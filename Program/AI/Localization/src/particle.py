__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

import numpy as np
import scipy.special as sp

# field = ((8, 1032), (60, 680), (-180, 180))
field = ((0, 1040), (0, 740), (-180, 180))


# -----------------------------------------------------------------------------
#   Class implementing a particle used on Particle Filter Localization
# -----------------------------------------------------------------------------

class Particle(object):
    # -------------------------------------------------------------------------
    #   Particle constructor
    # -------------------------------------------------------------------------
    def __init__(self,
                 x=None,
                 y=None,
                 rotation=None,
                 wfactor=0,
                 weight=1,
                 normals=None,
                 regions=None,
                 factors=None,
                 std=None,
                 spread=1):

        # This block sets the initial position values of the particles.
        #    If there was any given value, adopt it;
        #    else if there was a gaussian possible position given,
        #    generate a random position;
        #    else create a totally random one.

        # Note: normals is a Nx3x2 matrix, where
        #    the first line presents the mean and standard deviation of the x
        #    the second line presents the mean and standard deviation of the y
        #    the third line presents the coefficients of the rotation

        # Note2: regions is a 3x2 matrix, where
        #    the first line presents the min and max values of the x position
        #    the second line presents the min and max values of the y position
        #    the third line presents the min and max values of the rotation

        # Note3: factors are the factors for the motion model of the particles

        # Note4: std is a vector with the values used as standard deviation
        #    for computing particles' likelihood.
        #    the first for is used for the landmarks,
        #    in sequence blue, red, yellow, purple
        #    the last one is used for the IMU orientation

        # Note5: spread determines how much the particles will spread

        if regions is None:
            self.regions = field
        else:
            self.regions = regions

        if normals is not None:
            i = np.random.randint(len(normals))

        if x is not None:
            self.x = x
        elif normals is not None:
            self.x = np.random.normal(normals[i][0][0], normals[i][0][1])
        else:
            self.x = np.random.randint(self.regions[0][0], self.regions[0][1])

        if y is not None:
            self.y = y
        elif normals is not None:
            self.y = np.random.normal(normals[i][1][0], normals[i][1][1])
        else:
            self.y = np.random.randint(self.regions[1][0], self.regions[1][1])

        if rotation is not None:
            self.rotation = rotation
        elif normals is not None:
            self.rotation = np.random.normal(normals[i][2][0],
                                             normals[i][2][1])
        else:
            self.rotation = np.random.randint(self.regions[2][0],
                                              self.regions[2][1])

        # Holds particles weight, can come from previous iterations
        self.weight = weight

        # Motion error coefficients
        if factors is None:
            self.factors = [1, 2, 1, 500, 5,
                            1, 2, 1, 500, 7,
                            1, 2, 1, 100, 5]
            # self.factors = 15*[0]
            # self.factors = [1, 2, 1, 0, 10,  1, 2, 1, 0, 20,  1, 2, 1, 0, 10]
            # self.factors = [0, 0, 0, 0, 10,  0, 0, 0, 0, 20,  0, 0, 0, 0, 10]
            # self.factors = [0, 0, 0, 50, 0,  0, 0, 0, 50, 0,  0, 0, 0, 10, 0]
        else:
            self.factors = factors

        # Standard deviation used for computing angles likelihoods, in degrees.
        if std is None:
            self.std = [5, 30]
        else:
            self.std == std

        # Vars used to compute the particles likelihood
        self.gamma = 26.13
        self.Delta = 90

        self.psi = 0.7
        self.SigmaO = 70
        self.MuF = 700
        self.SigmaF = 10
        self.MuN = 10
        self.SigmaN = 1

        self.SigmaA = 5

        self.radius = (10, 50)

        self.SigmaIMU = 20

        # Used in order to implement the motion error.
        self.wfactor = wfactor

        # Probability factors used to compute particles weights.
        self.probfactors = (1, 0, 1, 1, 0.1, 0.3)
        # 0 - Factor
        # 1 - Factor
        # 2 - 0 0
        # 3 - 1 1
        # 4 - 1 0
        # 5 - 0 1

    # -------------------------------------------------------------------------
    #   Method that chooses which movement should be used
    # -------------------------------------------------------------------------
    def Movement(self,
                 straight=0,
                 drift=0,
                 rotational=0,
                 moving=1,
                 dt=0,
                 meanw=1):
        if moving == 1:
            self.Motion(straight, drift, rotational, moving, dt, meanw)
        elif moving == 2:
            self.GetUpBackUp()
        elif moving == 3:
            self.GetUpFrontUp()
        else:
            self.Motion(0, 0, 0, 0, dt, meanw)

    # -------------------------------------------------------------------------
    #   Method which moves particles around, reimplement.
    # -------------------------------------------------------------------------
    def Motion(self,
               straight=0,
               drift=0,
               rotational=0,
               moving=0,
               dt=0,
               meanw=1):
        # straight is the robot's forward speed in cm/s
        # drift is the robot's sideways speed in cm/s
        # rotational is the robot's angular speed in degrees/s

        # Computes the forward speed with error
        Forward = (straight +
                   NRnd(self.factors[0] * straight) +
                   NRnd(self.factors[1] * drift) +
                   NRnd(self.factors[2] * rotational) +
                   NRnd(self.factors[3] * self.wfactor) +
                   NRnd(self.factors[4]))

        # Computes the side speed with error
        Side = (drift +
                NRnd(self.factors[5] * straight) +
                NRnd(self.factors[6] * drift) +
                NRnd(self.factors[7] * rotational) +
                NRnd(self.factors[8] * self.wfactor) +
                NRnd(self.factors[9]))

        # Computes the angular speed with error
        Omega = (rotational +
                 NRnd(self.factors[10] * straight) +
                 NRnd(self.factors[11] * drift) +
                 NRnd(self.factors[12] * rotational) +
                 NRnd(self.factors[13] * self.wfactor) +
                 NRnd(self.factors[14]))

        # Converts angles to radians
        Omega = np.radians(Omega)
        Theta = np.radians(self.rotation)

        # Computes the new positions
        if Omega == 0:
            Direction = Theta
            x = (self.x +
                 Forward * np.cos(Theta) * dt +
                 Side * np.sin(Theta) * dt)
            y = (self.y -
                 Forward * np.sin(Theta) * dt +
                 Side * np.cos(Theta) * dt)
        else:
            Direction = Theta + Omega * dt
            Dir2 = -Theta + Omega * dt
            x = (self.x +
                 Forward / Omega * (np.sin(Direction) - np.sin(Theta)) -
                 Side / Omega * (np.cos(-Theta) - np.cos(Dir2)))
            y = (self.y -
                 Forward / Omega * (np.cos(Theta) - np.cos(Direction)) -
                 Side / Omega * (np.sin(-Theta) - np.sin(Dir2)))

        if (x < self.regions[0][0] or
            x > self.regions[0][1] or
            y < self.regions[1][0] or
            y > self.regions[1][1]):  # noqa E129
            return

        # Saves the new positions
        self.x = x
        self.y = y
        rot = np.degrees(Direction)
        if rot > 180:
            self.rotation = rot - 360
        elif rot < -180:
            self.rotation = rot + 360
        else:
            self.rotation = rot

    # -------------------------------------------------------------------------
    #   Method to replace particles after rising up
    # -------------------------------------------------------------------------
    def GetUpBackUp(self):
        self.x += NRnd(7)
        self.y += NRnd(7)
        self.rotation += NRnd(25)

    # -------------------------------------------------------------------------
    #   Method which replaces particles after turning on the ground
    # -------------------------------------------------------------------------
    def GetUpFrontUp(self):
        self.x += NRnd(7, -30) * np.sin(np.radians(self.rotation))
        self.y += NRnd(7, -30) * np.cos(np.radians(self.rotation))
        self.rotation += NRnd(25)
        self.GetUpBackUp()

    # -------------------------------------------------------------------------
    #   Likelihood computation
    # -------------------------------------------------------------------------
    def Sensor(self,
               distances=None,
               orientation=None,
               weight=1):
        factorweight = 1
        fwl = -1

        if distances is not None:
            pan = int(distances)
            aux = np.abs(pan - distances)
            dist = 1.0 / aux

            alpha = np.radians(-self.rotation + pan)
            alpha = np.abs(np.arctan2(np.sin(alpha), np.cos(alpha)))

            beta = np.radians(90 - self.rotation + pan)  # y -> 0
            beta = np.abs(np.arctan2(np.sin(beta), np.cos(beta)))

            gamma = np.radians(-90 - self.rotation + pan)  # y -> 740
            gamma = np.abs(np.arctan2(np.sin(gamma), np.cos(gamma)))

            delta = np.radians(180 - self.rotation + pan)  # x -> 0
            delta = np.abs(np.arctan2(np.sin(delta), np.cos(delta)))

            maxd = 9999

            if alpha < np.pi / 2:
                maxd = min(maxd, (1040 - self.x) / np.cos(alpha))
            if beta < np.pi / 2:
                maxd = min(maxd, self.y / np.cos(beta))
            if gamma < np.pi / 2:
                maxd = min(maxd, (740 - self.y) / np.cos(gamma))
            if delta < np.pi / 2:
                maxd = min(maxd, self.x / np.cos(delta))

            weight *= np.exp(-np.power(dist - maxd, 2) / (2 * np.power(50, 2)))

            factorweight *= np.exp(fwl)

        # If the IMU's orientation was given
        if orientation is not None:
            weight *= (ComputeAngLikelihoodDeg(np.degrees(orientation),
                                              self.rotation,  # noqa E128
                                              self.SigmaIMU) /  # noqa E128
                       (np.sqrt(2 * np.pi) * self.SigmaIMU))
            factorweight *= np.exp(fwl) / (np.sqrt(2 * np.pi) * self.SigmaIMU)

        self.weight = max(weight, 1e-300)

        if orientation is not None or distances is not None:
            self.wfactor = max(min(np.log(factorweight / self.weight), 2.), 0.)

        return self.weight

    # -------------------------------------------------------------------------
    #   Computes the distances for predicting the future.
    # -------------------------------------------------------------------------
    def GetDistance(self, pan=0):
        alpha = np.radians(- self.rotation + pan)
        alpha = np.abs(np.arctan2(np.sin(alpha), np.cos(alpha)))

        beta = np.radians(90 - self.rotation + pan)  # y -> 0
        beta = np.abs(np.arctan2(np.sin(beta), np.cos(beta)))

        gamma = np.radians(-90 - self.rotation + pan)  # y -> 740
        gamma = np.abs(np.arctan2(np.sin(gamma), np.cos(gamma)))

        delta = np.radians(180 - self.rotation + pan)   # x -> 0
        delta = np.abs(np.arctan2(np.sin(delta), np.cos(delta)))

        maxd = 9999

        if alpha < np.pi / 2:
            maxd = min(maxd, (1040 - self.x) / np.cos(alpha))
        if beta < np.pi / 2:
            maxd = min(maxd, self.y / np.cos(beta))
        if gamma < np.pi / 2:
            maxd = min(maxd, (740 - self.y) / np.cos(gamma))
        if delta < np.pi / 2:
            maxd = min(maxd, self.x / np.cos(delta))

        if pan < 0:
            return int(pan) - 1.0 / maxd
        else:
            return int(pan) + 1.0 / maxd

    # -------------------------------------------------------------------------
    #   Returns a string to print the particle's representation.
    # -------------------------------------------------------------------------
    def __repr__(self):
        return ("x: " + str(self.x) +
                " y: " + str(self.y) +
                " z: " + str(self.rotation) +
                " w: " + str(self.weight))

    # -------------------------------------------------------------------------
    #   Returns the variables used to create a new particle from this one.
    # -------------------------------------------------------------------------
    def copy(self):
        return self.x, self.y, self.rotation, self.wfactor, self.weight


# -----------------------------------------------------------------------------
#   Computes the likelihood between two angles in degrees.
# -----------------------------------------------------------------------------
def ComputeAngLikelihoodDeg(ang, base, std_deviation=0):
    # Note: the standard deviation also is in degrees

    # If the standard deviation is null
    if std_deviation == 0:
        # return a binary answer.
        if ang == base:
            return 1
        else:
            return 0
    else:
        # else computes the Cartesian points based on the angles,
        xa = np.cos(np.radians(ang))
        ya = np.sin(np.radians(ang))
        xb = np.cos(np.radians(base))
        yb = np.sin(np.radians(base))

        # computes the distance between these points,
        d = np.hypot(xa - xb, ya - yb)

        # converts the standard deviation into aa distance measure,
        sa = np.cos(np.radians(std_deviation))
        sb = np.sin(np.radians(std_deviation))
        s = np.hypot(sa - 1, sb)

        # returns the likelihood between the given angles.
        return np.exp(-np.power(d, 2) / (2 * np.power(s, 2)))


# -----------------------------------------------------------------------------
#   Returns a random number from a normal distribution.
# -----------------------------------------------------------------------------
def NRnd(sigma, mu=0):
    """ Returns a random number obtained from a normal distribution with:
    mean = mu and standard deviation = sigma.
    """
    if sigma == 0:
        return mu
    else:
        return np.random.normal(mu, np.abs(sigma))
