__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

import numpy as np
from particle import *


# -----------------------------------------------------------------------------
#   This class implements the Monte Carlo's Particle Filter
#   - Note that this is the most simples version of Monte Carlo Localization
# -----------------------------------------------------------------------------

class MonteCarlo():
    # ----------------------------------------------------------------------------------------------
    #   Constructor of the particle filter
    # ----------------------------------------------------------------------------------------------
    def __init__(self, max_qtd=1000, min_qtd=30, kidnap=True):
        # Holds the particles objects
        self.particles = []

        # Limits the quantity of particles the filter will have
        self.max_qtd = max_qtd
        self.min_qtd = min_qtd

        # Initializes with the max quantity of particles
        self.qtd = max_qtd

        # self.particles.append(Particle(350,350,0,factors=15*[0]))
        for i in range(self.qtd):
            # Randomly generates n particles
            if kidnap:
                self.particles.append(Particle())
            else:
                # self.particles.append(Particle(normals=[[[250,10],[370,10],[180,5]]]))
                self.particles.append(Particle(factors=[1, 2, 1, 0, 5,
                                                        1, 2, 1, 0, 7,
                                                        1, 2, 1, 0, 5]))

        self.totalweight = 0.  # Holds the total sum of particles' weights.
        self.meanweight = 0.

        # Holds the mean position of the estimated position.
        self.mean = [0, 0, 0]
        self.std = 1.

        self.VQPsigma = 3
        self.VQPmean = 10

    # -------------------------------------------------------------------------
    #   Prediction step
    # -------------------------------------------------------------------------
    def Prediction(self, u=None):
        # If there was movement, run the prediction step
        if u is not None:
            for particle in self.particles:
                particle.Movement(*u, meanw=self.meanweight)

    # -------------------------------------------------------------------------
    #   Update step
    # -------------------------------------------------------------------------
    def Update(self, z=None):
        # Clears the last total weight
        self.totalweight = 0

        # Applies the observation model to each particle
        for particle in self.particles:
            self.totalweight += particle.Sensor(*z)

    # -------------------------------------------------------------------------
    #   Resample step
    # -------------------------------------------------------------------------
    def Resample(self, qtd):
        parts = []  # Starts a empty list.

        np.random.shuffle(self.particles)  # Shuffles the particles in place.

        step = self.totalweight / (qtd + 1.)  # Computes the step size
        s = 0  # the first step is given by half the total.

        poses = []  # Holds all positions to compute the standard deviation

        i = 1  # Counts the quantity of selected particles
        j = len(self.particles)  # Counts down the quantity of particles
        # Until the quantity of particles is reached
        #   or there are no more particles to be selected

        self.meanweight = 0
        while i <= qtd and j >= 0:
            # If the cum. sum of steps is bigger than the cum. sum of weights
            if step * i > s:
                j -= 1  # Change the particle to be tested
                s += self.particles[j].weight  # Compute the new cum. weight
            else:
                i += 1  # Moves one step
                p = self.particles[j]  # Gets the particle
                # adds the particle to the list.
                parts.append(Particle(*p.copy(), factors=p.factors))
                self.meanweight += p.weight

                # Saves the position for computing the standard deviation
                poses.append([p.x, p.y,
                              np.cos(np.radians(p.rotation)) +
                              np.sin(np.radians(p.rotation)) * 1j])

        self.particles = parts  # Overwrites the previous particles.
        self.qtd = len(self.particles)  # Saves the new quantity of particles.

        self.totalweight = self.meanweight
        self.meanweight /= self.qtd

        m = np.mean(poses, 0)  # Computes the mean of the particles.

        self.mean[0] = int(np.real(m[0]))  # Get the mean x
        self.mean[1] = int(np.real(m[1]))  # Get the mean y
        self.mean[2] = int(np.angle(m[2], True))  # Get the mean angle

        poses = np.matrix(poses - m)  # Compute the error of each particle

        # Compute the standard deviation of the particle set
        self.std = np.power(np.sqrt(np.abs(np.linalg.det(((poses.T * poses) /
                            (self.qtd + 1))))), 1 / 3.)

    # -------------------------------------------------------------------------
    #   Tests which is the best information to be acquired.
    # -------------------------------------------------------------------------
    def PerfectInformation(self, u, pos, time=0):
        np.random.shuffle(self.particles)

        pan = [-90, 0, 90]
        utility = len(pan) * [0]

        poses = []
        for P in self.particles[0:30]:
            poses.append(Particle(*P.copy(), factors=P.factors))

        st = []
        for p in poses:
            uf = [u[0], u[1], u[2], time]
            p.Movement(*uf, meanw=self.meanweight)
            st.append([p.x, p.y,
                       np.cos(np.radians(p.rotation)) +
                       np.sin(np.radians(p.rotation)) * 1j])

        st = np.array(st)
        st = np.matrix(st - np.mean(st, 0))
        st = np.sqrt(np.abs(np.linalg.det(((st.T * st) / (31.)))))

        for c in xrange(len(pan)):
            z = [poses[0].GetDistance(pan[c]), poses[0].rotation]
            tw = 0
            for P in poses:
                tw += P.Sensor(*z)

            step = tw / (31.)
            s = 0

            i = 1
            j = 30

            nstd = []
            while i <= 31 and j >= 0:
                # If the cum. sum is bigger than the cum. sum of weights
                if step * i > s:
                    j -= 1  # Change the particle to be tested
                    s += poses[j].weight  # Compute the new cumulative weight
                else:
                    i += 1  # Moves one step
                    p = poses[j]
                    # Saves the position for computing the standard deviation
                    nstd.append([p.x, p.y,
                                 np.cos(np.radians(p.rotation)) +
                                 np.sin(np.radians(p.rotation)) * 1j])

            nstd = np.array(nstd)
            nstd = np.matrix(nstd - np.mean(nstd, 0))
            nstd = np.sqrt(np.abs(np.linalg.det(((nstd.T * nstd) / (31.)))))

            utility[c] = max(st - nstd - 30 * np.abs(pos - pan[c]) / 180., 0)

        return pan[np.argmax(utility)]

        rnd = np.random.random() * sum(utility)

        for c in xrange(len(pan)):
            if rnd < utility[c]:
                return pan[c]
            rnd -= utility[c]

        return -999

    # -------------------------------------------------------------------------
    #   Main algorithm
    # -------------------------------------------------------------------------
    def main(self, u=None, z=None):
        self.Prediction(u)  # Executes the prediction
        self.Update(z)  # Updates particles' weights
        self.Resample(self.qtd)  # Resamples the particles
        # Computes the quantity based on the standard deviation
        self.qtd = Qtd(self.std, mini=self.min_qtd, maxi=self.max_qtd)

        return self.mean, self.std  # Returns everything.


# -----------------------------------------------------------------------------
#   Computes the quantity of particles in function of the standard deviation
# -----------------------------------------------------------------------------
def Qtd(std, mean=6, sigma=2, mini=30, maxi=1000):
    return np.rint(np.max([mini, np.min([maxi,
                   std * (maxi - mini) / (4. * sigma) +
                   mini + (maxi - mini) / 2. -
                   (maxi - mini) * mean / (4. * sigma)])]))
