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
    # -------------------------------------------------------------------------
    #   Constructor of the particle filter
    # -------------------------------------------------------------------------
    def __init__(self,
                 max_qtd=1000,
                 min_qtd=30,
                 SensorReset=False,
                 aslow=0., afast=0.,
                 KLD=False, epslon=0.05, zdelta=2.33):
        # Holds the particles objects
        self.particles = []

        # Limits the quantity of particles the filter will have
        self.max_qtd = max_qtd
        self.min_qtd = min_qtd

        # Initializes with the max quantity of particles
        self.qtd = max_qtd

        self.factors = [1, 2, 1, 0, 5,
                        1, 2, 1, 0, 7,
                        1, 2, 1, 0, 5]

        # self.particles.append(Particle(350,350,0,factors=15*[0]))
        for i in range(self.qtd):
            self.particles.append(Particle(factors=self.factors))

        # Sensor Resseting
        self.SR = SensorReset

        # Variables for the AMCL
        self.wslow = 0.
        self.wfast = 0.
        self.aslow = aslow
        self.afast = afast

        # Variables for the KLD sampler
        self.KLD = KLD
        self.e = epslon
        self.zd = zdelta
        self.bins = [50, 50, 10]

        # Holds the total sum of particles' weights.
        self.totalweight = 0.
        self.meanweight = 0.

        # Holds the mean position of the estimated position.
        self.mean = [0, 0, 0]
        self.std = 1.

    # -------------------------------------------------------------------------
    #   Prediction step
    # -------------------------------------------------------------------------
    def Prediction(self, u=None):
        # If there was movement, run the prediction step
        if u is not None:
            for particle in self.particles:
                particle.Movement(*u)

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
    def Resample(self, z=None):
        if z[0] is not None:
            parts = []  # Starts a empty list.

            np.random.shuffle(self.particles)  # Shuffles the particles in place.

            # Refreshing wfast and wslow
            self.wslow += self.aslow * (self.totalweight / self.qtd - self.wslow)
            self.wfast += self.afast * (self.totalweight / self.qtd - self.wfast)

            poses = []  # Holds all positions to compute the standard deviation

            b = {}
            M = 0

            while ((len(parts) < M or len(parts) < self.min_qtd or not self.KLD) and # noqa E501
                   len(parts) < self.max_qtd):
                p = None
                if np.random.random() <= max(0., 1. - self.wfast / self.wslow):
                    if self.SR:
                        p = Particle(*self.SensorResetting(z), factors=self.factors)
                    else:
                        p = Particle(factors=self.factors)
                else:
                    s = self.totalweight * np.random.random()
                    j = -1
                    while s > 0:
                        j += 1
                        s -= self.particles[j].weight

                    p = Particle(*self.particles[j].copy(), factors=self.factors)
                
                    poses.append([p.x, p.y,
                                  np.cos(np.radians(p.rotation)) +
                                  np.sin(np.radians(p.rotation)) * 1j])

                parts.append(p)

                if self.KLD:
                    hb = self.HashBins(p.x, p.y, p.rotation)
                    if hb not in b:
                        b.update({hb: True})
                        k = len(b)

                        if k > 1:
                            M = (k - 1) / (2 * self.e) * np.power(
                                1 - 2 / (9 * (k - 1)) +
                                np.sqrt(2 / (9 * (k - 1))) * self.zd, 3)

            if self.SR and not self.KLD and self.aslow + self.afast == 0.:
                for i in xrange(30):
                    p = Particle(*self.SensorResetting(z), factors=self.factors)
                    parts.append(p)
                    poses.append([p.x, p.y,
                                  np.cos(np.radians(p.rotation)) +
                                  np.sin(np.radians(p.rotation)) * 1j])

            self.particles = parts  # Overwrites the previous particles.
            self.qtd = len(self.particles)  # Saves the new quantity of particles.

            self.meanweight = self.totalweight / self.qtd

            if len(poses) > 0:
                m = np.mean(poses, 0)  # Computes the mean of the particles.

                self.mean[0] = int(np.real(m[0]))  # Get the mean x
                self.mean[1] = int(np.real(m[1]))  # Get the mean y
                self.mean[2] = int(np.angle(m[2], True))  # Get the mean angle

                poses = np.matrix(poses - m)  # Compute the error of each particle

                # Compute the standard deviation of the particle set
                self.std = np.power(np.sqrt(np.abs(np.linalg.det(((poses.T * poses) /
                                    (self.qtd + 1))))), 1 / 3.)

    def SensorResetting(self, z=None):
        if z[0] is not None:
            theta = np.degrees(z[1])
            pan = int(z[0])
            aux = np.abs(pan - z[0])
            dist = 1.0 / aux

            ang = theta + pan

            if ang > 180:
                ang -= 360
            elif ang < -180:
                ang += 360

            limits = [[10, 1030], [10, 730], [None, None]]
            dx = dist * np.cos(np.radians(ang))
            dy = dist * np.sin(np.radians(ang))

            if dx >= -10 and dx <= 10:
                if dy > 10:
                    limits[1] = [dy, dy]
                    limits[2][1] = dy
                elif dy < -10:
                    limits[1] = [740 - dy, 740 - dy]
                    limits[2][1] = 740 - dy
            elif dy >= -10 and dy <= 10:
                if dx > 10:
                    limits[0] = [1040 - dx, 1040 - dx]
                    limits[2][0] = 1040 - dx
                elif dx < -10:
                    limits[0] = [-dx, -dx]
                    limits[2][0] = -dx
            else:
                if dx > 0:
                    limits[2][0] = 1040 - dx
                    limits[0][1] = 1040 - dx
                else:
                    limits[2][0] = -dx
                    limits[0][0] = -dx

                if dy > 0:
                    limits[2][1] = dy
                    limits[1][0] = dy
                else:
                    limits[2][1] = 740 - dy
                    limits[1][1] = 740 - dy

            X = limits[2][0]
            Y = limits[2][1]

            if X is not None and Y is not None:
                if np.random.random() >= 0.5:
                    X = np.random.randint(limits[0][0], limits[0][1])
                else:
                    Y = np.random.randint(limits[1][0], limits[1][1])
            elif X is None:
                X = np.random.randint(limits[0][0], limits[0][1])
            elif Y is None:
                Y = np.random.randint(limits[1][0], limits[1][1])

            return NRnd(10, X), NRnd(10, Y), NRnd(3, theta)

    def HashBins(self, x, y, z):
        i = int(x) / int(self.bins[0])
        j = int(y) / int(self.bins[1])
        if z < 0:
            z += 360
        k = int(z) / int(self.bins[2])

        a = 1040 / int(self.bins[0])
        b = 740 / int(self.bins[1])

        return i + j * a + k * a * b

    # -------------------------------------------------------------------------
    #   Main algorithm
    # -------------------------------------------------------------------------
    def main(self, u=None, z=None):
        self.Prediction(u)  # Executes the prediction
        self.Update(z)  # Updates particles' weights
        self.Resample(z)  # Resamples the particles

        return self.mean, self.std  # Returns everything.
