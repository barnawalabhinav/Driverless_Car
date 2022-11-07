import random
import util
import math
from util import Belief, pdf
from engine.const import Const

# Class: Estimator
# ----------------------
# Maintain and update a belief distribution over the probability of a car being in a tile.


class Estimator(object):
    def __init__(self, numRows: int, numCols: int):
        self.belief = util.Belief(numRows, numCols)
        self.transProb = util.loadTransProb()
        self.particles = [i*numCols + j for j in range(numCols) for i in range(numRows) for k in range(2)]

    ##################################################################################
    # [ Estimation Problem ]
    # Function: estimate (update the belief about a StdCar based on its observedDist)
    # ----------------------
    # Takes |self.belief| -- an object of class Belief, defined in util.py --
    # and updates it *inplace* based on the distance observation and your current position.
    #
    # - posX: x location of AutoCar
    # - posY: y location of AutoCar
    # - observedDist: current observed distance of the StdCar
    # - isParked: indicates whether the StdCar is parked or moving.
    #             If True then the StdCar remains parked at its initial position forever.
    #
    # Notes:
    # - Carefully understand and make use of the utilities provided in util.py !
    # - Remember that although we have a grid environment but \
    #   the given AutoCar position (posX, posY) is absolute (pixel location in simulator window).
    #   You might need to map these positions to the nearest grid cell. See util.py for relevant methods.
    # - Use util.pdf to get the probability density corresponding to the observedDist.
    # - Note that the probability density need not lie in [0, 1] but that's fine,
    #   you can use it as probability for this part without harm :)
    # - Do normalize self.belief after updating !!

    ###################################################################################
    def estimate(self, posX: float, posY: float, observedDist: float, isParked: bool) -> None:

        # BEGIN_YOUR_CODE

        rows = self.belief.numRows
        cols = self.belief.numCols

        weights = [0 for k in range(len(self.particles))]
        for particle in self.particles:
            y = util.rowToY(particle // cols)
            x = util.colToX(particle % cols)
            estimated_dist = math.sqrt(
                (x-posX) * (x-posX) + (y-posY) * (y-posY))
            weights[particle] = 1 / abs(estimated_dist - observedDist)

        for r in range(rows):
            for c in range(cols):
                self.belief.setProb(r, c, 0)

        self.particles = random.choices(self.particles, weights, k=len(self.particles))
        for particle in self.particles:
            r = particle // cols
            c = particle % cols
            self.belief.addProb(r, c, 1)
        self.belief.normalize()

        if not isParked:
            for i in range(len(self.particles)):
                particle = self.particles[i]
                y = particle // cols
                x = particle % cols
                moving_prob = [ self.transProb[((y, x), (y-1, x-1))] if ((y, x), (y-1, x-1)) in self.transProb else 0,
                                self.transProb[((y, x), (y, x-1))] if ((y, x), (y, x-1)) in self.transProb else 0,
                                self.transProb[((y, x), (y+1, x-1))] if ((y, x), (y+1, x-1)) in self.transProb else 0,
                                self.transProb[((y, x), (y-1, x))] if ((y, x), (y-1, x)) in self.transProb else 0,
                                self.transProb[((y, x), (y, x))] if ((y, x), (y, x)) in self.transProb else 0,
                                self.transProb[((y, x), (y+1, x))] if ((y, x), (y+1, x)) in self.transProb else 0,
                                self.transProb[((y, x), (y-1, x+1))] if ((y, x), (y-1, x+1)) in self.transProb else 0,
                                self.transProb[((y, x), (y, x+1))] if ((y, x), (y, x+1)) in self.transProb else 0,
                                self.transProb[((y, x), (y+1, x+1))] if ((y, x), (y+1, x+1)) in self.transProb else 0]
                total = sum(moving_prob)
                if total == 0:
                    # for i in [-1, 0, 1]:
                    #     for j in [-1, 0, 1]:
                    #         if x+i >= 0 and x+i < cols and y+j >= 0 and y+j < rows:
                    #             moving_prob[(i+1)*3+(j+1)] = 1
                    #             total += 1
                    moving_prob[4] = 1
                else:
                    moving_prob = [prob/total for prob in moving_prob]
                self.particles[i] = random.choices(
                    [x-1+(y-1)*cols, x-1+y*cols, x-1+(y+1)*cols, x+(y-1)*cols, x+y*cols, x+(y+1)*cols, x+1+(y-1)*cols, x+1+y*cols, x+1+(y+1)*cols], moving_prob, k=1)[0]
                    # [y-1+(x-1)*cols, y+(x-1)*cols, y+1+(x-1)*cols, y-1+x*cols, y+x*cols, y+1+x*cols, y-1+(x+1)*cols, y+(x+1)*cols, y+1+(x+1)*cols], moving_prob, k=1)[0]

        # END_YOUR_CODE
        return

    def getBelief(self) -> Belief:
        return self.belief
