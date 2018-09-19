# coding: utf-8

__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

from Viewer import *  # Imports the environment of the viewer
import MCL as MY
import LMCL as LIT
import time

# To pass arguments to the function
import argparse
# Import a shared memory
import sys
sys.path.append('../../Blackboard/src/')
from SharedMemory import SharedMemory # noqa E402

try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser

# To parse arguments on execution
parser = argparse.ArgumentParser(
    description='Robot Localization',
    epilog='Implements particle filters to self-localize a robot on the field.'
)
parser.add_argument(
    '-g',
    '--graphs',
    action="store_true",
    help='Shows graphical interface which visualizes the particles.'
)
parser.add_argument(
    '-l',
    '--log',
    action="store_true",
    help='Print variable logs.'
)

args = parser.parse_args()


# -----------------------------------------------------------------------------
#   Class implementing the Core of the Localization Process
# -----------------------------------------------------------------------------


class Localization():
    # -------------------------------------------------------------------------
    #   Class constructor and pre-processing.
    # -------------------------------------------------------------------------
    def __init__(self):
        self.bkb = SharedMemory()  # Instance of a blackboard
        config = ConfigParser()  # Configuration file

        try:
            # Reads the config archive
            config.read('../../Control/Data/config.ini')
            # Get the memory key
            mem_key = int(config.get('Communication', 'no_player_robofei')) * 100  # noqa E501
        except:  # noqa E722
            print "#----------------------------------#"
            print "#   Error loading config parser.   #"
            print "#----------------------------------#"
            sys.exit()

        # Create the link to the blackboard
        self.Mem = self.bkb.shd_constructor(mem_key)

        self.args = parser.parse_args()

        # Timestamp to use on the time step used for motion
        self.timestamp = time.time()

        # Clears the variables in the blackboard
        self.bkb.write_float(self.Mem, 'fVISION_FIELD', 0)

    # -------------------------------------------------------------------------
    #   Localization's main method.
    # -------------------------------------------------------------------------
    def main(self):
        # Creates a new screen
        screen = Screen(self.args.graphs)

        if self.args.graphs:
            simul = Simulation(screen)  # Creates the interface structure
            field = SoccerField(screen)  # Draws the field
            simul.field = field  # Passes the field to the simulation

        save = []
        exp = 0
        hpos = 0

        while True:
            # Setup the experiment
            bhv = self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING')
            if bhv >= 110 and bhv < 120:
                print 'Localization: Setting up experiment.'
                vipflag = False
                if bhv == 110:
                    PF = MY.MonteCarlo(1000, 1000, False)
                    vipflag = True
                    exp = 0
                elif bhv == 111:
                    PF = LIT.MonteCarlo(aslow=0.03, afast=0.1)
                    exp = 100
                elif bhv == 112:
                    PF = LIT.MonteCarlo(SensorReset=True)
                    exp = 200
                elif bhv == 113:
                    PF = MY.MonteCarlo(min_qtd=1000, kidnap=True)
                    exp = 300
                elif bhv == 114:
                    PF = LIT.MonteCarlo(KLD=True)
                    exp = 400
                elif bhv == 115:
                    PF = MY.MonteCarlo(kidnap=False)
                    exp = 500
                elif bhv == 116:
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 196)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') > 0:
                        time.sleep(0.1)
                    sixth = -self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING')
                    PF = MY.MonteCarlo(sixth, sixth, False)  # Change quantity
                    exp = 600
                elif bhv == 117:
                    PF = LIT.MonteCarlo(aslow=0.03, afast=0.1, SensorReset=True, KLD=True)  # noqa E501
                    exp = 700
                elif bhv == 118:
                    PF = MY.MonteCarlo()
                    exp = 800
                elif bhv == 119:
                    PF = MY.MonteCarlo()
                    vipflag = True
                    exp = 900

                std = 100
                hp = -999
                self.bkb.write_int(self.Mem, 'DECISION_LOCALIZATION', -999)
                weight = 1
                self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', bhv + 10)
                save = []

            if bhv == 200 or bhv == 600:

                # Process interactions events
                if self.args.graphs:
                    simul.perform_events()

                # Gets the motion command from the blackboard.
                u = self.GetU(self.bkb.read_int(self.Mem, 'CONTROL_ACTION'))

                x = self.bkb.read_float(self.Mem, 'fVISION_FIELD')
                self.bkb.write_float(self.Mem, 'fVISION_FIELD', 0)
                if x == 0:
                    distances = None
                else:
                    distances = x
                    hpos = np.int(x)

                orientation = self.bkb.read_float(self.Mem, 'IMU_EULER_Z')

                if (distances is None):
                    z = [None, None]
                else:
                    z = [distances, orientation]

                deltat = time.time()
                pos, std = PF.main(u, z)
                deltat = time.time() - deltat

                if not vipflag:
                    hp = -999
                    self.bkb.write_int(self.Mem, 'DECISION_LOCALIZATION', -999)
                elif (
                    distances is not None or
                    self.bkb.read_int(self.Mem, 'DECISION_LOCALIZATION') == 999
                ):
                    hp = PF.PerfectInformation(u, hpos, 1.)
                    self.bkb.write_int(self.Mem, 'DECISION_LOCALIZATION', hp)

                if PF.meanweight < 1:
                    weight = np.log(0.05) / np.log(PF.meanweight)

                save.append([time.time(), pos[0], pos[1], pos[2], std, PF.qtd, deltat])  # noqa E501

                if self.args.log:
                    print '\t\x1b[32mRobot at',  # Prints header
                    print 'ent\x1b[32m[x:\x1b[34m{} cm'.format(int(pos[0])),
                    print '\x1b[32m| y:\x1b[34m{} cm'.format(int(pos[1])),
                    print u'\x1b[32m| \u03B8:\x1b[34m{}\u00B0'.format(int(pos[2])),  # noqa E501
                    print u'\x1b[32m| \u03C3:\x1b[34m{} cm\x1b[32m]'.format(int(std))  # noqa E501

                # Write the robot's position on Black Board to be read by telemetry # noqa E501
                self.bkb.write_int(self.Mem, 'LOCALIZATION_X', int(pos[0]))
                self.bkb.write_int(self.Mem, 'LOCALIZATION_Y', int(pos[1]))
                self.bkb.write_int(self.Mem, 'LOCALIZATION_THETA', int(pos[2]))
                self.bkb.write_float(self.Mem, 'LOCALIZATION_RBT01_X', std)

            if bhv >= 400 and bhv < 500:
                print 'Localization: Saving data.'
                # Finish
                i = bhv - 400
                np.save(
                    '../../../../Experiments/loca' + str(exp + i),
                    np.array(save))
                self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', bhv + 100)

            if bhv == 910:
                print 'Localization: Finalized by the mindcontroller.'
                self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 0)
                exit()

            if self.args.graphs:
                # Redraws the screen background
                field.draw_soccer_field()
                try:
                    simul.DrawStd(pos, std, weight, hp)
                    # Draws all particles on screen
                    simul.display_update(PF.particles)
                except:  # noqa E722
                    pass

            # Updates for the next clock
            screen.clock.tick(5)

    # -------------------------------------------------------------------------
    #   This method returns a command instruction to the particles.
    # -------------------------------------------------------------------------
    def GetU(self, Action):
        fct = 0.9
        if Action in [0, 4, 5, 12, 13, 19, 20, 21, 22]:
            return (0, 0, 0, 0, self.dt())  # Stop or kick
        elif Action == 11:
            return (0, 0, 0, 1, self.dt())  # Gait
        elif Action == 1:
            return (fct * 20, 0, 0, 1, self.dt())  # Fast Walk Forward
        elif Action == 8:
            return (fct * 10, 0, 0, 1, self.dt())  # Slow Walk Forward
        elif Action == 17:
            return (fct * -20, 0, 0, 1, self.dt())  # Fast Walk Backward
        elif Action == 18:
            return (fct * -10, 0, 0, 1, self.dt())  # Slow Walk Backward
        elif Action == 6:
            return (0, fct * -10, 0, 1, self.dt())  # Walk Left
        elif Action == 7:
            return (0, fct * 10, 0, 1, self.dt())  # Walk Right
        elif Action == 3:
            return (0, 0, fct * 10, 1, self.dt())  # Turn Right
        elif Action == 2:
            return (0, 0, fct * -10, 1, self.dt())  # Turn Left
        elif Action == 9:
            return (0, fct * -10, fct * -20, 1, self.dt())  # Turn Left Around the Ball
        elif Action == 14:
            return (0, fct * 10, fct * 20, 1, self.dt())  # Turn Right Around the Ball
        elif Action == 16:
            return (0, 0, 0, 2, self.dt())  # Get up, back up
        elif Action == 15:
            return (0, 0, 0, 3, self.dt())  # Get up, front up
        elif Action == 10:
            print "ERROR - Please, edit Localization.GetU() for Goalkeeper before resuming!"  # noqa E501
            return (0, 0, 0, 0, self.dt())

    # -------------------------------------------------------------------------
    #   This method returns the time since the last update
    # -------------------------------------------------------------------------
    def dt(self):
        auxtime = time.time()
        timer = auxtime - self.timestamp
        self.timestamp = auxtime
        return timer


# Call the main function, start up the simulation
if __name__ == "__main__":
    Loc = Localization()
    Loc.main()
