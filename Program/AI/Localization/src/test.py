# coding: utf-8

__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

from Viewer import *  # Imports the environment of the viewer
from MCL import *
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

        PF = MonteCarlo(aslow=0.03, afast=0.1, SensorReset=True, KLD=True)

        while True:

            std = 100
            self.bkb.write_int(self.Mem, 'DECISION_LOCALIZATION', -999)
            weight = 1

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

            orientation = self.bkb.read_float(self.Mem, 'IMU_EULER_Z')

            if (distances is None):
                z = [None, None]
            else:
                z = [distances, orientation]

            pos, std = PF.main(u, z)
            hp = 0

            if PF.meanweight < 1:
                weight = np.log(0.05) / np.log(PF.meanweight)

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

            if self.args.graphs:
                # Redraws the screen background
                field.draw_soccer_field()
                try:
                    simul.DrawStd(pos, std, weight, hp)
                    # Draws all particles on screen
                    simul.display_update(PF.particles)
                except:  # noqa E722
                    print "Graphical error!!!"

            # Updates for the next clock
            screen.clock.tick(5)

    # -------------------------------------------------------------------------
    #   This method returns a command instruction to the particles.
    # -------------------------------------------------------------------------
    def GetU(self, Action):
        if Action in [0, 4, 5, 12, 13, 19, 20, 21, 22]:
            return (0, 0, 0, 0, self.dt())  # Stop or kick
        elif Action == 11:
            return (0, 0, 0, 1, self.dt())  # Gait
        elif Action == 1:
            return (20, 0, 0, 1, self.dt())  # Fast Walk Forward
        elif Action == 8:
            return (10, 0, 0, 1, self.dt())  # Slow Walk Forward
        elif Action == 17:
            return (-20, 0, 0, 1, self.dt())  # Fast Walk Backward
        elif Action == 18:
            return (-10, 0, 0, 1, self.dt())  # Slow Walk Backward
        elif Action == 6:
            return (0, -10, 0, 1, self.dt())  # Walk Left
        elif Action == 7:
            return (0, 10, 0, 1, self.dt())  # Walk Right
        elif Action == 3:
            return (0, 0, 10, 1, self.dt())  # Turn Right
        elif Action == 2:
            return (0, 0, -10, 1, self.dt())  # Turn Left
        elif Action == 9:
            return (0, -10, -20, 1, self.dt())  # Turn Left Around the Ball
        elif Action == 14:
            return (0, 10, 20, 1, self.dt())  # Turn Right Around the Ball
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
