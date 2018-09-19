__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

# from Viewer import *  # Imports the environment of the viewer
# from AMCL import * # Imports the Particle Filter Class
import time

# Import a shared memory
import sys
sys.path.append('../../Blackboard/src/')
from SharedMemory import SharedMemory

try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser

# To pass arguments to the function
import argparse
# To parse arguments on execution
parser = argparse.ArgumentParser(
    description='MindController',
    epilog='Takes control of ones mind!'
)
parser.add_argument(
    '-n',
    '--n',
    type=int,
    help='Quantity of experiments realized.'
)
parser.add_argument(
    '-q',
    '--q',
    type=int,
    help='Quantity of particles used for the sixth experiment.'
)
parser.add_argument(
    '-s',
    '--solo',
    type=int,
    help='Chooses which experiment will be executed.'
)

args = parser.parse_args()

if args.n is None:
    args.n = 1

# -----------------------------------------------------------------------------
#   Class implementing the Core of the Localization Process
# -----------------------------------------------------------------------------


class MINDREADER():
    def __init__(self):
        self.bkb = SharedMemory()  # Instance of a blackboard
        config = ConfigParser()  # Configuration file

        try:
            # Reads the config archive
            config.read('../../Control/Data/config.ini')
            # Get the memory key
            mem_key = int(config.get('Communication', 'no_player_robofei')) * 100
        except:
            print "#----------------------------------#"
            print "#   Error loading config parser.   #"
            print "#----------------------------------#"
            sys.exit()

        # Create the link to the blackboard
        self.Mem = self.bkb.shd_constructor(mem_key)

    def main(self):
        try:
            self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 0)
            if args.solo == 0 or args.solo is None:
                # Experiment zero - reference
                print 'mindcontroller: Experiment zero - Pure MCL'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 100)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 120:
                        time.sleep(0.1)

                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 1 or args.solo is None:
                # Experiment one - kidnap
                print 'mindcontroller: Experiment one - Kidnap with A-MCL'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 101)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 121:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 2 or args.solo is None:
                # Experiment two - kidnap
                print 'mindcontroller: Experiment two - Kidnap with SR'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 102)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 122:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 3 or args.solo is None:
                # Experiment three - kidnap
                print 'mindcontroller: Experiment three - Kidnap with W-MCL'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 103)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 123:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 4 or args.solo is None:
                # Experiment Four - Quantity of particles
                print 'mindcontroller: Experiment four - KLD-MCL'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 104)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 124:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 5 or args.solo is None:
                # Experiment Five - Quantity of particles
                print 'mindcontroller: Experiment five - SD-MCL'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 105)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 125:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 6 or args.solo is None and args.q is not None:
                # Experiment Six - Quantity of particles
                print 'mindcontroller: Experiment six - MCL - Static'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 106)
                    # Waits until localization asks for a number.
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 196:
                        time.sleep(0.1)
                    # Answers with the number.
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', -args.q)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 126:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 7 or args.solo is None:
                # Experiment seven - Literature
                print 'mindcontroller: Experiment seven - Literature Implementation'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 107)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 127:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 8 or args.solo is None:
                # Experiment eight - Mine
                print 'mindcontroller: Experiment eight - without VIP'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 108)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 128:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

            if args.solo == 9 or args.solo is None:
                # Experiment Nine - Mine
                print 'mindcontroller: Experiment nine - Proposal'
                for i in xrange(args.n):
                    # Sets up the experiment
                    print '\tSetting up experiment', i
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 109)
                    # Waits until all processes are ready
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 129:
                        time.sleep(0.1)
                    
                    print '\x1b[1F\x1b[2K\tRunning experiment', i
                    # Starts the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 200)

                    # State Machine
                    self.FirstTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
                    print 'Kidnapping!'
                    # Jump
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 600)
                    # Wait
                    time.sleep(1)
                    # State Machine
                    self.SecondTrack()
                    # Stop
                    self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)

                    print '\x1b[1F\x1b[2K\tFinishing experiment', i
                    # Finishes the experiment
                    self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 300+i)
                    while self.bkb.read_int(self.Mem, 'LOCALIZATION_WORKING') != 500+i:
                        time.sleep(0.1)

        except:
            print '\n\nLocalization: Force quit!'
            self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 0)
        finally:
            print '\n\nFinishing the Experiments'
            self.bkb.write_int(self.Mem, 'LOCALIZATION_WORKING', 900)
            exit()

    def FirstTrack(self):
        # WalkForward 16 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 1)
        time.sleep(16)

        # Turn Right 10 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 3)
        time.sleep(10)

        # WalkForward 15 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 1)
        time.sleep(15)

        # Turn Left 4 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 2)
        time.sleep(4)

        # WalkForward 15 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 1)
        time.sleep(15)

    def SecondTrack(self):
        # Walkforward 25 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 1)
        time.sleep(25)

        # Turnleft 10 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 2)
        time.sleep(10)

        # Walkforward 25 secs
        self.bkb.write_int(self.Mem, 'DECISION_ACTION_A', 1)
        time.sleep(25)

# Call the main function, start up the simulation
if __name__ == "__main__":
    MR = MINDREADER()
    MR.main()
