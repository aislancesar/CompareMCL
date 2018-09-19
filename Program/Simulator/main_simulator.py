#!/usr/bin/env python

__author__ = "RoboFEI-HT"
__authors__ = "Danilo H. Perico, Thiago P. D. Homem, Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

from world import *
from simulation import *

# To pass arguments to the function
import argparse
# To parse arguments on execution
parser = argparse.ArgumentParser(
    description='RoboFEI-HT - Simulator',
    epilog='Implements a simulator used by the RoboFEI Humanoid Team for implementing and testing AI algorithms.'
)
parser.add_argument(
    '-r',
    '--robot',
    action="store_true",
    help='Creates a robot in the middle of the field.'
)

args = parser.parse_args()

def main():

    screen = Screen()
    screen.start_simulation()


    simul = Simulation(screen)

    field = SoccerField(screen)

    simul.field = field

    pygame.display.set_icon(field.robofei_logo_scaled)

    if args.robot:
        simul.robots.append(Robot(520, 370, 0, 100, np.random.randint(0, 256, 3)))
        simul.robots[0].imu_initial_value = 0
        simul.robots[0].ball = simul.ball
        simul.robots[0].vision.clk = 10

    #Main loop
    while True:

        #Process events
        simul.perform_events()

        #Update object positions checking for collisions
        simul.update_pos()


        #update soccer field
        field.draw_soccer_field()

        #Ball searching
        #simul.searching()

        #Draw robots, ball and update the current frame
        simul.display_update()

        #Pause for the next frame
        screen.clock.tick(20)


    #Close window and exit
    #pygame.quit()

#Call the main function, start up the simulation
if __name__ == "__main__":
    main()
