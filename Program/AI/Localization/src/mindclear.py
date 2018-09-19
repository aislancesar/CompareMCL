# coding: utf-8

__author__ = "RoboFEI-HT"
__authors__ = "Aislan C. Almeida"
__license__ = "GNU General Public License v3.0"

import sys
sys.path.append('../../Blackboard/src/')
from SharedMemory import SharedMemory # noqa E402

try:
    from configparser import ConfigParser
except ImportError:
    from ConfigParser import ConfigParser

bkb = SharedMemory()  # Instance of a blackboard
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
Mem = bkb.shd_constructor(mem_key)

bkb.write_int(Mem, 'LOCALIZATION_WORKING', 0)
