#!/usr/bin/env python
"""this script reads the known navigation postions from a .csv and selects random postions as next goal.
0) ros setup
1) parse csv
2) select "random" entry
3) rosservice call
"""

__version__ = "1.0"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mueller.1@thws.de"

import signal
import sys
import argparse
import csv
import random
import rospy
from swot_msgs.srv import SwotNavigation

DEFAULT_CSV_FILE_NAME = "workstations.csv"

def signal_handler(sig, frame):
  print('Ctrl+C')
  sys.exit(0)

def parse_csv(csv_file_name):
  """ csv line
  WS,X,Y,Z,X,Y,Z,W
  START,-2.862,1.515,0,0,0,1,0.019
  END,-2.862,1.515,0,0,0,1,0.019
  WS01,-3.414,0.636,0,0,0,1,0.028
  WS01_controller,-3.414,0.636,0,0,0,1,0.028
  Turntable = TT, Shelf = SH
  """
  goals = []
  with open(csv_file_name, newline='') as csv_file:
    workstations = csv.DictReader(csv_file, delimiter=',')
    for ws in workstations:
      if('controller' not in ws['WS'] and 'FINISH' not in ws['WS'] and 'RECOVER' not in ws['WS']):
        goals.append(ws['WS'])
  return goals
  

def select_random_goal(goals):
   return random.choice(goals)

if __name__ == '__main__':
    """ Read possible positions from .csv 
    """
    signal.signal(signal.SIGINT, signal_handler)
    print("Init ROS node")
    rospy.init_node('ws_position_teacher', anonymous=True)

    #service call
    navigation_service = rospy.ServiceProxy('swot_next_destination', SwotNavigation)

    ## create console input parser with some basic parameters
    parser = argparse.ArgumentParser(
        description='nav test')
    parser.add_argument("file", type=str, nargs='?',
                        default=DEFAULT_CSV_FILE_NAME, help="CSV file with location data")
    args = parser.parse_args()
    csv_file_name = args.file
    goals = parse_csv(csv_file_name)

    while (not rospy.is_shutdown()):
      try:
        next_goal = select_random_goal(goals)
        print(f"Next Goal: {next_goal} \n") 
        # Sends the goal to the service and blocks.
        result = navigation_service(next_goal)
        # Prints out the result of executing the action
        print(f"Service Result: {result} \n") 

      except (Exception):
        print("ros action failed")
        exit()

    print('ros shutdown')
