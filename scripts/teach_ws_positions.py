#!/usr/bin/env python
"""this script reads the current AMCL-position and converts it to the format used in our WS_positions.csv (as of 14.09.2022 this is the name).
a name has to be entered for the output, after that it can be pasted into the .csv
1) subscribe to the rostopic that contains the current AMCL position
2) Extract the current position
3) Convert to .csv-format and print in console, print to .csv or store in intermediate file
"""

__author__ = "M.P.Heimbach"
__copyright__ = ""
__license__ = ""
__version__ = "1.0.1"
__maintainer__ = "Julian Mueller"
__email__ = "julian.mueller.1@thws.de"
__status__ = "Development"


import os
import signal
import sys
import argparse
import csv
import rospy
import tf2_ros

DEFAULT_CSV_FILE_NAME = "workstations.csv"


def add_ws_to_csv(csv_file_name, workstation, tf_data):
    """ Generate the csv line
        data is of type PoseWithCovarianceStamped
        ,X,Y,Z,X,Y,Z,W
        START,-2.862,1.515,0,0,0,1,0.019
        END,-2.862,1.515,0,0,0,1,0.019
        WS01,-3.414,0.636,0,0,0,1,0.028
        WS01_controller,-3.414,0.636,0,0,0,1,0.028
        Turntable = TT, Shelf = SH

    :param csv_file_name: csv file name
    :param workstation: current workstation 
    :param tf_data: current position estimate read from coordinate transform buffer
    """
    csv_line = {
        'WS': workstation,
        't_X': tf_data.transform.translation.x,
        't_Y': tf_data.transform.translation.y,
        't_Z': tf_data.transform.translation.z,
        'r_X': tf_data.transform.rotation.x,
        'r_Y': tf_data.transform.rotation.y,
        'r_Z': tf_data.transform.rotation.z,
        'r_W': tf_data.transform.rotation.w,
    }
    print(f"    {csv_line} \n")
    with open(csv_file_name, 'a', newline='') as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=csv_line.keys())
        if csv_file.tell() == 0:  # check if this is the first line
            writer.writeheader()  # write header if true
        writer.writerow(csv_line)


def delete_csv(file):
    if os.path.exists(file):
        os.remove(file)
        print(f"file: {file}  was deleted")
    else:
        print(f"The file: {file} does not exist")


def parse_console_input():
    input_var = input("Enter workstation name:\n"
                      "  Naming convention:\n"
                      "  WS01 = Workstation 1, TT01 = Turntable 1,\n"
                      "  SH01 = Shelf 1, PP01 = Precision Placement\n"
                      "  Write 'exit' to stop the program\n"
                      "Workstation:   "
                      )
    print("\n")
    # TODO: Do input validation here
    if (input_var == "exit"):
        exit()
    return input_var


def signal_handler(sig, frame):
    print('Ctrl+C')
    sys.exit(0)


if __name__ == '__main__':
    """ Read current position estimates of the robot and 
        generate the csv with syntax expected by the swot_robocup_navigation.cpp
    """
    signal.signal(signal.SIGINT, signal_handler)
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for the node so that multiple listeners can
    # run simultaneously.
    print("Init ROS node")
    rospy.init_node('ws_position_teacher', anonymous=True)

    # create tf buffer listener as source of current position estimate for the robot
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # create console input parser with some basic parameters
    parser = argparse.ArgumentParser(
        description='Script to automate the process of creating a new CSV file for workstation positions')
    parser.add_argument("file", type=str, nargs='?',
                        default=DEFAULT_CSV_FILE_NAME, help="Desired CSV file location")
    parser.add_argument("-c", "--cleanup", action="store_true",
                        help="use -c to delete default workstations csv")
    args = parser.parse_args()
    if args.cleanup:
        delete_csv(DEFAULT_CSV_FILE_NAME)
        exit()
    csv_file_name = args.file

    while (not rospy.is_shutdown()):
        workstation = parse_console_input()
        try:
            transform = tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print("ros transform failed")
            exit()
        add_ws_to_csv(csv_file_name, workstation, transform)

    print('ros shutdown')
