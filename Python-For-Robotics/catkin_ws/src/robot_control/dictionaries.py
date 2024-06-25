from robot_control_class import RobotControl
robotcontrol  = RobotControl()
laser_data = robotcontrol .get_laser_full()

laser_data_set = {"P0": laser_data[0], "P100": laser_data[100], "P200": laser_data[200], "P300": laser_data[300], "P400": laser_data[400], "P500": laser_data[500], "P600": laser_data[600], "P719": laser_data[719]}

print (laser_data_set)