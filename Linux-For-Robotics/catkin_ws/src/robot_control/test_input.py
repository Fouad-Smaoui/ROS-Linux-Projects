from robot_control_class import RobotControl

data_user = int(input("Enter a number between 0 and 719 "))

robotcontrol  = RobotControl()

laser_data_user = robotcontrol .get_laser(data_user)
print (laser_data_user)

