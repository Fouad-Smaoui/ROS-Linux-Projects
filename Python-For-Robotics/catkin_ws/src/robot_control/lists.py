from robot_control_class import RobotControl
robotcontrol  = RobotControl()
laser_data = robotcontrol .get_laser_full()

print ("Position 0: ", laser_data[0])
print ("Position 360: ", laser_data[360])
print ("Position 719: ", laser_data[719])