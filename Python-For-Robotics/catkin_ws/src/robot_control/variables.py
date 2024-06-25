from robot_control_class import RobotControl
robotcontrol  = RobotControl()
laser1= robotcontrol .get_laser(360)
print ("The laser value received is: ", laser1)

laser2= robotcontrol .get_laser(180)
print ("The laser value received is: ", laser2)

all_laser = robotcontrol.get_laser_full()
print ("The laser value received is: ", all_laser)


