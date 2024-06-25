from robot_control_class import RobotControl

robotcontrol  = RobotControl()
dataset = robotcontrol.get_laser_full()
max=0
for data in dataset:
 if data > max:
    max=data
    
print ("The higher value in the list is: ", max)
