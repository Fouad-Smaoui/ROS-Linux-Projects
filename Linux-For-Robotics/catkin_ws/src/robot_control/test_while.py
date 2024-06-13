from robot_control_class import RobotControl

robotcontrol  = RobotControl()
distance_from_wall = robotcontrol.get_laser(360)
while distance_from_wall > 0.5:
 robotcontrol.move_straight()
 distance_from_wall = robotcontrol.get_laser(360)

 print ("The laser value received was: ", distance_from_wall)

robotcontrol.stop_robot()   
print ("The laser value received was: ", distance_from_wall)
