from robot_control_class import RobotControl

robotcontrol  = RobotControl()
distance_from_wall = robotcontrol.get_laser(360)
if distance_from_wall <5:
 robotcontrol.stop_robot()
else : 
    robotcontrol.move_straight()
    
print ("The laser value received was: ", distance_from_wall)
