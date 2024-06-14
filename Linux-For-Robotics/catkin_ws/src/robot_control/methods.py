from robot_control_class import RobotControl
import time
robotcontrol  = RobotControl(robot_name="summit")
'''
def move_straight_with_time(secs):
    robotcontrol.move_straight()
    time.sleep(secs)
    robotcontrol.stop_robot()

move_straight_with_time(5)

def laser_reading(a,b,c):
    r1 = robotcontrol.get_laser_summit(a)
    r2 = robotcontrol.get_laser_summit(b)
    r3 = robotcontrol.get_laser_summit(c)
    return [r1, r2, r3]

laser_data = laser_reading(0, 500, 1000)

print ("Reading 1: ", laser_data[0])
print ("Reading 2: ", laser_data[1])
print ("Reading 3: ", laser_data[2])
'''
robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 6)
robotcontrol.turn("counter-clockwise", 0.3, 4)
robotcontrol.move_straight_time("forward", 0.3, 1.5)
robotcontrol.turn("counter-clockwise", 0.3, 7)
robotcontrol.move_straight_time("forward", 0.3, 7)