import time
from utility import Key

from triennale import TriennaleRobots

R_ID = "A"

key = Key()
tr = TriennaleRobots(robot_ids=[R_ID])

input()
tr.homeing(robot_id=R_ID)

while 1:
    if key.keyPress == "1":
        print("Reel out")
        tr.move_motors_simple(robot_id = R_ID, reel_out=True)
    if key.keyPress == "2":
        print("Reel in")
        tr.move_motors_simple(robot_id = R_ID, reel_out=False)
    if key.keyPress == "0":
        print("STOP")
        tr.stop_motors(robot_id= R_ID)

    # motor_ids = tr.robots[R_ID].motor_ids
    # curr = []
    # for id in motor_ids:
    #     current = tr.motors.read_current(ID=id)
    #     curr.append(current)
        
    #     limits = tr.robots[R_ID].current_limits_in_units
    #     if current > max(limits):
    #         tr.stop_motors(robot_id=R_ID)
    #         print("\n LIMIT reached on ID= ", id)
    #         tr.reel_out_a_bit(robot_id=R_ID)
    
    # print(curr)


#     print(curr)
# servo.end_communication()
