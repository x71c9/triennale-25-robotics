import time
import math

from dynamixel_controller import Dynamixel


U2D2_PORT="COM3"

ROBOT_A_MOTOR_IDS = [11,1]
ROBOT_A_MAX_CABLE_LENGTH_IN_M = 1.0

ROBOT_B_MOTOR_IDS = [10,3]
ROBOT_B_MAX_CABLE_LENGTH_IN_M = 1.0

ROBOT_C_MOTOR_IDS = [13,2]
ROBOT_C_MAX_CABLE_LENGTH_IN_M = 1.0

ROBOT_D_MOTOR_IDS = [12,0]
ROBOT_D_MAX_CABLE_LENGTH_IN_M = 1.0


MIN_CABLE_LENGTH_IN_M = 0.0

MOTOR_STEPS_PER_TURN = 4096
GEAR_RATIO = 234.0/30.0         # Big gear/small gear
DRUM_DIAMETER_M = 0.35          # Diameter with half cable wound up
CABLE_PER_DRUM_TURN = math.pi * DRUM_DIAMETER_M
CABLE_PER_MOTOR_TURN = CABLE_PER_DRUM_TURN / GEAR_RATIO



class Robot:
    def __init__(self, motor_ids, max_cable_length_in_m):
        self.motor_ids = motor_ids
        self.cable_length_in_m = None
        self.max_cable_length_in_m = max_cable_length_in_m

robot_a = Robot(ROBOT_A_MOTOR_IDS, ROBOT_A_MAX_CABLE_LENGTH_IN_M)
robot_b = Robot(ROBOT_B_MOTOR_IDS, ROBOT_B_MAX_CABLE_LENGTH_IN_M)
robot_c = Robot(ROBOT_C_MOTOR_IDS, ROBOT_C_MAX_CABLE_LENGTH_IN_M)
robot_d = Robot(ROBOT_D_MOTOR_IDS, ROBOT_D_MAX_CABLE_LENGTH_IN_M)



class TrienaleRobots:
    def __init__(self):
        self.robots = {"A": robot_a, "B": robot_b, "C": robot_c, "D": robot_d}

        motors_ids = ROBOT_A_MOTOR_IDS + ROBOT_B_MOTOR_IDS + ROBOT_C_MOTOR_IDS + ROBOT_D_MOTOR_IDS
        self.motors = Dynamixel(ID= motors_ids, descriptive_device_name="Trienale Robots", 
                            series_name=["xm", "xm"], baudrate=1000000, port_name=motors_ids)

        self.motors.begin_communication()
        self.motors.set_operating_mode("current-based position", ID = "all")

        for robot_id in self.robots.keys():
            self.set_position(robot_id, MIN_CABLE_LENGTH_IN_M)

    def clip(self, value, lower, upper):
        return lower if value < lower else upper if value > upper else value

    def meters_to_units(self, meters):
        units = int(round(meters / CABLE_PER_MOTOR_TURN))
        return units

    def set_position(self, robot_id: str, length_in_meters: float):
        length_in_meters = self.clip(length_in_meters, MIN_CABLE_LENGTH_IN_M, self.robots[robot_id].max_cable_length_in_m)

        if length_in_meters <= MIN_CABLE_LENGTH_IN_M:
            #TODO:
            # Do sth. when robot is supposed to go to minimum position
            print("Reseting robot.")
        else:
            length_in_units = self.meters_to_units(length_in_meters)
            
            motor_ids = self.robots[robot_id].motor_ids
            self.motors.write_position(motor_ids, length_in_units)

