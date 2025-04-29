import time
import math
from dynamixel_controller import Dynamixel

U2D2_PORT="COM3"
BAUDRATE = 1000000

# COMMON SETTINGS
HOMING_SPEED = 50
REEL_OUT_A_BIT_SPEED = 20


# INDIVIDUAL SETTINGS
ROBOT_A_MOTOR_IDS = [10,3]
ROBOT_A_MAX_CABLE_LENGTH_IN_M = 1.0
ROBOT_A_CURRENT_LIMITS_IN_UNITS = [250, 250]

ROBOT_B_MOTOR_IDS = [13,2]
ROBOT_B_MAX_CABLE_LENGTH_IN_M = 1.0
ROBOT_B_CURRENT_LIMITS_IN_UNITS = [200, 200]

ROBOT_C_MOTOR_IDS = [11,1]
ROBOT_C_MAX_CABLE_LENGTH_IN_M = 1.0
ROBOT_C_CURRENT_LIMITS_IN_UNITS = [180, 180]

ROBOT_D_MOTOR_IDS = [12,0]
ROBOT_D_MAX_CABLE_LENGTH_IN_M = 1.0
ROBOT_D_CURRENT_LIMITS_IN_UNITS = [150, 150]


MIN_CABLE_LENGTH_IN_M = 0.0

MOTOR_STEPS_PER_TURN = 4096
GEAR_RATIO = 234.0/30.0         # Big gear/small gear
DRUM_DIAMETER_M = 0.35          # Diameter with half cable wound up
CABLE_PER_DRUM_TURN = math.pi * DRUM_DIAMETER_M
CABLE_PER_MOTOR_TURN = CABLE_PER_DRUM_TURN / GEAR_RATIO

SMALL_CURRENT_THRESHOLD_IN_UNITS = 5

class Robot:
    def __init__(self, motor_ids, max_cable_length_in_m, current_limits_in_units):
        self.motor_ids = motor_ids
        self.cable_length_in_m = None
        self.max_cable_length_in_m = max_cable_length_in_m
        self.current_limits_in_units = current_limits_in_units

robot_a = Robot(ROBOT_A_MOTOR_IDS, ROBOT_A_MAX_CABLE_LENGTH_IN_M, ROBOT_A_CURRENT_LIMITS_IN_UNITS)
robot_b = Robot(ROBOT_B_MOTOR_IDS, ROBOT_B_MAX_CABLE_LENGTH_IN_M, ROBOT_B_CURRENT_LIMITS_IN_UNITS)
robot_c = Robot(ROBOT_C_MOTOR_IDS, ROBOT_C_MAX_CABLE_LENGTH_IN_M, ROBOT_C_CURRENT_LIMITS_IN_UNITS)
robot_d = Robot(ROBOT_D_MOTOR_IDS, ROBOT_D_MAX_CABLE_LENGTH_IN_M, ROBOT_D_CURRENT_LIMITS_IN_UNITS)

class TrienaleRobots:
    def __init__(self, robot_ids):
        self.robots = {"A": robot_a, "B": robot_b, "C": robot_c, "D": robot_d}
    
        motors_ids = []
        series_name = []
        for robot_id in robot_ids:
            motors_ids += self.robots[robot_id].motor_ids
            series_name += ["xm", "xm"]

        self.motors = Dynamixel(ID= motors_ids, descriptive_device_name="Trienale Robots", 
                                series_name=series_name, baudrate=BAUDRATE, port_name=U2D2_PORT)

        self.motors.begin_communication()
        self.motors.set_operating_mode("current-based position", ID = "all")
        
        for robot_id in robot_ids:
            self.apply_current_limit_settings(robot_id=robot_id)
            self.apply_homing_settings(robot_id=robot_id)

    def homeing(self, robot_id):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids

        self.apply_homing_settings(robot_id=robot_id)
        self.move_motors_simple(robot_id = robot_id, reel_out=False)

        while 1:
            curr = []
            for id in motor_ids:
                current = self.motors.read_current(ID=id)
                curr.append(current)
                
                limits = self.robots[robot_id].current_limits_in_units
                if current > max(limits):
                    self.toggle_torque(torque_on=False, robot_id=robot_id)
                    time.sleep(0.5)
                    self.toggle_torque(torque_on=True, robot_id=robot_id)

                    self.stop_motors(robot_id=robot_id)
                    print("\n LIMIT reached on ID= ", id)
                    self.reel_out_a_bit(robot_id=robot_id)
                    return
                    
            print(curr)

    def toggle_torque(self, torque_on, robot_id):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids

        if torque_on:
            self.motors.enable_torque(ID=motor_ids)
        else:
            self.motors.disable_torque(ID=motor_ids)

    def reel_out_a_bit(self, robot_id):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids
        self.motors.write_profile_velocity(REEL_OUT_A_BIT_SPEED, ID=motor_ids)

        for id in motor_ids:
            current_position = self.motors.read_position(ID = id)
            target_position = current_position - 1000
            self.motors.write_position(target_position, ID=id)

        time.sleep(2)
        self.apply_homing_settings(robot_id=robot_id)


    def apply_current_limit_settings(self, robot_id):
        robot = self.robots[robot_id]
        for lim, id in zip(robot.current_limits_in_units, robot.motor_ids):
            self.motors.write_current(lim, ID = id)

    def apply_homing_settings(self, robot_id):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids
        self.motors.write_profile_velocity(HOMING_SPEED, ID=robot.motor_ids)

    
    def move_motors_simple(self, robot_id, reel_out):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids

        if reel_out:
            demand = -900000
        else:
            demand = 900000

        self.motors.write_position(demand, ID = motor_ids)
    

    def stop_motors(self, robot_id):
        robot = self.robots[robot_id]
        motor_ids = robot.motor_ids

        for id in motor_ids:
            curr_pos = self.motors.read_position(ID = id)
            self.motors.write_position(curr_pos, ID = id)


    # def clip(self, value, lower, upper):
    #     return lower if value < lower else upper if value > upper else value

    # def meters_to_units(self, meters):
    #     return int(round(meters / CABLE_PER_MOTOR_TURN))

    # def units_to_meters(self, units):
    #     return units * CABLE_PER_MOTOR_TURN

    # def home_robot(self, robot_id):
    #     robot = self.robots[robot_id]
    #     motor_ids = robot.motor_ids
    #     current_limits_in_units = robot.current_limits_in_units
        
    #     self.motors.write_position(motor_ids, -sys.maxsize)

    #     currents_in_amp = self.motors.read_current(motor_ids)
    #     while (currents_in_amp[0] + SMALL_CURRENT_THRESHOLD_IN_UNITS < current_limits_in_units[0] or 
    #            currents_in_amp[1] + SMALL_CURRENT_THRESHOLD_IN_UNITS < current_limits_in_units[1]):
    #         time.sleep(1) 
    #         currents_in_amp = self.motors.read_current(motor_ids)

    # def set_position(self, robot_id: str, length_in_meters: float):
    #     length_in_meters = self.clip(length_in_meters, MIN_CABLE_LENGTH_IN_M, self.robots[robot_id].max_cable_length_in_m)
    #     length_in_units = self.meters_to_units(length_in_meters)
    #     motor_ids = self.robots[robot_id].motor_ids
    #     self.motors.write_position(motor_ids, length_in_units)

    # def get_position(self, robot_id: str):
        # motor_ids = self.robots[robot_id].motor_ids        
        # positions_in_units = self.motors.read_position(motor_ids)
        # average_motor_positions = sum(positions_in_units)/len(positions_in_units)
        # return self.units_to_meters(average_motor_positions)