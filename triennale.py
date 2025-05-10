import os
import time
import math
from dynamixel_controller import Dynamixel

U2D2_PORT="/dev/ttyUSB0"
BAUDRATE = 1000000

HOME_DIR = os.environ["HOME"]
ROBOT_FILES_DIRECTORY = os.path.join(HOME_DIR, "triennale-robots")
if not os.path.exists(ROBOT_FILES_DIRECTORY):
    os.makedirs(ROBOT_FILES_DIRECTORY)
    print(f"Created directory: {ROBOT_FILES_DIRECTORY}")

# COMMON SETTINGS
HOMING_SPEED = 50
REEL_OUT_A_BIT_SPEED = 30
MAX_VEL = 100
MIN_VEL = 20

MIN_CABLE_LENGTH_IN_M = 0.0

MOTOR_STEPS_PER_TURN = 4096
GEAR_RATIO = 234.0/30.0         # Big gear/small gear
DRUM_DIAMETER_M = 0.35          # Diameter with half cable wound up
CABLE_PER_DRUM_TURN = math.pi * DRUM_DIAMETER_M
CABLE_PER_MOTOR_TURN = CABLE_PER_DRUM_TURN / GEAR_RATIO
REVERSE_LENGTH = -1.0

# INDIVIDUAL SETTINGS
ROBOT_A_MOTOR_IDS = [10,3] # <-- DON'T TOUCH THIS
ROBOT_A_MAX_CABLE_LENGTH_IN_M = 5.4 # <-- DON'T TOUCH THIS
ROBOT_A_HOMING_CURRENT_LIMITS_IN_UNITS = [230, 230] # <-- EDIT THIS
ROBOT_A_CURRENT_LIMITS_IN_UNITS = [220, 200] # <-- EDIT THIS
ROBOT_A_FILE_PATH = os.path.join(ROBOT_FILES_DIRECTORY, "robotA.txt")

ROBOT_B_MOTOR_IDS = [13,2] # <-- DON'T TOUCH THIS
ROBOT_B_MAX_CABLE_LENGTH_IN_M = 4.5 # <-- DON'T TOUCH THIS
ROBOT_B_HOMING_CURRENT_LIMITS_IN_UNITS = [270, 270] # <-- EDIT THIS
ROBOT_B_CURRENT_LIMITS_IN_UNITS = [180, 160] # <-- EDIT THIS
ROBOT_B_FILE_PATH = os.path.join(ROBOT_FILES_DIRECTORY, "robotB.txt")

ROBOT_C_MOTOR_IDS = [11,1] # <-- DON'T TOUCH THIS
ROBOT_C_MAX_CABLE_LENGTH_IN_M = 4.8
ROBOT_C_HOMING_CURRENT_LIMITS_IN_UNITS = [240, 240] # <-- EDIT THIS
ROBOT_C_CURRENT_LIMITS_IN_UNITS = [180, 160] # <-- EDIT THIS
ROBOT_C_FILE_PATH = os.path.join(ROBOT_FILES_DIRECTORY, "robotC.txt")

ROBOT_D_MOTOR_IDS = [12,0] # <-- DON'T TOUCH THIS
ROBOT_D_MAX_CABLE_LENGTH_IN_M = 5.4 # <-- DON'T TOUCH THIS
ROBOT_D_CURRENT_LIMITS_IN_UNITS = [240, 240] # <-- EDIT THIS
ROBOT_D_HOMING_CURRENT_LIMITS_IN_UNITS = [180, 160] # <-- EDIT THIS
ROBOT_D_FILE_PATH = os.path.join(ROBOT_FILES_DIRECTORY, "robotD.txt")


ROBOT_IDS = {   "A": ROBOT_A_MOTOR_IDS, 
                "B": ROBOT_B_MOTOR_IDS, 
                "C": ROBOT_C_MOTOR_IDS, 
                "D": ROBOT_D_MOTOR_IDS}

ROBOT_MAX_CABLE_LENGTHS_IN_M = {    "A": ROBOT_A_MAX_CABLE_LENGTH_IN_M, 
                                    "B": ROBOT_B_MAX_CABLE_LENGTH_IN_M, 
                                    "C": ROBOT_C_MAX_CABLE_LENGTH_IN_M, 
                                    "D": ROBOT_D_MAX_CABLE_LENGTH_IN_M}

ROBOT_HOMING_CURRENT_LIMITS = {     "A": ROBOT_A_HOMING_CURRENT_LIMITS_IN_UNITS, 
                                    "B": ROBOT_B_HOMING_CURRENT_LIMITS_IN_UNITS, 
                                    "C": ROBOT_C_HOMING_CURRENT_LIMITS_IN_UNITS, 
                                    "D": ROBOT_D_HOMING_CURRENT_LIMITS_IN_UNITS}

ROBOT_CURRENT_LIMITS = {    "A": ROBOT_A_CURRENT_LIMITS_IN_UNITS, 
                            "B": ROBOT_B_CURRENT_LIMITS_IN_UNITS, 
                            "C": ROBOT_C_CURRENT_LIMITS_IN_UNITS, 
                            "D": ROBOT_D_CURRENT_LIMITS_IN_UNITS}

ROBOT_FILE_PATHS = {    "A": ROBOT_A_FILE_PATH, 
                        "B": ROBOT_B_FILE_PATH, 
                        "C": ROBOT_C_FILE_PATH, 
                        "D": ROBOT_D_FILE_PATH}


class TriennaleRobot:
    def __init__(self, robot_id, mode="read_write"):
        self.dbg = True

        self.motor_ids = ROBOT_IDS[robot_id]
        self.max_cable_length_in_m = ROBOT_MAX_CABLE_LENGTHS_IN_M[robot_id]
        self.current_limits = ROBOT_CURRENT_LIMITS[robot_id]
        self.homing_current_limits = ROBOT_HOMING_CURRENT_LIMITS[robot_id]
        self.file_path = ROBOT_FILE_PATHS[robot_id]
        
        self.motors = Dynamixel(ID= self.motor_ids, descriptive_device_name="TriennaleRobot", 
                                series_name=["xm", "xm"], baudrate=BAUDRATE, port_name=U2D2_PORT)

        self.motors.begin_communication()

        if mode is not "read_only":
            self.motors.set_operating_mode("current-based position", ID = "all")
            self.apply_current_limit_settings()

        self.zero_positions_in_units = self.read_zero_positions_from_file()
        if self.dbg:
            print(f"Read zero positions: {self.zero_positions_in_units}")


    def apply_current_limit_settings(self):
        for lim, id in zip(self.current_limits, self.motor_ids):
            self.motors.write_current(lim, ID = id)



    def homing(self):
        self.apply_homing_settings()
        time.sleep(0.5)
        self.move_motors_simple(reel_out=False)

        while 1:
            curr = []
            for id in self.motor_ids:
                current = self.motors.read_current(ID=id)
                curr.append(current)
                
                limits = self.homing_current_limits
                if current > max(limits):
                    self.toggle_torque(torque_on=False)
                    time.sleep(0.5)
                    self.toggle_torque(torque_on=True)

                    self.stop_motors()
                    print("\n LIMIT reached")
                    self.reel_out_a_bit()
                    
                    self.write_zero_positions_to_file()

                    return
                    
            print(curr)
    
    def toggle_torque(self, torque_on):
        if torque_on:
            self.motors.enable_torque(ID=self.motor_ids)
        else:
            self.motors.disable_torque(ID=self.motor_ids)

    def reel_out_a_bit(self):
        self.motors.write_profile_velocity(REEL_OUT_A_BIT_SPEED, ID=self.motor_ids)

        for id in self.motor_ids:
            current_position = self.motors.read_position(ID = id)
            target_position = current_position - 3000
            self.motors.write_position(target_position, ID=id)

        time.sleep(10)
        self.apply_homing_settings()
        self.apply_current_limit_settings()

    def apply_homing_settings(self):
        self.motors.write_profile_velocity(HOMING_SPEED, ID=self.motor_ids)

        for lim, id in zip(self.homing_current_limits, self.motor_ids):
            print("Setting limits to", lim)
            self.motors.write_current(lim, ID = id)


    def move_motors_simple(self, reel_out):
        if reel_out:
            demand = -900000
        else:
            demand = 900000

        self.motors.write_position(demand, ID = self.motor_ids)

    
    def stop_motors(self):
        for id in self.motor_ids:
            curr_pos = self.motors.read_position(ID = id)
            self.motors.write_position(curr_pos, ID = id)


    def read_motor_positions_in_units(self):
        motor_positions = []
        for id in self.motor_ids:
            motor_positions.append(self.motors.read_position(id))
        return motor_positions




    def write_zero_positions_to_file(self):
        motor_positions = [self.motors.read_position(ID=id) for id in self.motor_ids]
        try:
            with open(self.file_path, 'w') as f:
                f.write(','.join(str(p) for p in motor_positions))
        except IOError as e:
            raise RuntimeError(f"Failed writing zero positions to {self.file_path}: {e}")
        return motor_positions

    def read_zero_positions_from_file(self):
        try:
            with open(self.file_path, 'r') as f:
                data = f.read().strip()
        except FileNotFoundError:
            return self.write_zero_positions_to_file()
        except IOError as e:
            raise RuntimeError(f"Error reading zero positions from {self.file_path}: {e}")

        parts = data.split(',')
        if len(parts) != len(self.motor_ids):
            return self.write_zero_positions_to_file()
        try:
            return [int(p) for p in parts]
        except ValueError:
            return self.write_zero_positions_to_file()
        
        



    def set_position(self, length_in_meters: float, velocity:float):
        self.write_velocity(velocity)

        length_in_meters = self.clip(length_in_meters, MIN_CABLE_LENGTH_IN_M, self.max_cable_length_in_m)
        length_in_units = REVERSE_LENGTH * self.meters_to_units(length_in_meters)

        if self.dbg:
            print(f"Additional length in units: {length_in_units}")

        target_positions = [zp + length_in_units for zp in self.zero_positions_in_units]

        for id, position in zip(self.motor_ids, target_positions):

            self.motors.write_position(position, ID=id)
            if self.dbg:
                print(f"Setting motor {id} to length: {position}")

    def write_velocity(self, velocity:float):
        velocity_zero_to_one = self.clip(velocity, 0.0, 1.0)
        velocity_in_units = self.denormalize(velocity_zero_to_one, MIN_VEL, MAX_VEL)
        self.motors.write_profile_velocity(velocity_in_units, self.motor_ids)

    def denormalize(self, zero_to_one, lower, upper):
        return lower + zero_to_one * (upper - lower)

    def clip(self, value, lower, upper):
        return lower if value < lower else upper if value > upper else value

    def meters_to_units(self, meters):
        return int(round(meters / CABLE_PER_MOTOR_TURN * MOTOR_STEPS_PER_TURN))




    def get_position(self):
        motor_positions = [self.motors.read_position(ID=id) for id in self.motor_ids]                
        deltas = [mp - zp for mp, zp in zip(motor_positions, self.zero_positions_in_units)]
        if self.dbg:
            print(f"Motor deltas in units: {deltas}")
        meters = [self.units_to_meters(delta) for delta in deltas]

        if self.dbg:
            print(f"Meters: {meters}")
        
        return sum(meters) / len(meters)


    def units_to_meters(self, units):
        return units / MOTOR_STEPS_PER_TURN * CABLE_PER_MOTOR_TURN
