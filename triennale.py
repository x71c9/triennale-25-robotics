import time
import math
from dynamixel_controller import Dynamixel

U2D2_PORT="/dev/ttyUSB0"
BAUDRATE = 1000000

# COMMON SETTINGS
HOMING_SPEED = 50
REEL_OUT_A_BIT_SPEED = 20
MAX_VEL = 100
MIN_VEL = 20

MIN_CABLE_LENGTH_IN_M = 0.0

MOTOR_STEPS_PER_TURN = 4096
GEAR_RATIO = 234.0/30.0         # Big gear/small gear
DRUM_DIAMETER_M = 0.35          # Diameter with half cable wound up
CABLE_PER_DRUM_TURN = math.pi * DRUM_DIAMETER_M
CABLE_PER_MOTOR_TURN = CABLE_PER_DRUM_TURN / GEAR_RATIO



# INDIVIDUAL SETTINGS
ROBOT_A_MOTOR_IDS = [10,3]
ROBOT_A_MAX_CABLE_LENGTH_IN_M = 1.0
# ROBOT_A_CURRENT_LIMITS_IN_UNITS = [250, 250]
ROBOT_A_CURRENT_LIMITS_IN_UNITS = [25, 25]
ROBOT_A_FILE_PATH = "/home/max/robotA.txt"

ROBOT_B_MOTOR_IDS = [13,2]
ROBOT_B_MAX_CABLE_LENGTH_IN_M = 1.0
# ROBOT_B_CURRENT_LIMITS_IN_UNITS = [200, 200]
ROBOT_B_CURRENT_LIMITS_IN_UNITS = [120, 120]
ROBOT_B_FILE_PATH = "/home/max/robotB.txt"

ROBOT_C_MOTOR_IDS = [11,1]
ROBOT_C_MAX_CABLE_LENGTH_IN_M = 1.0
# ROBOT_C_CURRENT_LIMITS_IN_UNITS = [180, 180]
ROBOT_C_CURRENT_LIMITS_IN_UNITS = [18, 18]
ROBOT_C_FILE_PATH = "/home/max/robotC.txt"

ROBOT_D_MOTOR_IDS = [12,0]
ROBOT_D_MAX_CABLE_LENGTH_IN_M = 1.0
# ROBOT_D_CURRENT_LIMITS_IN_UNITS = [150, 150]
ROBOT_D_CURRENT_LIMITS_IN_UNITS = [15, 15]
ROBOT_D_FILE_PATH = "/home/max/robotD.txt"


ROBOT_IDS = { "A": ROBOT_A_MOTOR_IDS, 
            "B": ROBOT_B_MOTOR_IDS, 
            "C": ROBOT_C_MOTOR_IDS, 
            "D": ROBOT_D_MOTOR_IDS}

ROBOT_MAX_CABLE_LENGTHS_IN_M = {  "A": ROBOT_A_MAX_CABLE_LENGTH_IN_M, 
                                "B": ROBOT_B_MAX_CABLE_LENGTH_IN_M, 
                                "C": ROBOT_C_MAX_CABLE_LENGTH_IN_M, 
                                "D": ROBOT_D_MAX_CABLE_LENGTH_IN_M}

ROBOT_CURRENT_LIMITS = {  "A": ROBOT_A_CURRENT_LIMITS_IN_UNITS, 
                        "B": ROBOT_B_CURRENT_LIMITS_IN_UNITS, 
                        "C": ROBOT_C_CURRENT_LIMITS_IN_UNITS, 
                        "D": ROBOT_D_CURRENT_LIMITS_IN_UNITS}

ROBOT_FILE_PATHS = { "A": ROBOT_A_FILE_PATH, 
            "B": ROBOT_B_FILE_PATH, 
            "C": ROBOT_C_FILE_PATH, 
            "D": ROBOT_D_FILE_PATH}



class TriennaleRobot:
    def __init__(self, robot_id):
        self.dbg = True

        self.motor_ids = ROBOT_IDS[robot_id]
        self.max_cable_length_in_m = ROBOT_MAX_CABLE_LENGTHS_IN_M[robot_id]
        self.current_limits = ROBOT_CURRENT_LIMITS[robot_id]
        self.file_path = ROBOT_FILE_PATHS[robot_id]
        
        self.motors = Dynamixel(ID= self.motor_ids, descriptive_device_name="TriennaleRobot", 
                                series_name=["xm", "xm"], baudrate=BAUDRATE, port_name=U2D2_PORT)

        self.motors.begin_communication()
        self.motors.set_operating_mode("current-based position", ID = "all")
        
        self.apply_current_limit_settings()

        self.zero_position_in_units = self.read_zero_position_from_file()
        if self.dbg:
            print(f"Read zero position: {self.zero_position_in_units}")


    def apply_current_limit_settings(self):
        for lim, id in zip(self.current_limits, self.motor_ids):
            self.motors.write_current(lim, ID = id)



    def homeing(self):
        self.apply_homing_settings()
        self.move_motors_simple(reel_out=False)

        while 1:
            curr = []
            for id in self.motor_ids:
                current = self.motors.read_current(ID=id)
                curr.append(current)
                
                limits = self.current_limits
                if current > max(limits):
                    self.toggle_torque(torque_on=False)
                    time.sleep(0.5)
                    self.toggle_torque(torque_on=True)

                    self.stop_motors()
                    print("\n LIMIT reached")
                    self.reel_out_a_bit()
                    
                    self.write_zero_position_to_file()

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
            target_position = current_position - 1000
            self.motors.write_position(target_position, ID=id)

        time.sleep(2)
        self.apply_homing_settings()

    def apply_homing_settings(self):
        self.motors.write_profile_velocity(HOMING_SPEED, ID=self.motor_ids)

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


    def read_average_motor_position_in_units(self):
        positions_in_units = []
        for id in self.motor_ids:
            positions_in_units.append(self.motors.read_position(id))
        return int(sum(positions_in_units)/len(positions_in_units))




    def write_zero_position_to_file(self):
        motor_positions = self.read_average_motor_position_in_units()
        try:
            with open(self.file_path, 'w') as f:
                f.write(str(motor_positions))
        except IOError as e:
            raise RuntimeError(f"Failed to write zero position to {self.file_path}: {e}")
        return motor_positions


    def read_zero_position_from_file(self):
        try:
            with open(self.file_path, 'r') as f:
                content = f.read().strip()
        except FileNotFoundError:
            print(f"Zero position file not found: {self.file_path}")
            zero_position_in_units = self.write_zero_position_to_file()
            return zero_position_in_units
        except IOError as e:
            raise RuntimeError(f"Failed to read zero position from {self.file_path}: {e}")

        try:
            zero_position_in_units = int(content)
        except ValueError:
            raise ValueError(f"Invalid zero position value in {self.file_path}: '{content}'")
        return zero_position_in_units



    def set_position(self, length_in_meters: float, velocity:float):
        length_in_meters = self.clip(length_in_meters, MIN_CABLE_LENGTH_IN_M, self.max_cable_length_in_m)
        length_in_units = self.meters_to_units(length_in_meters)

        target_length_in_units = length_in_units + self.zero_position_in_units
        
        self.write_velocity(velocity)
        self.motors.write_position(self.motor_ids, target_length_in_units)

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
        average_motor_positions = self.read_average_motor_position_in_units()
        if self.dbg:
            print(f"Average Motor position in units: {average_motor_positions}")     
        
        motor_position_in_units = average_motor_positions - self.zero_position_in_units
        if self.dbg:
            print(f"Motor position in units: {motor_position_in_units}")
            
        return self.units_to_meters(motor_position_in_units)

    def units_to_meters(self, units):
        return units / MOTOR_STEPS_PER_TURN * CABLE_PER_MOTOR_TURN
