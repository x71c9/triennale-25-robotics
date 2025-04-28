from utility import *
from dynamixel_controller import Dynamixel

# motors_ids = [10,3]
# motors_ids = [13,2]
# motors_ids = [12,0]
motors_ids = [11,1]
servo = Dynamixel(ID= motors_ids, descriptive_device_name="XM430 test motor", 
                    series_name=["xm", "xm"], baudrate=1000000, port_name="COM3")

key = Key()
servo.begin_communication()
servo.set_operating_mode("velocity", ID = "all")

def write_speed(speed):
    for id in motors_ids:
        servo.write_velocity(speed, ID = id)

spd = 100
while 1:
    time.sleep(0.01)
    if key.keyPress == "1":
        write_speed(spd)
    if key.keyPress == "2":
        write_speed(-spd)
    if key.keyPress == "0":
        write_speed(0)

    curr = []
    for id in motors_ids:
        curr.append(servo.read_current(ID=id))

    print(curr)
servo.end_communication()
