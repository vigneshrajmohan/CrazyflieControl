from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
import cflib.crtp
crazyflie = Crazyflie()
crazyflie.connected.add_callback(connected)

def connect():
    crazyflie.open_link("radio://0/80/250K")


def disconnect():
    crazyflie.close_link()

def connected():
    # Code here
    crazyflie.param.set_value("motors.motorPowerM1","50000")
    crazyflie.param.set_value("motors.motorPowerM2","50000")
    crazyflie.param.set_value("motors.motorPowerM3","50000")
    crazyflie.param.set_value("motors.motorPowerM4","50000")
