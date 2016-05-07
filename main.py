from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
import cflib.crtp


def connect():
    available = cflib.crtp.scan_interfaces()
    for i in available:
        print("Interface with URI {0} found and name/comment {1}".format(i[0], i[1]))
    address = input("Please enter the connection string:  ")
    crazyflie.open_link("radio://0/80/250K")


def disconnect():
    crazyflie.close_link()

def connected(*args):
    # Code here
    crazyflie.param.set_value("motors.motorPowerM1","50000")
    crazyflie.param.set_value("motors.motorPowerM2","50000")
    crazyflie.param.set_value("motors.motorPowerM3","50000")
    crazyflie.param.set_value("motors.motorPowerM4","50000")

try:
    cflib.crtp.init_drivers()
    crazyflie = Crazyflie()
    crazyflie.connected.add_callback(connected)
    connect()
except:
    disconnect()
