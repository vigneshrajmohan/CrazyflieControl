from cflib.crazyflie import Crazyflie
from cflib.utils import callbacks
import cflib.crtp
import time
import logging
logging.basicConfig(level=logging.ERROR)
#from time import sleep


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
    #crazyflie.param.set_value("motors.motorPowerM1","5000")
    #crazyflie.param.set_value("motors.motorPowerM2","5000")
    #crazyflie.param.set_value("motors.motorPowerM3","5000")
    #crazyflie.param.set_value("motors.motorPowerM4","5000")
    for i in range(10000, 35001, 50):
        crazyflie.param.set_value("motors.motorPowerM1", str(i))
        crazyflie.param.set_value("motors.motorPowerM2", str(i))
        crazyflie.param.set_value("motors.motorPowerM3", str(i))
        crazyflie.param.set_value("motors.motorPowerM4", str(i))

        print(i)

    #time.sleep(3)

    for j in range(35000, 0, -50):
        crazyflie.param.set_value("motors.motorPowerM1", str(j))
        crazyflie.param.set_value("motors.motorPowerM2", str(j))
        crazyflie.param.set_value("motors.motorPowerM3", str(j))
        crazyflie.param.set_value("motors.motorPowerM4", str(j))

        print(j)

    cutthewings()
    # print("This is the end")

def fly():
#function
    crazyflie.param.set_value("motors.motorPowerM1","5000")
    crazyflie.param.set_value("motors.motorPowerM2","5000")
    crazyflie.param.set_value("motors.motorPowerM3","5000")
    crazyflie.param.set_value("motors.motorPowerM4","5000")

def cutthewings():
#function
    crazyflie.param.set_value("motors.motorPowerM1","0")
    crazyflie.param.set_value("motors.motorPowerM2","0")
    crazyflie.param.set_value("motors.motorPowerM3","0")
    crazyflie.param.set_value("motors.motorPowerM4","0")



cflib.crtp.init_drivers()
crazyflie = Crazyflie()
crazyflie.connected.add_callback(connected)
connect()
