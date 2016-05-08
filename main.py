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
    #
    # roll    = 0.0
    # pitch   = 0.0
    # yawrate = 0
    # thrust  = 10000
    #
    # send_setpoint(roll, pitch, yaw, thrust)
    # """
        # Send a new control set-point for roll/pitch/yaw/thust to the copter
        #
        # The arguments roll/pitch/yaw/trust is the new set-points that should
        # be sent to the copter
        # """

    #crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust)

    for i in range(10000, 50001, 1000):
        crazyflie.param.set_value("motors.motorPowerM1", str(i))
        crazyflie.param.set_value("motors.motorPowerM2", str(i))
        crazyflie.param.set_value("motors.motorPowerM3", str(i))
        crazyflie.param.set_value("motors.motorPowerM4", str(i))
        # fly(i)
        print(i)

    #time.sleep(3)

    for j in range(50000, 0, -1000):
        crazyflie.param.set_value("motors.motorPowerM1", str(j))
        crazyflie.param.set_value("motors.motorPowerM2", str(j))
        crazyflie.param.set_value("motors.motorPowerM3", str(j))
        crazyflie.param.set_value("motors.motorPowerM4", str(j))
        # fly(j)
        print(j)

    cutthewings()
    # print("This is the end")

def fly(ii):
#function
    crazyflie.param.set_value("motors.motorPowerM1",str[ii])
    crazyflie.param.set_value("motors.motorPowerM2",str[ii])
    crazyflie.param.set_value("motors.motorPowerM3",str[ii])
    crazyflie.param.set_value("motors.motorPowerM4",str[ii])

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
