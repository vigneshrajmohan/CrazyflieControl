# from cflib.crazyflie import Crazyflie
# from cflib.utils import callbacks
# import cflib.crtp
# import time
# import logging
# logging.basicConfig(level=logging.ERROR)
# #from time import sleep
#
#
# def connect():
#     available = cflib.crtp.scan_interfaces()
#     for i in available:
#         print("Interface with URI {0} found and name/comment {1}".format(i[0], i[1]))
#     address = input("Please enter the connection string:  ")
#     crazyflie.open_link("radio://0/80/250K")
#
#
# def disconnect():
#     crazyflie.close_link()
#
# def connected(*args):
#     # Code here
#     #
#     # roll    = 0.0
#     # pitch   = 0.0
#     # yawrate = 0
#     # thrust  = 10000
#     #
#     # send_setpoint(roll, pitch, yaw, thrust)
#     # """
#         # Send a new control set-point for roll/pitch/yaw/thust to the copter
#         #
#         # The arguments roll/pitch/yaw/trust is the new set-points that should
#         # be sent to the copter
#         # """
#
#     #crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust)
#
#     for i in range(10000, 50001, 1000):
#         crazyflie.param.set_value("motors.motorPowerM1", str(i))
#         crazyflie.param.set_value("motors.motorPowerM2", str(i))
#         crazyflie.param.set_value("motors.motorPowerM3", str(i))
#         crazyflie.param.set_value("motors.motorPowerM4", str(i))
#         # fly(i)
#         print(i)
#
#
#     # time.sleep(3)
#     #
#     for j in range(50000, 0, -1000):
#         crazyflie.param.set_value("motors.motorPowerM1", str(j))
#         crazyflie.param.set_value("motors.motorPowerM2", str(j))
#         crazyflie.param.set_value("motors.motorPowerM3", str(j))
#         crazyflie.param.set_value("motors.motorPowerM4", str(j))
#         # fly(j)
#         print(j)
#     print("This is a test yo")
#     print (crazyflie.param.get_value("gyro.x"))
#     print("This is the bottom of a test ahah")
#     # cutthewings()
#     # print("This is the end")
#
# def fly(ii):
# #function
#     crazyflie.param.set_value("motors.motorPowerM1",str[ii])
#     crazyflie.param.set_value("motors.motorPowerM2",str[ii])
#     crazyflie.param.set_value("motors.motorPowerM3",str[ii])
#     crazyflie.param.set_value("motors.motorPowerM4",str[ii])
#
# def cutthewings():
# #function
#     crazyflie.param.set_value("motors.motorPowerM1","0")
#     crazyflie.param.set_value("motors.motorPowerM2","0")
#     crazyflie.param.set_value("motors.motorPowerM3","0")
#     crazyflie.param.set_value("motors.motorPowerM4","0")
#
# # def gyro():
# #     crazyflie.param.get_value("gyro.x")
#
#
#
#
# cflib.crtp.init_drivers()
# crazyflie = Crazyflie()
# crazyflie.connected.add_callback(connected)
# connect()
import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie

logging.basicConfig(level=logging.ERROR)


class MotorRampExample:
    """Example that connects to a Crazyflie and ramps the motors up/down and
    the disconnects"""

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)
        link_uri = "radio://0/80/250K"
        self._cf.open_link(link_uri)

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._hover_motion).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)

    def _hover_motion(self):

        thrustchange = 1000
        startthrust = 60000

        #measurements
        thrust = 55000
        pitch = 0
        roll = 0
        yawrate = 0

        num = 0
        itime = 0.25

        self._cf.commander.send_setpoint(0, 0, 0, 0)

        # self._cf.param.set_value("flightmode.althold","True")
        # self._cf.commander.send_setpoint(0, 0, 0, startthrust)
        time.sleep(itime*2)
        print(num, thrust, itime*2)

        for num in range(1,6,1):
            # self._cf.param.set_value("flightmode.althold","True")
            # self._cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(itime)
            print(num, thrust, itime)
            thrust += num * thrustchange

        for num in range(7,3,-2):
            # self._cf.param.set_value("flightmode.althold","True")
            # self._cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(itime)
            print(num, thrust, itime)
            thrust -= num * thrustchange


        # thrust = 15000
        # self._cf.param.set_value("flightmode.althold","True")
        # self._cf.commander.send_setpoint(0, 0, 0, thrust)
        # time.sleep(.05)


        # while thrust >= 15000:
        #     # originally 20000
        #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
        #     time.sleep(0.1)
        #     if thrust >= 21000:
        #         # originally it was 25000
        #         thrust_coeff = -5
        #
        #     thrust += thrustchange * thrust_coeff


        self._cf.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
        self._cf.close_link()



if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        le = MotorRampExample(available[0][0])
    else:
        print('No Crazyflies found, cannot run code')
