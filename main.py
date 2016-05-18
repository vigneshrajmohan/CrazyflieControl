# from cflib.crazyflie import Crazyflie
# from cflib.utils import callbacks
# import cflib.crtp
# import time
# import logging
# logging.basicConfig(level=logging.ERROR)
# #from time import sleep


#--------------- original file codes
# import logging
# import time
# from threading import Thread
#
# import cflib
# from cflib.crazyflie import Crazyflie
#
# logging.basicConfig(level=logging.ERROR)
#
#
# class MotorRampExample:
#     """Example that connects to a Crazyflie and ramps the motors up/down and
#     the disconnects"""
#
#     def __init__(self, link_uri):
#         """ Initialize and run the example with the specified link_uri """
#
#         self._cf = Crazyflie()
#
#         self._cf.connected.add_callback(self._connected)
#         self._cf.disconnected.add_callback(self._disconnected)
#         self._cf.connection_failed.add_callback(self._connection_failed)
#         self._cf.connection_lost.add_callback(self._connection_lost)
#         link_uri = "radio://0/80/250K"
#         self._cf.open_link(link_uri)
#
#         print('Connecting to %s' % link_uri)
#
#     def _connected(self, link_uri):
#         """ This callback is called form the Crazyflie API when a Crazyflie
#         has been connected and the TOCs have been downloaded."""
#
#         # Start a separate thread to do the motor test.
#         # Do not hijack the calling thread!
#         Thread(target=self._hover_motion).start()
#
#     def _connection_failed(self, link_uri, msg):
#         """Callback when connection initial connection fails (i.e no Crazyflie
#         at the specified address)"""
#         print('Connection to %s failed: %s' % (link_uri, msg))
#
#     def _connection_lost(self, link_uri, msg):
#         """Callback when disconnected after a connection has been made (i.e
#         Crazyflie moves out of range)"""
#         print('Connection to %s lost: %s' % (link_uri, msg))
#
#     def _disconnected(self, link_uri):
#         """Callback when the Crazyflie is disconnected (called in all cases)"""
#         print('Disconnected from %s' % link_uri)

# --- Shanthi's find begins -----------------------------------------------*****

'''
Sensors
'''

import time
from threading import Thread, Timer

#Has to be launched from within the autonomous folder

import cflib
from cflib.crazyflie import Crazyflie

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig


import logging
logging.basicConfig(level=logging.ERROR)

import crazy_auto1
from pykalman import KalmanFilter
import numpy as np


class logs:

     def __init__(self, cf):

        #self.log_file_next_stuff = open("log_file_next.txt", "w+")
        #self.log_file_stab = open("log_file_stab.txt", "w+")

        #local copy of crazy_Auto
         self._cf = cf

        # Roll, Pitch, Yaw
         self.init_state = [0,0,0]
         self.state = [0,0,0]
         self.next_state = [0,0]
         # X, Y, Z, Mag
         self.init_accel_state = [0,0,0,0]
         self.accel_state = [0,0,0,0]

         # X, Y, Z
         self.init_gyro_state = [0,0,0]
         self.gyro_state = [0,0,0]

         # ground average, current
         self.altitude = [0,0]

        #current battery voltage
         self.battery_state = 0

         #[roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]
         self.measurements = [0 for i in range(12)]

         self.logging_count = 0

         #Setup logs once connection to crazyflie has been established
         self._cf._cf.connected.add_callback(self._init_flight_var)

         # specifying the size of the state and the observation space
         #self.transition_matrices = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
         #self.transition_matrices.resize(4,1)
         #self.kf = KalmanFilter(transition_matrices=self.transition_matrices, n_dim_obs= 4) #transition_matrices=self.transition_matrices,


         #self._cf.connected.add_callback(self._init_flight_var) #fill in base parameters for flight



     def _init_flight_var(self, link_uri):

         print ("Connected to %s" % link_uri)

         self.RPY_log = LogConfig(name="Stabilizer", period_in_ms=10)

         self.RPY_log.add_variable("stabilizer.roll", "float")
         self.RPY_log.add_variable("stabilizer.pitch", "float")
         self.RPY_log.add_variable("stabilizer.yaw", "int16_t")

         self.RPY_log.add_variable("baro.asl", "float") #barometer above sea level

         self.RPY_log.add_variable("gyro.x", "float")
         self.RPY_log.add_variable("gyro.y", "float")
         self.RPY_log.add_variable("gyro.z", "float")
         self.battery_log = LogConfig(name="Battery", period_in_ms=1000)
         self.battery_log.add_variable("pm.vbat", "float")

         self.acc_log = LogConfig(name="Acc", period_in_ms = 10)
         self.acc_log.add_variable("acc.x", "float")
         self.acc_log.add_variable("acc.y", "float")
         self.acc_log.add_variable("acc.z", "float")

         self._cf._cf.log.add_config(self.RPY_log) #add the log to the CrazyFlie
         self._cf._cf.log.add_config(self.battery_log)
         self._cf._cf.log.add_config(self.acc_log)

         self.RPY_log.data_received_cb.add_callback(self.update_flight_params)
         self.RPY_log.error_cb.add_callback(self.update_error)
         self.battery_log.data_received_cb.add_callback(self.update_battery)
         self.battery_log.error_cb.add_callback(self.update_error)

         self.acc_log.data_received_cb.add_callback(self.update_acc)
         self.acc_log.error_cb.add_callback(self.update_error)


         self.RPY_log.start()
         self.battery_log.start()
         self.acc_log.start()
         print ("Logging Started\n")

         self._cf._cf.connected.add_callback(self._cf._connected)

     def update_error(self, logconf, msg):
         print ("Error when logging %s: %s" % (logconf.name, msg))

     def update_battery(self, timestamp, data, logconf):
         self.battery_state = data["pm.vbat"]
     def update_acc(self, timestamp, data, logconf):
         if not self._cf.is_flying:
             self.init_accel_state[0] = data["acc.x"]
             self.init_accel_state[1] = data["acc.y"]
             self.init_accel_state[2] = data["acc.z"]
             self.init_accel_state[3] = data["acc.mag2"]
             return
         self.accel_state[0] = data["acc.x"]
         self.accel_state[1] = data["acc.y"]
         self.accel_state[2] = data["acc.z"]
         self.accel_state[3] = data["acc.mag2"]

     def update_flight_params(self, timestamp, data, logconf):
         #print data
         #print self._cf.is_flying
         if not self._cf.is_flying:
             self.init_state[0] = float(data["stabilizer.roll"])
             self.init_state[1] = float(data["stabilizer.pitch"])
             self.init_state[2] = data["stabilizer.yaw"]

             #self.init_accel_state[0] = data["acc.x"]
             #self.init_accel_state[1] = data["acc.y"]
             #self.init_accel_state[2] = data["acc.z"]
             #self.init_accel_state[3] = data["acc.mag2"]

             self.init_gyro_state[0] = data["gyro.x"]
             self.init_gyro_state[1] = data["gyro.y"]
             self.init_gyro_state[2] = data["gyro.z"]

             #self.battery_state =  data["pm.vbat"]

             #if self.logging_count is 0:
             self.altitude[0] = data["baro.asl"]
             #    self.logging_count += 1
             #else:

             #    self.altitude[0] = float((data["baro.asl"] + self.logging_count*self.altitude[0])/(self.logging_count+1)) #take the running average of the inital altitude for better ground reading
             #    self.logging_count += 1

             #self._cf.altitude[1] = self.altitude[0] #<-----------this shouldnt be here....

             #[roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]

             #print self.altitude
             return

         self.state[0] = data["stabilizer.roll"]
         self.state[1] = data["stabilizer.pitch"]
         self.state[2] = data["stabilizer.yaw"]

         #self.accel_state[0] = data["acc.x"]
         #self.accel_state[1] = data["acc.y"]
         #self.accel_state[2] = data["acc.z"]
         #self.accel_state[3] = data["acc.mag2"]

         self.gyro_state[0] = data["gyro.x"]
         self.gyro_state[1] = data["gyro.y"]
         self.gyro_state[2] = data["gyro.z"]

         #self.battery_state =  data["pm.vbat"]

         self.altitude[1] = data["baro.asl"]
         #print self.altitude
         self.next_state[0] = self.state[0] - self.init_state[0]
         self.next_state[1] = self.state[1] - self.init_state[0]


     def log_file_print(self, file, data):
         for i in range(len(data)):
             file.write(str(data[i]))
             file.write(',')
         file.write('\n')



     def get_measurements(self):
         pass
         #return self.measurements
         #self.kf = self.kf.em(self.measurements, n_iter=1)
         #return self.kf.filter(self.measurements)

     def get_gyro(self, value):

         if (value == 0):
             print ("X value:  ", self.gyro_state[0])

         elif(value == 1):
             print ("Y value:  ", self.gyro_state[1])

         elif(value == 2):
             print ("Z value:  ", self.gyro_state[2])
         else:
             return 0


     def get_altitude(self):
         #print "altitude:  ", self.baro_sensor[0]
         return self.altitude[0]


     def get_roll(self):
         print ("roll:  ", self.state[0])

     def get_pitch(self):
         print ("pitch:  ", self.state[1])

     def get_yaw(self):
         print ("yaw:  ", self.state[2])


     def get_baro(self, value):

         if (value == 0):
             print ("lRaw:  ", self.baro_sensor[0])

         elif(value == 1):
             print ("lLong:  ", self.baro_sensor[0])

         elif(value == 2):
             print ("Pressure:  ", self.baro_sensor[0])
         else:
             return 0

if __name__ == '__main__':
     # Initialize the low-level drivers (don't list the debug drivers)
     cflib.crtp.init_drivers(enable_debug_driver=False)
     # Scan for Crazyflies and use the first one found
     print ("Scanning interfaces for Crazyflies...")
     available = cflib.crtp.scan_interfaces()
     print ("Crazyflies found:")
     for i in available:
         print (i[0])

     if len(available) > 0:
         le = crazy_auto1.Crazy_Auto(available[0][0])
         #t = logs(le)
         while le.is_connected:
             time.sleep(1)
         # t.log_file_stab.close()
         # t.log_file_next_stuff.close()
         # print "Roll, Pitch, Yaw",t.state
         # print "Gyro",t.gyro_state
         # t.get_gyro(1)

     else:
         print ("No Crazyflies found, cannot run example")







""" Autonomous flight library for Crazyflie2 """

import sys
sys.path.append("../lib")

import time

from threading import Thread, Timer

#Has to be launched from within the autonomous folder

import cflib
from cflib.crazyflie import Crazyflie

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig

import numpy as np

import logging
logging.basicConfig(level=logging.ERROR)

import Sensors


class Crazy_Auto:
     """ Basic calls and functions to enable autonomous flight """
     def __init__(self, link_uri):
         """ Initialize crazyflie using passed in link"""

        #trim variables that will be updated during flight for stabilization
         self.roll_trim = 2
         self.pitch_trim = -1

         #-------------- PID stuff -------------------#

         self.avg_state = [0,0,0]
         self.last_state = [0,0,0]

         self.num_readings = 0
         self.state_sum = [0,0,0]

         self.kproportional = .1

         #----------------- END PID --------------------#

         #----------------- Flight Vars ----------------#
         self.next_roll = 0
         self.next_pitch = 0
         self.next_thrust = 0

         self.gyro_threash = 15

         self.take_off_thrust_step = 500

         self.kill_threash = 10000
         self.take_off_thrust = 38000

         self.hover_alt_step = 3
         self.thrust_step = 500

         self.hover_thrust_step = 250

         self.is_hovering = False

         self.min_thrust = 0
         self.is_flying = False

         #---------------- End Flight Vars --------------#

         #self.altitude = [0,0]  #[base/initial, current]
         #self.current_altitude = 0

         self.curr_thrust = 10000 #10000 is the base thrust to get the props moving

         self.max_thrust = 60000 #max thrust we ever want to give it


         self._cf = Crazyflie() #class declaration is in ..\lib\cflib\crazyflie\__init__.py
         self.t = Sensors.logs(self)

         #the three function calls below setup threads for connection monitoring
         self._cf.disconnected.add_callback(self._disconnected) #first monitor thread checking for disconnections
         self._cf.connection_failed.add_callback(self._connection_failed) #second monitor thread for checking for back connection to crazyflie
         self._cf.connection_lost.add_callback(self._connection_lost) # third monitor thread checking for lost connection

         print ("Connecting to %s" % link_uri)

         self._cf.open_link(link_uri) #connects to crazyflie and downloads TOC/Params

         self.is_connected = True

### ---------------------------------------------------------- Main Flight States ---------------------------------------------------------------###
     def hover_mode(self):
         self.is_hovering = True
         print ("hovering")
         #t = Timer(10, self._land_copter())
         #t.start()
         yawrate = 0
         sleep_step = .1


         while self.is_hovering:
             #print self.t.accel_state
             if self.t.gyro_state[2] >= self.gyro_threash:
                 self.next_thrust += self.hover_thrust_step
             elif self.t.gyro_state[2] <= -self.gyro_threash:
                 self.next_thrust -= self.hover_thrust_step*2
             print (self.next_thrust)
             #self.next_thrust = self.curr_thrust + (self.hover_alt - self.altitude[1])*self.thrust_step
             #print self.t.gyro_state
             self._cf.commander.send_setpoint(self.next_roll + self.roll_trim, self.next_pitch+self.pitch_trim, yawrate, self.next_thrust)
             time.sleep(sleep_step)


     def _take_off(self):

         #self.altitude[0] = self.t.get_altitude()
         self.hover_alt = self.t.altitude[0] + self.hover_alt_step

         while self.hover_alt == self.hover_alt_step or self.t.altitude[0] == 0:
             #print self.altitude
             #self.altitude[0] = self.t.get_altitude()
             self.hover_alt = self.t.altitude[0] + self.hover_alt_step
         time.sleep(5)
         self.is_flying = True

         yawrate = 0
         sleep_step = .1
         co = 0
         #print self.t.altitude, self.hover_alt
         print ("taking off")
         while self.t.altitude[1] <= self.hover_alt or self.curr_thrust < self.take_off_thrust:
             if self.t.altitude[1] < self.t.altitude[0]:
                 self.t.altitude[0] = self.t.altitude[1]
                 self.hover_alt = self.t.altitude[0] + self.hover_alt_step
             #print self.t.altitude, self.hover_alt
             print ("roll: ",self.next_roll, "pitch: ",self.next_pitch, "gyro: ",self.t.gyro_state)
             #print self.altitude, self.hover_alt
             self._cf.commander.send_setpoint(self.next_roll+self.roll_trim, self.next_pitch + self.pitch_trim, yawrate,self.curr_thrust)
             self.curr_thrust += self.take_off_thrust_step
             #print self.curr_thrust
             #print self.altitude[1], self.hover_alt
             time.sleep(sleep_step)
             co += 1

         #print self.altitude
         self.hover_alt = self.t.altitude[1]
         self.next_thrust = self.curr_thrust - self.take_off_thrust_step*2
         Thread(target=self.fly_circle(20)).start()
         #Thread(target=self._land_copter).start()

     def _land_copter(self):
         self.is_hovering = False
         print ("landing")
         landing_thrust = self.curr_thrust
         while self.altitude[1]  <= self.altitude[0]:# or abs(self.accel_state[2]) > 0:
             landing_thrust -= self.thrust_step
             self._cf.commander.send_setpoint(self.next_roll, self.next_pitch, 0, landing_thrust)

         self.is_flying = False
         time.sleep(.1)
         self._cf.close_link()

### --------------------------------------------------------------------------- END MAIN FLIGHT STATES -----------


# ----- circle fly begins ------------------------

     def fly_circle(self, max_angle):
         yaw = 0
         count = 0
         print ("start circle")
         increment = (max_angle/2)*.1
         self.is_hovering = True
         if self.is_hovering:
             # 3 degrees per time slic
             roll_circle = max_angle
             pitch_circle = 0
             while (roll_circle > 0 and pitch_circle < max_angle):
                 self._cf.commander.send_setpoint( roll_circle,+pitch_circle, yaw, self.next_thrust)
                 print ('1')
                 roll_circle -= increment
                 pitch_circle += increment
                 time.sleep(.1)

             roll_circle = 0
             pitch_circle = max_angle
             while (roll_circle > -(max_angle) and pitch_circle > 0):
                 self._cf.commander.send_setpoint(roll_circle, pitch_circle, yaw, self.next_thrust)
                 print ('2')

                 roll_circle -= increment
                 pitch_circle -= increment
                 time.sleep(.1)

             roll_circle = -max_angle
             pitch_circle = 0
             while (roll_circle < 0 and pitch_circle > -(max_angle)):
                 self._cf.commander.send_setpoint(roll_circle,pitch_circle, yaw, self.next_thrust)
                 print ('3')
                 roll_circle += increment
                 pitch_circle -= increment
                 time.sleep(.1)


             roll_circle = 0
             pitch_circle = -max_angle
             while (roll_circle < (max_angle) and pitch_circle < 0):
                 self._cf.commander.send_setpoint(roll_circle,pitch_circle, yaw, self.next_thrust)
                 print ('4')
                 roll_circle += increment
                 pitch_circle += increment
                 time.sleep(.1)

     def _ramp_motors(self):
         print ("Starting Flight")
         print ("Start Roll, Pitch", self.init_state)
         timer_start = False
         self.is_flying = True
         thrust_mult = 1
         thrust_step = 500
         thrust = 20000
         pitch = 0
         roll = 0
         yawrate = 0
         sleep_step = .1
         while thrust >= 20000:
             self._cf.commander.send_setpoint(self.next_roll, self.next_pitch, yawrate, thrust)
             time.sleep(sleep_step)
             if thrust >= 45000:
                 if timer_start is False:
                     t = Timer(10, self._cf.close_link)
                     t.start()
                     timer_start = True
             else:
                 thrust += thrust_step * thrust_mult
         self._cf.commander.send_setpoint(0, 0, 0, 0)

         # Make sure that the last packet leaves before the link is closed
         # since the message queue is not flushed before closing
         time.sleep(0.1)
         self._cf.close_link()

     def kill_check(self): #
         if self.t.accel_state[3] >= self.kill_threash:
             self._cf.commander.send_setpoint(0,0,0,0);
             print ("QUADCOPTER KILLED --- Acceleration Mag: ", self.t.accel_state)
             self.is_hovering = False
             self.is_flying = False
             self._cf.close_link()
         else:
             Timer(.5, self.kill_check).start()

# -------- circle fly ends -------------------------


 ###------------------------------------------------------------------------------ END Flight Functions -------------------------------------------------------###


     def update_error(self, logconf, msg):
         print ("Error when logging %s: %s" % (logconf.name, msg))

     def update_vals(self):
         #m = self.t.get_measurements() #  m = [roll,pitch,yaw,altitude,acc.x,acc.y,acc.z,acc.mag,gyro.x, gyro.y, gyro.z, batteryV]

         #print m
         pre_roll = self.t.next_state[0]#float(m[0])
         pre_pitch = self.t.next_state[1]#float(m[1])
         self.state_PID( pre_roll, pre_pitch)

         Timer(.1, self.update_vals).start()

     def state_PID(self, p_r, p_p):
         error = [p_r,p_p, float(self.next_thrust)]

         kp = self.kproportional#[self.avg_state[i] - error[i] for i in range(len(error))]

         #self.state_sum = [self.state_sum[i] + error[i] for i in range(len(error))]

         p = [error[i]*kp for i in range(len(error))]

         #ii = [self.state_sum[i]*kp for i in range(len(self.state_sum))]

         #d = [float((error[i]-self.last_state[i])/.1) for i in range(len(error))]

         #self.pid_file.write(str(p[0])+','+str(p[1])+','+str(p[2]))

         #self.pid_file.write(str(ii)+' ')
         #self.pid_file.write(str(d))
         #self.pid_file.write('\n')
         #if (self.t.gyro_state
         self.next_roll = p[0] #+ ii[0] + d[0]
         self.next_pitch = p[1]# + ii[1] + d[1]
         #self.next_thrust = p[2]# + ii[2] + d[2]


 ### ------------------------------------------------------- Callbacks ----------------------------------------------------------------------###
     def _connected(self, link_uri):
         """ This callback is called form the Crazyflie API when a Crazyflie
         has been connected and the TOCs have been downloaded."""

         # Start a separate thread to do the motor test.
         # Do not hijack the calling thread!
         Thread(target=self.update_vals).start()
         #Thread(target=self.kill_check).start()
         print ("Waiting for logs to initalize...")
         Thread(target=self._take_off).start()


     def _connection_failed(self, link_uri, msg):
         """Callback when connection initial connection fails (i.e no Crazyflie
         at the speficied address)"""
         print ("Connection to %s failed: %s" % (link_uri, msg))
         self.is_connected = False

     def _connection_lost(self, link_uri, msg):
         """Callback when disconnected after a connection has been made (i.e
         Crazyflie moves out of range)"""
         print ("Connection to %s lost: %s" % (link_uri, msg))
         self.is_connected = False

     def _disconnected(self, link_uri):
         """Callback when the Crazyflie is disconnected (called in all cases)"""

         print ("Disconnected from %s" % link_uri)
         self.is_connected = False
 ### ------------------------------------------------------ END CALLBACKS ------------------------------------------



















# ----- baseline2a codes begins ---------------------------------------------
#     def _hover_motion(self):
#
#         thrustchange = 1000
#         startthrust = 50000
#
#         #measurements
#         thrust = 45000
#         pitch = 0
#         roll = 0
#         yawrate = 0
#
#         num = 0
#         itime = 0.1
#         itime_num = 5
#
#         self._cf.commander.send_setpoint(0, 0, 0, 0)
#
#         self._cf.param.set_value("flightmode.althold","True")
#         self._cf.commander.send_setpoint(0, 0, 0, startthrust)
#         time.sleep(itime * itime_num)
#         print(num, thrust, itime*2)
#
#         for num in range(0,4,1):
#             thrust += num * thrustchange
#             self._cf.param.set_value("flightmode.althold","True")
#             self._cf.commander.send_setpoint(0, 0, 0, thrust)
#             time.sleep(itime)
#             print(num, thrust, itime)
#
#
#         for num in range(4,1,-1):
#             thrust -= num * thrustchange
#             self._cf.param.set_value("flightmode.althold","True")
#             self._cf.commander.send_setpoint(0, 0, 0, thrust)
#             time.sleep(itime)
#             print(num, thrust, itime)
#             thrust -= num * thrustchange
#             self._cf.param.set_value("flightmode.althold","True")
#             self._cf.commander.send_setpoint(0, 0, 0, thrust)
#             time.sleep(itime)
#             print(num, thrust, itime)
#             thrust -= num * thrustchange
#             self._cf.param.set_value("flightmode.althold","True")
#             self._cf.commander.send_setpoint(0, 0, 0, thrust)
#             time.sleep(itime)
#             print(num, thrust, itime)
#
#
#         # thrust = 15000
#         # self._cf.param.set_value("flightmode.althold","True")
#         # self._cf.commander.send_setpoint(0, 0, 0, thrust)
#         # time.sleep(.05)
#
#
#         # while thrust >= 15000:
#         #     # originally 20000
#         #     self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
#         #     time.sleep(0.1)
#         #     if thrust >= 21000:
#         #         # originally it was 25000
#         #         thrust_coeff = -5
#         #
#         #     thrust += thrustchange * thrust_coeff
#
#
#         self._cf.commander.send_setpoint(0, 0, 0, 0)
#         # Make sure that the last packet leaves before the link is closed
#         # since the message queue is not flushed before closing
#         time.sleep(0.1)
#         self._cf.close_link()
#
#------ original baseline2a code ends --------


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
