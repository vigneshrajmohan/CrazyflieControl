require 'artoo'

connection :crazyflie, :adaptor => :crazyflie, :port => '/dev/tty.Bluetooth-Incoming-Port', :supports_hover => true
device :drone, :driver => :crazyflie, :connection => :crazyflie, :interval => 0.1

work do
  drone.start
  drone.take_off

  after(1.seconds) {drone.stop}
end
