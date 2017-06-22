from um7 import UM7array

name1 = 'sensor1'
port1 = '/dev/tty.RNBT-835C-RNI-SPP'

name2 = 'sensor2'
port2 = '/dev/tty.usbserial-A903AAV3'

names = [name1, name2]
ports = [port1, port2]
measurements = ['xaccel','roll', 'pitch', 'yaw']

sensor_array = UM7array([name1], [port1], measurements, baud=115200)
# sensor_array = UM7array([name2], [port2], measurements, baud=115200)
# sensor_array = UM7array(names, ports, measurements, baud=115200)
sensor_array.btstart() #include this line if using rn-42 bluetooth modules
sensor_array.settimer()
while True:
    sensor_array.catchsample()
    print sensor_array.state


# Notes
# only use um7 array
# info held in object.state, which is a dict
# be careful of syncing data
# if no data is received for variable, nan is returned to flag interpolation
# use btstart for bluetooth (initiates rn-42 fast mode)
# sometimes you get double data
