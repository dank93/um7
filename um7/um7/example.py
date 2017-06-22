from um7 import UM7array

name1 = 'sensor1'
name2 = 'sensor2'
port1 = '/dev/tty.usbserial-A903AAV3'

# names = [name1, name2]
# ports = [port1, port2]
names = [name1]
ports = [port1]
measurements = ['xaccel', 'roll', 'pitch', 'yaw']

sensor_array = UM7array(names, ports, measurements, baud=115200)
sensor_array.btstart() #include if using 2n-42 bluetooth module
sensor_array.settimer()

while True:
    sensor_array.catchsample()
    print sensor_array.state

# UM7array.btstart() puts bt module into fast module
# settimer resets the timestamps
# variables given 'nan' value to if there is no new value (better than repeating)
