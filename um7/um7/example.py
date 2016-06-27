from um7 import UM7

name1 = 'sensor1'
name2 = 'sensor2'
port1 = '/dev/tty.RNBT-8C0B-RNI-SPP'
port2 = '/dev/tty.RNBT-E341-RNI-SPP'
# port = '/dev/tty.usbserial-A903AAV1'

# sensor1 = UM7(name1, port1)
sensor2 = UM7(name2, port2)
# sensors = [sensor1, sensor2]
# sensors = [sensor1]
sensors = [sensor2]
for i in sensors:
    i.zerogyros()
    i.resetekf()
    i.settimer()

while True:
    # sensor.grabsample(['xaccel', 'yaccel', 'zaccel', 'xgyro', 'rollpitch', 'yaw'])
    for i in sensors:
        i.catchsample()
        print i.state


