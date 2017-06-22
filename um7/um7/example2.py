#!/usr/bin/env python

import um7
import time

name = sensor1
port = '/dev/tty.usbserial-A903AAV1'
measurements = ['roll']
timevar = name + ' time'

sensor = um7.UM7(name, port, measurements, baud=115200)
time.sleep(2)
sensor.settimer()
print 'Starting BT...'
sensor.btstart()
while not sensor.zerogyros():
    pass
while not sensor.resetekf():
    pass
data = {}
try:
    print 'Taking data...'
    while True:
        while True:
            newmsg = sensor.catchsample()
            if newmsg: data.update(newmsg)
            print data
            if set(sensor.statevars) - set(data.keys()) == set([timevar]):
                break
        try:
            for i in data.keys():
                if i != timevar:
                    data[i] = data[i]*3.14159/180.0
                data[i] = str(data[i])
            socket.send_json(data)
            data = {}
        except TypeError as e:
            print('TypeError')
except KeyboardInterrupt:
    print('Done')
    del sensor
