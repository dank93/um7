"""Module to interface with CHR UM7 IMU

    Important Notes:
        Timestamps are based on OS time, not sensor's internal timer
        Sensor object's most recent data points are stored in sensor.state (dict)
        Most functions return False if invalid data type is found
        This module does not check incoming data checksums
        Valid arguments for object.grabsample(datatype) can be found below

"""

# Daniel Kurek
# d'Arbeloff Lab, MIT
# January, 2016
# Module that holds UM7 class
# Creates serial objects, contains functions to parse serial data

#####################################################################
# TODO: Broadcast Rate Settings, Optimize Data Collection
#####################################################################


import serial
import time
import binascii
import struct
import numpy
import sys


class UM7array(object):

    def __init__(self, names, ports, statevars, baud=115200):
        self.t0 = time.time()  # Reference time
        self.state = {}  # Dict that holds current state
        self.statemask = {}  # dict of NaNs to mask out old data
        self.sensors = []  # List of um7 objects in array
        for i in range(len(names)):
            svars = [j for j in statevars]
            s = UM7(names[i], ports[i], svars, baud)
            self.sensors.append(s)
        statevars[:] = [i + ' ' + j for i in names for j in statevars]
        statevars = ['time'] + statevars
        self.statevars = statevars
        self.history = numpy.zeros(len(statevars))
        for i in statevars:
            self.state.update({i: float('NaN')})
            self.statemask.update({i: float('NaN')})

    def __del__(self):
        for i in self.sensors:
            i.serial.close()
        print 'Array closed.'

    def settimer(self):
        self.t0 = time.time()

    def btstart(self):
        for i in self.sensors:
            i.btstart()

    def catchsample(self):
        for i in self.sensors:
            i.catchsample()
        self.updatestate()
        self.updatehistory()

    def catchallsamples(self, timeout=0.02):  # How do we update state/history?!?!
        for i in self.sensors:
            i.catchallsamples(timeout=timeout)
            # self.updatestate(i)
            # self.updatehistory()
        self.updatestate()
        self.updatehistory()

    # def updatestate(self, s):
        # sensorstate = {k: v for k, v in s.state.items()}
        # sensorstate.pop('time')
        # mask = {k: v for k, v in self.statemask.items()}
        # mask.update(sensorstate)
        # mask.update({'time': time.time() - self.t0})
        # self.state.update(mask)

    def updatestate(self):
        self.state.update({'time': time.time() - self.t0})  # maybe mask other sensor states to avoid oversampling
        for i in self.sensors:                              # also it lets you take more accurate time measurements
            sensorstate = {k: v for k, v in i.state.items()}
            sensorstate.pop(i.name + ' time')
            self.state.update(sensorstate)

    def updatehistory(self):
        state = numpy.array([])
        for i in self.statevars:
            state = numpy.append(state, self.state[i])
        self.history = numpy.vstack((self.history, state))

    def checkbuffer(self, numbytes):
        for i in self.sensors:
            if i.serial.inWaiting() > numbytes:
                print("flush")
                i.serial.flushInput()


class UM7(object):
    """ Class that handles UM7 interfacing. Creates serial object for communication, contains functions to request specific
        data samples, catch any incoming data, check input buffer, and set various data broadcast rates. Currently only
        handles processed accel, gyro, and euler angle data.  Data is timed by OS.
    """

    def __init__(self, name, port, statevars, baud=115200):
        """Create new UM7 serial object.
        Defuault Baud Rate = 115200
        Byte Size = 8 bits
        No Parity, 1 Stop Bit, 0 second timeout
        Initializes port, name, OS timer, and sensor state (dict)
        :param port: Virtual COM port to which the IMU is connected (str)
                name: name of object (str)
        :return: UM7 Object
        """
        statevars[:] = [name + ' ' + i for i in statevars]
        statevars = [name + ' time'] + statevars
        self.name = name
        self.t0 = time.time()
        self.state = {}
        self.statemask = {}
        self.statevars = statevars
        for i in statevars:
            self.state.update({i: float('NaN')})
            self.statemask.update({i: float('NaN')})
        try:
            self.serial = serial.Serial(port, baudrate=baud, bytesize=8, parity='N', stopbits=1, timeout=0)  # Open serial device
            self.serial.flushInput()
            self.serial.write('$$$')
            print 'Successfully connected to %s UM7!' % self.name
        except OSError:
            print 'Could not connect to %s UM7. Is it plugged in or being used by another program?' % self.name

    def __del__(self):
        """Closes virtual com port

        :return: None
        """
        self.serial.close()
        print '%s serial device closed' % self.name

    def __name__(self):
        return self.name

    def catchsample(self):
        """Function that catches and parses incoming data, and then updates the sensor's state to include new data. Old
        data in state is overwritten. Data is timed by OS

        :return: Newly obtained data, and updates internal sensor state
        """
        [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
        if not foundpacket:
            return False
        sample = parsedatabatch(data, startaddress, self.name)
        if sample:
            copy = sample.copy()
            self.updatestate(copy)
        return sample

    def catchallsamples(self, timeout):
        sample = {}  # Initialize empty dict for new samples
        t0 = time.time()  # Initialize timeout timer
        while time.time() - t0 < timeout:  # While elapsed time is less than timeout
            foundpacket = 0
            # if self.serial.inWaiting() > 500:
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()  # Read a packet
            if foundpacket:  # If you got one
                newsample = parsedatabatch(data, startaddress, self.name)  # extract data
                if newsample:  # If it works
                    sample.update(newsample)  # Update sample with new sample
            if list(set(self.statevars)-set(sample.keys())) == [self.name + ' time']:  # If we have a new data key for every
                                                                            # var in statevar minus 'time'
                break  # Then we have all new data and can move on
        if list(set(self.statevars) - set(sample.keys())) != [self.name + ' time']:  # In case we timed out before we caught every var we want
            print 'Missed some vars!', self.serial.inWaiting()
            # return False
        if sample:  # If we have any new data
            self.updatestate(sample)  # Update the sensor state
        return sample  # Return the sample

    def grabsample(self, datatype):
        """Function that flushes buffers and then requests and then waits for specific datatype. ONLY WORKS IF BROADCAST
        SETTINGS FOR REQUESTED DATA TYPE ARE ALREADY SET TO ZERO. Generally much slower than catchsample()

        :param datatype: 'xaccel', 'yaccel', 'zaccel', 'xgyro', 'ygyro', 'zgyro', 'rollpitch', 'yaw', 'rollpitchrate',
         and/or 'yawrate', given in list form
        :return: Newly obtained data, and updates internal sensor state
        """
        sample = {}
        for i in datatype:
            address = name2hex_reg[i]
            returnaddress = []
            self.serial.flushInput()
            self.request(i)
            while address != returnaddress:
                [foundpacket, hasdata, returnaddress, data, commandfailed] = self.readpacket()
            sample.update(parsedata(data, returnaddress, self.name))
        if sample:
            self.updatestate(sample)
        return sample

    def readpacket(self):
        """Scans for and partially parses new data packets. Binary data can then be sent to data parser

        :return: Parsed packet info
        """
        foundpacket = 0
        count = 0
        t = time.time()
        while True:
            count += 1
            if self.serial.inWaiting() > 100:
                byte = self.serial.read(size=1)
                if byte == 's':
                    byte2 = self.serial.read(size=1)
                    if byte2 == 'n':
                        byte3 = self.serial.read(size=1)
                        if byte3 == 'p':
                            foundpacket = 1
                            break
        if foundpacket == 0:
            hasdata = 0
            commandfailed = 0
            startaddress = 0
            data = 0
        else:
            try:
                ptbyte = bin(int(binascii.hexlify(self.serial.read(size=1)), 16))[2:]
                ptbyte = ptbyte.zfill(8)
                hasdata = int(ptbyte[0], 2)
                numdatabytes = (int(ptbyte[2:6], 2))*4+4
                commandfailed = int(ptbyte[7], 2)
                startaddress = int(binascii.hexlify(self.serial.read(size=1)), 16)
                if hasdata:
                    data = binascii.hexlify(self.serial.read(size=numdatabytes))
                else:
                    data = False
            except ValueError:
                hasdata = 0
                commandfailed = 0
                startaddress = 0
                data = 0
        return [foundpacket, hasdata, startaddress, data, commandfailed]

    def request(self, datatype):
        """Sends data or command request to sensor.  Does not wait for any sort of response

        :param: Same as grab sample
        :return: Nothing
        """
        init = [0x73, 0x6e, 0x70, 0x00]
        address = name2hex_reg[datatype]
        decimalchecksum = 337 + address
        decimalchecksum1, decimalchecksum2 = divmod(decimalchecksum, 0x100)
        init.append(address)
        init.append(decimalchecksum1)
        init.append(decimalchecksum2)
        self.serial.write(init)

    def settimer(self, t=False):
        """Resets internal UM7 class timer

        :param t: If given, sets class timer to t.  If not, all new data is timed relative to instant that settimer()
        is called
        :return:
        """
        if t:
            self.t0 = t
        else:
            self.t0 = time.time()

    def zerogyros(self):
        """Sends request to zero gyros and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        print 'Zeroing ' + self.name + ' gyros...'
        self.serial.write('F,1\n')
        self.request('zerogyros')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('zerogyros')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['zerogyros'] and commandfailed == 0:
                print 'Successfully zeroed gyros.'
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print 'Could not zero gyros.'
        return False

    def resetekf(self):
        """Sends request to reset ekf and waits for confirmation from sensor

        :return: True or False based on success of request
        """
        print 'Resetting ' + self.name + ' EFK...'
        self.serial.write('F,1\n')
        self.request('resetekf')
        timeout = time.time() + 0.5
        while time.time() < timeout:
            self.request('resetekf')
            [foundpacket, hasdata, startaddress, data, commandfailed] = self.readpacket()
            if startaddress == name2hex_reg['resetekf'] and commandfailed == 0:
                print 'Successfully reset EKF.'
                return True
            if self.serial.inWaiting() > 500:
                self.serial.flushInput()
        print 'Could not reset EKF.'
        return False

    def btstart(self):
        self.serial.flushInput()
        buff = 0
        while not buff:
            self.serial.write('F,1\n')
            buff = self.serial.inWaiting()
            print buff
            time.sleep(0.1)

    def updatestate(self, sample):
        sample.update({self.name + ' time': time.time() - self.t0})
        todelete = list(set(sample.keys()).difference(self.state.keys()))
        for i in todelete:
            sample.pop(i)
        mask = {k: v for k, v in self.statemask.items()}
        mask.update(sample)
        self.state.update(mask)

    def checkbuffer(self, numbytes):
        if self.serial.inWaiting() > numbytes:
            print("flush")
            self.serial.flushInput()


def parsedata(data, address, devicename):
    """Function called by class to parse binary data packets

    :param data:
    :param address:
    :param devicename:
    :return:
    """

    datatype = devicename + ' ' + dec2name_reg[address]

    if datatype == 'xgyro' or datatype == 'ygyro' or datatype == 'zgyro':
        data = struct.unpack('!f', data.decode('hex'))[0]
        output = {datatype: data}

    elif datatype == 'xaccel' or datatype == 'yaccel' or datatype == 'zaccel':
        data = struct.unpack('!f', data.decode('hex'))[0]
        output = {datatype: data}

    elif datatype == 'rollpitch':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        for j in range(len(datasplit)):
            datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 91.02222
        output = {'roll': datasplit[0], 'pitch': datasplit[1]}

    elif datatype == 'yaw':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        datasplit[0] = struct.unpack('!h', datasplit[0].decode('hex'))[0] / 91.02222
        output = {datatype: datasplit[0]}

    elif datatype == 'rollpitchrate':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        for j in range(len(datasplit)):
            datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 16.0
        output = {'rollrate': datasplit[0], 'pitchrate': datasplit[1]}

    elif datatype == 'yawrate':
        datasplit = [data[i:i + 4] for i in range(0, len(data), 4)]
        datasplit[0] = struct.unpack('!h', datasplit[0].decode('hex'))[0] / 16.0
        output = {datatype: datasplit[0]}

    else:
        return False

    return output


def parsedatabatch(data, startaddress, devicename):
    xg = devicename + ' xgyro'
    yg = devicename + ' ygyro'
    zg = devicename + ' zgyro'
    xa = devicename + ' xaccel'
    ya = devicename + ' yaccel'
    za = devicename + ' zaccel'
    r = devicename + ' roll'
    p = devicename + ' pitch'
    y = devicename + ' yaw'
    rr = devicename + ' rollrate'
    pr = devicename + ' pitchrate'
    yr = devicename + ' yawrate'
    try:
        if startaddress == 97:  # Processed Gyro Data
            n = 8
            datasplit = [data[i:i + n] for i
                         in range(0, len(data), n)]  # Split data string into array of data bytes (n hex chars each)
            del datasplit[-1]
            for j in range(len(datasplit)):
                datasplit[j] = struct.unpack('!f', datasplit[j].decode('hex'))[0]  # Convert hex string to IEEE 754 floating point
            output = {xg: datasplit[0], yg: datasplit[1], zg: datasplit[2]}
        elif startaddress == 101:  # Processed Accel Data:
            n = 8
            datasplit = [data[i:i + n] for i
                         in range(0, len(data), n)]  # Split data string into array of data bytes (n hex chars each)
            del datasplit[-1]
            for j in range(len(datasplit)):
                datasplit[j] = struct.unpack('!f', datasplit[j].decode('hex'))[0]  # Convert hex string to IEEE 754 floating point
            output = {xa: datasplit[0], ya: datasplit[1], za: datasplit[2]}
        elif startaddress == 112:  # Processed Euler Data:
            n = 4
            datasplit = [data[i:i + n] for i
                         in range(0, len(data), n)]  # Split data string into array of data bytes (n hex chars each)
            del datasplit[9]
            del datasplit[8]
            del datasplit[7]
            del datasplit[3]  # Delete unused data bytes
            for j in range(len(datasplit)):
                if j < len(datasplit) - 3:  # Euler angle bytes
                    datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 91.02222  # Convert hex str to floating point
                    # and convert using constant  # Euler angle rate bytes
                else:
                    datasplit[j] = struct.unpack('!h', datasplit[j].decode('hex'))[0] / 16.0  # Convert hex str to floating
                    # point and convert using constant
            output = {r: datasplit[0], p: datasplit[1], y: datasplit[2], rr: datasplit[3], yr: datasplit[4], pr: datasplit[5]}
        else:
            return False
    except:
        return False
    return output


name2hex_reg = {'health': 0x55,
               'xgyro': 0x61,
               'ygyro': 0x62,
               'zgyro': 0x63,
               'xaccel': 0x65,
               'yaccel': 0x66,
               'zaccel': 0x67,
               'rollpitch': 0x70,
               'yaw': 0x71,
               'rollpitchrate': 0x72,
               'yawrate': 0x73,
                'zerogyros': 0xAD,
                'resetekf': 0xB3}

dec2name_reg = {85: 'health',
                97: 'xgyro',
                98: 'ygyro',
                99: 'zgyro',
                101: 'xaccel',
                102: 'yaccel',
                103: 'zaccel',
                112: 'rollpitch',
                113: 'yaw',
                114: 'rollpitchrate',
                115: 'yawrate'}
