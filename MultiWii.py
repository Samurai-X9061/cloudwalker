#!/usr/bin/env python

"""multiwii.py: Handles Multiwii Serial Protocol."""

__author__ = "Aldo Vargas and Samurai-X"
__copyright__ = "Copyright 2017 Altax.net"

__license__ = "GPL"
__version__ = "1.6"


import serial, time, struct


class MultiWii:

    """Multiwii Serial Protocol message ID"""
    """ notice: just attitude, rc channels and raw imu, set raw rc are implemented at the moment """
    IDENT = 100
    STATUS = MODE = BOX = 101
    RAW_IMU = 102
    SERVO = 103
    MOTOR = 104
    RC = 105
    RAW_GPS = 106
    COMP_GPS = 107
    ATTITUDE = 108
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    MISC = 114
    MOTOR_PINS = 115
    BOXNAMES = 116
    PIDNAMES = 117
    WP = 118
    BOXIDS = 119
    RC_RAW_IMU = 121
    SET_RAW_RC = 200
    SET_RAW_GPS = 201
    SET_PID = 202
    SET_BOX = 203
    SET_RC_TUNING = 204
    ACC_CALIBRATION = 205
    MAG_CALIBRATION = 206
    SET_MISC = 207
    RESET_CONF = 208
    SET_WP = 209
    SWITCH_RC_SERIAL = 210
    IS_SERIAL = 211
    DEBUG = 254
    VTX_CONFIG = 88
    VTX_SET_CONFIG = 89
    EEPROM_WRITE = 250
    REBOOT = 68


    """Class initialization"""
    def __init__(self, serPort):

        """Global variables of data"""
        self.PIDcoef = {'rp':0,'ri':0,'rd':0,'pp':0,'pi':0,'pd':0,'yp':0,'yi':0,'yd':0}
        self.rcChannels = {'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.rawIMU = {'ax':0,'ay':0,'az':0,'gx':0,'gy':0,'gz':0,'mx':0,'my':0,'mz':0,'elapsed':0,'timestamp':0}
        self.motor = {'m1':0,'m2':0,'m3':0,'m4':0,'elapsed':0,'timestamp':0}
        self.attitude = {'angx':0,'angy':0,'heading':0,'elapsed':0,'timestamp':0}
        self.altitude = {'estalt':0,'vario':0,'elapsed':0,'timestamp':0}
        self.message = {'angx':0,'angy':0,'heading':0,'roll':0,'pitch':0,'yaw':0,'throttle':0,'elapsed':0,'timestamp':0}
        self.vtxConfig = {'device':0, 'band':0, 'channel':0, 'power':0, 'pit':0, 'unknown':0}
        self.analog = {'voltage': 0, 'current': 0, 'mahDrawn': 0,'rssi': 0}
        self.modes = {'arm' : False, 'angle' : False}
        self.temp = ();
        self.temp2 = ();
        self.elapsed = 0
        self.PRINT = 1

        self.ser = serial.Serial()
        self.ser.port = serPort
        self.ser.baudrate = 115200
        self.ser.bytesize = serial.EIGHTBITS
        self.ser.parity = serial.PARITY_NONE
        self.ser.stopbits = serial.STOPBITS_ONE
        self.ser.timeout = 1
        self.ser.xonxoff = False
        self.ser.rtscts = False
        self.ser.dsrdtr = False
        self.ser.writeTimeout = 2
        """Time to wait until the board becomes operational"""
        wakeup = 2
        try:
            self.ser.open()
            if self.PRINT:
                print ("Waking up board on "+self.ser.port+"...")
            for i in range(1,wakeup):
                if self.PRINT:
                    print (wakeup-i)
                    time.sleep(1)
                else:
                    time.sleep(1)
        except Exception as error:
            print ("\n\nError opening "+self.ser.port+" port.\n"+str(error)+"\n\n")

    """Function for sending a command to the board"""
    def sendCMD(self, data_length, code, data, data_format):
        checksum = 0
        total_data = ['$'.encode('utf-8'), 'M'.encode('utf-8'), '<'.encode('utf-8'), data_length, code] + data
        for i in struct.pack('<2B' + data_format, *total_data[3:len(total_data)]):
            checksum = checksum ^ i
        total_data.append(checksum)
        try:
            b = None
            b = self.ser.write(struct.pack('<3c2B'+ data_format + 'B', *total_data))
        except Exception as error:
            print ("\n\nError in sendCMD.")
            print ("("+str(error)+")\n\n")
            pass

    """Function for sending a command to the board and receive attitude"""
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def sendCMDreceiveATT(self, code, data, ATT = True):
        checksum = 0
        total_data = ['$'.encode('utf-8'), 'M'.encode('utf-8'), '<'.encode('utf-8'), len(data), code] + data
        for i in struct.pack('<2B%dH' % len(data), *total_data[3:len(total_data)]):
            checksum = checksum ^ i
        total_data.append(checksum)
        try:
            start = time.time()
            b = None
            b = self.ser.write(struct.pack('<3c2B%dHB' % len(data), *total_data))
            return self.getData(MultiWii.ATTITUDE) if ATT else True
        
        except Exception as error:
            print ("\n\nError in sendCMDreceiveATT.")
            print ("("+str(error)+")\n\n")
            pass

    """Function to arm / disarm """
    """
    Modification required on Multiwii firmware to Protocol.cpp in evaluateCommand:

    case MSP_SET_RAW_RC:
      s_struct_w((uint8_t*)&rcSerial,16);
      rcSerialCount = 50; // 1s transition 
      s_struct((uint8_t*)&att,6);
      break;

    """
    def arm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1500,2000]
            self.sendCMDreceiveATT(8,MultiWii.SET_RAW_RC,data, ATT = False)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()

    def disarm(self):
        timer = 0
        start = time.time()
        while timer < 0.5:
            data = [1500,1500,1000,1500,1000]
            self.sendCMDreceiveATT(8,MultiWii.SET_RAW_RC,data, ATT =  False)
            time.sleep(0.05)
            timer = timer + (time.time() - start)
            start =  time.time()
    
    def setPID(self,pd):
        nd=[]
        for i in nd.arange(1,len(pd),2):
            nd.append(pd[i]+pd[i+1]*256)
        data = pd
        print ("PID sending:", data)
        self.sendCMD(30,MultiWii.SET_PID,data)
        self.sendCMD(0,MultiWii.EEPROM_WRITE,[])

    def setVTX(self,band,channel,power):
        band_channel = ((band-1) << 3)|(channel-1)
        t = None
        while t == None :
            t = self.getData(MultiWii.VTX_CONFIG)
        different = (self.vtxConfig['band'] != band) | (self.vtxConfig['channel'] != channel) | (self.vtxConfig['power'] != power)
        data = [band_channel,power,self.vtxConfig['pit']]
        while different :
            self.sendCMD(4,MultiWii.VTX_SET_CONFIG,data, 'H2B')
            time.sleep(1)
            self.sendCMD(0,MultiWii.EEPROM_WRITE,[],'')
            self.ser.close()
            time.sleep(3)
            self.ser.open()
            time.sleep(3)
            t = None
            while t == None :
                t = self.getData(MultiWii.VTX_CONFIG)
            print(t)
            different = (self.vtxConfig['band'] != band) | (self.vtxConfig['channel'] != channel) | (self.vtxConfig['power'] != power)

    """Function to receive a data packet from the board"""
    def getData(self, cmd, short = False):
        while True:
            try:
                start = time.time()
                self.sendCMD(0,cmd,[],'')
                while True:
                    header = self.ser.read().decode('utf-8')
                    if header == '$':
                        header = header+self.ser.read(2).decode('utf-8')
                        break
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                
                self.ser.flushInput()
                self.ser.flushOutput()
                elapsed = time.time() - start
                if cmd == MultiWii.ATTITUDE:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)                
                    self.attitude['angx']=float(temp[0]/10.0)
                    self.attitude['angy']=float(temp[1]/10.0)
                    self.attitude['heading']=float(temp[2])
                    self.attitude['elapsed']=round(elapsed,3)
                    self.attitude['timestamp']="%0.2f" % (time.time(),) 
                    return self.attitude
                elif cmd == MultiWii.ALTITUDE:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)
                    self.altitude['estalt']=float(temp[0])
                    self.altitude['vario']=float(temp[1])
                    self.altitude['elapsed']=round(elapsed,3)
                    self.altitude['timestamp']="%0.2f" % (time.time(),) 
                    return self.altitude
                elif cmd == MultiWii.RC:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)
                    self.rcChannels.clear()
                    self.rcChannels['roll']=temp[0]
                    self.rcChannels['pitch']=temp[1]
                    self.rcChannels['yaw']=temp[2]
                    self.rcChannels['throttle']=temp[3]
                    if not short:
                        self.rcChannels['aux1']=temp[4]
                        self.rcChannels['aux2']=temp[5]
                        self.rcChannels['aux3']=temp[6]
                        self.rcChannels['aux4']=temp[7]
                        self.rcChannels['aux5']=temp[8]
                        self.rcChannels['aux6']=temp[9]
            
                    self.rcChannels['elapsed']=round(elapsed,3)
                    self.rcChannels['timestamp']="%0.2f" % (time.time(),)
                    return self.rcChannels
                elif cmd == MultiWii.RAW_IMU:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)
                    self.rawIMU['ax']=float(temp[0])
                    self.rawIMU['ay']=float(temp[1])
                    self.rawIMU['az']=float(temp[2])
                    self.rawIMU['gx']=float(temp[3])
                    self.rawIMU['gy']=float(temp[4])
                    self.rawIMU['gz']=float(temp[5])
                    self.rawIMU['mx']=float(temp[6])
                    self.rawIMU['my']=float(temp[7])
                    self.rawIMU['mz']=float(temp[8])
                    self.rawIMU['elapsed']=round(elapsed,3)
                    self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                    return self.rawIMU
                elif cmd == MultiWii.MOTOR:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)
                    self.motor['m1']=float(temp[0])
                    self.motor['m2']=float(temp[1])
                    self.motor['m3']=float(temp[2])
                    self.motor['m4']=float(temp[3])
                    self.motor['elapsed']="%0.3f" % (elapsed,)
                    self.motor['timestamp']="%0.2f" % (time.time(),)
                    return self.motor
                elif cmd == MultiWii.PID:
                    temp = struct.unpack('<'+'h'*int(datalength/2),data)
                    dataPID=[]
                    if len(temp)>1:
                        d=0
                        for t in temp:
                            dataPID.append(t%256)
                            dataPID.append(t/256)
                        for p in [0,3,6,9]:
                            dataPID[p]=dataPID[p]/10.0
                            dataPID[p+1]=dataPID[p+1]/1000.0
                        self.PIDcoef['rp']= dataPID=[0]
                        self.PIDcoef['ri']= dataPID=[1]
                        self.PIDcoef['rd']= dataPID=[2]
                        self.PIDcoef['pp']= dataPID=[3]
                        self.PIDcoef['pi']= dataPID=[4]
                        self.PIDcoef['pd']= dataPID=[5]
                        self.PIDcoef['yp']= dataPID=[6]
                        self.PIDcoef['yi']= dataPID=[7]
                        self.PIDcoef['yd']= dataPID=[8]
                    return self.PIDcoef
                elif cmd == MultiWii.VTX_CONFIG:
                    if datalength > 1:
                        temp = struct.unpack('<bbbbb',data)
                        self.vtxConfig['device'] = temp[0]
                        self.vtxConfig['band'] = temp[1]
                        self.vtxConfig['channel'] = temp[2]
                        self.vtxConfig['power'] = temp[3]
                        self.vtxConfig['pit'] = temp[4]
                        self.vtxConfig['unknown'] = 0
                        return self.vtxConfig
                    else:
                        temp = struct.unpack('<b',data)
                        self.vtxConfig['unknown'] = temp[0]
                        return self.vtxConfig
                elif cmd == MultiWii.ANALOG:
                    if datalength > 1:
                        print(len(data))
                        temp = struct.unpack('<BHHhH', data)
                        print(data)
                        self.analog['voltage'] = temp[4]/100
                        self.analog['mahDrawn'] = temp[1]
                        self.analog['current'] = temp[3]/100
                        self.analog['rssi'] = temp[2]*100/1023
                        return self.analog
                elif cmd == MultiWii.MODE:
                    if datalength > 1:
                        payload = data
                        #idk how this works but see the difference in raw payloads to find what byte determines what

                        cycle_time = int.from_bytes(payload[0:2], byteorder='little')  # us
                        i2c_errors = int.from_bytes(payload[2:4], byteorder='little')
                        sensors = int.from_bytes(payload[4:6], byteorder='little')
                        flight_mode_flags = int.from_bytes(payload[6:10], byteorder='little')
                        arming_flags = int.from_bytes(payload[19:23], byteorder='little')
                        cpu_usage = int.from_bytes(payload[11:13], byteorder='little')//10
                        
                        # Flight modes in Betaflight 4.5
                        sensors_bitmask = {
                            0: "Accelerometer",
                            1: "Barometer",
                            2: "Magnetometer",
                            3: "GPS",
                            4: "Rangefinder",
                            5: "Gyroscope",
                            6: "Optical Flow",
                        }

                        flight_modes = {
                                0: "ARM",
                                1: "ANGLE",
                                2: "HORIZON",
                                17: "BEEPER",
                                12: "AIRMODE",
                            }
                    
                    # Updated arming failure flags in Betaflight 4.5
                        arming_failure_flags = {
                            0: "NO_GYRO",                  # No gyro detected
                            1: "FAILSAFE",                 # Failsafe is active
                            2: "RX_FAILSAFE",              # Receiver failsafe is active  
                            3: "BAD_RX_RECOVERY",          # Recovering from bad RX connection
                            4: "BOXFAILSAFE",              # Failsafe switch is active
                            5: "THROTTLE",                 # Throttle not low enough
                            6: "ANGLE",                    # Aircraft angle too high
                            7: "BOOT_GRACE_TIME",          # Still in boot grace time
                            8: "NOPREARM",                 # Pre-arm switch not activated
                            9: "LOAD",                     # System load too high
                            10: "CALIBRATING",             # Calibration in progress
                            11: "CLI",                     # CLI is active
                            12: "CMS_MENU",                # CMS menu is active
                            13: "BST",                     # BlackBox is active
                            14: "MSP",                     # MSP connection is active
                            15: "PARALYZE",                # Aircraft is paralyzed
                            16: "GPS",                     # Waiting for GPS
                            17: "RESCUE_SW",               # Rescue switch is on
                            18: "RESC_HW",                 # Hardware rescue is active
                            19: "RESC_SW_ENDSTOP",         # Software rescue endstop reached
                            20: "RESC_HW_ENDSTOP",         # Hardware rescue endstop reached
                            21: "DSHOT_BITBANG",           # DSHOT bitbang not available
                            22: "ACC_CALIBRATION",         # Accelerometer calibration needed
                            23: "MOTOR_PROTOCOL",          # Motor protocol not configured
                            24: "ARM_SWITCH",              # Arm switch in wrong position
                        }
                        # Determine active flight modes
                        active_modes = []
                        for bit, mode in flight_modes.items():
                            if flight_mode_flags & (1 << bit):
                                active_modes.append(mode)
                        
                        # Determine arming failure reasons
                        arm_fail_reasons = []
                        for bit, failure in arming_failure_flags.items():
                            if arming_flags & (1 << bit):
                                arm_fail_reasons.append(failure)
                        
                        return {
                            "cycle_time_us": cycle_time,
                            "i2c_errors": i2c_errors,
                            #"sensors": sensors,
                            "flight_mode_flags": bin(flight_mode_flags),
                            "active_modes": active_modes,
                            "arming_flags": arming_flags,
                            "arm_fail_reasons": arm_fail_reasons,}
                else:
                    return "No return error!"
            except Exception as error:
                print (error)
                time.sleep(0.001)
                continue
            

    """Function to receive a data packet from the board. Note: easier to use on threads"""
    def getDataInf(self, cmd):
        while True:
            try:
                start = time.clock()
                self.sendCMD(0,cmd,[])
                while True:
                    header = self.ser.read().decode('utf-8')
                    if header == '$':
                        header = header+self.ser.read(2).decode('utf-8')
                        break
                datalength = struct.unpack('<b', self.ser.read())[0]
                code = struct.unpack('<b', self.ser.read())
                data = self.ser.read(datalength)
                temp = struct.unpack('<'+'h'*int(datalength/2),data)
                elapsed = time.clock() - start
                self.ser.flushInput()
                self.ser.flushOutput()
                if cmd == MultiWii.ATTITUDE:
                    self.attitude['angx']=float(temp[0]/10.0)
                    self.attitude['angy']=float(temp[1]/10.0)
                    self.attitude['heading']=float(temp[2])
                    self.attitude['elapsed']="%0.3f" % (elapsed,)
                    self.attitude['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RC:
                    self.rcChannels['roll']=temp[0]
                    self.rcChannels['pitch']=temp[1]
                    self.rcChannels['yaw']=temp[2]
                    self.rcChannels['throttle']=temp[3]
                    self.rcChannels['elapsed']="%0.3f" % (elapsed,)
                    self.rcChannels['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.RAW_IMU:
                    self.rawIMU['ax']=float(temp[0])
                    self.rawIMU['ay']=float(temp[1])
                    self.rawIMU['az']=float(temp[2])
                    self.rawIMU['gx']=float(temp[3])
                    self.rawIMU['gy']=float(temp[4])
                    self.rawIMU['gz']=float(temp[5])
                    self.rawIMU['elapsed']="%0.3f" % (elapsed,)
                    self.rawIMU['timestamp']="%0.2f" % (time.time(),)
                elif cmd == MultiWii.MOTOR:
                    self.motor['m1']=float(temp[0])
                    self.motor['m2']=float(temp[1])
                    self.motor['m3']=float(temp[2])
                    self.motor['m4']=float(temp[3])
                    self.motor['elapsed']="%0.3f" % (elapsed,)
                    self.motor['timestamp']="%0.2f" % (time.time(),)
            except Exception as error:
                print(error)
                pass

    """Function to ask for 2 fixed cmds, attitude and rc channels, and receive them. Note: is a bit slower than others"""
    def getData2cmd(self, cmd):
        try:
            start = time.time()
            self.sendCMD(0,self.ATTITUDE,[])
            while True:
                header = self.ser.read().decode('utf-8')
                if header == '$':
                    header = header+self.ser.read(2).decode('utf-8')
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp = struct.unpack('<'+'h'*int(datalength/2),data)
            self.ser.flushInput()
            self.ser.flushOutput()

            self.sendCMD(0,self.RC,[])
            while True:
                header = self.ser.read().decode('utf-8')
                if header == '$':
                    header = header+self.ser.read(2).decode('utf-8')
                    break
            datalength = struct.unpack('<b', self.ser.read())[0]
            code = struct.unpack('<b', self.ser.read())
            data = self.ser.read(datalength)
            temp2 = struct.unpack('<'+'h'*int(datalength/2),data)
            elapsed = time.time() - start
            self.ser.flushInput()
            self.ser.flushOutput()

            if cmd == MultiWii.ATTITUDE:
                self.message['angx']=float(temp[0]/10.0)
                self.message['angy']=float(temp[1]/10.0)
                self.message['heading']=float(temp[2])
                self.message['roll']=temp2[0]
                self.message['pitch']=temp2[1]
                self.message['yaw']=temp2[2]
                self.message['throttle']=temp2[3]
                self.message['elapsed']=round(elapsed,3)
                self.message['timestamp']="%0.2f" % (time.time(),) 
                return self.message
            else:
                return "No return error!"
        except Exception as error:
            print (error)

