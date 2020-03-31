#  Date        : 03/31/2020
#  Author      : mcalyer
#  Module      : scanse_control.py
#  Description : Code to explore Scanse LIDAR capabilites
#  Python      : 2.7
#  Version     : 0.2
#  References  : 
# 
#  1. Scanse User Manual V1.0 , 04/20/2017
#  2. Scanse sweep-ardunio source   
#  3. sweep-sdk-1.30_10_27_2017          
#  
#  Hardware : PC , Scanse Hardware version 1.0 , FW version 01 , 2016 KickStarter
#             Not available today
#
#  Notes: 
#  0. Really ?! , Unit appears to wooble during spinning 
#  1. 
#  2. Power : 5V at 450 - 500 ma
#  3. Motor Speed : if setting is '0''0' motor off but when power cycled resets to 5HZ
#  4. Embedded use : power control scanse : control motor , power usage , fail safe device reset
#  5. There is python using driver example for Linux , see sweepy in SDK
#  6. Need to look at driver source 
#  7. Scanse Status LED :
#     Blinking Green = Start up OK , no ouput
#     Solid Blue     = Normal operation
#     Solid Red      = Internal communication error 
#  8. Example Scan Seetings :
#      Speed             : 5HZ 
#      Sample rate       : 500 - 600 HZ 
#      Time required     : .2 sec (approx) 
#      Number of samples : 60 (approx) for 1 rev (360) ? , see angle problem
#      Angle Delta       : Generally 3.XX degrees (approx) 
#                          Angle problem : See in 0 - 120 degreee range , large (10 degres) angle deltas                        
#      Revolution        : 1 rev , 360 degrees 
#      Zero Angle        : One near zero reading in samples 
#  9.  Angular resolution : 1.4 - 7.2 degrees based on rotational speed   (Other factors ?) 
#
#  Acknownledgements : None
#
#  Releases:
#  03/28/2020 : First
#  03/31/2020 : Version 0.2
#    1. Fixed DX stop issue:  Fixed scanse DX command return number of bytes ,
#       added scanse flush routine ,   class Scanse_Control : scanse_flush()
#    2. Added get scan based on number of samples requested , also does not rely on large serial input buffer , 
#       class Scanse_Control : rx_scan_samples().
#       Times observed for 60 samples .150 - .23 seconds @ motor speed = 5HZ , LIDAR sample rate = 500 - 600 HZ
#    3. Added scan data to PGM file , helps visualize point cloud
#
###########################################################################



################################### Imports ###############################


import time
import serial
import sys
from scanse_pgm import *


################################## Scanse Serial Port #########################################  


class Scanse_Control:
    def __init__(self):  
        self.uart = None  
        self.port = None        

    def connect(self, port = None):
        if self.uart:
            return 0  
        if port is None : port = self.port        
        # Open serial port connection
        # port is a string based on OS:
        # Examples: Windows 'COM12' , Linux:  '/dev/ttyACM0'
        try:   
            self.uart = serial.Serial(port, baudrate=115200, timeout=1)   
            self.port = port            
            return 0 , None           
        except:           
            self.uart = None  
            self.port = None            
            return 1 , 'Serial port connection error !'        
     
    def disconnect(self):
        if self.uart:            
            self.uart.close()
            self.uart = None       
       
    def tx(self,cmd_list):       
        try:             
            #self.uart.write(''.join(chr(e) for e in cmd_list))    
            self.uart.write(cmd_list)               
            return 0 , None
        except serial.SerialException:         
            return 1 ,'Command: Serial Port Failed'  
            
    def rx(self, n, delay = 0):          
        if delay != 0 : time.sleep(delay)      
        try:           
            nb = self.uart.inWaiting()   
            #print(nb)            
            if nb == 0: return 1 , 'RxBytes: Zero serial bytes'   
            if n == '!': n = nb                    
            if n != nb:  
                self.uart.flush()            
                return 1 , 'RxBytes: Expected : ' + str(n) + ' Received : ' + str(nb)      
            data = self.uart.read(n)                   
            return 0 , data            
        except serial.SerialException:              
            return 1, 'RxBytes: Serial Port Failed'   
            
    def rx_scan(self):
        try:
            nb = self.uart.inWaiting()                  
            data = self.uart.read(nb)    
        except:           
            return None        
        return bytes(data)
        
    def rx_scan_samples(self, nb):
        data = bytes[0]
        b = []      
        t = 0
        try:
            while(nb > 0):
                t = t + 1
                time.sleep(.001) 
                n = self.uart.inWaiting() 
                if n == 0 :                
                    continue                
                b = self.uart.read(n)
                data = data + b
                nb = nb - n                         
        except:               
            return 1 , t , 'rx_scan_sample error'  
        return 0 , t, data    

    def scanse_flush(self):
        nb = self.uart.inWaiting() 
        t = 1000        
        while(nb != 0): 
            d  = self.uart.read(nb)                       
            time.sleep(.001)
            nb = self.uart.inWaiting()     
            t = t - 1
            if t == 0:
                break;                        
        return t  

        
    def flush(self):        
        self.uart.flush()              
            
scanse_ctrl =  Scanse_Control()


################################## Scanse Interface #########################################  

class Scanse_IF():
    def __init__ (self, IF , cmd , rx_bytes , decode = None):
        self.IF     = IF
        self.cmd    = cmd #['I', 'V'] + ['\n']
        self.rx_nb  = rx_bytes
        self.data   = None
        self._decode = decode  
        self.delay = .050          
        
    def txrx(self, arg = None):
        if arg is not None : self.cmd = self.cmd + arg
        self.IF.tx(self.cmd + ['\n'])
        if 0 == self.rx_nb : return 0, None
        time.sleep(self.delay)       
        result, self.data = self.IF.rx(self.rx_nb)
        if result : return 1, self.data
        if self.data[0] != self.cmd[0] or self.data[1] != self.cmd[1] : return 1, None       
        return 0, self.data
        
    def decode(self):
        if self._decode is None : return self.data
        return self._decode(self.data)

# IV Decode Model , Protocol , FWV , HWV , Serial Number
iv_decode =  lambda x : (x[2:7] , x[7:9][::-1] , x[9:11][::-1] , x[11] , x[12:20]) 
scanse_iv = Scanse_IF(scanse_ctrl,['I' , 'V'] , 21 , iv_decode )

# Set Motor_Speed
# speed 0 - 10 hz , ['0','0'] - ['1','0']
scanse_ms = Scanse_IF(scanse_ctrl,['M' , 'S'] , 9)

# Motor Info
mi_decode =  lambda x : (x[2:4]) 
scanse_mi = Scanse_IF(scanse_ctrl,['M' , 'I'] , 5 , mi_decode)

# Motor Ready
mz_decode =  lambda x : (x[2:4]) 
scanse_mz = Scanse_IF(scanse_ctrl,['M' , 'Z'] , 5 , mz_decode)

# Device Information
di_decode =  lambda x : (x[2:8] , x[8] , x[9] , x[10] , x[11:13] , x[13:17]) 
scanse_di =  Scanse_IF(scanse_ctrl,['I' , 'D'] , 18 , di_decode)

# LIDAR Get Sample Rate
lidar_decode =  lambda x : (x[2:4]) 
scanse_lidar_get_sr = Scanse_IF(scanse_ctrl,['L' , 'I'] , 5 , lidar_decode)

# LIDAR , Set Sample Rate
# ['0','1'] = 500  - 600 HZ
# ['0','2'] = 750  - 800 HZ
# ['0','3'] = 1000 - 1075 HZ
lidar_sr_decode =  lambda x : (x[5:7]) 
scanse_lidar_set_sr = Scanse_IF(scanse_ctrl,['L' , 'R'] , 9 , lidar_sr_decode)

# Reset Device
scanse_reset = Scanse_IF(scanse_ctrl,['R' , 'R'] , 0)

# Stop Data Aquisition
scanse_stop_data = Scanse_IF(scanse_ctrl,['D' , 'X'] , 6)

# Start Data Aquisition
scanse_start_data = Scanse_IF(scanse_ctrl,['D' , 'S'] , 7)


############################## Data Acquisition ############################################# 

def measurement(s):   
    d     = (ord(s[4]) << 8) + ord(s[3])
    a_int = (ord(s[2]) << 8) + ord(s[1])          
    return [d, a_int/16.0]
    

def get_scan(delay): 
    scanse_ctrl.flush()    
    # Send DS Command , start acquisition
    scanse_ctrl.tx(['D' , 'S']  + ['\n'])
    # Wait for data   
    time.sleep(delay)
    # Get data
    scan = scanse_ctrl.rx_scan()  
    if scan is None or len(scan) < 2 : return 1,0,0, 'No Scan Data'    
    # Check header bytes
    if scan[0] != 'D' or scan[1] != 'S' : return 1, 0, 0, 'No Scan DS header'
    # Create List of samples
    scan_data = []
    l = len(scan) 
    ns = ((l - 6)/7) - 1  
    s = scan[6:(l - 6)]  
    x = 0
    z = None
    n = ns
    for i in range(0,n):
        x = i * 7    
        q = s[x:x+7] 
        w = ord(q[0]) 
        if w & 0x01 : z = i 
        if w & 0xFE : return 1, i, w, 'Scan Packet Error' 
        da = measurement(q)
        # Filter out measurements with d == 1 , error
        if da[0] == 1: 
            ns = ns - 1
            continue
        scan_data.append(da) 
        
    # Send DX Command , stop acquisition    
    scanse_stop_data.txrx()  
    # Fluah scanse uart
    scanse_ctrl.scanse_flush()  
    return 0, ns, z, scan_data
 
############################### Test ######################################################## 

def main(sys_argv):
    
    if len(sys_argv) < 2: print("More Args Please !") ; exit(0) 
    port = sys_argv[1] 
    
    # Scanse Connect   
    result , message = scanse_ctrl.connect(port)
    if result: print message ; exit(0) 
    print "\n"  
     
    # Scanse Flush
    scanse_ctrl.scanse_flush()    
        
    # Get Version Information   
    scanse_ctrl.flush()
    result , info = scanse_iv.txrx()    
    print(info if result else 'Version :' + str(scanse_iv.decode()))   
    
    #Get Device Information
    scanse_ctrl.flush()
    result , info = scanse_di.txrx() 
    print(info if result else 'Device Info : ' + str(scanse_di.decode()))     

    # Set LIDAR sample rate
    # Lower sample rate , more light , range measurements more accurate
    result , status = scanse_lidar_set_sr.txrx(['0','1'])  
    print(status if result else 'LIDAR Set Sample Rates Status : ' + str(scanse_lidar_set_sr.decode())) 

    # Get Motor Speed 
    result, motor_speed = scanse_mi.txrx()  
    ms = scanse_mi.decode()    
    print(motor_speed if result else 'Motor Speed : ' + str(ms))          
        
    #Get LIDAR Info   
    result , info = scanse_lidar_get_sr.txrx()
    print(info if result else 'LIDAR Sample Rate : ' + str(scanse_lidar_get_sr.decode()))    
    
    # Get 10 Scans
    data = []
    for i in range(0,10):
        r, n, z , data = get_scan(.225)
        if r : print(data) ; break
        if data != []:
            print('Samples : ' + str(n) + '  Zero Index : ' + str(z))
            for i in range(0,n):
                print(i,data[i])       
        print('\n')   
        
        # Scan sorted by distance 
        ds  = sorted(data,key = lambda data: data[0])
        # Scan sorted by angle
        ans = sorted(data,key = lambda data: data[1])
        print('Distance Min :' + str(ds[0]))
        print('Angle    Min :' + str(ans[0]))
        print('\n')     
    
        # PGM File          
        try:          
            scan_2_pgm(ds, int(ds[::-1][0][0]))    
        except:
            pass 
        
        
    # Exit    
    scanse_ctrl.disconnect() 
    exit(0)   
    

if __name__ == "__main__":
     # one argument COM port ,  Example:  Windows 'COM12' , Linux:  '/dev/ttyACM0'
     main(sys.argv)  
