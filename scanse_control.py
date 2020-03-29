#  Date: 03/28/2020
#  Author: mcalyer
#  Description: Code to explore Scanse LIDAR capabilites
#  References: 
#  1. Scanse User Manual V0.9 
#  2. Scanse User Manual V1.0
#  3. Scanse sweep-ardunio source   
#  4. sweep-sdk-1.30_10_27_2017          
#  
#  Hardware : PC , Scanse Hardware version 1.0 , FW version 01
#  Notes: 
#  1. Stop scan Commnad does not seem to work , what works disconnect/reconnect uart
#  2. Can start motor , but need disconnect/reconnect uart to continue
#  3. Can not reliably stop motor
#  4. Embedded use : power control to scanse 
#  5. There is python using driver example for Linux , see sweepy in SDK
#  6. Need to look at driver source 
#   
#  Acknownledgements : None
###########################################################################



################################### Imports ###############################


import time
import serial
import sys
 

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
iv_decode =  lambda x : (x[2:7] , x[7:9] , x[9:11] , x[11] , x[12:20]) 
scanse_iv = Scanse_IF(scanse_ctrl,['I' , 'V'] , 21 , iv_decode )

# Set Motor_Speed
# speed 0 - 10 hz , ['0','0'] - ['1','0']
scanse_ms = Scanse_IF(scanse_ctrl,['M' , 'S'] , 8)

# Motor Info
mi_decode =  lambda x : (x[2:4]) 
scanse_mi = Scanse_IF(scanse_ctrl,['M' , 'I'] , 5 , mi_decode)

# Device Information
di_decode =  lambda x : (x[2:8] , x[8] , x[9] , x[10:12] , x[12:16]) 
scanse_di =  Scanse_IF(scanse_ctrl,['I' , 'D'] , 18 , di_decode)

# LIDAR Get Sample Rate
lidar_decode =  lambda x : (x[2:4]) 
scanse_lidar_get_sr = Scanse_IF(scanse_ctrl,['L' , 'I'] , 5 , lidar_decode)

# LIDAR , Set Sample Rate
# ['0','1'] = 500  - 600 HZ
# ['0','2'] = 750  - 800 HZ
# ['0','3'] = 1000 - 1075 HZ
scanse_lidar_set_sr = Scanse_IF(scanse_ctrl,['L' , 'R'] , 8)

# Reset Device
scanse_reset = Scanse_IF(scanse_ctrl,['R' , 'R'] , 0)

# Stop Data Aquisition
scanse_stop_data = Scanse_IF(scanse_ctrl,['D' , 'X'] , 5)

# Start Data Aquisition
scanse_start_data = Scanse_IF(scanse_ctrl,['D' , 'S'] , 7)


############################## Data Acquisition ############################################# 

def measurement(s):    
    # angular resolution: 1.4 - 7.2 degrees based on rotational speed    
    d     = (ord(s[4]) << 8) + ord(s[3])
    a_int = (ord(s[2]) << 8) + ord(s[1])          
    return [d, a_int/16.0]
    

def get_scan(delay): 
    scanse_ctrl.flush()    
    # Send DS Command , start acquisition
    scanse_ctrl.tx(['D' , 'S']  + ['\n'])
    # Wait for data 
    # Delay time: For 5HZ motor speed  , 1 rev = .2 , resulting in 60 samples 
    # Approxmately 1 msec per sample
    time.sleep(delay)
    # Get data
    scan = scanse_ctrl.rx_scan()  
    if scan is None or len(scan) < 2 : return 1,0,0, 'No Scan Data'    
    # Check header bytes
    if scan[0] != 'D' or scan[1] != 'S' : return 1, 0, 0, 'No Scan DS header'
    # Create List of samples
    scan_data = []
    l = len(scan) 
    n = ((l - 6)/7) - 1  
    s = scan[6:(l - 6)]  
    x = 0
    z = None
    for i in range(0,n):
        x = i * 7    
        q = s[x:x+7] 
        w = ord(q[0]) 
        if w & 0x01 : z = i 
        if w & 0xFE : return 1, i, w, 'Scan Packet Error' 
        scan_data.append(measurement(q)) 
        
    # Send DX Command , stop acquisition , this does not work but leave it here for now
    scanse_ctrl.tx(['D' , 'X']  + ['\n'])  
    return 0, n, z, scan_data
 
############################### Test ######################################################## 
def main(sys_argv):
    
    if len(sys_argv) < 2: print("More Args Please !") ; exit(0) 
    port = sys_argv[1] 
    
    # Scanse Connect   
    result , message = scanse_ctrl.connect(port)
    if result: print message ; exit(0) 
    print "\n"   
    
        
    # Get Version Information   
    scanse_ctrl.flush()
    result , info = scanse_iv.txrx()    
    print(info if result else scanse_iv.decode())    
    
    #Get Device Information
    scanse_ctrl.flush()
    result , info = scanse_di.txrx() 
    print(info if result else scanse_di.decode())       
  
    # Get Motor info 
    result, motor_speed = scanse_mi.txrx()  
    ms = scanse_mi.decode()    
    print(motor_speed if result else ms)      
    
    # Start Motor , set speed = 5 HZ
    if ms == '00':       
        scanse_ms.txrx(['0','5'])
        scanse_ctrl.disconnect() 
        scanse_ctrl.connect()
        time.sleep(10)
        
        
    #Get LIDAR Info   
    result , info = scanse_lidar_get_sr.txrx()
    print(info if result else scanse_lidar_get_sr.decode())     
    
    
    # Get 10 Scans
    for i in range(0,10):
        r, n, z , data = get_scan(.22)
        if r : print(data) ; break
        if data != []:
            print(n,z)
            for i in range(0,n):
                print(i,data[i])       
        
        scanse_ctrl.disconnect() 
        scanse_ctrl.connect()
        print('\n')   
   
        
    # Exit
    scanse_ctrl.disconnect() 
    exit(0)   
    

if __name__ == "__main__":
     # one argument COM port ,  Example:  Windows 'COM12' , Linux:  '/dev/ttyACM0'
     main(sys.argv)  
