#  Date   : 03/31/2020
#  Module : scanse_pgm.py
#  Author : mcalyer
#
#  Description: Code to explore Scanse LIDAR capabilites
#               Create PGM image file of scan data.
#               Scan data format : [  [distance in cm(float) , angle in degrees(float)] , ....... [dn,an] ]
#
#  Version    : 0.1
#
#  References : 
#  1. Scanse User Manual V1.0 04/20/2017
#  
#  Hardware : 
#  1. PC 
#  2. Scanse LIDAR : Hardware version 1.0 , FW version 01 
#     2016 KickStarter , not available today
#
#  Python :  2.7
#
#  Notes : 
#  
#
#  Acknownledgements : 
#  1. Got idea to create pgm file of scan data after looking at simondlevy/BreezySLAM 
#     https://github.com/simondlevy/BreezySLAM/tree/master/python
# 
#  Releases:
#  03/31/2020 : First
#  
###########################################################################


################################### Imports ###############################

import math
import numpy as np
from datetime import datetime


################################## PGM ###############################################

# Degrees to Rad 
pi_180  =  math.pi/180.0   

def scan_2_pgm(scan , d_max , d_default = None): 

    # d_max : max distance (integer value) used to determine image size , offset into map
    # Using the same max distance for multiple images allows for
    # better viewing when flipping thru a number of images
        
    _d = d_max
    if d_default is not None : _d = d_default   
    
    # image size
    h = (_d + 10) * 2
    w = (_d + 10) * 2 
    
    # image array of pixels
    img = np.zeros((h,w))  
    
    # Put Scanse Marker with direction indicator in image array
    w_l = w/2 - 5
    w_r = w/2 + 13  
    h_b = h/2 - 5   
    h_t = h/2 + 6    
    for i in range(w_l,w_r):
        img[h/2,i] = 255
    for i in range(h_b,h_t):
        img[i,w/2] = 255         
    
    # Convert polar to x,y , map scan point cloud point into image array space 
    for p in scan:    
        d = p[0]       
        a = p[1] * pi_180 # Convert degrees to rads
        x = int(d * math.cos(a)) + _d      
        y = int(d * math.sin(a)) + _d   
        img[y,x] = 255
       
    # Create PGM File 
    # Get time string use it to create unique file name   
    now = datetime.now()
    current_time = now.strftime("%H_%M_%S")
    
    output = open("scanse_" + current_time + ".pgm", "w+")  
    # PGM Header File:    
    output.write('P2\n' + str(w) + ' ' + str(h) + '\n' + '255' + '\n') 

    # Convert img array to rows of ASCII strings representing image pixel array
    # Image : Black background with scan point cloud points as white dots
    # Scanse location and orientation indicated by white cross
    # Pixel :  '0' = black , '255' = white 
    for i in range(h):
        ls = ''
        for j in range(w): 
            if img[j,i] == 0: 
                ls = ls + '0' + ' '
            else: 
                ls = ls + '255' + ' '                    
        ls = ls[:-1]  # Remove last space   
        output.write(ls)
        output.write('\n')  
        
    output.close()  
   

