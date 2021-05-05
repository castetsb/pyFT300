#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2021-01-04

This is a simple example showing of to get measurement data of FT300 in stream mode using python.

Hardward preparation:
---------------------
The ft300 have to be connected to the PC via USB and power with a 24V power supply.

Dependencies:
*************
MinimalModbus: https://pypi.org/project/MinimalModbus/

@author: Benoit CASTETS
"""
######################
#Libraries importation
######################

import time
from math import *
import serial
import minimalmodbus as mm
import io
import libscrc

######################
#Functions
######################

def forceFromSerialMessage(serialMessage,zeroRef=[0,0,0,0,0,0]):
  """Return a list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] correcponding to the dataArray
  
  Parameters
  ----------
  serialMessage:
    bytearray which contents the serial message send by the FT300. 
    [0x20,0x4e,LSBdata1,MSBdata2,...,LSBdata6,MSBdata6,crc1,crc2]
    Check FT300 manual for details.
  zeroRef:
    list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] use the set the zero reference of the sensor.
  
  Return
  ------
  forceTorque:
    list with force and torque values [Fx,Fy,Fz,Tx,Ty,Tz] correcponding to the dataArray
  """
  #Initialize variable
  forceTorque=[0,0,0,0,0,0]
  
  #converte bytearray values to integer. Apply the zero offset and round at 2 decimals
  forceTorque[0]=round(int.from_bytes(serialMessage[2:4], byteorder='little', signed=True)/100-zeroRef[0],2)
  forceTorque[1]=round(int.from_bytes(serialMessage[4:6], byteorder='little', signed=True)/100-zeroRef[1],2)
  forceTorque[2]=round(int.from_bytes(serialMessage[6:8], byteorder='little', signed=True)/100-zeroRef[2],2)
  forceTorque[3]=round(int.from_bytes(serialMessage[8:10], byteorder='little', signed=True)/1000-zeroRef[3],2)
  forceTorque[4]=round(int.from_bytes(serialMessage[10:12], byteorder='little', signed=True)/1000-zeroRef[4],2)
  forceTorque[5]=round(int.from_bytes(serialMessage[12:14], byteorder='little', signed=True)/1000-zeroRef[5],2)
  
  return forceTorque

def crcCheck(serialMessage):
  """Check if the serial message have a valid CRC.
  
  Parameters
  -----------
  serialMessage:
    bytearray which contents the serial message send by the FT300. 
    [0x20,0x4e,LSBdata1,MSBdata2,...,LSBdata6,MSBdata6,crc1,crc2]
    Check FT300 manual for details.
    
  Return
  ------
  checkResult:
    bool, return True if the message have a valid CRC and False if not.
  """
  checkResult = False
  
  #CRC from serial message
  crc = int.from_bytes(serialMessage[14:16], byteorder='little', signed=False)
  #calculated CRC
  crcCalc = libscrc.modbus(serialMessage[0:14])
  
  if crc == crcCalc:
    checkResult = True
  
  return checkResult


#############################
#Serial connection parameters
#############################

#Communication setup
BAUDRATE=19200
BYTESIZE=8
PARITY="N"
STOPBITS=1
TIMEOUT=1

#Change portname according the port on which is connected the FT300

#For Ubuntu
############
#Name of the port (string) where is connected the gripper. Usually
#/dev/ttyUSB0 on Linux. It is necesary to allow permission to access
#this connection using the bash command sudo chmod 666 /dev/ttyUSB0

#For windows
############
#Check the name of the port using robotiq user interface. It should be something
#like: COM12

PORTNAME="COM3"

#Set the slave ID of the gripper. By default it is 9.

SLAVEADDRESS=9

#####################################################################################################
#Main program
#####################################################################################################

if __name__ == '__main__':
  
  try:
    ############################
    #Desactivate streaming mode
    ############################


    #To stop the data stream, communication must be interrupted by sending a series of 0xff characters to the Sensor. Sending for about
    #0.5s (50 times)will ensure that the Sensor stops the stream.
    ser=serial.Serial(port=PORTNAME, baudrate=BAUDRATE, bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS, timeout=TIMEOUT)
    packet = bytearray()
    sendCount=0
    while sendCount<50:
      packet.append(0xff)
      sendCount=sendCount+1
    ser.write(packet)
    ser.close()
    del packet
    del sendCount    
    del ser
    

    ############################
    #Activate streaming mode
    ############################
    
    
    #Setup minimalmodbus
    ####################

    #Communication setup
    mm.BAUDRATE=BAUDRATE
    mm.BYTESIZE=BYTESIZE
    mm.PARITY=PARITY
    mm.STOPBITS=STOPBITS
    mm.TIMEOUT=TIMEOUT

    #Create FT300 object
    ft300=mm.Instrument(PORTNAME, slaveaddress=SLAVEADDRESS)
    ft300.close_port_after_each_call=True

    #Uncomment to see binary messages for debug
    #ft300.debug=True
    #ft300.mode=mm.MODE_RTU
    
    #Write 0x0200 in 410 register to start streaming
    ###############################################
    registers=ft300.write_register(410,0x0200)
    
    del ft300
    
    ############################
    #Open serial connection
    ############################
    
    
    ser=serial.Serial(port=PORTNAME, baudrate=BAUDRATE, bytesize=BYTESIZE, parity=PARITY, stopbits=STOPBITS, timeout=TIMEOUT)
    startTime=time.time()
    
    
    ############################
    #Initialize stream reading
    ############################
    
    #Bytes use to itendify the beginning of the serial message
    STARTBYTES = bytes([0x20,0x4e])
    
    #Read serial buffer until founding the bytes [0x20,0x4e]
    #First serial reading.
    #This message in uncomplete in most cases so it is ignored.
    data = ser.read_until(STARTBYTES)
    
    #Second serial reading.
    #This message is use to make the zero of the sensor.
    data = ser.read_until(STARTBYTES)
    #convert from byte to bytearray
    dataArray = bytearray(data)
    #Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
    dataArray = STARTBYTES+dataArray[:-2]
    #Check if the serial message have a valid CRC
    if crcCheck(dataArray) is False:
      raise Exception("CRC ERROR: Serial message and the CRC does not match")
    
    #Save sensor value force and torue value in a variable which will be use as a zero reference
    #The FT300 is suppose to be at rest when starting this program.
    zeroRef = forceFromSerialMessage(dataArray)
    
    
    #Program variables
    ##################
    
    #Number of received messages
    nbrMessages = 0
    #Data rate frequency in Hz
    frequency = 0
    
    while True:
      #Read serial message
      ####################
      
      data = ser.read_until(STARTBYTES)
      #convert from byte to bytearray
      dataArray = bytearray(data)
      #Delete the end bytes [0x20,0x4e] and place it at the beginning of the bytearray
      dataArray = STARTBYTES+dataArray[:-2]
      
      #calulate force and torque form serial message
      #############################################
      forceTorque=forceFromSerialMessage(dataArray,zeroRef)
      
      #CRC validation
      ################
      if crcCheck(dataArray) is False:
        raise Exception("CRC ERROR: Serial message and the CRC does not match")
      
      #Frequency
      ###############
      #Update message counter
      nbrMessages+=1
      #Update timer
      elapsedTime=time.time()-startTime
      #Calculate average frequency
      frequency=round(nbrMessages/elapsedTime)
      
      #Data printing
      ###############
      print("F: ",frequency,"Hz - force Vector : ",forceTorque)
      
  except KeyboardInterrupt:
    print("Program ended")
    ser.close()
    pass