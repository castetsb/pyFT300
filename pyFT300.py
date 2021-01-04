#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2021-01-04

This is a simple example showing of to get measurement data of FT300 using python.

Hardward preparation:
---------------------
The ft300 have to be connected to the PC via USB and power with a 24V power supply.

Dependencies:
*************
MinimalModbus: https://pypi.org/project/MinimalModbus/

@author: Benoit CASTETS
"""
#Libraries importation
import minimalmodbus as mm
import time
from math import *
import serial

######################
#Connection parameters
######################

#Communication setup
BAUDRATE=19200
BYTESIZE=8
PARITY="N"
STOPBITS=1
TIMEOUT=0.2

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

SLAVEADDRESS=9

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

####################
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

#Uncomment to see binary messages for debug
#ft300.debug=True
#ft300.mode=mm.MODE_RTU

####################
#Functions
####################

def forceConverter(forceRegisterValue):
  """Return the force corresponding to force register value.
  
  input:
    forceRegisterValue: Value of the force register
    
  output:
    force: force corresponding to force register value in N
  """
  force=0

  forceRegisterBin=bin(forceRegisterValue)[2:]
  forceRegisterBin="0"*(16-len(forceRegisterBin))+forceRegisterBin
  if forceRegisterBin[0]=="1":
    #negative force
    force=-1*(int("0111111111111111",2)-int(forceRegisterBin[1:],2))/100
  else:
    #positive force
    force=int(forceRegisterBin,2)/100
  return force

def torqueConverter(torqueRegisterValue):
  """Return the torque corresponding to torque register value.
  
  input:
    torqueRegisterValue: Value of the torque register
    
  output:
    torque: torque corresponding to force register value in N.m
  """
  torque=0

  torqueRegisterBin=bin(torqueRegisterValue)[2:]
  torqueRegisterBin="0"*(16-len(torqueRegisterBin))+torqueRegisterBin
  if torqueRegisterBin[0]=="1":
    #negative force
    torque=-1*(int("0111111111111111",2)-int(torqueRegisterBin[1:],2))/1000
  else:
    #positive force
    torque=int(torqueRegisterBin,2)/1000
  return torque


####################
#Main program
####################

if __name__ == '__main__':
#Get FT300 force and torque
  try:
    #Initialisation
  
    #Read registers where are saved force and torque values.
    registers=ft300.read_registers(180,6)

    #Save measured values at rest. Those values are use to make the zero of the sensor.
    fxZero=forceConverter(registers[0])
    fyZero=forceConverter(registers[1])
    fzZero=forceConverter(registers[2])
    txZero=torqueConverter(registers[3])
    tyZero=torqueConverter(registers[4])
    tzZero=torqueConverter(registers[5])
    
    #main loop
    while True:
      #Read registers where are saved force and torque values.
      registers=ft300.read_registers(180,6)
      
      #Calculate measured value form register values
      fx=round(forceConverter(registers[0])-fxZero,0)
      fy=round(forceConverter(registers[1])-fyZero,0)
      fz=round(forceConverter(registers[2])-fzZero,0)
      tx=round(torqueConverter(registers[3])-txZero,2)
      ty=round(torqueConverter(registers[4])-tyZero,2)
      tz=round(torqueConverter(registers[5])-tzZero,2)
      
      #Display result
      print("***Press Ctrl+C to stop the program***")
      print("fx=",fx,"N")
      print("fy=",fy,"N")
      print("fz=",fz,"N")
      print("tx=",tx,"N.m")
      print("ty=",ty,"N.m")
      print("tz=",tz,"N.m")
      
      time.sleep(1)
      
  except KeyboardInterrupt:
    print("Program ended")
    pass