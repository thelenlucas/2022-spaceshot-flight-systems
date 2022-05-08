# -*- coding: utf-8 -*-
"""
Created on Tue May  3 11:28:31 2022

@author: techn
"""
import serial, time, tkinter, threading

class dummySerial(object):
    def __init__(self):
        pass
    def write(self, toWrite):
        pass
    def readline(self):
        pass
    def writelines(self):
        pass

try:
    ser = serial.Serial('COM8', 9600)
except:
    print("caught")
    ser = dummySerial()

#Transmit commands here
def sendString(toSend):
    toWrite = "<"+str(toSend)+">"
    ser.write(toWrite.encode("UTF-8"))
    print("OUT: " + str((toSend)))

#controls if normal operations or simulation mode
mode = "normal"

#This function triggers simulation mode
def StartSimulationMode():
    sendString("CMD,1091,SIM,ENABLE")
    return "transmitting"

def SendSimPressure(pressure):
    sendString("CMD,1091,SIMP," + str(pressure))

def SetTime(toSet):
    sendString("CMD,1091,ST,"+toSet)

def ActivateTelemetry():
    sendString("CMD,1091,CX,ON")

def ResetPackets():
    sendString("RST_PACKET")
    
#main loop
def tick(startTime, lastTime, timeSinceLastTransmit):
    #limiter to check rates
    #always do these
    currTime = time.time_ns()
    deltaTime = currTime-lastTime
    executionTime = currTime - startTime
    lastTime = currTime
    timeSinceLastTransmit += deltaTime;
    
    inln = ser.readline()
    
    if inln is not None:
        inln = str(inln)[2:-5]
        
        inData = inln.split(",")
        print(inData)
    
    #todo graph live data
    #sendString(deltaTime)
    if mode == "normal":
        #normal operation function - parse input data, ect
        pass
    elif mode == "simulation":
        if timeSinceLastTransmit > 1:
            pass 
        #simulation mode - transmit pressure data at 1Hz
        pass
    
    time.sleep(0.1)
    tick(startTime, lastTime, timeSinceLastTransmit)

constThread = threading.Thread(target = tick, args=(time.time_ns(),time.time_ns(),0,))
constThread.start()

root = tkinter.Tk()
root.title("Counting Seconds")
label = tkinter.Label(root, fg="dark green")
label.pack()
button = tkinter.Button(root, text='Reset Packets', width=25, command=ResetPackets)
button.pack()
root.mainloop()