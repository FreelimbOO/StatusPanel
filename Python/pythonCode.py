# -*- coding: utf-8 -*-
import serial
import time
from datetime import datetime
from datetime import timedelta
import wmi # Windows Management INstrumentation
import math

startMarker = '<'
endMarker = '>'
dataStarted = False
dataBuf = ""
messageComplete = False
isInitial = True

def setupSerial(baudRate, serialPortName):
    global  serialPort
    serialPort = serial.Serial(port= serialPortName, baudrate = baudRate, timeout=0, rtscts=True)
    print("Serial port " + serialPortName + " opened  Baudrate " + str(baudRate))
    waitForArduino()

def sendToArduino(stringToSend):
    global startMarker, endMarker, serialPort

    stringWithMarkers = (startMarker)
    stringWithMarkers += stringToSend
    stringWithMarkers += (endMarker)
    
    serialPort.write(stringWithMarkers.encode('utf-8')) # encode needed for Python3

def recvLikeArduino():
    global startMarker, endMarker, serialPort, dataStarted, dataBuf, messageComplete

    if serialPort.inWaiting() > 0 and messageComplete == False:
        x = serialPort.read().decode("utf-8") # decode needed for Python3

        if dataStarted == True:
            if x != endMarker:
                dataBuf = dataBuf + x
            else:
                dataStarted = False
                messageComplete = True
        elif x == startMarker:
            dataBuf = ''
            dataStarted = True
    if (messageComplete == True):
        messageComplete = False
        return dataBuf
    else:
        return "XXX" 

def waitForArduino():
    print("Waiting for Arduino to reset")

    msg = ""
    while msg.find("Arduino is ready") == -1:
        msg = recvLikeArduino()
        if not (msg == 'XXX'): 
            print(msg)

#===== Main program ======#
setupSerial(115200, "COM3") # For Windows
#setupSerial(115200, "/dev/ttyACM0") # For Ubuntu
cpu_temp_index = 0;
cpu_load_indices = [0, 0, 0, 0, 0, 0, 0, 0]
gpu_temp_index = 0;
gpu_load_index = 0;
mem_index = 0;
gddr_index = 0;
w = wmi.WMI(namespace="root\\OpenHardwareMonitor")
sensors = w.Sensor()



def searchIndices():
    global w, sensors, cpu_temp_index, cpu_load_indices, gpu_temp_index, gpu_load_index, mem_index, gddr_index

    print("Checking index...")
    time.sleep(1)
    sensors = w.Sensor()

    dummyIdx = 0;

    for i, sensor in enumerate(sensors):
        if sensor.SensorType==u'Temperature' and 'CPU Package' == sensor.Name:
            cpu_temp_index = i
            print(i, sensor.Name, sensor.SensorType, sensor.Value)
        elif sensor.SensorType==u'Load' and 'CPU Core #' in sensor.Name:
            cpu_load_indices[dummyIdx] = i
            dummyIdx += 1
            print(i, sensor.Name, sensor.SensorType, sensor.Value)
        elif sensor.SensorType==u'Temperature' and 'GPU Core' == sensor.Name:
            gpu_temp_index = i
            print(i, sensor.Name, sensor.SensorType, sensor.Value)
        elif sensor.SensorType==u'Load' and 'GPU Core' == sensor.Name:
        #elif sensor.SensorType==u'Power' and 'GPU Total' == sensor.Name:
            gpu_load_index = i
            print(i, sensor.Name, sensor.SensorType, sensor.Value/27.7)
        elif sensor.SensorType==u'Load' and 'Memory' == sensor.Name:
            mem_index = i
            print(i, sensor.Name, sensor.SensorType, sensor.Value)
        elif sensor.SensorType==u'Load' and 'GPU Memory' == sensor.Name:
        #elif sensor.SensorType==u'Clock' and 'GPU Memory' == sensor.Name:
            gddr_index = i
            print(i, sensor.Name, sensor.SensorType, sensor.Value/1500)


prevDateTime = datetime.now()
deltaThreshold = timedelta( seconds = 20)
diff = timedelta( seconds = 0)
deg = 0;
while True:
    arduinoReq = recvLikeArduino()
    if not (arduinoReq == u'XXX'):
        sensors = w.Sensor()
        currDateTime = datetime.now()
        diff = currDateTime - prevDateTime
        #print("now =", currDateTime, ", diff = ", diff)
        
        #print("isInitial: ", isInitial)
        #print("diff: ", diff)
        if isInitial or diff > deltaThreshold:
            searchIndices()
            isInitial = False

        prevDateTime = datetime.now()
        #print("now =", prevDateTime)

        try:
            cpu_temp = min(99.9, sensors[cpu_temp_index].Value)
            if cpu_temp == 0:
                isInitial = True
                continue
        except IndexError:
            print("cpu_temp_index out of range.")
            isInitial = True
            continue
            
        try:
            cpu_load = 0
            for i in cpu_load_indices:
                cpu_load += sensors[i].Value
            cpu_load /= len(cpu_load_indices)
            cpu_load = min(99.9, cpu_load)
            if cpu_load == 0:
                isInitial = True
                continue
        except IndexError:
            print("cpu_load_index out of range.")
            isInitial = True
            continue
        
        try:
            gpu_temp = min(99.9, sensors[gpu_temp_index].Value)
            if gpu_temp == 0:
                isInitial = True;
                continue
        except IndexError:
            print("gpu_temp_index out of range.")
            isInitial = True
            continue

        try:
            gpu_load = min(99.9, sensors[gpu_load_index].Value/27.7*100)
        except IndexError:
            print("gpu_load_index out of range.")
            isInitial = True
            continue

        try:
            mem = min(99.9, sensors[mem_index].Value)
        except IndexError:
            print("mem_index out of range.")
            isInitial = True
            continue
        
        try:
            gddr = min(99.9, sensors[gddr_index].Value/1500*100)
        except IndexError:
            print("gddr_index out of range.")
            isInitial = True
            continue

        rad = 6*deg / 180.0 * 3.1415926;
        mem = 20 + 20* math.sin(rad)
        deg += 1

        newString = "{:.1f}".format(cpu_temp)
        returnMsg = newString
        #print(cpu_temp_index, "CPU Temp: " + newString)

        newString = ",{:.1f}".format(cpu_load)
        returnMsg += newString
        #print("Avg CPU Load: " + newString)

        newString = ",{:.1f}".format(gpu_temp)
        returnMsg += newString
        #print("GPU Temp: " + newString)

        newString = ",{:.1f}".format(gpu_load)
        returnMsg += newString
        #print("GPU Load: " + newString)

        newString = ",{:.1f}".format(mem)
        returnMsg += newString
        #print("Generic Memory: " + newString)

        newString = ",{:.1f}".format(gddr)
        returnMsg += newString
        #print("GPU Memory: " + newString)

        sendToArduino(returnMsg)
