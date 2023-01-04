# Ultrasonic Multi-threaded example
# Reads sensor with some failsafes against faulty readings.
# 
#
# Author: Net-time, 2019
#
"""
<plugin key="Ultrasonic" name="Ultrasonic GPIO sensor" author="Nettime" version="1.0.1" >
    <description>
        <h2>Ultrasonic</h2><br/>
        Reads sensor with some failsafes against faulty readings.<br/>
    </description>
    <params>
        <param field="Mode1" label="TX GPIO pin (BCM) " width="50px"  default="18" required="true"/>
        <param field="Mode2" label="RX GPIO pin (BCM) " width="50px"  default="24" required="true"/>
        <param field="Mode3" label="Min true value " width="50px" default="10" required="true"/>
        <param field="Mode4" label="Max true value " width="50px"   default="70" required="true"/>
        <param field="Mode5" label="Max span between 3 readings " width="50px"  default="10" required="true"/>
        <param field="Mode6" label="Debug" width="150px">
            <options>
                <option label="Normal" value="0"  default="Normal" />
                <option label="All" value="1"/>
            </options>
        </param>
    </params>
</plugin>
"""
import Domoticz
import sys,os
import threading
import RPi.GPIO as GPIO
import time
from collections import deque

sensorHistory = deque([0.0]*3)
ticks = 0
#Variables for GPIO Pins
GPIO_TRIGGER = 0
GPIO_ECHO = 0
SENSOR_MIN = 0
SENSOR_MAX = 0
SENSOR_SPAN = 0
image = 0

class BasePlugin:
    terminate = False

    def handleSensor(self):
        global sensorHistory,GPIO_TRIGGER, GPIO_ECHO
        try:
            Domoticz.Log("Entering sensor handler")
            while True:
                GPIO.output(GPIO_TRIGGER, True)
                # set Trigger after 0.01ms to LOW
                time.sleep(0.00001)
                GPIO.output(GPIO_TRIGGER, False) 
                StartTime = time.time()
                StopTime = time.time() 
                # save StartTime
                while GPIO.input(GPIO_ECHO) == 0:
                    StartTime = time.time() 
                # save time of arrival
                while GPIO.input(GPIO_ECHO) == 1:
                    StopTime = time.time() 
                # time difference between start and arrival
                TimeElapsed = StopTime - StartTime
                # multiply with the sonic speed (34300 cm/s)
                # and divide by 2, because there and back
                distance = round(((TimeElapsed * 34300) / 4),1)
                # store last 3 values for comparison against faulty readings-
                sensorHistory.rotate() ; sensorHistory[0]= distance
                #Domoticz.Log(str(sensorHistory))
                for x in range(10): # Delay to spare the sensor and cpu load.
                    #Domoticz.Log(str(x))
                    time.sleep(1)
                    #Message = self.messageQueue.get()
                    if self.terminate == True:
                        GPIO.cleanup()
                        Domoticz.Log("Exiting sensor handler")
                        break
        except Exception as err:
            Domoticz.Log("handleSensor: "+str(err))
    
    def onStart(self):
        global GPIO_TRIGGER, GPIO_ECHO, SENSOR_MIN, SENSOR_MAX, SENSOR_SPAN, image
        GPIO_TRIGGER = int(Parameters["Mode1"])
        GPIO_ECHO = int(Parameters["Mode2"])
        SENSOR_MIN = int(Parameters["Mode3"])
        SENSOR_MAX = int(Parameters["Mode4"])
        SENSOR_SPAN = int(Parameters["Mode5"])
        VerBose("GPIO_TRIGGER/ECHO: "+ str(GPIO_TRIGGER)+"/"+str(GPIO_ECHO))
        try:
            if "UltrasonicSG" not in Images:
                Domoticz.Image("UltrasonicSG.zip").Create()
                image = Images["UltrasonicSG"].ID
                Domoticz.Log("Image created. ID: "+str(image))
            else:
                image = Images["UltrasonicSG"].ID
        except:
            image = 0
        if len(Devices)==0:
            Domoticz.Device("UltraSonic", Unit=1, Type= 243, Subtype=31, Image=image, Options={"Custom": "1;cm"}).Create()
            Domoticz.Log("Created device: ")
        #GPIO Mode (BOARD / BCM)
        GPIO.setmode(GPIO.BCM)
        #set GPIO direction (IN / OUT)
        GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
        GPIO.setup(int(GPIO_ECHO), GPIO.IN)
        self.messageThread = threading.Thread(name="QueueThread", target=BasePlugin.handleSensor, args=(self,))
        self.messageThread.start()
        


    def onHeartbeat(self):
        global ticks, sensorHistory, SENSOR_MIN, SENSOR_MAX, SENSOR_SPAN, image
        if ticks>6:
            low =  min(sensorHistory)
            high = max(sensorHistory)
            if low != 0.0: # After restart, wait for 3 measurements before before displaying  data
                if low>SENSOR_MIN and high<SENSOR_MAX and (high-low < SENSOR_SPAN):
                    Domoticz.Log("Ultrasonic: "+str(sensorHistory[0]))
                    
                    if (1 in Devices):
                        #if (Devices[Unit].nValue != nValue) or (Devices[Unit].sValue != sValue):
                        Devices[1].Update(0, str(sensorHistory[0]),Image=image)
                            #Domoticz.Log("Update "+str(nValue)+":'"+str(sValue)+"' ("+Devices[Unit].Name+")")
                    
            #Domoticz.Log("Heartbeat")
            ticks=0
        ticks +=1

    def onStop(self):
        # signal sensor thread to exit
        self.terminate = True
        # Wait until sensor thread has exited
        Domoticz.Log("Threads still active: "+str(threading.active_count())+", should be 1.")
        while (threading.active_count() > 1):
            for thread in threading.enumerate():
                if (thread.name != threading.current_thread().name):
                    Domoticz.Log("'"+thread.name+"' is still running, waiting otherwise Domoticz will abort on plugin exit.")
            time.sleep(1.0)

global _plugin
_plugin = BasePlugin()

def onStart():
    global _plugin
    _plugin.onStart()

def onStop():
    global _plugin
    _plugin.onStop()

def onHeartbeat():
    global _plugin
    _plugin.onHeartbeat()

# Generic helper functions

def VerBose(text):
    if Parameters["Mode6"] != "Normal":
        Domoticz.Log(text)
    return