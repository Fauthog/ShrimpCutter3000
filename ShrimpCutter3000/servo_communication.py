import serial
from serial.tools import list_ports
import ast

class Arduino():   
    def __init__(self):
        self.port = None
        self.shutdown = False
        self.connected = False
 
    def Shutdown(self):
        self.shutdown = True        
    
    def parametersFromConfig(self, config):
        self.minAngle=[float(config["servo0"]["min_angle"]), 
                       float(config["servo1"]["min_angle"]), 
                       float(config["servo2"]["min_angle"]),
                       float(config["servo3"]["min_angle"]),
                       float(config["servo4"]["min_angle"]),
                       float(config["servo5"]["min_angle"])
                       ]
        self.maxAngle=[float(config["servo0"]["max_angle"]), 
                       float(config["servo1"]["max_angle"]), 
                       float(config["servo2"]["max_angle"]),
                       float(config["servo3"]["max_angle"]),
                       float(config["servo4"]["max_angle"]),
                       float(config["servo5"]["max_angle"])
                       ]
        self.printCmd=ast.literal_eval(config["Console"]["printCmd"])
        
        self.conversionMicrosecondsToDegrees = 2000/270
        self.scan()

    def scan(self)->str:
        print("scanning for arduino")
        # scan all connected serial ports for arduino. If arduino is found, set self.port to the found port
        # after that call the function to setup the serial connection to the arduino
        VID = '1A86'
        PID = '7523'
        device_list = list_ports.comports()
        if self.port == None:
            for device in device_list:            
                if (device.vid != None or device.pid != None):                    
                    if ('{:04X}'.format(device.vid) == VID and
                        '{:04X}'.format(device.pid) == PID):
                        self.port = device.device                    
                        break
                    self.port = None
                
            if self.port is not None:
                self.setupSerialToArduino(self.port)
                self.connected = True
            else:
                self.connected = False
    
    def setupSerialToArduino(self, port)->None:
            # establish serial connection to arduino and send the default position values for all servos. 
            self.arduino = serial.Serial(port=port, baudrate=9600, timeout=2)

    def writeToArduino(self, cmd:str)->None:  
        # write the position cmd value to the arduino via the serial port
        if not self.arduino.is_open:
                self.connected=False
                print("not open")
        try:
            self.arduino.write((cmd + "\r\n").encode('utf-8'))
            if self.printCmd:
                print("arduino cmd:", cmd)
        except:
            print("cannot write to Arduino")

    def activateRelays(self, relays:int, state:bool)->None:
        if state:
            active="1"
        else:
            active="0"
        cmd="<" + str(relays) + "," + active + ">"
        if self.port is not None:
            self.writeToArduino(cmd)
        if self.printCmd:
            print("relay cmd", cmd)
        
    def cleanUpArduinoConnection(self)->None:
        # close the srial connection to the arduino
        
        self.writeToArduino("<77, 1>")
        print("closing Arduino")
        try:
            if self.port is not None:
                if self.arduino !="None":
                    self.arduino.flushInput()
                    self.arduino.flushOutput()
                    self.arduino.close()
        except:
            print("no connection to cleanup")
            
        return

    def moveServoToAngle(self, servo:int, angle:float)->None:
        clamped_angle = min(max(angle, self.minAngle[servo]), self.maxAngle[servo])
        ms =max(min(round((clamped_angle*self.conversionMicrosecondsToDegrees)+500), 2500) ,500)
        cmd="<" + str(servo) + "," + str(ms) + ">"
        if self.port is not None:
            self.writeToArduino(cmd)
        if self.printCmd:
            print("servo cmd", cmd)
