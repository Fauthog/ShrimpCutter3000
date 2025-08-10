from alpha5 import Alpha5Client
import serial
from serial.tools import list_ports
import time
import RPi.GPIO as GPIO

class linearDrive(): 
    def __init__(self):
        self.port=None
        self.scanForLinearDrive()
        if self.port is not None:
            self.alpha5 = Alpha5Client(self.port)
            self.alpha5.create_client()
            self.alpha5.connect()
            self.alpha5.set_show_query_response(True)
            self.homingDone=False
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(21, GPIO.IN)
            self.loop()
    
    def loop(self):
        self.homing()
        while True:
            if GPIO.input(21):
                self.immediateOperation(5000, 5000)
                time.sleep(1)
                self.immediateOperation(-5000, 5000)
                
            

            # userInput = input("command: ")
            # if userInput[0]=="e":
            #     break
            # elif userInput[0]=="h":
            #     self.homing()
            #     self.homingDone=True
            # elif userInput[0]=="s":
            #     self.getPos()
            # elif (userInput[0]!="e" or userInput[0]!="h") and self.homingDone==True:
            #     value=int(userInput.strip())
            #     self.immediateOperation(value, 5000)
                
    def getPos(self):
        print("position: ", self.alpha5.get_feedback_position(id=1))
    
    def scanForLinearDrive(self)->str:
        VID = '10C4'
        PID = 'EA60'
        device_list = list_ports.comports()
        if self.port == None:
            for device in device_list:  
                print("device", device.vid, device.pid, '{:04X}'.format(device.vid),  '{:04X}'.format(device.pid))          
                if (device.vid != None or device.pid != None):                    
                    if ('{:04X}'.format(device.vid) == VID and
                        '{:04X}'.format(device.pid) == PID):
                        self.port = device.device 
                        print("port", self.port)                   
                        break
                    self.port = None
    
    def homing(self)->None:
        print('Set [S-ON] to ON')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
        time.sleep(0.1)
        print('Issue homing command')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='ON') # [ORG]->CONT11
        time.sleep(0.1)
        print("Homing...")
        print('Wait for the operation to complete')
        time.sleep(5)
        self.alpha5.wait_operation_complete(id=1)
        print('done!')        
     
        print('Set [S-ON] to OFF')
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
        self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='OFF') # [ORG]->CONT11



    def immediateOperation(self, Units:int, Speed:int)->None:
        try:
                # print('Set [S-ON] to ON')
                self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
                # ~ time.sleep(0.1)
                # print('Send immediate operation setting')
                self.alpha5.send_immediate_operation_setting(id=1, units=Units, speed=Speed)
                # ~ time.sleep(0.1)
                # print('Set [START] to ON')
                self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='ON') # [START]->CONT10
                # ~ time.sleep(0.1)
                # print('Set [START] to OFF')
                self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='OFF') # [START]->CONT10
                time.sleep(0.1)


                # print('Wait for the operation to complete')
                self.alpha5.wait_operation_complete(id=1)


                # print('Set [S-ON] to OFF')
                self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
        except:
                print("cannot write immediate operation to linear drive") 


def test():
	print("test")
	alpha5 = Alpha5Client("/dev/ttyUSB1")
	alpha5.create_client()
	alpha5.connect()
	alpha5.set_show_query_response(True)
	
	# Homing
	print('Set [S-ON] to ON')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
	print('Issue homing command')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='ON') # [ORG]->CONT11
	print("Homing...")
	print('Wait for the operation to complete')
	alpha5.wait_operation_complete(id=1)
	print('done!')        
	print('Set [S-ON] to OFF')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=2, state='OFF') # [ORG]->CONT11

	# Move some units
	print("============= Move some units =============")
	print('Set [S-ON] to ON')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='ON') # [S-ON]->CONT9
	print('Send immediate operation setting')
	alpha5.send_immediate_operation_setting(id=1, units=1000, speed=5000)
	print('Set [START] to ON')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='ON') # [START]->CONT10
	print('Set [START] to OFF')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=1, state='OFF') # [START]->CONT10
	print('Wait for the operation to complete')
	alpha5.wait_operation_complete(id=1)
	print('Set [S-ON] to OFF')
	alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
	
def main():
    ld=linearDrive()
    # ~ test()


if __name__ == "__main__":
    main()
