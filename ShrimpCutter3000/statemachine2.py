from threading import Thread
import time
from servo_communication import Arduino
from LDcommunication import LD
import ConfigurationManager
from multiprocessing import Process, Queue


import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import busio
import board

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(21, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(20, GPIO.IN, GPIO.PUD_DOWN)
GPIO.setup(16, GPIO.IN, GPIO.PUD_DOWN)


class statemachine:
    def __init__(self):
        self.configManager = ConfigurationManager.configManager()
        self.config = self.configManager.readConfig()
        self.armConnection = False
        self.ldConnection = False
        self.Shutdown = False
        self.process_list = []
        self.GUI = None
        self.QueueRequestedState = Queue()
        self.QueueCurrentState = Queue()
        self.QueueArduinoCommand = Queue()
        self.QueueLinearDriveCommand = Queue()
        self.QueueLDPosition = Queue()

        self.CurrentMachineState = {}
        process = Process(
            target=self.Arduino,
            daemon=True,
            args=(
                self.QueueArduinoCommand,
                self.QueueCurrentState,
            ),
        )
        process.start()
        time.sleep(2)
        process = Process(
            target=self.LinearDrive,
            daemon=True,
            args=(
                self.QueueLinearDriveCommand,
                self.QueueLDPosition,
                self.QueueCurrentState,
            ),
        )
        process.start()
        process = Process(
            target=self.statemachine,
            daemon=True,
            args=(
                self.QueueCurrentState,
                self.QueueRequestedState,
                self.QueueArduinoCommand,
                self.QueueLinearDriveCommand,
                self.QueueLDPosition,
            ),
        )
        process.start()
        process = Thread(
            target=self.CurrentState, daemon=True, args=(self.QueueCurrentState,)
        )
        process.start()

    def getParametersfromConfig(self):
        self.config = self.configManager.readConfig()
        # ~ Servos
        self.servo0Pos1 = float(self.config["servo0"]["pos1_angle"])
        self.servo0Pos2 = float(self.config["servo0"]["pos2_angle"])
        self.servo0Pos3 = float(self.config["servo0"]["pos3_angle"])
        self.servo0Pos4 = float(self.config["servo0"]["pos4_angle"])

        self.servo1Pos1 = float(self.config["servo1"]["pos1_angle"])
        self.servo1Pos2 = float(self.config["servo1"]["pos2_angle"])

        self.servo2Pos1 = float(self.config["servo2"]["pos1_angle"])
        self.servo2Pos2 = float(self.config["servo2"]["pos2_angle"])
        self.servo2Pos3 = float(self.config["servo2"]["pos3_angle"])
        self.servo2Pos4 = float(self.config["servo2"]["pos4_angle"])

        self.servo3Pos1 = float(self.config["servo3"]["pos1_angle"])
        self.servo3Pos2 = float(self.config["servo3"]["pos2_angle"])
        self.servo3Pos3 = float(self.config["servo3"]["pos3_angle"])
        self.servo3Pos4 = float(self.config["servo3"]["pos4_angle"])

        self.servo4Pos1 = float(self.config["servo4"]["pos1_angle"])
        self.servo4Pos2 = float(self.config["servo4"]["pos2_angle"])
        self.servo4Pos3 = float(self.config["servo4"]["pos3_angle"])
        self.servo4Pos4 = float(self.config["servo4"]["pos4_angle"])

        self.servo5Pos1 = float(self.config["servo5"]["pos1_angle"])
        self.servo5Pos2 = float(self.config["servo5"]["pos2_angle"])
        # ~ timings
        self.T1 = float(self.config["wait_times"]["T1"])
        self.T2 = float(self.config["wait_times"]["T2"])
        self.T3 = float(self.config["wait_times"]["T3"])
        self.T4 = float(self.config["wait_times"]["T4"])
        self.T5 = float(self.config["wait_times"]["T5"])
        self.T6 = float(self.config["wait_times"]["T6"])
        self.T7 = float(self.config["wait_times"]["T7"])
        self.T8 = float(self.config["wait_times"]["T8"])
        self.T9 = float(self.config["wait_times"]["T9"])
        self.T10 = float(self.config["wait_times"]["T10"])
        self.T11 = float(self.config["wait_times"]["T11"])
        self.T12 = float(self.config["wait_times"]["T12"])
        self.T13 = float(self.config["wait_times"]["T13"])
        self.T14 = float(self.config["wait_times"]["T14"])
        self.T15 = float(self.config["wait_times"]["T15"])
        self.T16 = float(self.config["wait_times"]["T16"])
        self.T17 = float(self.config["wait_times"]["T17"])
        self.T18 = float(self.config["wait_times"]["T18"])
        self.T19 = float(self.config["wait_times"]["T19"])
        self.T20 = float(self.config["wait_times"]["T20"])
        self.T21 = float(self.config["wait_times"]["T21"])
        self.T22 = float(self.config["wait_times"]["T22"])
        self.T23 = float(self.config["wait_times"]["T23"])
        self.T24 = float(self.config["wait_times"]["T24"])
        self.T25 = float(self.config["wait_times"]["T25"])
        self.T26 = float(self.config["wait_times"]["T26"])
        self.T27 = float(self.config["wait_times"]["T27"])
        self.T28 = float(self.config["wait_times"]["T28"])
        self.T29 = float(self.config["wait_times"]["T29"])
        self.T30 = float(self.config["wait_times"]["T30"])
        self.T31 = float(self.config["wait_times"]["T31"])
        self.T32 = float(self.config["wait_times"]["T32"])
        self.T33 = float(self.config["wait_times"]["T33"])
        self.T34 = float(self.config["wait_times"]["T34"])
        self.T35 = float(self.config["wait_times"]["T35"])
        # ~ linear drive
        # ~ positions
        self.WaitPosition = int(self.config["LinearDrive"]["Wait"])
        self.CutPosition = int(self.config["LinearDrive"]["Cut"])
        self.DropOffPosition = int(self.config["LinearDrive"]["DropOff"])
        self.ArrivalPosition = int(self.config["LinearDrive"]["Arrival"])
        # ~ tolerances
        self.PositionTolerance = int(self.config["LinearDrive"]["Tolerance"])
        self.TargetTolerance = int(self.config["LinearDrive"]["TargetTolerance"])
        # ~ speeds
        self.MoveSpeed = int(self.config["LinearDrive"]["MoveSpeed"])
        self.CutInSpeed = int(self.config["LinearDrive"]["CutInSpeed"])
        self.CutOutSpeed = int(self.config["LinearDrive"]["CutOutSpeed"])
        self.CustomSpeed = int(self.config["LinearDrive"]["CustomSpeed"])
        self.RestSpeed = int(self.config["LinearDrive"]["RestSpeed"])

    def shutdown(self):
        print("statemachine shutdown")
        # self.QueueShutdown.put(True)
        # self.arm.Shutdown()
        # self.Shutdown = True
        # time.sleep(2)

    def Arduino(self, QueueArduinoCommand, QueueCurrentState) -> None:
        self.arduino = Arduino()
        self.getParametersfromConfig()
        config = self.configManager.readConfig()
        self.arduino.parametersFromConfig(config)
        command = [77]
        QueueCurrentState.put(["arduino", self.arduino.connected])
        connected = None
        while True:
            if self.arduino.connected != connected:
                connected = self.arduino.connected
                QueueCurrentState.put(["arduino", self.arduino.connected])
            while not QueueArduinoCommand.empty():
                command = QueueArduinoCommand.get()
                command[0] = int(command[0])
                if 0 <= command[0] <= 5:
                    self.arduino.moveServoToAngle(command[0], float(command[1]))
                    QueueCurrentState.put([str(command[0]), str(command[1])])
                elif command[0] in [36, 42, 44, 46, 48]:
                    if command[1] == "0":
                        self.arduino.activateRelays(command[0], False)
                        QueueCurrentState.put([str(command[0]), False])
                    else:
                        self.arduino.activateRelays(command[0], True)
                        QueueCurrentState.put([str(command[0]), True])               
                elif command[0] == 77:
                    self.arduino.writeToArduino("<77>")
            time.sleep(0.01)

    def LinearDrive(
        self, QueueLinearDriveCommand, QueueLDPosition, QueueCurrentState
    ) -> None:
        self.LD = LD()
        self.homing = False
        connected = None
        while True:
            if self.LD.connected != connected:
                connected = self.LD.connected
                QueueCurrentState.put(["LD", self.LD.connected])
            while not QueueLinearDriveCommand.empty():
                command = QueueLinearDriveCommand.get()
                match command[0]:
                    case "homing":
                        if self.LD.connected:
                            self.homing = False
                            self.LD.homing()
                            self.homing = True
                    case "goto":
                        if self.LD.connected:
                            self.LD.immediateOperation(command[1], command[2])
                    case "Soff":
                        if self.LD.connected:
                            self.LD.Soff()
                    case "Pos":
                        if self.LD.connected:
                            position = self.LD.LDgetPosition()
                            QueueLDPosition.put(position)
            time.sleep(0.01)

    def requestState(self, state: str):
        self.QueueRequestedState.put(state)

    def getCurrentState(self):
        return self.CurrentMachineState

    def updateConfig(self):
        self.getParametersfromConfig()
        config = self.configManager.readConfig()
        return config

    def getConfig(self):
        config = self.configManager.readConfig()
        return config

    def CurrentState(self, QueueCurrentState):
        while True:
            if not QueueCurrentState.empty():
                while not QueueCurrentState.empty():
                    state = QueueCurrentState.get()
                    self.CurrentMachineState[state[0]] = state[1]
            else:
                time.sleep(0.01)

    def statemachine(
        self,
        QueueCurrentState,
        QueueRequestedState,
        QueueArduinoCommand,
        QueueLDCommand,
        QueueLDPosition,
    ):
        print("statemachine")
        self.getParametersfromConfig()
        # ProcessState = "init"
        # ProcessStateBelt = "init"
        # ProcessStateRotator = "init"
        # ProcessStateG1= "init"
        # ProcessStateGuillotine2 = "init"
        # ProcessStateSuction = "init"
        PS1 = "init"
        PS2 = "init"
        PS3 = "init"
        PS4 = "init"
        PS5 = "init"
        PS6 = "init"
        ProcessStateString_minus = None

        homing = False
        nextstep="init"
        hold = False
        G1 = None
        G1Ready = False
        G2Ready = False
        WantToEnd = False
        counter = 0
        LaserFeedback = False

        target = None

        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        ads.gain = 1
        chan = AnalogIn(ads, ADS.P0)
        LDPosition = None

        timer1 = time.perf_counter()
        timer2 = time.perf_counter()
        timer3 = time.perf_counter()
        timer4 = time.perf_counter()
        timer5 = time.perf_counter()
        timer6 = time.perf_counter()
        timerLD = time.perf_counter()

        EmergencyState = None
        PSW = None
        LS = None

        waitingFor=""

        

        beltCounter = 0

        while True:
            current_time = time.perf_counter()

            if PSW != GPIO.input(21):
                PSW = GPIO.input(21)
                QueueCurrentState.put(["psw", GPIO.input(21)])
            if LS != GPIO.input(20):
                LS = GPIO.input(20)
                QueueCurrentState.put(["ls", GPIO.input(20)])
            if not QueueLDPosition.empty():
                while not QueueLDPosition.empty():
                    LDPosition = QueueLDPosition.get()
            if not QueueRequestedState.empty():
                while not QueueRequestedState.empty():
                    state = str(QueueRequestedState.get())
                    state = state.split(", ")
                    for i in range(0, len(state)):
                        state[i] = state[i].strip()

                match str(state[0]):
                    case "start":
                        hold = False
                        PS1 = "ArmReadyForHoming"
                        nextstep="run"
                    case "hold":
                        hold = True
                    case "pause":
                        hold = True
                    case "continue":
                        if WantToEnd:
                            PS1 = "ArmReadyForHoming"
                            WantToEnd = False
                        hold = False
                    case "stop":
                        hold = False
                        WantToEnd = True
                    case "ArmReadyForHoming":
                        hold = False
                        PS1 = "ArmReadyForHoming"
                    case "prepareStart":
                        hold = False
                        PS1 = "ArmReadyForHoming"
                    case "loadFeederBelt":
                        hold = False
                        PS1 = "LoadFeederBelt"
                    case "release":
                        hold = False
                        PS1 = "Release"
                    case "direct":
                        hold = False
                        PS1 = "direct"
                        PS2 = "init"
                        PS3 = "init"
                        PS4 = "init"
                        PS3 = "init"
                        PS6 = "init"
                        DirectTarget = [state[1], state[2]]
                    case "LDGoTo":
                        hold = False
                        LDPosTarget = int(state[1])
                        print("statemachine LDGoTo", LDPosTarget)
                        PS1 = "MoveLDToCustomPosition"
                    case "homing":
                        hold = False
                        PS1 = "OnlyHoming"
                    case "StopAndGo":
                        hold = False
                        PS1 = "ReturnToPickupFromAnywhere"
                    case "End":
                        hold = False
                        PS1 = "End"
                    case "LaserFeedback":

                        if LaserFeedback:
                            LaserFeedback = False
                        else:
                            LaserFeedback = True
                        print("LaserFeedback", LaserFeedback)
            if LaserFeedback:
                QueueCurrentState.put(["LaserFeedback", chan.value])
            if EmergencyState != GPIO.input(16):
                EmergencyState = GPIO.input(16)
                QueueCurrentState.put(["EmergencyButton", GPIO.input(16)])
            if EmergencyState:
                PS1 = "Emergency"
                hold = True
                homing=False
                nextstep="init"
                if counter <= 2:
                    QueueArduinoCommand.put([42, "0"])
                    QueueArduinoCommand.put([44, "0"])
                    QueueArduinoCommand.put([46, "0"])
                    QueueArduinoCommand.put([48, "0"])
                    QueueArduinoCommand.put([36, "0"])
                    counter += 1
                if not QueueRequestedState.empty():
                    while not QueueRequestedState.empty():
                        QueueRequestedState.get()

                time.sleep(0.1)
            else:
                counter = 0
            ProcessStateString = (
                waitingFor+
                " M: "
                + PS1
                + " B:"
                + PS2
                + " R:"
                + PS3
                + " G1:"
                + PS4
                + " G2:"
                + PS5
                + " S:"
                + PS6
            )
            if ProcessStateString != ProcessStateString_minus:
                ProcessStateString_minus = ProcessStateString
                QueueCurrentState.put(["ProcessState", ProcessStateString])

            if not hold:
                waitingFor=""
                match PS1:
                    case "init":
                        pass
                    case "Emergency":
                        pass
                    case "ArmReadyForHoming":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "ArmReadyForHoming2a"
                    case "ArmReadyForHoming2a":
                        if current_time - timer1 >= self.T4:
                            if GPIO.input(20):
                                timer1 = time.perf_counter()
                                PS1 = "ArmReadyForHoming2"
                            else:
                                PS1 = "ArmReadyForHoming"
                                waitingFor="Laser Clear"
                    case "ArmReadyForHoming2":
                        if current_time - timer1 >= self.T4:
                            PS3 = "startup"
                            PS1 = "ArmReadyForHoming3"
                    case "ArmReadyForHoming3":                       
                        if PS3 == "Ready":
                            if homing:
                                PS1="GoToWaitPosition"
                            else:
                                PS1 = "Homing"

                            
                    case "OnlyHoming":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "OnlyHoming2"
                    case "OnlyHoming2":
                        if current_time - timer1 >= self.T4:
                            if GPIO.input(20):
                                timer1 = time.perf_counter()
                                PS1 = "Homing"  
                            else:
                                waitingFor="Laser Clear"                   
                    case "Homing":
                        QueueLDCommand.put(["homing"])
                        timer1 = time.perf_counter()
                        PS1 = "Homing2"
                    case "Homing2":
                        if current_time - timer1 >= self.T5:
                            homing = True
                            timer1 = time.perf_counter()
                            if nextstep=="run":
                                PS1 = "GoToWaitPosition"
                            else:
                                PS1 = "init"
                    case "End":
                        QueueArduinoCommand.put([42, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([44, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([46, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([48, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([2, self.servo2Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([3, self.servo3Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([4, self.servo4Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "End2"
                    case "End2":
                        if current_time - timer1 >= self.T1:
                            timer1 = time.perf_counter()
                            LDPosition = None
                            if homing == True:
                                hold = True
                                QueueLDCommand.put(["Pos"])
                                PS1 = "LDPositionBeforeRest"
                            else:
                                hold = True
                                PS1 = "init"
                    case "Release":
                        QueueArduinoCommand.put([42, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([44, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([46, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([48, "0"])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([2, self.servo2Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([3, self.servo3Pos1])
                        time.sleep(0.05)
                        QueueArduinoCommand.put([4, self.servo4Pos1])
                        PS1 = "End2"
                    case "LDPositionBeforeRest":
                        if LDPosition != None:
                            delta = 0 - LDPosition
                            if abs(delta) > 100:
                                if GPIO.input(20):
                                    QueueLDCommand.put(["goto", delta, self.RestSpeed])
                                    hold = True
                                    PS1 = "init"
                                else:
                                    waitingFor="Laser Clear"            
                    case "direct":                    
                        QueueArduinoCommand.put(DirectTarget)
                        PS1 = "init"
                        hold = True
                    case "MoveLDToCustomPosition":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "MoveLDToCustomPosition2"
                    case "MoveLDToCustomPosition2":
                        if current_time - timer1 >= self.T4:
                            if homing:
                                LDPosition = None
                                QueueLDCommand.put(["Pos"])
                                timer1 = time.perf_counter()
                                PS1 = "MoveLDToCustomPosition3"
                            else:
                                PS1 = "MoveLDToCustomPosition2homing"
                    case "MoveLDToCustomPosition2homing":
                        QueueLDCommand.put(["homing"])
                        timer1 = time.perf_counter()
                        PS1 = "MoveLDToCustomPosition2homing2"
                    case "MoveLDToCustomPosition2homing2":
                        if current_time - timer1 >= self.T5:
                            homing = True
                            PS1 = "MoveLDToCustomPosition2"
                    case "MoveLDToCustomPosition3":
                        if LDPosition != None:
                            delta = LDPosTarget - LDPosition
                            if GPIO.input(20):
                                QueueLDCommand.put(["goto", delta, self.CustomSpeed])
                                hold = True
                                PS1 = "init"
                            else:
                                waitingFor="Laser Clear"
                            
                        else:
                            if current_time - timer1 >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "LoadFeederBelt":
                        PS2 = "init"
                        QueueArduinoCommand.put([44, "0"])
                        timer1 = time.perf_counter()
                        beltCounter = 0
                        PS1 = "LoadFeederBelt1"
                    case "LoadFeederBelt1":
                        if current_time - timer1 >= self.T2:
                            QueueArduinoCommand.put([44, "1"])
                            timer1 = time.perf_counter()
                            PS1 = "LoadFeederBelt2"
                    case "LoadFeederBelt2":
                        if beltCounter < 14:
                            if current_time - timer1 >= self.T3:
                                if not GPIO.input(21):
                                    beltCounter += 1
                                    timer1 = time.perf_counter()
                        else:
                            QueueArduinoCommand.put([44, "0"])
                            PS1 = "init"
                            PS2 = "init"
                    
                        if PS3 == "Ready":
                            if LDPosition != None:
                                delta = self.WaitPosition - LDPosition
                                if GPIO.input(20):
                                    QueueLDCommand.put([["goto", delta, self.MoveSpeed]])
                                    PS1 = "StartKnife"
                                else:
                                    waitingFor="Laser Clear"
                    case "GoToWaitPosition":
                        if homing == True:
                            QueueLDCommand.put(
                                ["goto", self.WaitPosition, self.MoveSpeed]
                            )
                            timerLD = time.perf_counter()
                            PS1 = "StartKnife"
                    case "StartKnife":
                        QueueArduinoCommand.put([42, "1"])
                        timer1 = time.perf_counter()
                        PS1 = "StartingKnife1"
                    case "StartingKnife1":
                        if current_time - timer1 >= self.T7:
                            PS1 = "CheckLDPosition"
                    case "CheckLDPosition":
                        if WantToEnd:
                            PS1 = "End"
                        else:
                            if LDPosition == None:
                                if current_time - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                            else:
                                if target != None:
                                    if (
                                        (target - self.PositionTolerance)
                                        < LDPosition
                                        < (target + self.PositionTolerance)
                                    ):
                                        PS1 = "WaitingForBelt"
                                    else:
                                        if current_time - timerLD >= self.T8:
                                            QueueLDCommand.put(["Pos"])
                                            timerLD = time.perf_counter()
                                else:
                                    if (
                                        (self.WaitPosition - self.PositionTolerance)
                                        < LDPosition
                                        < (self.WaitPosition + self.PositionTolerance)
                                    ):
                                        PS1 = "WaitingForBelt"
                                    else:
                                        if current_time - timerLD >= self.T8:
                                            QueueLDCommand.put(["Pos"])
                                            timerLD = time.perf_counter()
                    case "WaitingForBelt":
                        if PS2 == "init":
                            PS2 = "StopAndGo"
                        elif PS2 == "Stopped":
                            PS1 = "CheckWhereWeAre"
                    case "CheckWhereWeAre":
                        QueueCurrentState.put(["LaserAnalog", str(chan.value)])
                        if chan.value > 19500:
                            LDPosition = None
                            PS2 = "StopAndGo"
                            PS1 = "WaitingForBelt"
                        elif 18900 < chan.value <= 19500:
                            target = 28000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 17600 < chan.value <= 18900:
                            target = 27500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 16500 < chan.value <= 17600:
                            target = 27000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 15500 < chan.value <= 16500:
                            target = 26500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 15000 < chan.value <= 15500:
                            target = 26000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 14000 < chan.value <= 15000:
                            target = 25500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 13000 < chan.value <= 14000:
                            target = 25000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 12000 < chan.value <= 13000:
                            target = 24500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif 11500 < chan.value <= 12000:
                            target = 24000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            PS1 = "MoveLDToTarget"
                        elif chan.value < 11500:
                            LDPosition = None
                            PS2 = "StopAndGo"
                            PS1 = "WaitingForBelt"
                    case "MoveLDToTarget":
                        if LDPosition == None:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                        else:
                            if target != None:
                                delta = target - LDPosition
                            if abs(delta) > 5:
                                if GPIO.input(20):
                                    QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                    LDPosition = None
                                    timer1 = time.perf_counter()
                                    PS1 = "checkPickupPosition0"
                                else:
                                    waitingFor="Laser Clear"
                            else:
                                PS1 = "Pickup"
                    case "checkPickupPosition0":
                        if current_time - timer1 >= self.T8:
                            timerLD = time.perf_counter()
                            PS1 = "CheckPickupPosition"
                    case "CheckPickupPosition":
                        if LDPosition == None:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                        else:
                            if (
                                (target - self.TargetTolerance)
                                < LDPosition
                                < (target + self.TargetTolerance)
                            ):
                                PS1 = "Pickup"
                            else:
                                if current_time - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                    case "Pickup":
                        QueueArduinoCommand.put([0, self.servo0Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "Pickup1"
                    case "Pickup1":
                        if current_time - timer1 >= self.T9:
                            QueueArduinoCommand.put([1, self.servo1Pos2])
                            timer1 = time.perf_counter()
                            PS1 = "Pickup2"
                    case "Pickup2":
                        if current_time - timer1 >= self.T10:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            PS1 = "Pickup3"
                    case "Pickup3":
                        if current_time - timer1 >= self.T11:
                            PS2 = "StopAndGo"
                            PS1 = "CheckLSBeforeMoving"
                    case "CheckLSBeforeMoving":
                        # ~ PS1 = "GoToKnife"
                        # ~ TODO remove comment after laser is adjusted
                        if GPIO.input(20):
                            PS1 = "GoToKnife"
                        else:
                            waitingFor="Laser Clear"
                    case "GoToKnife":
                        delta = self.ArrivalPosition - LDPosition
                        QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                        LDPosition = None
                        timerLD = time.perf_counter()
                        PS1 = "WaitingForArrivalAtKnife"
                    case "WaitingForArrivalAtKnife":
                        if LDPosition != None:
                            if (
                                (self.ArrivalPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.ArrivalPosition + self.PositionTolerance)
                            ):
                                timer1 = time.perf_counter()
                                PS1 = "precut"
                            else:
                                if current_time - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "precut":
                        if current_time - timer1 >= self.T8:
                            if GPIO.input(20):
                                PS1 = "cut"
                            else:
                                waitingFor="Laser Clear"
                    case "cut":
                        delta = self.CutPosition - self.ArrivalPosition
                        QueueLDCommand.put(["goto", delta, self.CutInSpeed])
                        LDPosition = None
                        QueueLDCommand.put(["Pos"])
                        timerLD = time.perf_counter()
                        PS1 = "cut2"
                    case "cut2":
                        if LDPosition != None:
                            if (
                                (self.CutPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.CutPosition + self.PositionTolerance)
                            ):
                                PS1 = "cut3"
                            else:
                                if current_time - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "cut3":
                        delta = self.DropOffPosition - self.CutPosition
                        QueueLDCommand.put(["goto", delta, self.CutOutSpeed])
                        LDPosition = None
                        QueueLDCommand.put(["Pos"])
                        timerLD = time.perf_counter()
                        PS1 = "cut4"
                    case "cut4":
                        if LDPosition != None:
                            if (
                                (self.DropOffPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.DropOffPosition + self.PositionTolerance)
                            ):
                                PS1 = "PreDropOff"
                            else:
                                if current_time - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "PreDropOff":
                        if G1Ready and PS4 == "Ready":
                            PS1 = "DropOff0"
                        if G2Ready and PS5 == "Ready":
                            PS1 = "DropOff0"
                    case "DropOff0":
                        if G1Ready:
                            PS4 = "OpenClamps"
                            timer1 = time.perf_counter()
                            PS1 = "DropOff"
                        if G2Ready:
                            PS5 = "OpenClamps"
                            timer1 = time.perf_counter()
                            PS1 = "DropOff"
                    case "DropOff":
                        if current_time - timer1 >= self.T12:
                            QueueArduinoCommand.put([0, self.servo0Pos4])
                            timer1 = time.perf_counter()
                            PS1 = "DropOff1a"
                    case "DropOff1a":
                        if current_time - timer1 >= self.T13:
                            if G1Ready:
                                PS4 = "HandOver"
                                timer1 = time.perf_counter()
                                PS1 = "DropOff1"
                            if G2Ready:
                                PS5 = "HandOver"
                                timer1 = time.perf_counter()
                                PS1 = "DropOff1"
                    case "DropOff1":
                        if G1Ready:
                            if PS4 == "Loaded":
                                QueueArduinoCommand.put([1, self.servo1Pos1])
                                timer1 = time.perf_counter()
                                PS1 = "DropOff2"
                        if G2Ready:
                            if PS5 == "Loaded":
                                QueueArduinoCommand.put([1, self.servo1Pos1])
                                timer1 = time.perf_counter()
                                PS1 = "DropOff2"
                    case "DropOff2":
                        if current_time - timer1 >= self.T14:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            PS1 = "DropOff3"
                    case "DropOff3":
                        if current_time - timer1 >= self.T15:
                            if GPIO.input(20):
                                PS1 = "DropOff4"
                            else:
                                waitingFor="Laser Clear"
                    case "DropOff4":
                        LDPosition = None
                        timerLD = time.perf_counter()
                        if G1Ready:
                            if PS4 == "Loaded":
                                PS4 = "Loaded1"
                        if G2Ready:
                            if PS5 == "Loaded":
                                PS5 = "Loaded1"
                        PS1 = "ReturnToPickup"
                    case "ReturnToPickup":
                        if LDPosition != None:
                            if target != None:
                                if GPIO.input(20):
                                    delta = target - LDPosition
                                    QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                    PS1 = "CheckLDPosition"
                                else:
                                    waitingFor="Laser Clear"
                               
                            else:
                                if GPIO.input(20):
                                    delta = self.WaitPosition - LDPosition
                                    QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                    PS1 = "CheckLDPosition"
                                else:
                                    waitingFor="Laser Clear"
                        else:
                            if current_time - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "ReturnToPickupFromAnywhere":
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        PS1 = "ReturnToPickupFromAnywhere1"
                    case "ReturnToPickupFromAnywhere1":
                        if current_time - timer1 >= self.T16:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            PS1 = "ReturnToPickupFromAnywhere2"
                    case "ReturnToPickupFromAnywhere2":
                        if current_time - timer1 >= self.T17:
                            QueueLDCommand.put(["Pos"])
                            timer1 = time.perf_counter()
                            PS1 = "ReturnToPickupFromAnywhere3"
                    case "ReturnToPickupFromAnywhere3":
                        PS1 = "ReturnToPickup"
                    case _:
                        pass
                # --------Belt-----------------
                match PS2:
                    case "init":
                        pass
                    case "StopAndGo":
                        target = None
                        QueueArduinoCommand.put([44, "1"])
                        timer2 = time.perf_counter()
                        PS2 = "WaitingForPSW"
                    case "WaitingForPSW":
                        if current_time - timer2 >= self.T28:
                            if not GPIO.input(21):
                                timer2 = time.perf_counter()
                                PS2 = "WaitingForMeasurement"
                    case "WaitingForMeasurement":
                        if chan.value < 22000:
                            QueueArduinoCommand.put([44, "0"])
                            timer2 = time.perf_counter()
                            PS2 = "WaitingForMeasurement2"
                    case "WaitingForMeasurement2":
                        if current_time - timer2 >= self.T29:
                            PS2 = "CheckWhereWeAre"
                    case "MoveABit":
                        if current_time - timer2 >= self.T30:
                            LDPosition = None
                            PS2 = "WaitingForMeasurement"
                    case "CheckWhereWeAre":
                        QueueCurrentState.put(["LaserAnalog", str(chan.value)])
                        if chan.value > 19500:
                            QueueArduinoCommand.put([44, "1"])
                            PS2 = "MoveABit"
                        elif 18900 < chan.value <= 19500:
                            target = 28000
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 17600 < chan.value <= 18900:
                            target = 27500
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 16500 < chan.value <= 17600:
                            target = 27000
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 15500 < chan.value <= 16500:
                            target = 26500
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 15000 < chan.value <= 15500:
                            target = 26000
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 14000 < chan.value <= 15000:
                            target = 25500
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 13000 < chan.value <= 14000:
                            target = 25000
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 12000 < chan.value <= 13000:
                            target = 24500
                            LDPosition = None
                            PS2 = "Stopped"
                        elif 11500 < chan.value <= 12000:
                            target = 24000
                            LDPosition = None
                            PS2 = "Stopped"
                        elif chan.value < 11500:
                            LDPosition = None
                            PS2 = "StopAndGo"
                    case "Stopped":
                        pass
                    case _:
                        pass
                #-------------Rotator---------------------------- 
                match PS3:
                    case "init":
                        pass
                    case "startup":
                        timer5 = time.perf_counter()
                        PS6 = "startup"
                        PS3 = "startup0"
                    case "startup0":
                        if current_time - timer5 >= self.T18:
                            QueueArduinoCommand.put([5, self.servo5Pos1])
                            timer5 = time.perf_counter()
                            PS3 = "startup1"
                    case "startup1":
                        if current_time - timer5 >= self.T19:
                            PS5 = "getReady"
                            timer5 = time.perf_counter()
                            PS3 = "startup2a"
                    case "startup2a":
                        if current_time - timer5 >= self.T20:
                            QueueArduinoCommand.put([5, self.servo5Pos2])
                            timer5 = time.perf_counter()
                            PS3 = "startup2"
                    case "startup2":
                        if current_time - timer5 >= self.T21:
                            PS4 = "getReady"
                            timer5 = time.perf_counter()
                            PS3 = "startup3a"
                    case "startup3a":
                        if current_time - timer5 >= self.T22:
                            QueueArduinoCommand.put([5, self.servo5Pos1])
                            timer5 = time.perf_counter()
                            PS3 = "startup3"
                    case "startup3":
                        if current_time - timer5 >= self.T23:
                            G1 = True
                            G1Ready = True
                            G2Ready = False
                            PS3 = "startup4"
                    case "startup4":
                        if PS4 == "Ready" and PS5 == "Ready":
                            PS3 = "Ready"
                    case "Ready":
                        if G1:
                            if PS4 == "Loaded1":
                                if PS5 == "Ready" and PS6 == "init":
                                    QueueArduinoCommand.put([5, self.servo5Pos2])
                                    canDropShrimp = False
                                    timer5 = time.perf_counter()
                                    PS3 = "Rotating"
                        else:
                            if PS5 == "Loaded1":
                                if PS4 == "Ready" and PS6 == "init":
                                    QueueArduinoCommand.put([5, self.servo5Pos1])
                                    canDropShrimp = False
                                    timer5 = time.perf_counter()
                                    PS3 = "Rotating"
                    case "Rotating":
                        if current_time - timer5 >= self.T24:
                            if G1:
                                G1Ready = False
                                G2Ready = True
                            else:
                                G1Ready = True
                                G2Ready = False
                            PS6 = "GoToSuction"
                            PS3 = "ProcessingShrimp"
                    case "ProcessingShrimp":
                        if PS6 == "init":
                            if G1:
                                PS4 = "HeadOff"
                                PS3 = "WaitingForShrimp"
                            else:
                                PS5 = "HeadOff"
                                PS3 = "WaitingForShrimp"
                    case "WaitingForShrimp":
                        if G1:
                            if PS4 == "Ready":
                                G1 = False
                                PS3 = "Ready"
                        else:
                            if PS5 == "Ready":
                                G1 = True
                                PS3 = "Ready"
                    case _:
                        pass
                # --------G1----------
                match PS4:
                    case "init":
                        pass
                    case "getReady":
                        QueueArduinoCommand.put([2, self.servo2Pos2])
                        QueueArduinoCommand.put([48, "1"])
                        timer3 = time.perf_counter()
                        PS4 = "MoveToReady"
                    case "MoveToReady":
                        if current_time - timer3 >= self.T31:
                            PS4 = "Ready"
                    case "Ready":
                        pass
                    case "OpenClamps":
                        QueueArduinoCommand.put([48, "1"])
                        PS4 = "ClampsOpen"
                    case "ClampsOpen":
                        pass
                    case "HandOver":
                        QueueArduinoCommand.put([48, "0"])
                        QueueArduinoCommand.put([2, self.servo2Pos3])
                        timer3 = time.perf_counter()
                        PS4 = "PreLoaded"
                    case "PreLoaded":
                        if current_time - timer3 >= self.T32:
                            PS4 = "Loaded"
                    case "Loaded":
                        pass
                    case "Loaded1":
                        pass
                    case "HeadOff":
                        QueueArduinoCommand.put([2, self.servo2Pos4])
                        timer3 = time.perf_counter()
                        PS4 = "WithoutHead"
                    case "WithoutHead":
                        if current_time - timer3 >= self.T33:
                            QueueArduinoCommand.put([2, self.servo2Pos1])
                            timer3 = time.perf_counter()
                            PS4 = "ReadyForDrop"
                    case "ReadyForDrop":
                        if current_time - timer3 >= self.T34:
                            QueueArduinoCommand.put([48, "1"])
                            timer3 = time.perf_counter()
                            PS4 = "Empty"
                    case "Empty":
                        if current_time - timer3 >= self.T35:
                            PS4 = "getReady"
                    case _:
                        pass
                # -------G2-----------                
                match PS5:
                    case "init":
                        pass
                    case "getReady":
                        QueueArduinoCommand.put([4, self.servo4Pos2])
                        QueueArduinoCommand.put([46, "1"])
                        timer4 = time.perf_counter()
                        PS5 = "MoveToReady"
                    case "MoveToReady":
                        if current_time - timer4 >= self.T31:
                            PS5 = "Ready"
                    case "Ready":
                        pass
                    case "OpenClamps":
                        QueueArduinoCommand.put([46, "1"])
                        PS5 = "ClampsOpen"
                    case "ClampsOpen":
                        pass
                    case "HandOver":
                        QueueArduinoCommand.put([46, "0"])
                        QueueArduinoCommand.put([4, self.servo4Pos3])
                        timer4 = time.perf_counter()
                        PS5 = "PreLoaded"
                    case "PreLoaded":
                        if current_time - timer4 >= self.T32:
                            PS5 = "Loaded"
                    case "Loaded":
                        pass
                    case "Loaded1":
                        pass
                    case "HeadOff":
                        QueueArduinoCommand.put([4, self.servo4Pos4])
                        timer4 = time.perf_counter()
                        PS5 = "WithoutHead"
                    case "WithoutHead":
                        if current_time - timer4 >= self.T33:
                            QueueArduinoCommand.put([4, self.servo4Pos1])
                            timer4 = time.perf_counter()
                            PS5 = "ReadyForDrop"
                    case "ReadyForDrop":
                        if current_time - timer4 >= self.T34:
                            QueueArduinoCommand.put([46, "1"])
                            timer4 = time.perf_counter()
                            PS5 = "Empty"
                    case "Empty":
                        if current_time - timer4 >= self.T35:
                            PS5 = "getReady"
                    case _:
                        pass                
                # ------------Suction----------------------------
                match PS6:
                    case "init":
                        pass
                    case "startup":
                        QueueArduinoCommand.put([3, self.servo3Pos1])
                        PS6 = "init"
                    case "GoToSuction":
                        QueueArduinoCommand.put([3, self.servo3Pos3])
                        timer6 = time.perf_counter()
                        PS6 = "WaitingForArrival"
                    case "WaitingForArrival":
                        if current_time - timer6 >= self.T25:
                            timer6 = time.perf_counter()
                            PS6 = "Sucking"
                    case "Sucking":
                        if current_time - timer6 >= self.T26:
                            QueueArduinoCommand.put([3, self.servo3Pos1])
                            timer6 = time.perf_counter()
                            PS6 = "Retreat"
                    case "Retreat":
                        if current_time - timer6 >= self.T27:
                            PS6 = "init"
                    case _:
                        pass
            else:
                time.sleep(0.1)
