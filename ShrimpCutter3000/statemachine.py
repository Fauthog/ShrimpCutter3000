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
                elif 42 <= command[0] <= 48:
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

        ProcessState = "init"
        ProcessStateBelt = "init"
        homing = False
        hold = False
        mode = "step"
        print("statemachine")
        target = None
        self.getParametersfromConfig()
        i2c = busio.I2C(board.SCL, board.SDA)
        ads = ADS.ADS1115(i2c)
        ads.gain = 1
        chan = AnalogIn(ads, ADS.P0)
        LDPosition = None
        target = None
        timer1 = time.perf_counter()
        timer2 = time.perf_counter()
        timer3 = time.perf_counter()
        timer4 = time.perf_counter()
        timer5 = time.perf_counter()
        timer6 = time.perf_counter()
        timerLD = time.perf_counter()
        timerDeEnergize = time.perf_counter()
        EmergencyState = None
        PSW = None
        LS = None
        ProcessStateString_minus = None
        beltCounter = 0
        ProcessStateRotator = "init"
        ProcessStateG1 = "init"
        ProcessStateGuillotine2 = "init"
        ProcessStateSuction = "init"
        DeEnergize = False
        G1 = None
        G1Ready = False
        G2Ready = False
        WantToEnd = False
        counter = 0
        LaserFeedback = False
        while True:
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
                        ProcessState = "ArmReadyForHoming"
                        # ~ if homing:
                            # ~ if WantToEnd:
                                # ~ ProcessState = "StartAfterEnd"
                            # ~ else:
                                # ~ ProcessState = "ReturnToPickupFromAnywhere"
                        # ~ else:
                            # ~ ProcessState = "ArmReadyForHoming"
                    case "hold":
                        hold = True
                    case "pause":
                        hold = True
                    case "continue":
                        if WantToEnd:
                                ProcessState = "ArmReadyForHoming"
                                WantToEnd=False
                        hold = False
                    case "stop":
                        hold = False
                        WantToEnd = True
                    case "ArmReadyForHoming":
                        hold = False
                        ProcessState = "ArmReadyForHoming"
                    case "prepareStart":
                        hold = False
                        ProcessState = "ArmReadyForHoming"
                        # ~ if homing:
                            # ~ ProcessState = "ReturnToPickupFromAnywhere"
                        # ~ else:
                            # ~ ProcessState = "ArmReadyForHoming"
                    case "loadFeederBelt":
                        hold = False
                        ProcessState = "LoadFeederBelt"
                    case "release":
                        hold = False
                        ProcessState = "Release"
                    case "direct":
                        hold = False
                        # ~ QueueArduinoCommand.put([77,"0"])
                        ProcessState = "direct"
                        ProcessStateBelt = "init"
                        ProcessStateG1="init"
                        ProcessStateG2="init"
                        ProcessStateRotator="init" 
                        ProcessStateSuction="init"                       
                        DirectTarget = [state[1], state[2]]
                    case "LDGoTo":
                        hold = False
                        LDPosTarget = int(state[1])
                        print("statemachine LDGoTo", LDPosTarget)
                        ProcessState = "MoveLDToCustomPosition"
                    case "homing":
                        hold = False
                        ProcessState = "Homing"
                    case "StopAndGo":
                        hold = False
                        ProcessState = "ReturnToPickupFromAnywhere"
                    case "End":
                        hold = False
                        ProcessState = "End"
                    case "LaserFeedback":
                            
                            if LaserFeedback:
                                    LaserFeedback = False
                            else:
                                    LaserFeedback = True
                            print("LaserFeedback", LaserFeedback)
            if LaserFeedback:
                    # ~ QueueCurrentState.put(["LaserFeedback", chan.value])
                    # ~ time.sleep(0.1)
                    continue
            if EmergencyState != GPIO.input(16):
                EmergencyState = GPIO.input(16)
                QueueCurrentState.put(["EmergencyButton", GPIO.input(16)])
            if EmergencyState:
                ProcessState = "Emergency"
                hold = True
                if counter <= 2:
                    QueueArduinoCommand.put([42, "0"])
                    QueueArduinoCommand.put([44, "0"])
                    QueueArduinoCommand.put([46, "0"])
                    QueueArduinoCommand.put([48, "0"])
                    counter += 1
                if not QueueRequestedState.empty():
                    while not QueueRequestedState.empty():
                        QueueRequestedState.get()

                time.sleep(0.1)
            else:
                counter = 0
            ProcessStateString = (
                "M: "
                + ProcessState
                + " B:"
                + ProcessStateBelt
                + " R:"
                + ProcessStateRotator
                + " G1:"
                + ProcessStateG1
                + " G2:"
                + ProcessStateGuillotine2
                + " S:"
                + ProcessStateSuction
            )
            if ProcessStateString != ProcessStateString_minus:
                ProcessStateString_minus = ProcessStateString
                QueueCurrentState.put(["ProcessState", ProcessStateString])
            # ~ print(
            # ~ ProcessState,
            # ~ "B",
            # ~ ProcessStateBelt,
            # ~ "R",
            # ~ ProcessStateRotator,
            # ~ "G1",
            # ~ ProcessStateG1,
            # ~ "G2",
            # ~ ProcessStateGuillotine2,
            # ~ "S",
            # ~ ProcessStateSuction,
            # ~ "E",
            # ~ EmergencyState,
            # ~ )
            if not hold:
                match ProcessState:
                    case "init":
                        pass
                    case "Emergency":
                        pass
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
                        ProcessState = "End2"
                    case "End2":
                        if time.perf_counter() - timer1 >= self.T1:
                            timer1 = time.perf_counter()
                            LDPosition = None
                            if homing == True:
                                hold = True
                                QueueLDCommand.put(["Pos"])
                                ProcessState = "LDPositionBeforeRest"
                            else:
                                hold = True
                                ProcessState = "init"

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
                        ProcessState = "End2"
                    case "LDPositionBeforeRest":
                        if LDPosition != None:
                            delta = 0 - LDPosition
                            if abs(delta) > 100:
                                QueueLDCommand.put(["goto", delta, self.RestSpeed])
                            hold = True
                            ProcessState = "init"
                    case "direct":
                        QueueArduinoCommand.put(DirectTarget)
                        ProcessState = "init"
                        hold = True
                    case "MoveLDToCustomPosition":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        ProcessState = "MoveLDToCustomPosition2"
                    case "MoveLDToCustomPosition2":
                        if time.perf_counter() - timer1 >= self.T4:
                            if homing:
                                LDPosition = None
                                QueueLDCommand.put(["Pos"])
                                timer1 = time.perf_counter()
                                ProcessState = "MoveLDToCustomPosition3"
                            else:
                                ProcessState = "MoveLDToCustomPosition2homing"
                    case "MoveLDToCustomPosition2homing":
                        QueueLDCommand.put(["homing"])
                        timer1 = time.perf_counter()
                        ProcessState = "MoveLDToCustomPosition2homing2"
                    case "MoveLDToCustomPosition2homing2":
                        if time.perf_counter() - timer1 >= self.T5:
                            homing = True
                            ProcessState = "MoveLDToCustomPosition2"
                    case "MoveLDToCustomPosition3":
                        if LDPosition != None:
                            delta = LDPosTarget - LDPosition
                            QueueLDCommand.put(["goto", delta, self.CustomSpeed])
                            hold = True
                            ProcessState = "init"
                        else:
                            if time.perf_counter() - timer1 >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "LoadFeederBelt":
                        ProcessStateBelt = "init"
                        QueueArduinoCommand.put([44, "0"])
                        timer1 = time.perf_counter()
                        beltCounter = 0
                        ProcessState = "LoadFeederBelt1"
                    case "LoadFeederBelt1":
                        if time.perf_counter() - timer1 >= self.T2:
                            QueueArduinoCommand.put([44, "1"])
                            timer1 = time.perf_counter()
                            ProcessState = "LoadFeederBelt2"
                    case "LoadFeederBelt2":
                        if beltCounter < 14:
                            if time.perf_counter() - timer1 >= self.T3:
                                if not GPIO.input(21):
                                    beltCounter += 1
                                    timer1 = time.perf_counter()
                        else:
                            QueueArduinoCommand.put([44, "0"])
                            ProcessState = "init"
                            ProcessStateBelt = "init"
                    case "ArmReadyForHoming":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        ProcessState = "ArmReadyForHoming2a"
                    case "ArmReadyForHoming2a":
                        if time.perf_counter() - timer1 >=self.T4:
                                if GPIO.input(20):
                                        timer1=time.perf_counter()
                                        ProcessState = "ArmReadyForHoming2"
                                else:
                                        ProcessState="ArmReadyForHoming"
                    case "ArmReadyForHoming2":
                        if time.perf_counter() - timer1 >= self.T4:
                            ProcessStateRotator = "startup"
                            ProcessState = "ArmReadyForHoming3"
                    case "ArmReadyForHoming3":
                        if ProcessStateRotator == "Ready":
                            ProcessState = "Homing"
                    case "Homing":
                        QueueLDCommand.put(["homing"])
                        timer1 = time.perf_counter()
                        ProcessState = "Homing2"
                    case "Homing2":
                        if time.perf_counter() - timer1 >= self.T5:
                            homing = True
                            timer1 = time.perf_counter()
                            ProcessState = "GoToWaitPosition"
                    case "StartAfterEnd":
                        QueueArduinoCommand.put([0, self.servo0Pos2])
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        LDPosition = None
                        QueueLDCommand.put(["Pos"])
                        ProcessState = "StartAfterEnd1"
                    case "StartAfterEnd1":
                        if time.perf_counter() - timer1 >= self.T6:
                            ProcessStateRotator = "startup"
                            ProcessState = "StartAfterEnd2"
                    case "StartAfterEnd2":
                        if ProcessStateRotator == "Ready":
                            if LDPosition != None:
                                delta = self.WaitPosition - LDPosition
                                QueueLDCommand.put([["goto", delta, self.MoveSpeed]])
                                ProcessState = "StartKnife"
                    case "GoToWaitPosition":
                        if homing == True:
                            QueueLDCommand.put(
                                ["goto", self.WaitPosition, self.MoveSpeed]
                            )
                            timerLD = time.perf_counter()
                            ProcessState = "StartKnife"
                    case "StartKnife":
                        QueueArduinoCommand.put([42, "1"])
                        timer1 = time.perf_counter()
                        ProcessState = "StartingKnife1"
                    case "StartingKnife1":
                        if time.perf_counter() - timer1 >= self.T7:
                            ProcessState = "CheckLDPosition"

                    case "CheckLDPosition":
                        if WantToEnd:
                            ProcessState = "End"
                        else:
                            if LDPosition == None:
                                if time.perf_counter() - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                            else:
                                if target != None:
                                    if (
                                        (target - self.PositionTolerance)
                                        < LDPosition
                                        < (target + self.PositionTolerance)
                                    ):
                                        ProcessState = "WaitingForBelt"
                                    else:
                                        if time.perf_counter() - timerLD >= self.T8:
                                            QueueLDCommand.put(["Pos"])
                                            timerLD = time.perf_counter()
                                else:
                                    if (
                                        (self.WaitPosition - self.PositionTolerance)
                                        < LDPosition
                                        < (self.WaitPosition + self.PositionTolerance)
                                    ):
                                        ProcessState = "WaitingForBelt"
                                    else:
                                        if time.perf_counter() - timerLD >= self.T8:
                                            QueueLDCommand.put(["Pos"])
                                            timerLD = time.perf_counter()

                    case "WaitingForBelt":
                        if ProcessStateBelt == "init":
                            ProcessStateBelt = "StopAndGo"
                        elif ProcessStateBelt == "Stopped":
                            ProcessState = "CheckWhereWeAre"

                    case "CheckWhereWeAre":
                        QueueCurrentState.put(["LaserAnalog", str(chan.value)])
                        if chan.value > 19500:
                            LDPosition = None
                            ProcessStateBelt = "StopAndGo"
                            ProcessState = "WaitingForBelt"
                        elif 18900 < chan.value <= 19500:
                            target = 28000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 17600 < chan.value <= 18900:
                            target = 27500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 16500 < chan.value <= 17600:
                            target = 27000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 15500 < chan.value <= 16500:
                            target = 26500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 15000 < chan.value <= 15500:
                            target = 26000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 14000 < chan.value <= 15000:
                            target = 25500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 13000 < chan.value <= 14000:
                            target = 25000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 12000 < chan.value <= 13000:
                            target = 24500
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif 11500 < chan.value <= 12000:
                            target = 24000
                            LDPosition = None
                            timerLD = time.perf_counter()
                            ProcessState = "MoveLDToTarget"
                        elif chan.value < 11500:
                            LDPosition = None
                            ProcessStateBelt = "StopAndGo"
                            ProcessState = "WaitingForBelt"
                    case "MoveLDToTarget":
                        if LDPosition == None:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                        else:
                            if target != None:
                                delta = target - LDPosition
                            if abs(delta) > 5:
                                QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                LDPosition = None
                                timer1 = time.perf_counter()
                                ProcessState = "checkPickupPosition0"
                            else:
                                ProcessState = "Pickup"
                    case "checkPickupPosition0":
                        if time.perf_counter() - timer1 >= self.T8:
                            timerLD = time.perf_counter()
                            ProcessState = "CheckPickupPosition"
                    case "CheckPickupPosition":
                        if LDPosition == None:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                        else:
                            if (
                                (target - self.TargetTolerance)
                                < LDPosition
                                < (target + self.TargetTolerance)
                            ):
                                ProcessState = "Pickup"
                            else:
                                if time.perf_counter() - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                    case "Pickup":
                        QueueArduinoCommand.put([0, self.servo0Pos1])
                        timer1 = time.perf_counter()
                        ProcessState = "Pickup1"
                    case "Pickup1":
                        if time.perf_counter() - timer1 >= self.T9:
                            QueueArduinoCommand.put([1, self.servo1Pos2])
                            timer1 = time.perf_counter()
                            ProcessState = "Pickup2"
                    case "Pickup2":
                        if time.perf_counter() - timer1 >= self.T10:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            ProcessState = "Pickup3"
                    case "Pickup3":
                        if time.perf_counter() - timer1 >= self.T11:
                            ProcessStateBelt = "StopAndGo"
                            ProcessState = "CheckLSBeforeMoving"
                    case "CheckLSBeforeMoving":
                        # ~ ProcessState = "GoToKnife"
                        # ~ TODO remove comment after laser is adjusted
                        if GPIO.input(20):
                                ProcessState = "GoToKnife"
                    case "GoToKnife":
                        delta = self.ArrivalPosition - LDPosition
                        QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                        LDPosition = None
                        timerLD = time.perf_counter()
                        ProcessState = "WaitingForArrivalAtKnife"
                    case "WaitingForArrivalAtKnife":
                        if LDPosition != None:
                            if (
                                (self.ArrivalPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.ArrivalPosition + self.PositionTolerance)
                            ):
                                timer1 = time.perf_counter()
                                ProcessState = "precut"
                            else:
                                if time.perf_counter() - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "precut":
                        if time.perf_counter() - timer1 >= self.T8:
                            if GPIO.input(20):
                                ProcessState = "cut"
                    case "cut":
                        delta = self.CutPosition - self.ArrivalPosition
                        QueueLDCommand.put(["goto", delta, self.CutInSpeed])
                        LDPosition = None
                        QueueLDCommand.put(["Pos"])
                        timerLD = time.perf_counter()
                        ProcessState = "cut2"
                    case "cut2":
                        if LDPosition != None:
                            if (
                                (self.CutPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.CutPosition + self.PositionTolerance)
                            ):
                                ProcessState = "cut3"
                            else:
                                if time.perf_counter() - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "cut3":
                        delta = self.DropOffPosition - self.CutPosition
                        QueueLDCommand.put(["goto", delta, self.CutOutSpeed])
                        LDPosition = None
                        QueueLDCommand.put(["Pos"])
                        timerLD = time.perf_counter()
                        ProcessState = "cut4"
                    case "cut4":
                        if LDPosition != None:
                            if (
                                (self.DropOffPosition - self.PositionTolerance)
                                < LDPosition
                                < (self.DropOffPosition + self.PositionTolerance)
                            ):
                                ProcessState = "PreDropOff"
                            else:
                                if time.perf_counter() - timerLD >= self.T8:
                                    QueueLDCommand.put(["Pos"])
                                    timerLD = time.perf_counter()
                        else:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "PreDropOff":
                        if G1Ready and ProcessStateG1 == "Ready":
                            ProcessState = "DropOff0"
                        if G2Ready and ProcessStateGuillotine2 == "Ready":
                            ProcessState = "DropOff0"
                    case "DropOff0":
                        if G1Ready:
                            ProcessStateG1 = "OpenClamps"
                            timer1 = time.perf_counter()
                            ProcessState = "DropOff"
                        if G2Ready:
                            ProcessStateGuillotine2 = "OpenClamps"
                            timer1 = time.perf_counter()
                            ProcessState = "DropOff"
                    case "DropOff":
                        if time.perf_counter() - timer1 >= self.T12:
                            QueueArduinoCommand.put([0, self.servo0Pos4])
                            timer1 = time.perf_counter()
                            ProcessState = "DropOff1a"
                    case "DropOff1a":
                        if time.perf_counter() - timer1 >= self.T13:
                            if G1Ready:
                                ProcessStateG1 = "HandOver"
                                timer1 = time.perf_counter()
                                ProcessState = "DropOff1"
                            if G2Ready:
                                ProcessStateGuillotine2 = "HandOver"
                                timer1 = time.perf_counter()
                                ProcessState = "DropOff1"
                    case "DropOff1":
                        if G1Ready:
                            if ProcessStateG1 == "Loaded":
                                QueueArduinoCommand.put([1, self.servo1Pos1])
                                timer1 = time.perf_counter()
                                ProcessState = "DropOff2"
                        if G2Ready:
                            if ProcessStateGuillotine2 == "Loaded":
                                QueueArduinoCommand.put([1, self.servo1Pos1])
                                timer1 = time.perf_counter()
                                ProcessState = "DropOff2"
                    case "DropOff2":
                        if time.perf_counter() - timer1 >= self.T14:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            ProcessState = "DropOff3"
                    case "DropOff3":
                        if time.perf_counter() - timer1 >= self.T15:
                            LDPosition = None
                            timerLD = time.perf_counter()
                            if G1Ready:
                                if ProcessStateG1 == "Loaded":
                                    ProcessStateG1 = "Loaded1"
                            if G2Ready:
                                if ProcessStateGuillotine2 == "Loaded":
                                    ProcessStateGuillotine2 = "Loaded1"
                            ProcessState = "ReturnToPickup"
                    case "ReturnToPickup":
                        if LDPosition != None:
                            if target != None:
                                delta = target - LDPosition
                                QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                ProcessState = "CheckLDPosition"
                            else:
                                delta = self.WaitPosition - LDPosition
                                QueueLDCommand.put(["goto", delta, self.MoveSpeed])
                                ProcessState = "CheckLDPosition"
                        else:
                            if time.perf_counter() - timerLD >= self.T8:
                                QueueLDCommand.put(["Pos"])
                                timerLD = time.perf_counter()
                    case "ReturnToPickupFromAnywhere":
                        QueueArduinoCommand.put([1, self.servo1Pos1])
                        timer1 = time.perf_counter()
                        ProcessState = "ReturnToPickupFromAnywhere1"
                    case "ReturnToPickupFromAnywhere1":
                        if time.perf_counter() - timer1 >= self.T16:
                            QueueArduinoCommand.put([0, self.servo0Pos2])
                            timer1 = time.perf_counter()
                            ProcessState = "ReturnToPickupFromAnywhere2"
                    case "ReturnToPickupFromAnywhere2":
                        if time.perf_counter() - timer1 >= self.T17:
                            QueueLDCommand.put(["Pos"])
                            timer1 = time.perf_counter()
                            ProcessState = "ReturnToPickupFromAnywhere3"
                    case "ReturnToPickupFromAnywhere3":
                        ProcessState = "ReturnToPickup"
                    case _:
                        pass
                match ProcessStateRotator:
                    case "init":
                        pass
                    case "startup":
                        timer5 = time.perf_counter()
                        ProcessStateSuction = "startup"
                        ProcessStateRotator = "startup0"
                    case "startup0":
                        if time.perf_counter() - timer5 >= self.T18:
                            QueueArduinoCommand.put([5, self.servo5Pos1])
                            timer5 = time.perf_counter()
                            ProcessStateRotator = "startup1"
                    case "startup1":
                        if time.perf_counter() - timer5 >= self.T19:
                            ProcessStateGuillotine2 = "getReady"
                            timer5 = time.perf_counter()
                            ProcessStateRotator = "startup2a"
                    case "startup2a":
                        if time.perf_counter() - timer5 >= self.T20:
                            QueueArduinoCommand.put([5, self.servo5Pos2])
                            timer5 = time.perf_counter()
                            ProcessStateRotator = "startup2"
                    case "startup2":
                        if time.perf_counter() - timer5 >= self.T21:
                            ProcessStateG1 = "getReady"
                            timer5 = time.perf_counter()
                            ProcessStateRotator = "startup3a"
                    case "startup3a":
                        if time.perf_counter() - timer5 >= self.T22:
                            QueueArduinoCommand.put([5, self.servo5Pos1])
                            timer5 = time.perf_counter()
                            ProcessStateRotator = "startup3"
                    case "startup3":
                        if time.perf_counter() - timer5 >= self.T23:
                            G1 = True
                            G1Ready = True
                            G2Ready = False
                            ProcessStateRotator = "startup4"
                    case "startup4":
                        if (
                            ProcessStateG1 == "Ready"
                            and ProcessStateGuillotine2 == "Ready"
                        ):
                            ProcessStateRotator = "Ready"
                    case "Ready":
                        if G1:
                            if ProcessStateG1 == "Loaded1":
                                if (
                                    ProcessStateGuillotine2 == "Ready"
                                    and ProcessStateSuction == "init"
                                ):
                                    QueueArduinoCommand.put([5, self.servo5Pos2])
                                    canDropShrimp = False
                                    timer5 = time.perf_counter()
                                    ProcessStateRotator = "Rotating"
                        else:
                            if ProcessStateGuillotine2 == "Loaded1":
                                if (
                                    ProcessStateG1 == "Ready"
                                    and ProcessStateSuction == "init"
                                ):
                                    QueueArduinoCommand.put([5, self.servo5Pos1])
                                    canDropShrimp = False
                                    timer5 = time.perf_counter()
                                    ProcessStateRotator = "Rotating"
                    case "Rotating":
                        if time.perf_counter() - timer5 >= self.T24:
                            if G1:
                                G1Ready = False
                                G2Ready = True
                            else:
                                G1Ready = True
                                G2Ready = False
                            ProcessStateSuction = "GoToSuction"
                            ProcessStateRotator = "ProcessingShrimp"
                    case "ProcessingShrimp":
                        if ProcessStateSuction == "init":
                            if G1:
                                ProcessStateG1 = "HeadOff"
                                ProcessStateRotator = "WaitingForShrimp"
                            else:
                                ProcessStateGuillotine2 = "HeadOff"
                                ProcessStateRotator = "WaitingForShrimp"
                    case "WaitingForShrimp":
                        if G1:
                            if ProcessStateG1 == "Ready":
                                G1 = False
                                ProcessStateRotator = "Ready"
                        else:
                            if ProcessStateGuillotine2 == "Ready":
                                G1 = True
                                ProcessStateRotator = "Ready"
                    case _:
                        pass
                match ProcessStateSuction:
                    case "init":
                        pass
                    case "startup":
                        QueueArduinoCommand.put([3, self.servo3Pos1])
                        ProcessStateSuction = "init"
                    case "GoToSuction":
                        QueueArduinoCommand.put([3, self.servo3Pos3])
                        timer6 = time.perf_counter()
                        ProcessStateSuction = "WaitingForArrival"
                    case "WaitingForArrival":
                        if time.perf_counter() - timer6 >= self.T25:
                            timer6 = time.perf_counter()
                            ProcessStateSuction = "Sucking"
                    case "Sucking":
                        if time.perf_counter() - timer6 >= self.T26:
                            QueueArduinoCommand.put([3, self.servo3Pos1])
                            timer6 = time.perf_counter()
                            ProcessStateSuction = "Retreat"
                    case "Retreat":
                        if time.perf_counter() - timer6 >= self.T27:
                            ProcessStateSuction = "init"
                    case _:
                        pass
                match ProcessStateBelt:
                    case "init":
                        pass
                    case "StopAndGo":
                        target = None
                        QueueArduinoCommand.put([44, "1"])
                        timer2 = time.perf_counter()
                        ProcessStateBelt = "WaitingForPSW"
                    case "WaitingForPSW":
                        if time.perf_counter() - timer2 >= self.T28:
                            if not GPIO.input(21):
                                timer2 = time.perf_counter()
                                ProcessStateBelt = "WaitingForMeasurement"
                    case "WaitingForMeasurement":
                        if chan.value < 22000:
                            QueueArduinoCommand.put([44, "0"])
                            timer2 = time.perf_counter()
                            ProcessStateBelt = "WaitingForMeasurement2"
                    case "WaitingForMeasurement2":
                        if time.perf_counter() - timer2 >= self.T29:
                            ProcessStateBelt = "CheckWhereWeAre"
                    case "MoveABit":
                        if time.perf_counter() - timer2 >= self.T30:
                            LDPosition = None
                            ProcessStateBelt = "WaitingForMeasurement"
                    case "CheckWhereWeAre":
                        QueueCurrentState.put(["LaserAnalog", str(chan.value)])
                        if chan.value > 19500:
                            QueueArduinoCommand.put([44, "1"])
                            ProcessStateBelt = "MoveABit"
                        elif 18900 < chan.value <= 19500:
                            target = 28000
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 17600 < chan.value <= 18900:
                            target = 27500
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 16500 < chan.value <= 17600:
                            target = 27000
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 15500 < chan.value <= 16500:
                            target = 26500
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 15000 < chan.value <= 15500:
                            target = 26000
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 14000 < chan.value <= 15000:
                            target = 25500
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 13000 < chan.value <= 14000:
                            target = 25000
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 12000 < chan.value <= 13000:
                            target = 24500
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif 11500 < chan.value <= 12000:
                            target = 24000
                            LDPosition = None
                            ProcessStateBelt = "Stopped"
                        elif chan.value < 11500:
                            LDPosition = None
                            ProcessStateBelt = "StopAndGo"
                    case "Stopped":
                        pass
                    case _:
                        pass
                match ProcessStateG1:
                    case "init":
                        pass
                    case "getReady":
                        QueueArduinoCommand.put([2, self.servo2Pos2])
                        QueueArduinoCommand.put([48, "1"])
                        timer3 = time.perf_counter()
                        ProcessStateG1 = "MoveToReady"
                    case "MoveToReady":
                        if time.perf_counter() - timer3 >= self.T31:
                            ProcessStateG1 = "Ready"
                    case "Ready":
                        pass
                    case "OpenClamps":
                        QueueArduinoCommand.put([48, "1"])
                        ProcessStateG1 = "ClampsOpen"
                    case "ClampsOpen":
                        pass
                    case "HandOver":
                        QueueArduinoCommand.put([48, "0"])
                        QueueArduinoCommand.put([2, self.servo2Pos3])
                        timer3 = time.perf_counter()
                        ProcessStateG1 = "PreLoaded"
                    case "PreLoaded":
                        if time.perf_counter() - timer3 >= self.T32:
                            ProcessStateG1 = "Loaded"
                    case "Loaded":
                        pass
                    case "Loaded1":
                        pass
                    case "HeadOff":
                        QueueArduinoCommand.put([2, self.servo2Pos4])
                        timer3 = time.perf_counter()
                        ProcessStateG1 = "WithoutHead"
                    case "WithoutHead":
                        if time.perf_counter() - timer3 >= self.T33:
                            QueueArduinoCommand.put([2, self.servo2Pos1])
                            timer3 = time.perf_counter()
                            ProcessStateG1 = "ReadyForDrop"
                    case "ReadyForDrop":
                        if time.perf_counter() - timer3 >= self.T34:
                            QueueArduinoCommand.put([48, "1"])
                            timer3 = time.perf_counter()
                            ProcessStateG1 = "Empty"
                    case "Empty":
                        if time.perf_counter() - timer3 >= self.T35:
                            ProcessStateG1 = "getReady"
                    case _:
                        pass
                match ProcessStateGuillotine2:
                    case "init":
                        pass
                    case "getReady":
                        QueueArduinoCommand.put([4, self.servo4Pos2])
                        QueueArduinoCommand.put([46, "1"])
                        timer4 = time.perf_counter()
                        ProcessStateGuillotine2 = "MoveToReady"
                    case "MoveToReady":
                        if time.perf_counter() - timer4 >= self.T31:
                            ProcessStateGuillotine2 = "Ready"
                    case "Ready":
                        pass
                    case "OpenClamps":
                        QueueArduinoCommand.put([46, "1"])
                        ProcessStateGuillotine2 = "ClampsOpen"
                    case "ClampsOpen":
                        pass
                    case "HandOver":
                        QueueArduinoCommand.put([46, "0"])
                        QueueArduinoCommand.put([4, self.servo4Pos3])
                        timer4 = time.perf_counter()
                        ProcessStateGuillotine2 = "PreLoaded"
                    case "PreLoaded":
                        if time.perf_counter() - timer4 >= self.T32:
                            ProcessStateGuillotine2 = "Loaded"
                    case "Loaded":
                        pass
                    case "Loaded1":
                        pass
                    case "HeadOff":
                        QueueArduinoCommand.put([4, self.servo4Pos4])
                        timer4 = time.perf_counter()
                        ProcessStateGuillotine2 = "WithoutHead"
                    case "WithoutHead":
                        if time.perf_counter() - timer4 >= self.T33:
                            QueueArduinoCommand.put([4, self.servo4Pos1])
                            timer4 = time.perf_counter()
                            ProcessStateGuillotine2 = "ReadyForDrop"
                    case "ReadyForDrop":
                        if time.perf_counter() - timer4 >= self.T34:
                            QueueArduinoCommand.put([46, "1"])
                            timer4 = time.perf_counter()
                            ProcessStateGuillotine2 = "Empty"
                    case "Empty":
                        if time.perf_counter() - timer4 >= self.T35:
                            ProcessStateGuillotine2 = "getReady"
                    case _:
                        pass
            else:
                    time.sleep(0.1)
