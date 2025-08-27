import customtkinter
from statemachine import statemachine
import time
import ast


customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme(
    "blue"
)  # Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_widget_scaling(0.9)


class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()
        self.statemachine = statemachine()
        config = self.statemachine.getConfig()
        self.commissioning = ast.literal_eval(config["Commissioning"]["commissioning"])

        # configure window
        self.title("ShrimpCutter3000")
        self.geometry(f"{1280}x{700}")

        # create tabview
        self.tabview = customtkinter.CTkTabview(self, width=1280, height=700)
        self.tabview.grid(row=0, column=0)
        self.tabview.add("GUI")
        if self.commissioning:
            self.tabview.add("Commissioning")

        self.GUI_Tab = GUI_TabFrame(self.tabview.tab("GUI"), self.statemachine)
        self.GUI_Tab.grid(row=0, column=0)
        if self.commissioning:
            self.Commissioning_Tab = Commissioning_TabFrame(
                self.tabview.tab("Commissioning"), self.statemachine
            )
            self.Commissioning_Tab.grid(
                row=0, column=0, padx=10, pady=(10, 0), sticky="sw"
            )

        self.protocol("WM_DELETE_WINDOW", self.closed)

    def shutdown(self):
        print("shutting down")
        self.statemachine.shutdown()

    def closed(self):
        self.shutdown()
        time.sleep(0.1)
        self.quit()


class GUI_TabFrame(customtkinter.CTkFrame):
    def __init__(self, master, statemachine):
        super().__init__(master)
        self.statemachine = statemachine

        self.BaseFrame2 = customtkinter.CTkFrame(self, width=1280, height=700)
        self.BaseFrame2.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")

        self.arduino_btn = customtkinter.CTkButton(
            self.BaseFrame2,
            text="Arduino",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.arduino_btn.grid(row=0, column=0, padx=40, pady=(0, 0))
        self.ld_btn = customtkinter.CTkButton(
            self.BaseFrame2,
            text="Linear Drive",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.ld_btn.grid(row=0, column=1, padx=20, pady=(0, 0))

        self.EmergencyButton_btn = customtkinter.CTkButton(
            self.BaseFrame2,
            text="Emergency Stop",
            font=customtkinter.CTkFont(size=20, weight="bold"),
            state="disabled",
        )
        self.EmergencyButton_btn.grid(row=0, column=2, padx=20, pady=(0, 0))

        self.BaseFrame = customtkinter.CTkFrame(self, width=1280, height=700)
        self.BaseFrame.grid(row=1, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")

        self.load_belt_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="load feeder belt",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("loadFeederBelt"),
        )
        self.load_belt_btn.grid(row=1, column=1, padx=0, pady=(20, 0))
        self.start_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="start",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("start"),
        )
        self.start_btn.grid(row=2, column=1, padx=0, pady=(20, 0))
        self.pause_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="pause",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("pause"),
        )
        self.pause_btn.grid(row=3, column=0, padx=0, pady=(20, 0))
        self.continue_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="continue",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("continue"),
        )
        self.continue_btn.grid(row=3, column=1, padx=0, pady=(20, 0))
        self.stop_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="stop",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("stop"),
        )
        self.stop_btn.grid(row=3, column=2, padx=0, pady=(20, 0))
        self.release_btn = customtkinter.CTkButton(
            self.BaseFrame,
            text="release",
            font=customtkinter.CTkFont(size=40, weight="bold"),
            command=lambda: self.state_callback("release"),
        )
        self.release_btn.grid(row=4, column=1, padx=0, pady=(20, 0))

        self.BaseFrame3 = customtkinter.CTkFrame(self, width=1280, height=700)
        self.BaseFrame3.grid(row=1, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.relay42_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="42", state="disabled"
        )
        self.relay42_btn.grid(row=0, column=0, padx=20, pady=(10, 0))
        self.relay44_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="44", state="disabled"
        )
        self.relay44_btn.grid(row=0, column=1, padx=20, pady=(10, 0))
        self.relay46_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="46", state="disabled"
        )
        self.relay46_btn.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.relay48_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="48", state="disabled"
        )
        self.relay48_btn.grid(row=1, column=1, padx=20, pady=(10, 0))
        self.psw_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="psw", state="disabled"
        )
        self.psw_btn.grid(row=2, column=0, padx=20, pady=(10, 0))
        self.laserAnalog_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="0", state="disabled"
        )
        self.laserAnalog_btn.grid(row=2, column=1, padx=20, pady=(10, 0))
        self.servo0_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo0_btn.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.servo1_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo1_btn.grid(row=3, column=1, padx=20, pady=(10, 0))
        self.servo2_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo2_btn.grid(row=4, column=0, padx=20, pady=(10, 0))
        self.servo3_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo3_btn.grid(row=4, column=1, padx=20, pady=(10, 0))
        self.servo4_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo4_btn.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.servo5_btn = customtkinter.CTkButton(
            self.BaseFrame3, text="servo0", state="disabled"
        )
        self.servo5_btn.grid(row=5, column=1, padx=20, pady=(10, 0))
        self.getCurrentState()

    def getCurrentState(self):
        self.currentState = self.statemachine.getCurrentState()
        self.updateGUItoCurrentState()
        self.after(1000, self.getCurrentState)

    def updateGUItoCurrentState(self):
        if "arduino" in self.currentState:
            if self.currentState["arduino"]:
                self.arduino_btn.configure(fg_color="green")
            else:
                self.arduino_btn.configure(fg_color="red")
        if "LD" in self.currentState:
            if self.currentState["LD"]:
                self.ld_btn.configure(fg_color="green")
            else:
                self.ld_btn.configure(fg_color="red")
        if "0" in self.currentState:
            self.servo0_btn.configure(text="servo0 " + str(self.currentState["0"]))
        if "1" in self.currentState:
            self.servo1_btn.configure(text="servo1 " + str(self.currentState["1"]))
        if "2" in self.currentState:
            self.servo2_btn.configure(text="servo2 " + str(self.currentState["2"]))
        if "3" in self.currentState:
            self.servo3_btn.configure(text="servo3 " + str(self.currentState["3"]))
        if "4" in self.currentState:
            self.servo4_btn.configure(text="servo4 " + str(self.currentState["4"]))
        if "5" in self.currentState:
            self.servo5_btn.configure(text="servo5 " + str(self.currentState["5"]))
        if "42" in self.currentState:
            if self.currentState["42"]:
                self.relay42_btn.configure(fg_color="green")
            else:
                self.relay42_btn.configure(fg_color="red")
        if "44" in self.currentState:
            if self.currentState["44"]:
                self.relay44_btn.configure(fg_color="green")
            else:
                self.relay44_btn.configure(fg_color="red")
        if "46" in self.currentState:
            if self.currentState["46"]:
                self.relay46_btn.configure(fg_color="green")
            else:
                self.relay46_btn.configure(fg_color="red")
        if "48" in self.currentState:
            if self.currentState["48"]:
                self.relay48_btn.configure(fg_color="green")
            else:
                self.relay48_btn.configure(fg_color="red")
        if "psw" in self.currentState:
            if self.currentState["psw"]:
                self.psw_btn.configure(fg_color="green")
            else:
                self.psw_btn.configure(fg_color="red")
        if "LaserAnalog" in self.currentState:
            self.laserAnalog_btn.configure(text=str(self.currentState["LaserAnalog"]))
        if "EmergencyButton" in self.currentState:
            if self.currentState["EmergencyButton"]:
                self.EmergencyButton_btn.configure(fg_color="red")
            else:
                self.EmergencyButton_btn.configure(fg_color="green")

    def state_callback(self, text):
        # print("state", text)
        if "[" in text:
            self.statemachine.requestState(text[: text.index("[")].strip())
        else:
            self.statemachine.requestState(text.strip())


class Commissioning_TabFrame(customtkinter.CTkFrame):
    def __init__(self, master, statemachine):
        super().__init__(master)
        self.statemachine = statemachine

        self.Mode: str = "step"
        self.LoopState: str = ""
        self.arm_wait = 0.0
        self.arm_pickup = 0.0
        self.arm_traverse = 0.0
        self.arm_camera = 0.0
        self.arm_cut = 0.0
        self.shrimp_clamp_open = 0.0
        self.shrimp_clamp_close = 0.0
        self.guillotine_fully_open = 0.0
        self.guillotine_open = 0.0
        self.guillotine_grip = 0.0
        self.guillotine_close = 0.0
        self.ArduinoEnabled = True

        self.BaseFrame = customtkinter.CTkFrame(self, width=1280, height=700)
        self.BaseFrame.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")

        self.leftFrame = customtkinter.CTkFrame(self.BaseFrame)
        self.leftFrame.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.middleFrame = customtkinter.CTkScrollableFrame(
            self.BaseFrame, width=400, height=400
        )
        self.middleFrame.grid(
            row=0, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew"
        )
        self.rightFrame = customtkinter.CTkFrame(self.BaseFrame)
        self.rightFrame.grid(row=0, column=2, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.bottomFrame = customtkinter.CTkFrame(self.BaseFrame)
        self.bottomFrame.grid(
            row=1, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew"
        )
        self.FlowFrame = customtkinter.CTkFrame(self.BaseFrame)
        self.FlowFrame.grid(row=1, column=1, padx=(20, 0), pady=(20, 0), sticky="nsew")
        self.BoolFrame = customtkinter.CTkFrame(self.BaseFrame)
        self.BoolFrame.grid(row=1, column=2, padx=(20, 0), pady=(20, 0), sticky="nsew")

        self.directAngleFrame(self.rightFrame)
        self.loopFrame(self.leftFrame)
        self.positionFrame(self.middleFrame)
        self.statusFrame(self.bottomFrame)
        self.flowFrame(self.FlowFrame)
        self.boolFrame(self.BoolFrame)

        self.setConfig(self.statemachine.getConfig())
        self.getCurrentState()

    def getCurrentState(self):
        self.currentState = self.statemachine.getCurrentState()
        self.updateGUItoCurrentState()
        self.after(1000, self.getCurrentState)

    def statusFrame(self, frame):
        self.config_btn = customtkinter.CTkButton(
            frame, text="update configuration", command=self.updateConfig
        )
        self.config_btn.grid(row=0, column=0, padx=20, pady=(10, 0))
        self.arduino_btn = customtkinter.CTkButton(frame, text="Arduino")
        self.arduino_btn.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.ld_btn = customtkinter.CTkButton(frame, text="Linear Drive")
        self.ld_btn.grid(row=2, column=0, padx=20, pady=(10, 0))
        self.EmergencyButton_btn = customtkinter.CTkButton(frame, text="Emergency Stop")
        self.EmergencyButton_btn.grid(row=3, column=0, padx=20, pady=(10, 0))

    def updateConfig(self) -> None:
        self.setConfig(self.statemachine.updateConfig())

    def setConfig(self, config) -> None:
        self.config = config
        self.servo0_close_label.configure(
            text="up " + str(self.config["servo0"]["min_angle"])
        )
        self.servo0_open_label.configure(
            text="down " + str(self.config["servo0"]["max_angle"])
        )
        self.servo1_open_label.configure(
            text="open " + str(self.config["servo1"]["min_angle"])
        )
        self.servo1_close_label.configure(
            text="close " + str(self.config["servo1"]["max_angle"])
        )
        self.servo2_open_label.configure(
            text="open " + str(self.config["servo2"]["max_angle"])
        )
        self.servo2_close_labell.configure(
            text="close " + str(self.config["servo2"]["min_angle"])
        )
        self.servo3_open_label.configure(
            text="open " + str(self.config["servo3"]["max_angle"])
        )
        self.servo3_close_label.configure(
            text="away from shrimp " + str(self.config["servo3"]["min_angle"])
        )
        self.servo4_open_label.configure(
            text="touch shrimp " + str(self.config["servo4"]["max_angle"])
        )
        self.servo4_close_label.configure(
            text="close " + str(self.config["servo4"]["min_angle"])
        )
        self.servo5_open_label.configure(
            text="open " + str(self.config["servo5"]["max_angle"])
        )
        self.servo5_close_label.configure(
            text="close " + str(self.config["servo5"]["min_angle"])
        )
        self.servo0_Pos1_btn.configure(
            text="pickup [" + self.config["servo0"]["pos1_angle"] + "]"
        )
        self.servo0_Pos2_btn.configure(
            text="traverse [" + self.config["servo0"]["pos2_angle"] + "]"
        )
        self.servo0_Pos3_btn.configure(
            text="cut [" + self.config["servo0"]["pos3_angle"] + "]"
        )
        self.servo0_Pos4_btn.configure(
            text="dropoff [" + self.config["servo0"]["pos4_angle"] + "]"
        )
        self.servo1_Pos1_btn.configure(
            text="open [" + self.config["servo1"]["pos1_angle"] + "]"
        )
        self.servo1_Pos2_btn.configure(
            text="close [" + self.config["servo1"]["pos2_angle"] + "]"
        )
        self.servo2_Pos1_btn.configure(
            text="fully open [" + self.config["servo2"]["pos1_angle"] + "]"
        )
        self.servo2_Pos2_btn.configure(
            text="open [" + self.config["servo2"]["pos2_angle"] + "]"
        )
        self.servo2_Pos3_btn.configure(
            text="grip [" + self.config["servo2"]["pos3_angle"] + "]"
        )
        self.servo2_Pos4_btn.configure(
            text="nose cut [" + self.config["servo2"]["pos4_angle"] + "]"
        )
        self.servo3_Pos1_btn.configure(
            text="Far away from shrimp [" + self.config["servo3"]["pos1_angle"] + "]"
        )
        self.servo3_Pos2_btn.configure(
            text="Close to Shrimp [" + self.config["servo3"]["pos2_angle"] + "]"
        )
        self.servo3_Pos3_btn.configure(
            text="Touch shrimp [" + self.config["servo3"]["pos3_angle"] + "]"
        )
        self.servo3_Pos4_btn.configure(
            text="Pos4 [" + self.config["servo3"]["pos4_angle"] + "]"
        )
        self.servo4_Pos1_btn.configure(
            text="fully open [" + self.config["servo4"]["pos1_angle"] + "]"
        )
        self.servo4_Pos2_btn.configure(
            text="open [" + self.config["servo4"]["pos2_angle"] + "]"
        )
        self.servo4_Pos3_btn.configure(
            text="grip [" + self.config["servo4"]["pos3_angle"] + "]"
        )
        self.servo4_Pos4_btn.configure(
            text="nose cut [" + self.config["servo4"]["pos4_angle"] + "]"
        )
        self.servo5_Pos1_btn.configure(
            text="1 up [" + self.config["servo5"]["pos1_angle"] + "]"
        )
        self.servo5_Pos2_btn.configure(
            text="2 up [" + self.config["servo5"]["pos2_angle"] + "]"
        )

    def directAngleFrame(self, frame):
        # arm
        self.servo0GoToAngle_label = customtkinter.CTkLabel(frame, text="servo0")
        self.servo0GoToAngle_label.grid(row=0, column=1, padx=20, pady=(10, 0))
        self.servo0GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo0GoToAngle_entry.grid(row=1, column=1, sticky="we")
        self.servo0GoToAngle_entry.bind(
            "<Return>", self.servo0GoToAngle_entry_return_key_event
        )
        self.servo0_open_label = customtkinter.CTkLabel(frame)
        self.servo0_open_label.grid(row=1, column=2, padx=20, pady=(10, 0))
        self.servo0_close_label = customtkinter.CTkLabel(frame)
        self.servo0_close_label.grid(row=1, column=0, padx=20, pady=(10, 0))

        # shrimp clamp
        self.servo1GoToAngle_label = customtkinter.CTkLabel(frame, text="servo1")
        self.servo1GoToAngle_label.grid(row=2, column=1, padx=20, pady=(10, 0))
        self.servo1_open_label = customtkinter.CTkLabel(frame)
        self.servo1_open_label.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.servo1_close_label = customtkinter.CTkLabel(frame)
        self.servo1_close_label.grid(row=3, column=2, padx=20, pady=(10, 0))
        self.servo1GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo1GoToAngle_entry.grid(row=3, column=1, sticky="we")
        self.servo1GoToAngle_entry.bind(
            "<Return>", self.servo1GoToAngle_entry_return_key_event
        )

        # head cutter
        self.servo2GoToAngle_label = customtkinter.CTkLabel(frame, text="servo2")
        self.servo2GoToAngle_label.grid(row=4, column=1, padx=20, pady=(10, 0))
        self.servo2_open_label = customtkinter.CTkLabel(frame)
        self.servo2_open_label.grid(row=5, column=2, padx=20, pady=(10, 0))
        self.servo2_close_labell = customtkinter.CTkLabel(frame)
        self.servo2_close_labell.grid(row=5, column=0, padx=20, pady=(10, 0))

        self.servo2GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo2GoToAngle_entry.grid(row=5, column=1, sticky="we")
        self.servo2GoToAngle_entry.bind(
            "<Return>", self.servo2GoToAngle_entry_return_key_event
        )
        #  suction cleaning
        self.servo3GoToAngle_label = customtkinter.CTkLabel(frame, text="servo3")
        self.servo3GoToAngle_label.grid(row=6, column=1, padx=20, pady=(10, 0))
        self.servo3_open_label = customtkinter.CTkLabel(frame)
        self.servo3_open_label.grid(row=7, column=2, padx=20, pady=(10, 0))
        self.servo3_close_label = customtkinter.CTkLabel(frame)
        self.servo3_close_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.servo3GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo3GoToAngle_entry.grid(row=7, column=1, sticky="we")
        self.servo3GoToAngle_entry.bind(
            "<Return>", self.servo3GoToAngle_entry_return_key_event
        )
        # servo4
        self.servo4GoToAngle_label = customtkinter.CTkLabel(frame, text="servo4")
        self.servo4GoToAngle_label.grid(row=8, column=1, padx=20, pady=(10, 0))
        self.servo4_open_label = customtkinter.CTkLabel(frame)
        self.servo4_open_label.grid(row=9, column=2, padx=20, pady=(10, 0))
        self.servo4_close_label = customtkinter.CTkLabel(frame)
        self.servo4_close_label.grid(row=9, column=0, padx=20, pady=(10, 0))
        self.servo4GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo4GoToAngle_entry.grid(row=9, column=1, sticky="we")
        self.servo4GoToAngle_entry.bind(
            "<Return>", self.servo4GoToAngle_entry_return_key_event
        )
        # servo5
        self.servo5GoToAngle_label = customtkinter.CTkLabel(frame, text="servo5")
        self.servo5GoToAngle_label.grid(row=10, column=1, padx=20, pady=(10, 0))
        self.servo5_open_label = customtkinter.CTkLabel(frame)
        self.servo5_open_label.grid(row=11, column=2, padx=20, pady=(10, 0))
        self.servo5_close_label = customtkinter.CTkLabel(frame)
        self.servo5_close_label.grid(row=11, column=0, padx=20, pady=(10, 0))
        self.servo5GoToAngle_entry = customtkinter.CTkEntry(
            frame, placeholder_text="angle"
        )
        self.servo5GoToAngle_entry.grid(row=11, column=1, sticky="we")
        self.servo5GoToAngle_entry.bind(
            "<Return>", self.servo5GoToAngle_entry_return_key_event
        )

    def boolFrame(self, frame):
        self.relay42_btn = customtkinter.CTkButton(frame, text="42", state="disabled")
        self.relay42_btn.grid(row=0, column=0, padx=20, pady=(10, 0))
        self.relay44_btn = customtkinter.CTkButton(frame, text="44", state="disabled")
        self.relay44_btn.grid(row=0, column=1, padx=20, pady=(10, 0))
        self.relay46_btn = customtkinter.CTkButton(frame, text="46", state="disabled")
        self.relay46_btn.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.relay48_btn = customtkinter.CTkButton(frame, text="48", state="disabled")
        self.relay48_btn.grid(row=1, column=1, padx=20, pady=(10, 0))
        self.psw_btn = customtkinter.CTkButton(frame, text="psw", state="disabled")
        self.psw_btn.grid(row=2, column=0, padx=20, pady=(10, 0))
        self.laserAnalog_btn = customtkinter.CTkButton(
            frame, text="0", state="disabled"
        )
        self.laserAnalog_btn.grid(row=2, column=1, padx=20, pady=(10, 0))
        self.ls_btn = customtkinter.CTkButton(frame, text="LS", state="disabled")
        self.ls_btn.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.state_text = customtkinter.CTkButton(frame, text="state", state="disabled")
        self.state_text.grid(row=4, column=0, padx=20, pady=(10, 0))
        self.LaserFeedback_btn = customtkinter.CTkButton(frame, text="LaserFeedback", command=lambda: self.state_callback("LaserFeedback"))
        self.LaserFeedback_btn.grid(row=4, column=1, padx=20, pady=(10, 0))

    def loopFrame(self, frame):
        self.loopStates = []
        self.loop_frame_label = customtkinter.CTkLabel(frame, text="loop")
        self.loop_frame_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")

        self.homing_arm_btn = customtkinter.CTkButton(
            frame,
            text="Homing",
            anchor="s",
            command=lambda: self.state_callback("ArmReadyForHoming"),
        )
        self.homing_arm_btn.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.loopStates.append(self.homing_arm_btn)

        self.start_position_btn = customtkinter.CTkButton(
            frame,
            text="Start",
            anchor="s",
            command=lambda: self.state_callback("prepareStart"),
        )
        self.start_position_btn.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.loopStates.append(self.start_position_btn)
        
        self.cut_and_clean_btn = customtkinter.CTkButton(
            frame, text="End", anchor="s", command=lambda: self.state_callback("End")
        )
        self.cut_and_clean_btn.grid(row=8, column=0, padx=20, pady=(10, 0))
        self.loopStates.append(self.cut_and_clean_btn)
        
        self.pause_btn = customtkinter.CTkButton(
            frame,
            text="pause",            
            command=lambda: self.state_callback("pause"),
        )
        self.pause_btn.grid(row=9, column=0, padx=0, pady=(10, 0))
        self.continue_btn = customtkinter.CTkButton(
            frame,
            text="continue",    
            command=lambda: self.state_callback("continue"),
        )
        self.continue_btn.grid(row=10, column=0, padx=0, pady=(10, 0))

    def flowFrame(self, frame):
        self.ServoSetup = customtkinter.CTkLabel(
            frame, text="servo setup (connect to Servo5 cable)"
        )
        self.ServoSetup.grid(row=0, column=1, padx=20, pady=(10, 0))

        self.progressbar = customtkinter.CTkProgressBar(frame)
        self.progressbar.grid(row=1, column=1, padx=20, pady=(10, 0))

        self.slider = customtkinter.CTkSlider(
            frame, command=self.slider_callback, from_=0, to=270
        )
        self.slider.grid(row=2, column=1, padx=20, pady=(10, 0))
        self.slider.set(135)

        self.ServoSetupClose = customtkinter.CTkLabel(frame, text="0")
        self.ServoSetupClose.grid(row=2, column=0, padx=20, pady=(10, 0))

        self.ServoSetupOpen = customtkinter.CTkLabel(frame, text="270")
        self.ServoSetupOpen.grid(row=2, column=2, padx=20, pady=(10, 0))
        
        self.LDLabel = customtkinter.CTkLabel(
            frame, text="LD go to:"
        )
        self.LDLabel.grid(row=3, column=1, padx=20, pady=(10, 0))
        
        self.LDGoToEntry=customtkinter.CTkEntry(
            frame, placeholder_text="Position"
        )
        self.LDGoToEntry.grid(row=4, column=1, sticky="we")
        self.LDGoToEntry.bind(
            "<Return>", self.LDGoToEntry_return_key_event
        )
        
        self.LDMOT = customtkinter.CTkLabel(frame, text="-21500")
        self.LDMOT.grid(row=4, column=0, padx=20, pady=(10, 0))

        self.LDPOT = customtkinter.CTkLabel(frame, text="28000")
        self.LDPOT.grid(row=4, column=2, padx=20, pady=(10, 0))

    def slider_callback(self, value):
        value = max(min(value, 270), 0)
        self.progressbar.set(value / 270)
        self.state_callback("direct, 5, " + str(value))

    def positionFrame(self, frame):
        # arm
        self.servo0_label = customtkinter.CTkLabel(
            frame, text="arm", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.servo0_label.grid(row=2, column=0, padx=20, pady=20, sticky="nsew")
        self.servo0_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="pickup",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 0, " + self.config["servo0"]["pos1_angle"]
            ),
        )
        self.servo0_Pos1_btn.grid(row=3, column=0, padx=20, pady=(10, 0))
        self.servo0_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="traverse",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 0, " + self.config["servo0"]["pos2_angle"]
            ),
        )
        self.servo0_Pos2_btn.grid(row=3, column=1, padx=20, pady=(10, 0))
        self.servo0_Pos3_btn = customtkinter.CTkButton(
            frame,
            # ~ text="camera",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 0, " + self.config["servo0"]["pos3_angle"]
            ),
        )
        self.servo0_Pos3_btn.grid(row=4, column=0, padx=20, pady=(10, 0))
        self.servo0_Pos4_btn = customtkinter.CTkButton(
            frame,
            # ~ text="cut",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 0, " + self.config["servo0"]["pos4_angle"]
            ),
        )
        self.servo0_Pos4_btn.grid(row=4, column=1, padx=20, pady=(10, 0))

        # shrimp clamp
        self.servo1_label = customtkinter.CTkLabel(
            frame,
            text="shrimp clamp",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.servo1_label.grid(row=0, column=0, padx=20, pady=20, sticky="nsew")
        self.servo1_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Close",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 1, " + self.config["servo1"]["pos1_angle"]
            ),
        )
        self.servo1_Pos1_btn.grid(row=1, column=0, padx=20, pady=(10, 0))
        self.servo1_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 1, " + self.config["servo1"]["pos2_angle"]
            ),
        )
        self.servo1_Pos2_btn.grid(row=1, column=1, padx=20, pady=(10, 0))
        # head cutter
        self.servo2_label = customtkinter.CTkLabel(
            frame,
            text="nose cutter",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.servo2_label.grid(row=6, column=0, padx=20, pady=20, sticky="nsew")
        self.servo2_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="fully Open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 2, " + self.config["servo2"]["pos1_angle"]
            ),
        )
        self.servo2_Pos1_btn.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.servo2_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 2, " + self.config["servo2"]["pos2_angle"]
            ),
        )
        self.servo2_Pos2_btn.grid(row=7, column=1, padx=20, pady=(10, 0))
        self.servo2_Pos3_btn = customtkinter.CTkButton(
            frame,
            # ~ text="grip",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 2, " + self.config["servo2"]["pos3_angle"]
            ),
        )
        self.servo2_Pos3_btn.grid(row=8, column=0, padx=20, pady=(10, 0))
        self.servo2_Pos4_btn = customtkinter.CTkButton(
            frame,
            # ~ text="nose Cut",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 2, " + self.config["servo2"]["pos4_angle"]
            ),
        )
        self.servo2_Pos4_btn.grid(row=8, column=1, padx=20, pady=(10, 0))
        #  suction cleaning
        self.servo3_label = customtkinter.CTkLabel(
            frame,
            text="suction cleaning",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.servo3_label.grid(row=9, column=0, padx=20, pady=20, sticky="nsew")
        self.servo3_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Pos 1",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 3, " + self.config["servo3"]["pos1_angle"]
            ),
        )
        self.servo3_Pos1_btn.grid(row=10, column=0, padx=20, pady=(10, 0))
        self.servo3_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Pos 2",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 3, " + self.config["servo3"]["pos2_angle"]
            ),
        )
        self.servo3_Pos2_btn.grid(row=10, column=1, padx=20, pady=(10, 0))
        self.servo3_Pos3_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Pos 3",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 3, " + self.config["servo3"]["pos3_angle"]
            ),
        )
        self.servo3_Pos3_btn.grid(row=11, column=0, padx=20, pady=(10, 0))
        self.servo3_Pos4_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Pos 4",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 3, " + self.config["servo3"]["pos4_angle"]
            ),
        )
        self.servo3_Pos4_btn.grid(row=11, column=1, padx=20, pady=(10, 0))

        # head cutter 2
        self.servo4_label = customtkinter.CTkLabel(
            frame,
            text="nose cutter 2",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.servo4_label.grid(row=12, column=0, padx=20, pady=20, sticky="nsew")
        self.servo4_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="fully Open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 4, " + self.config["servo4"]["pos1_angle"]
            ),
        )
        self.servo4_Pos1_btn.grid(row=13, column=0, padx=20, pady=(10, 0))
        self.servo4_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 4, " + self.config["servo4"]["pos2_angle"]
            ),
        )
        self.servo4_Pos2_btn.grid(row=13, column=1, padx=20, pady=(10, 0))
        self.servo4_Pos3_btn = customtkinter.CTkButton(
            frame,
            # ~ text="grip",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 4, " + self.config["servo4"]["pos3_angle"]
            ),
        )
        self.servo4_Pos3_btn.grid(row=14, column=0, padx=20, pady=(10, 0))
        self.servo4_Pos4_btn = customtkinter.CTkButton(
            frame,
            # ~ text="nose Cut",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 4, " + self.config["servo4"]["pos4_angle"]
            ),
        )
        self.servo4_Pos4_btn.grid(row=14, column=1, padx=20, pady=(10, 0))

        # servo 5
        self.servo5_label = customtkinter.CTkLabel(
            frame, text="Rotator", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.servo5_label.grid(row=15, column=0, padx=20, pady=20, sticky="nsew")
        self.servo5_Pos1_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Open",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 5, " + self.config["servo5"]["pos1_angle"]
            ),
        )
        self.servo5_Pos1_btn.grid(row=16, column=0, padx=20, pady=(10, 0))
        self.servo5_Pos2_btn = customtkinter.CTkButton(
            frame,
            # ~ text="Close",
            anchor="s",
            command=lambda: self.state_callback(
                "direct, 5, " + self.config["servo5"]["pos2_angle"]
            ),
        )
        self.servo5_Pos2_btn.grid(row=16, column=1, padx=20, pady=(10, 0))

        # relay 42
        self.relay42_label = customtkinter.CTkLabel(
            frame, text="relay 42", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.relay42_label.grid(row=17, column=0, padx=20, pady=20, sticky="nsew")
        self.relay42_On_btn = customtkinter.CTkButton(
            frame,
            text="relay On",
            anchor="s",
            command=lambda: self.state_callback("direct, 42, 1"),
        )
        self.relay42_On_btn.grid(row=18, column=0, padx=20, pady=(10, 0))
        self.relay42_Off_btn = customtkinter.CTkButton(
            frame,
            text="relay Off",
            anchor="s",
            command=lambda: self.state_callback("direct, 42, 0"),
        )
        self.relay42_Off_btn.grid(row=18, column=1, padx=20, pady=(10, 0))

        # relay 44
        self.relay44_label = customtkinter.CTkLabel(
            frame, text="relay 44", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.relay44_label.grid(row=19, column=0, padx=20, pady=20, sticky="nsew")
        self.relay44_On_btn = customtkinter.CTkButton(
            frame,
            text="relay On",
            anchor="s",
            command=lambda: self.state_callback("direct, 44, 1"),
        )
        self.relay44_On_btn.grid(row=20, column=0, padx=20, pady=(10, 0))
        self.relay44_Off_btn = customtkinter.CTkButton(
            frame,
            text="relay Off",
            anchor="s",
            command=lambda: self.state_callback("direct, 44, 0"),
        )
        self.relay44_Off_btn.grid(row=20, column=1, padx=20, pady=(10, 0))

        # relay 46
        self.relay46_label = customtkinter.CTkLabel(
            frame, text="relay 46", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.relay46_label.grid(row=21, column=0, padx=20, pady=20, sticky="nsew")
        self.relay46_On_btn = customtkinter.CTkButton(
            frame,
            text="relay On",
            anchor="s",
            command=lambda: self.state_callback("direct, 46, 1"),
        )
        self.relay46_On_btn.grid(row=22, column=0, padx=20, pady=(10, 0))
        self.relay46_Off_btn = customtkinter.CTkButton(
            frame,
            text="relay Off",
            anchor="s",
            command=lambda: self.state_callback("direct, 46, 0"),
        )
        self.relay46_Off_btn.grid(row=22, column=1, padx=20, pady=(10, 0))

        # relay 48
        self.relay48_label = customtkinter.CTkLabel(
            frame, text="relay 48", font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.relay48_label.grid(row=23, column=0, padx=20, pady=20, sticky="nsew")
        self.relay48_On_btn = customtkinter.CTkButton(
            frame,
            text="relay On",
            anchor="s",
            command=lambda: self.state_callback("direct, 48, 1"),
        )
        self.relay48_On_btn.grid(row=24, column=0, padx=20, pady=(10, 0))
        self.relay48_Off_btn = customtkinter.CTkButton(
            frame,
            text="relay Off",
            anchor="s",
            command=lambda: self.state_callback("direct, 48, 0"),
        )
        self.relay48_Off_btn.grid(row=24, column=1, padx=20, pady=(10, 0))

        self.LD_label = customtkinter.CTkLabel(
            frame,
            text="Linear Drive",
            font=customtkinter.CTkFont(size=20, weight="bold"),
        )
        self.LD_label.grid(row=25, column=0, padx=20, pady=20, sticky="nsew")
        self.LD_home_btn = customtkinter.CTkButton(
            frame,
            text="Homing",
            anchor="s",
            command=lambda: self.state_callback("homing"),
        )
        self.LD_home_btn.grid(row=26, column=0, padx=20, pady=(10, 0))

    def updateGUItoCurrentState(self):
        if "arduino" in self.currentState:
            if self.currentState["arduino"]:
                self.arduino_btn.configure(fg_color="green")
            else:
                self.arduino_btn.configure(fg_color="red")
        if "LD" in self.currentState:
            if self.currentState["LD"]:
                self.ld_btn.configure(fg_color="green")
            else:
                self.ld_btn.configure(fg_color="red")
        if "0" in self.currentState:
            self.servo0GoToAngle_label.configure(
                text="servo0 " + str(self.currentState["0"])
            )
        if "1" in self.currentState:
            self.servo1GoToAngle_label.configure(
                text="servo1 " + str(self.currentState["1"])
            )
        if "2" in self.currentState:
            self.servo2GoToAngle_label.configure(
                text="servo2 " + str(self.currentState["2"])
            )
        if "3" in self.currentState:
            self.servo3GoToAngle_label.configure(
                text="servo3 " + str(self.currentState["3"])
            )
        if "4" in self.currentState:
            self.servo4GoToAngle_label.configure(
                text="servo4 " + str(self.currentState["4"])
            )
        if "5" in self.currentState:
            self.servo5GoToAngle_label.configure(
                text="servo5 " + str(self.currentState["5"])
            )
        if "42" in self.currentState:
            if self.currentState["42"]:
                self.relay42_btn.configure(fg_color="green")
            else:
                self.relay42_btn.configure(fg_color="red")
        if "44" in self.currentState:
            if self.currentState["44"]:
                self.relay44_btn.configure(fg_color="green")
            else:
                self.relay44_btn.configure(fg_color="red")
        if "46" in self.currentState:
            if self.currentState["46"]:
                self.relay46_btn.configure(fg_color="green")
            else:
                self.relay46_btn.configure(fg_color="red")
        if "48" in self.currentState:
            if self.currentState["48"]:
                self.relay48_btn.configure(fg_color="green")
            else:
                self.relay48_btn.configure(fg_color="red")
        if "psw" in self.currentState:
            if self.currentState["psw"]:
                self.psw_btn.configure(fg_color="red")
            else:
                self.psw_btn.configure(fg_color="green")
        if "LaserAnalog" in self.currentState:
            self.laserAnalog_btn.configure(text=str(self.currentState["LaserAnalog"]))
        if "Mode" in self.currentState:
            self.Flow_label.configure(text=str(self.currentState["Mode"]))
        if "EmergencyButton" in self.currentState:
            if self.currentState["EmergencyButton"]:
                self.EmergencyButton_btn.configure(fg_color="red")
            else:
                self.EmergencyButton_btn.configure(fg_color="green")
        if "ls" in self.currentState:
            if self.currentState["ls"]:
                self.ls_btn.configure(fg_color="green")
            else:
                self.ls_btn.configure(fg_color="red")
        if "ProcessState" in self.currentState:
            self.state_text.configure(text=self.currentState["ProcessState"])
        if "LaserFeedback" in self.currentState:
            self.LaserFeedback_btn.configure(text="LaserFeedback" + str(self.currentState["LaserFeedback"]))
            

    def state_callback(self, text):
        if "[" in text:
            self.statemachine.requestState(text[: text.index("[")].strip())
        else:
            self.statemachine.requestState(text.strip())

    def servo0GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo0GoToAngle_entry.get().strip())
            self.state_callback("direct, 0, " + str(angle))
        except:
            print("NaN", self.servo0GoToAngle_entry.get().strip())

    def servo1GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo1GoToAngle_entry.get().strip())
            self.state_callback("direct, 1, " + str(angle))
        except:
            print("NaN")

    def servo2GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo2GoToAngle_entry.get().strip())
            self.state_callback("direct, 2, " + str(angle))
        except:
            print("NaN")

    def servo3GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo3GoToAngle_entry.get().strip())
            self.state_callback("direct, 3, " + str(angle))
        except:
            print("NaN")

    def servo4GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo4GoToAngle_entry.get().strip())
            self.state_callback("direct, 4, " + str(angle))
        except:
            print("NaN")

    def servo5GoToAngle_entry_return_key_event(self, event):
        try:
            angle = float(self.servo5GoToAngle_entry.get().strip())
            self.state_callback("direct, 5, " + str(angle))
        except:
            print("NaN")
    
    def LDGoToEntry_return_key_event(self, event):
        try:            
            pos=int(self.LDGoToEntry.get().strip())
            print("LDGOTO:",pos) 
            self.state_callback("LDGoTo, " + str(pos))
        except:
            print("NaN")

if __name__ == "__main__":
    global app
    app = App()
    app.mainloop()

