from alpha5 import Alpha5Client

# import serial
from serial.tools import list_ports


class LD:
    def __init__(self):
        self.port = None
        self.shutdown = False
        self.connected = False
        self.scanForLinearDrive()

        if self.port is not None:
            self.alpha5 = Alpha5Client(self.port)
            self.alpha5.create_client()
            self.alpha5.connect()
            self.alpha5.set_show_query_response(True)

    def scanForLinearDrive(self) -> str:
        print("scanning for linear drive")
        VID = "10C4"
        PID = "EA60"
        device_list = list_ports.comports()
        if self.port == None:
            for device in device_list:
                if device.vid != None or device.pid != None:
                    if (
                        "{:04X}".format(device.vid) == VID
                        and "{:04X}".format(device.pid) == PID
                    ):
                        self.port = device.device
                        self.connected = True
                        break
                    self.port = None
                    self.connected = False

    def homing(self) -> None:
        if self.alpha5 is not None:
            print("Set [S-ON] to ON")
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=0, state="ON"
            )  # [S-ON]->CONT9

            print("Issue homing command")
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=2, state="ON"
            )  # [ORG]->CONT11

            print("Homing...")
            print("Wait for the operation to complete")
            # time.sleep(10)
            self.alpha5.wait_operation_complete(id=1)
            print("done!")

            print("Set [S-ON] to OFF")
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=0, state="OFF"
            )  # [S-ON]->CONT9
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=2, state="OFF"
            )  # [ORG]->CONT11
        else:
            print("No  Linear Drive attached")

    def immediateOperation(self, Units: int, Speed: int) -> None:
        if self.alpha5 is not None:

            # print('Set [S-ON] to ON')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=0, state="ON"
            )  # [S-ON]->CONT9

            # print('Send immediate operation setting')
            self.alpha5.send_immediate_operation_setting(id=1, units=Units, speed=Speed)
            # print('Set [START] to ON')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=1, state="ON"
            )  # [START]->CONT10
            # print('Set [START] to OFF')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=1, state="OFF"
            )  # [START]->CONT10

            # print('Wait for the operation to complete')
            self.alpha5.wait_operation_complete(id=1)

            # print('Set [S-ON] to OFF')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=0, state="OFF"
            )  # [S-ON]->CONT9
        else:
            print("No  Linear Drive attached")
            # time.sleep(1)

    def immediateOperationNonBlocking(self, Units: int, Speed: int) -> None:
        if self.alpha5 is not None:

            # print('Set [S-ON] to ON')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=0, state="ON"
            )  # [S-ON]->CONT9

            # print('Send immediate operation setting')
            self.alpha5.send_immediate_operation_setting(id=1, units=Units, speed=Speed)
            # print('Set [START] to ON')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=1, state="ON"
            )  # [START]->CONT10
            # print('Set [START] to OFF')
            self.alpha5.manipulate_virtualcont_bits(
                id=1, bit_index=1, state="OFF"
            )  # [START]->CONT10

            # print('Wait for the operation to complete')
            # self.alpha5.wait_operation_complete(id=1)

            # print('Set [S-ON] to OFF')
            # self.alpha5.manipulate_virtualcont_bits(id=1, bit_index=0, state='OFF') # [S-ON]->CONT9
        else:
            print("No  Linear Drive attached")
            # time.sleep(1)

    def Soff(self):
        self.alpha5.manipulate_virtualcont_bits(
            id=1, bit_index=0, state="OFF"
        )  # [S-ON]->CONT9

    def LDgetPosition(self) -> int:
        if self.alpha5 is not None:
            return self.alpha5.get_feedback_position(id=1)
        else:
            print("No  Linear Drive attached")
            return 0
