import configparser
import os


class configManager:
    def __init__(self):
        self.config = None

    def readConfig(self):
        self.config = configparser.ConfigParser()
        # self.config.read('config.ini')
        # self.config.read('D:\Github\Conveyor\ShrimpCutter3000\config.ini')
        cwd = os.path.dirname(os.path.abspath(__file__))
        self.config.read(cwd + "/config.ini")
        # print(self.config["arm_servo"]["min_angle"])
        return self.config


def main():
    c = configManager()
    # c.writeConfig()
    c.readConfig()


if __name__ == "__main__":
    main()
