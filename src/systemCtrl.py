#_*_ coding: utf-8 _*_

import os
# import pyttsx
# import time


class systemCtrl():

    def __init__(self):
        pass
        # self.voice = pyttsx.init()
        # self.say('robot on')

    def shut_down(self):
        # self.say('shutdown after 5 seconds')
        # time.sleep(5)
        os.system('init 0')

    def reboot(self):
        # self.say('restart after 6 seconds')
        # time.sleep(5)
        os.system('init 6')

    def say(self, data):
        pass
        # self.voice.say(data)
        # self.voice.runAndWait()


if __name__ == '__main__':
    voice = systemCtrl()
    # voice.say(123)
