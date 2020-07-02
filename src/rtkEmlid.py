#!/usr/bin/python
# coding:utf-8

import serial
import time


class rtk_dev(object):

    def __init__(self,port = '/dev/ttyACM0'):
        self.port = port
        self.bauterate = 9600
        self.bytesize = 8
        self.timeout = 1.2
        self.write_timeout = 0.2
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.bauterate, timeout=self.timeout, write_timeout=self.write_timeout)
        except Exception as e:
            print(e)

    def get_position(self):
        try:
            response = self.serial.read_until()
            response = response.split(' ')
            res_pos = [
                    response[3],
                    response[6],
                    response[9],
                    response[12],
                    response[17],
                    response[20],
                    response[23],
                    ]
            for i in range(len(res_pos)):
                if i == 3:
                    res_pos[i] = int(res_pos[i])
                    continue
                res_pos[i] = float(res_pos[i])
            return res_pos
        except Exception as e:
            return None



if __name__ == '__main__':
    print('start:')
    rtk = rtk_dev()
    while True:
        pos = rtk.get_position()
        if(pos):
            print(pos)
