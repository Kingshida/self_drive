# coding:utf-8
import serial
import time
import threading


class motor:
    def __init__(self, port = '/dev/ttyUSB1'):
        self.motor_ser = serial.Serial(port, 19200, timeout=0.5, write_timeout=0.5)
        # 分辨率 /deg
        self.reso = 91
        # 270度位置
        self.max_pos = self.reso * 270
        # 90度位置
        self.min_pos = self.reso * 90
        # enable 命令
        self.en_cmd = [0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x48, 0x0a]
        # max位置cmd
        self.max_cmd = [0x01, 0x7b, 0x00, 0x00, 0x60, 0x00, 0xcd, 0xc0]
        # min位置cmd
        self.min_cmd = [0x01, 0x7b, 0x00, 0x00, 0x20, 0x00, 0xfc, 0x00]
        # 读取位置cmd
        self.get_pos_cmd = [0x01, 0x03, 0x00, 0x16, 0x00, 0x01, 0x65, 0xce]
        # 主线程控制读取位置开关 
        self.flag = False
        # 子线程读取完成标志,读取完成为False
        self.pos_flag = True
        # < 0 cur_pos无效
        self.cur_pos = self.min_pos
        # 当前角度
        self.cur_rad = 0

        self.enabale()
        self.go_min()


    def enabale(self):
        self.motor_ser.reset_input_buffer()
        self.motor_ser.write(self.en_cmd)
        data = self.motor_ser.read(8)
        if len(data) == 8:
            data = [ord(i) for i in data]
            print(data)
            return True
        else:
            return False

    def go_max(self):
        self.motor_ser.reset_input_buffer()
        self.motor_ser.write(self.max_cmd)
        data = self.motor_ser.read(8)
        if len(data) == 8:
            data = [ord(i) for i in data]
            print(data)
            return True
        else:
            return False

    def go_min(self):
        self.motor_ser.reset_input_buffer()
        self.motor_ser.write(self.min_cmd)
        data = self.motor_ser.read(8)
        if len(data) == 8:
            data = [ord(i) for i in data]
            print(data)
            return True
        else:
            return False


    def read_pos(self):
        while True:
            if self.flag and self.pos_flag:
                try:
                    # pre_time = time.time()
                    self.motor_ser.reset_input_buffer()
                    self.motor_ser.write(self.get_pos_cmd)
                    data = self.motor_ser.read(7)
                    if len(data) == 7:
                        data = [ord(i) for i in data]
                        if data[0] == 1 and data[1] == 3:
                            self.cur_pos = (data[3] << 8) + data[4]
                            self.cur_rad = 3.1416 * self.cur_pos / self.reso / 180
                        else:
                            self.cur_pos = -1
                    else:
                        self.cur_pos = -1
                    # now = time.time()
                    # print('t1: %.6f, %.6f, %.6f'%(pre_time, now, now-pre_time))
                except:
                    pass
                self.pos_flag = False
            time.sleep(0.005)

    def main(self):
        time.sleep(5)
        t1 = threading.Thread(target=self.read_pos)
        t1.setDaemon(True)
        t1.start()

        self.flag = True
        go_flag = True
        while True:
            try:
                if abs(self.cur_pos - self.max_pos) < 20 and go_flag:
                    self.go_min()
                    go_flag = False
                if abs(self.cur_pos - self.min_pos) < 20 and go_flag:
                    self.go_max()
                    go_flag = False

                if self.cur_pos > self.min_pos + 30 and self.cur_pos < self.max_pos -30:
                    go_flag = True
                if not self.pos_flag:
                    self.flag = False
                    self.pos_flag = True
                    self.flag = True
            except:
                pass
            # time.sleep(1)


class ray:
    def __init__(self, port = '/dev/ttyS6'):
        self.ray_ser = serial.Serial(port, 9600, timeout=0.5, write_timeout=0.5)
        # 读取位置cmd
        self.get_dist_cmd = [0x01, 0x03, 0x00, 0x00, 0x00, 0x01, 0x84, 0x0a]
        # 主线程控制读取位置开关 
        self.flag = False
        # 子线程读取完成标志,读取完成为False
        self.ray_flag = True
        # < 0 cur_pos无效
        self.cur_dist = 0


    def read_dist(self):
        while True:
            if self.flag and self.ray_flag:
                try:
                    # pre_time = time.time()
                    self.ray_ser.reset_input_buffer()
                    self.ray_ser.write(self.get_dist_cmd)
                    data = self.ray_ser.read(7)
                    if len(data) == 7:
                        data = [ord(i) for i in data]
                        if data[0] == 1 and data[1] == 3:
                            self.cur_dist = (data[3] << 8) + data[4]
                        else:
                            self.cur_dist = -1
                    else:
                        self.cur_dist = -1
                    # now = time.time()
                    # print('t2: %.6f, %.6f, %.6f'%(pre_time, now, now-pre_time))
                except:
                    pass
                self.ray_flag = False

    def main(self):
        time.sleep(5)
        t1 = threading.Thread(target=self.read_dist)
        t1.setDaemon(True)
        t1.start()
        self.flag = True
        while True:
            try:
                if not self.ray_flag:
                    self.flag = False
                    self.ray_flag = True
                    self.flag = True
            except:
                pass
            # time.sleep(1)

class radar(motor, ray):
    def __init__(self):
        motor.__init__(self)
        ray.__init__(self)


    def main(self):
        t1 = threading.Thread(target=self.read_pos)
        t1.setDaemon(True)
        t1.start()

        t2 = threading.Thread(target=self.read_dist)
        t2.setDaemon(True)
        t2.start()

        self.flag = True
        go_flag = True
        while True:
            try:
                if abs(self.cur_pos - self.max_pos) < 20 and go_flag:
                    self.go_min()
                    go_flag = False
                if abs(self.cur_pos - self.min_pos) < 20 and go_flag:
                    self.go_max()
                    go_flag = False

                if self.cur_pos > self.min_pos + 30 and self.cur_pos < self.max_pos -30:
                    go_flag = True
                if not (self.pos_flag or self.ray_flag):
                    print(round(self.cur_rad, 4), self.cur_dist)
                    self.flag = False
                    self.pos_flag = True
                    self.ray_flag= True
                    self.flag = True
            except:
                pass



# dev = motor()
# dev = ray()
dev = radar()
dev.main()
