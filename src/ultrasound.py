# _*_ coding: UTF-8 _*_

import serial
import time
from redisHandler import redisHandler

class ultrasound(redisHandler):

    def __init__(self,port = None):
        # 超声波模块信息类
        if not port:
            self.port = '/dev/ttyS5'
        else:
            self.port = port
        # 数据读取频率
        self.hz = 0.8
        # 遇障碍物停止的最小距离
        self.min_dist = 500
        self.ser = serial.Serial(self.port, 9600, timeout=0.5, write_timeout=0.5)
        # 获取四个探头数据的命令
        self.send_cmd = [0x55, 0xAA, 0x01, 0x01, 0x01, 0x01]
        # 信息头
        self.head = [0x55, 0xAA, 0x01, 0x01]
        # 信息头之和
        self.sum_head=0
        for i in self.head: self.sum_head += i
        # redis
        redisHandler.__init__(self)

    def run(self):
        # 主运行方法
        self.pub_topics = ['move_base_in']
        pre_time = time.time()
        timeout = 1.0 / self.hz
        sleep_time = timeout / 5.0
        pub_data = {
                    'header': 'ob_invalid',
                    'data':''
                }
        while True:
            # 按一定频率hz发送数据至move base
            # 障碍物在避障范围之外
            now = time.time()
            if now - pre_time >= timeout:
                data = self.get_data()
                # print(data)
                if data:
                    is_ob = False
                    for i_data in data:
                        if i_data < self.min_dist and i_data != 0:
                            is_ob = True
                    if not is_ob:
                        self.pub_all(pub_data)
            else:
                time.sleep(sleep_time)

    def get_data(self):
        self.send_data()
        return self.read_data()

    def send_data(self):
        # self.ser.reset_input_buffer()
        if self.ser.is_open:
            self.ser.write(self.send_cmd)
        else:
            self.ser.open()
            self.ser.write(self.send_cmd)

    def read_data(self):
        # 读取四个超声波探头测得的数据
        data = []
        data_len = 9
        data_sum = self.sum_head
        timeout = 2.0
        pre_t = time.time()
        head_flag = False
        while True:
            now = time.time()
            if now - pre_t >= timeout:
                return None
            for i in self.head:
                # 读取数据头
                while not self.ser.in_waiting:
                    # 超时返回
                    now = time.time()
                    time.sleep(0.2)
                    if now - pre_t >= timeout:
                        return None
                head_byte = ord(self.ser.read())
                if head_byte == i:
                    head_flag = True
                else:
                    head_flag = False
                    continue
            if head_flag:
                for i in range(data_len):
                    # 读取四个超声波探头数据+checksum
                    while not self.ser.in_waiting:
                        # 超时返回
                        now = time.time()
                        if now - pre_t >= timeout:
                            return None
                    data_byte = ord(self.ser.read())
                    data.append(data_byte)
                    # 校验和
                    if i < data_len-1:
                        data_sum += data_byte
                    
            if len(data) == data_len and (data_sum & 0xff) == data[-1]:
                # 数据长度和校验和正确则输出数据
                real_data = []
                for i in range(0,data_len-1, 2):
                    real_data.append((data[i] << 8) + data[i+1])
                return real_data
            else:
                return None


if __name__ == '__main__':
    print('start:')
    dev = ultrasound('/dev/ttyS5')
    dev.run()
    while True:
        dev.get_data()
        time.sleep(0.5)
