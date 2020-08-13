# _*_ coding: UTF-8 _*_

import time
import minimalmodbus
import serial


class Motor:
    """
    驱动轮类模块
    """
    def __init__(self, dev_id, port):
        """
        初始化函数
        :param id: 电机从地址id
        :param port: 电机控制端口号
        """
        '''
        # 系统状态寄存器地址0x465d: 说明
        '''
        self._addr_data = {
            '485_en':0x00,          # 485使能
            'fix_speed': 0x02,      # 写电机速度 
            'cur_speed': 0x10,      # 当前电机速度
            'enable': 0x01,         # 电机使能
            'err':0x0e,             # 系统错误代码
            'I':0x0f,               # 电机IQ电流值0.1A
            'U':0x11,               # 电机电压值
            'zero': 0x19,           # 自动找原点寄存器地址
            'pos_l': 0x16,          # 绝对位置低16位, 只读
            'pos_h': 0x17,          # 绝对位置高16位，只读
            'pu':0x000c,            # 相对位置寄存器低16位，只写，32位
            'dir':0x09,             # 极性
            'acc': 0x03,            # acc r/min/s
        }
        self._max_speed = 3000
        self._enable_status = False
        self._id = dev_id
        self._motor = minimalmodbus.Instrument(port, self._id, debug=False)
        self._motor.serial.baudrate = 19200
        self.ini_motor()

    def ini_motor(self):
        try:
            print('init motoring')
            self._motor.write_register(self._addr_data['485_en'], 1, functioncode=6)
            self._motor.write_register(self._addr_data['acc'], 6000, functioncode=6)
            print('----------')
            self._enable_status = self.enable()
            self.write_speed(0)
            return True
        except BaseException as e:
            # print e
            time.sleep(2)
            return False

    def enable(self):
        """
        电机使能
        :return:
        """
        try:
            self._motor.write_register(self._addr_data['enable'], 1, functioncode=6)
            self._enable_status = True
            return True
        except BaseException as e:
            # print e
            return False

    def go_zero(self):
        """
        回原点
        :return:
        """
        try:
            self._motor.write_register(self._addr_data['485_en'], 1, functioncode=6)
            self.write_speed(0.1)
            self._motor.write_register(self._addr_data['dir'], 1, functioncode=6)
            self._motor.write_register(self._addr_data['zero'], 1, functioncode=6)
            while True:
                flag = self._motor.read_register(self._addr_data['485_en'])
                if flag == 0:
                    break
                time.sleep(1)
            self._motor.write_register(self._addr_data['485_en'], 1, functioncode=6)
            return True
        except BaseException as e:
            print e
            return False

    def go_abpos(self, pos):
        """
        到绝对位置
        :return:
        """
        try:
            if pos == 0:
                pos = 1
            abs_cmd = [self._id, 0x7b]
            abs_cmd.append(pos >> 24 & 0xff)
            abs_cmd.append(pos >> 16 & 0xff)
            abs_cmd.append(pos >> 8 & 0xff)
            abs_cmd.append(pos & 0xff)
            self.crc_calc(abs_cmd)
            self._motor.serial.write(abs_cmd)
            return True
        except BaseException as e:
            print e
            return False

    def crc_calc(self, data):
        crc = 0xffff
        for i  in data:
            crc ^= i
            for j in range(8):
                if (crc & 1):
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        data.append(crc & 0xff)
        data.append(crc >>8 & 0xff)

    def disable(self):
        """
        电机取消使能
        :return:
        """
        try:
            self._motor.write_register(self._addr_data['enable'], 0, functioncode=6)
            self._enable_status = False
            return True
        except BaseException as e:
            # print e
            return False

    def read_speed(self):
        """
        # 读速度，单位:r/min
        :return: speed
        """
        try:
            speed = self._motor.read_register(self._addr_data['cur_speed'])
            flag = (speed&0b1000000000000000)>>15       # < 0
            if flag:
                print('-----------------')
                speed=-(0xffff-speed)
            speed /=10.0
            return round(speed)
        except BaseException as e:
            # print e
            # self.ini_motor()
            return -1

    def write_speed(self, speed):
        """
        写速度：单位0.1r/min
        :param speed: -1 ~ +1
        :return: bool
        """
        try:
            if speed > 1.0:
                speed = 1.0
            elif speed < -1:
                speed = -1
            speed = int(speed * self._max_speed)
            if self._enable_status:
                speed = speed & 0xffff
                # self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
                self._motor.write_register(self._addr_data['fix_speed'], speed, functioncode=6)
            else:
                self.enable()
                speed = speed & 0xffff
                self._motor.write_register(self._addr_data['fix_speed'], speed, functioncode=6)
                # self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
            return True
        except BaseException as e:
            print e
            self.ini_motor()
            return False


    def read_base_info(self):
        """
        # 读速度, 电压，电流
        :return: speed
        """
        speed = self.read_speed()
        u, i = self.read_u_i()
        err = self.read_err()
        data = [speed, u, i, err]
        return data

    def read_u_i(self):
        """
        # 读电压，电流，单位:V，A
        :return: u, i 
        """
        try:
            u = self._motor.read_register(self._addr_data['U']) / 327.0
            i = self._motor.read_register(self._addr_data['I'])
            u, i = round(u, 1), round(i, 1)
            return u, i
        except BaseException as e:
            # print e
            # self.ini_motor()
            return -1, -1

    def read_err(self):
        """
        # 读电压，电流，单位:V，A
        :return: u, i 
        """
        try:
            err = self._motor.read_register(self._addr_data['err'])
            print("err code", err)
            return err
        except BaseException as e:
            # print e
            # self.ini_motor()
            return -1

    def read_pos(self):
        """
        # 读电压，电流，单位:V，A
        :return: u, i 
        """
        try:
            pos = self._motor.read_long(self._addr_data['pos_l'], byteorder=3)
            return pos
        except BaseException as e:
            # print e
            # self.ini_motor()
            return None



if __name__ == '__main__':
    wheel = Motor(01, '/dev/ttyS5')
    print "####enable############"
    # print wheel.ini_motor()
    wheel.go_zero()
    wheel.write_speed(01.0)
    wheel.read_pos()
    wheel.go_abpos(int(32767*5.2))
    time.sleep(5)
    wheel.read_pos()
    '''
    wheel.go_abpos(0)
    time.sleep(2)
    wheel.read_pos()
    '''
    '''
    time.sleep(5)
    wheel.write_speed(0.01)
    time.sleep(5)
    print(wheel.read_base_info())
    wheel.write_speed(-0.01)
    time.sleep(5)
    print(wheel.read_base_info())
    wheel.write_speed(0.01)
    time.sleep(5)
    print(wheel.read_base_info())
    wheel.write_speed(-0.01)
    time.sleep(5)
    print(wheel.read_base_info())
    wheel.write_speed(-0.0)
    print "####enable############"
    print "####read_speed############"
    '''
