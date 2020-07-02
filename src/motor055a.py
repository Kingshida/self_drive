# _*_ coding: UTF-8 _*_

import time
import minimalmodbus
import serial


class Motor:
    """
    驱动轮类模块
    """
    def __init__(self, id, port):
        """
        初始化函数
        :param id: 电机从地址id
        :param port: 电机控制端口号
        """
        '''
        # 系统状态寄存器地址0x465d: 说明
        '''
        self._addr_data = {
            '485_en':0x00,        # 485使能
            'fix_speed': 0x02,    # 写电机速度 
            'cur_speed': 0x10,    # 当前电机速度
            'enable': 0x01,       # 电机使能
            'err':0x0e,           # 系统错误代码
            'I':0x0f,             # 电机IQ电流值0.1A
            'U':0x11,             # 电机电压值
        }
        self._max_speed = 3000
        self._enable_status = False
        self._id = id
        self._motor = minimalmodbus.Instrument(port, id, debug=False)
        self._motor.serial.baudrate = 19200
        self.ini_motor()

    def ini_motor(self):
        try:
            print('init motoring')
            self._motor.write_register(self._addr_data['485_en'], 1, functioncode=6)
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



if __name__ == '__main__':
    wheel = Motor(01, '/dev/ttyS3')
    print "####enable############"
    # print wheel.ini_motor()
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
