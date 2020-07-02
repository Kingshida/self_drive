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
            # 固定速度             编码器线数               电机圈数值            系统状态寄存器：
            'fix_speed': 0x4420,    # 写电机速度 
            'cur_speed': 0x42d1,    # 当前电机速度
            'rec_line_num': 0x42ea, # 编码器线数
            'position': 0x42ff,     # 电机圈数值
            'sys_status': 0x465d,   # 系统状态寄存器，bit17参数保存
            'enable': 0x4657,       # 电机使能
            'id':0x466c,            # 电机通信地址
            'mode':0x465a,          # 控制模式选择，1力， 2速度， 3位置
            'err':0x466e,           # 系统错误代码
            'I':0x42cc,             # 电机IQ电流值0.1A
            'U':0x426b,             # 电机电压值
            'nominal_speed':0x42e8, # 电机额定速度0.1r/min
        }
        self._max_speed = 3000
        self._enable_status = False
        self._id = id
        self._motor = minimalmodbus.Instrument(port, id, debug=False    )
        self._motor.serial.baudrate = 9600
        self.ini_motor()

    def ini_motor(self):
        try:
            print('init motoring')
            self._motor.address = 0
            self._id = self._motor.read_register(self._addr_data['id'])
            self._motor.address = self._id
            self._max_speed = int(self._motor.read_register(self._addr_data['nominal_speed']))
            self._enable_status = self.enable()
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
            self._motor.write_register(self._addr_data['enable'], 1)
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
            self._motor.write_register(self._addr_data['enable'], 0)
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
                # print('-----------------')
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
            if speed > 1:
                speed = 1
            elif speed < -1:
                speed = -1
            speed = speed * self._max_speed * 10
            speed = int(speed)
            # 速度不为零
            if self._enable_status:
                speed = speed & 0xffffffff
                self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
            else:
                self.enable()
                speed = speed & 0xffffffff
                self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
            '''
            if speed != 0:
                # 速度不为零
                if self._enable_status:
                    speed = speed & 0xffffffff
                    self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
                else:
                    self.enable()
                    speed = speed & 0xffffffff
                    self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
            else:
                # 速度为零，先写速度0，再取消使能
                speed = speed & 0xffffffff
                self._motor.write_long(self._addr_data['fix_speed'], speed, byteorder=minimalmodbus.BYTEORDER_LITTLE_SWAP)
                time.sleep(0.2)
                self.disable()
            '''
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
            u = self._motor.read_register(self._addr_data['U']) / 100.0*2
            i = self._motor.read_register(self._addr_data['I']) / 100.0
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
            # print("err code", err)
            return err
        except BaseException as e:
            # print e
            # self.ini_motor()
            return -1



if __name__ == '__main__':
    wheel = Motor(01, '/dev/ttyS2')
    print "####enable############"
    print wheel.ini_motor()
    print "####enable############"
    print "####read_speed############"
    print wheel.read_speed()
