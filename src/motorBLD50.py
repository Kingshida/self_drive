# _*_ coding: UTF-8 _*_

import time
import minimalmodbus
import serial


class Motor:
    """
    驱动轮类模块, 蓝色驱动器
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
            'fix_speed': 0x0056,    # 电机速度
            '485_enable': 0x00b6,   # 485使能,1有效，2无效
            'enable': 0x0066,       # 电机使能,0停止，1正转，2反转，3刹车, 4刹车解除
            'speed':0x0056,         # 实际速度
            'u':0x00c8,             # 电压
            'i':0x00c6,             # 电流
        }
        self._max_speed = 2500
        self._id = dev_id
        self._motor = minimalmodbus.Instrument(port, self._id, debug=False)
        self._motor.serial.baudrate = 9600
        self.ini_motor()

    def ini_motor(self):
        try:
            print('init motoring')
            self._motor.write_register(self._addr_data['485_enable'], 1, functioncode=6)
            self._motor.write_register(self._addr_data['fix_speed'], 0, functioncode=6)
            self._motor.write_register(self._addr_data['enable'], 1, functioncode=6)
        except Exception as e:
            print(e)


    def write_speed(self, speed):
        """
        写速度：单位r/min
        :param speed: -1 ~ +1
        :return: bool
        """
        if speed > 1:
            speed = 1
        elif speed < -1:
            speed = -1
        speed = int(speed * self._max_speed)
        if speed >= 0:
            # 速度大于等于零
            try:
                self._motor.write_register(self._addr_data['fix_speed'], speed, functioncode=6)
                self._motor.write_register(self._addr_data['enable'], 1, functioncode=6)
            except:
                pass
        elif speed < 0:
            # 速度小于零
            try:
                speed = abs(speed) & 0xffff
                self._motor.write_register(self._addr_data['fix_speed'], speed, functioncode=6)
                self._motor.write_register(self._addr_data['enable'], 2, functioncode=6)
            except:
                pass
        return True


    def read_base_info(self):
        """
        # 读速度, 电压，电流
        :return: speed
        """
        data = [-1, -1, -1, -1]
        # '''
        data[0] = self.read_speed()
        data[1] = self._motor.read_register(self._addr_data['u'])
        data[2] = self._motor.read_register(self._addr_data['i'])
        # '''
        return data

    def read_speed(self):
        """
        # 读速度
        :return: speed
        """
        speed = self._motor.read_register(self._addr_data['speed'])
        return speed


if __name__ == '__main__':
    wheel = Motor(02, '/dev/ttyS2')
    print "####enable############"
    print wheel.ini_motor()
    print(wheel.write_speed(0.2))
    print(wheel.read_base_info())
    time.sleep(5)
    print(wheel.write_speed(-0.2))
    print(wheel.read_base_info())
    time.sleep(5)
    print(wheel.write_speed(0.0))
