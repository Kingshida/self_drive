# _*_ coding: UTF-8 _*_

import time
import minimalmodbus
import serial


class sensorLinear:
    """
    驱动轮类模块
    """
    def __init__(self, dev_id, port):
        """
        初始化函数
        :param id: 从地址id
        :param port: 控制端口号
        """
        '''
        # 系统状态寄存器地址: 说明
        '''
        self._addr_data = {
            'pos': 0x0001,          # 位置,两寄存器, 只读
            'id': 0x0020,           # id, 读写
            'baudrate': 0x0021,     # 01:4800, 02:9600, 03:19200, 04:38400
        }
        self._id = dev_id
        self.dev = minimalmodbus.Instrument(port, self._id, debug=False)
        self.dev.serial.baudrate = 19200

    def set_id(self, dev_id):
        """
        设置地址
        """
        try:
            self.dev.write_register(self._addr_data['id'], dev_id, functioncode=6)
            self._id = dev_id
            # self.dev = minimalmodbus.Instrument(port, self._id, debug=False)
            # self.dev.serial.baudrate = 9600
            return True
        except BaseException as e:
            print(e)
            return False

    def read_pos(self):
        """
        读位置
        """
        try:
            pos = self.dev.read_long(self._addr_data['pos'], byteorder=0) / 100.0
            return pos
        except BaseException as e:
            print(e)
            return 0



if __name__ == '__main__':
    dev = sensorLinear(03, '/dev/ttyS5')
    print "####enable############"
    # dev.set_id(3)
    dev.read_pos()
    time.sleep(2)
    dev.read_pos()
    time.sleep(2)
    dev.read_pos()
