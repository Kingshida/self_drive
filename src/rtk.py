#coding=utf-8

import sys
import math
from math import sin, cos
import time
import serial
import threading
import numpy as np
from ntripClient import ntripClient
from redisHandler import redisHandler
from wgs84 import WGS84


class rtk(redisHandler):
    def init(self, port = '/dev/ttyS6', bps = 115200, timeout=2):
        self.pub_topics = ['rtk_out']
        self.base_lon = 121.24554536
        self.base_lat = 30.87471633
        self.base_h = 27.427
        self.wgs84 = WGS84(self.base_lon, self.base_lat, self.base_h)
        self.__ser = serial.Serial(port, bps, timeout=timeout)
        self.__position = {
                'p':[0,0],
                'angle': 0.0,
                'precision': 0.0,
                # 'angle_precision':0.0,
                'rtcm':'',
                'rtk_mod':0
                }
        self.start_sub()
        # ntrip 挂载获取rtcm消息
        # ntrip = ntripClient()
        ntrip = ntripClient(mount_pt='sweet_bds')
        self.__p_rtcm = threading.Thread(target=ntrip.run)
        self.__p_rtcm.setDaemon(True)
        self.__p_rtcm.start()


    def run(self):
        # rtk
        self.init()
        ser = self.__ser
        hz = 3  # 发送频率
        pre_time = time.time()
        while True:
            try:
                if ser.in_waiting:
                    #print('s')
                    data = ser.read_until('\n')
                    # print(data)
                    
                    # 报文处理
                    if 'GPGGA' in data or 'GNGGA' in data:
                        data = data.split(',')
                        # print(data)
                        self.__position['rtk_mod'] = int(data[6])
                        # rtk 差分数据源传入时间间隔
                        self.__position['rtcm'] = data[13]
                        # 经度dddmm.mmmmm转度
                        # 纬度ddmm.mmmmm转度
                        lon = float(data[4]) / 100.0
                        lat = float(data[2]) / 100.0
                        h = float(data[9])
                        lon = int(lon) + (lon - int(lon)) * 100 / 60.0
                        lat = int(lat) + (lat - int(lat)) * 100 / 60.0
                        # 经纬度转xyz
                        self.__position['p'] = self.wgs84.calc_xyz(lon, lat, h)
                        # 精度因子
                        self.__position['precision'] = float(data[8])
                        #　坐标转换-->基站坐标系
                        p = self.__position['p']
                        p = p + self.wgs84.translation
                        # print(p.shape, self.wgs84.rotation.shape)
                        self.__position['p'] =[round(v, 3) for v in np.dot(p, self.wgs84.rotation).tolist()]
                        # print(self.__position)
                    elif 'HEADINGA' in data:
                        # 联适rtk
                        data = data.split(';')[1]
                        data = data.split(',')
                        if not data[3]:
                            self.__position['rtk_mod'] = -1
                            continue
                        self.__position['angle'] = round(math.radians(360 - float(data[3])), 1)
                        # redis 发布
                        now = time.time()
                        if now - pre_time >= 1.0 / hz:
                            pre_time = now
                            pub_data = {
                                    'header': 'rtk_position',
                                    'data':self.__position
                                    }
                            self.pub_all(pub_data)
                        # self.__position['precision_angle'] = float(data[6])
                    elif 'GNHDT' in data:
                        # 梦芯rtk
                        data = data.split(',')
                        if not data[1]:
                            self.__position['rtk_mod'] = -1
                            continue
                        angle = (540 - float(data[1])) % 360
                        # '''
                        # 平移至车体中心
                        x = self.__position['p'][0]
                        y = self.__position['p'][1]
                        rx = -0.5
                        ry = 0.0
                        tx = cos(angle) * rx - sin(angle) * ry
                        ty = sin(angle) * rx + cos(angle) * ry
                        self.__position['p'][0] += tx
                        self.__position['p'][1] += ty
                        # '''
                        self.__position['angle'] = round(math.radians(angle), 3)
                        # redis 发布
                        pub_data = {
                                'header': 'rtk_position',
                                'data':self.__position
                                }
                        self.pub_all(pub_data)
                        # print(angle)
            except Exception as e:
                pass
                # print('rtk err')
                # print(e)





if __name__ == '__main__':
    rtk_rover = rtk()
    rtk_rover.run()
