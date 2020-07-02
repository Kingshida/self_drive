# _*_coding:utf-8_*_
import os
import requests
import threading
import cv2 as cv
import time
import numpy
import socket
import json
import struct
import sys
from redisHandler import redisHandler
from config import config

reload(sys)
sys.setdefaultencoding('utf8')


class pushImg(redisHandler):
    # 推流类
    def init(self):
        # 初始化
        self.conf = config()
        self.key = self.conf.robot['key']
        self.sub_topics = ['pushimg_in']
        self.__run_flag = False
        self.time_fps = 1/12.0  # 推流频率
        ip = '117.184.129.18'
        self.address_server = (ip, 8020)  # 服务器地址
        self.__push_time = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.__img_quality = [(240,320), (240, 320)]
        self.__img_threshold = 0.05     # 摄像头推流质量控制阈值
        self.__img_quality_index = 1
        # 获取摄像头列表
        self.__caps, self.__cap_count = self.__get_caps()
        self.__cap_index = 0
        self.cap_next = None
        if self.__cap_count > 0:
            self.cap = self.__caps[self.__cap_index]
            if self.__cap_count > 1:
                self.cap_next = self.__caps[1]
        else:
            self.cap = None
        # 认证头
        self.header = {
                'key':self.key,
                'method':'push_img',
                'size':0
                }
        self.sock = self.__conn_sock()
        # 启动订阅主题
        self.start_sub()
        t = threading.Thread(target=self.next_cap)
        t.setDaemon(True)
        # t.start()

    def next_cap(self):
        if self.__cap_count <= 1:
            return
        while True:
            try:
                print('next')
                if self.cap_next:
                    r, f = self.cap_next.read()
                else:
                    return
            except:
                pass

    def __conn_sock(self):
        while True:
            # connect to tcp server
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect(self.address_server)
                print('connected to server')
                break
            except Exception as e:
                print(e)
                print('connecting to server')
                time.sleep(2)
        return sock



    def __get_caps(self):
        # 初始化，获取摄像头数量
        caps = []
        os.system('arp -a > tmp.txt')
        with open('tmp.txt') as fp:
            for line in fp:
                s = line.find('(')
                e = line.find(')')
                try:
                    cam_ip = line[s+1:e]
                    print(cam_ip)
                
                    response = requests.get('http://%s/asp/mainview.asp'%cam_ip, timeout=1)
                    if response.ok:
                        cap_addr = "rtsp://admin:@" + cam_ip + ":554/h264/ch1/main/av_stream"
                        cap = cv.VideoCapture(cap_addr)
                        cap.set(cv.CAP_PROP_BUFFERSIZE, 2)
                        caps.append(cap)
                except Exception as e:
                    print(e)
        caps_count = len(caps)
        print('caps count is: %d'%caps_count)
        return caps, caps_count

    def __del__(self):
        print('del')
        self.release()

    def q_get(self):
        # 重写q_get方法
        try:
            data = self.q.get_nowait()
            if data:
                header = data['header']
                if header == 'camera_on':
                    # 切换摄像头
                    self.__run_flag = True
                    tmp_ind = self.__cap_index + 1
                    if tmp_ind >= self.__cap_count:
                        self.__cap_index = 0
                    else:
                        self.__cap_index = tmp_ind
                    if len(self.__caps) > 0:
                        self.cap = self.__caps[self.__cap_index]
                    ind_next = self.__cap_index + 1
                    if ind_next > self.__cap_count:
                        ind_next = 0
                    if self.__cap_count > 1:
                        self.cap_next = self.__caps[ind_next]
                    else:
                        self.cap_next = None
                elif header == 'camera_off':
                    self.__run_flag = False
                    # 初始化
        except Exception as e:
            # print(e)
            pass



    def run(self):
        # 推流主线程
        print('camera on')
        self.init()
        pre_time = time.time()
        if not self.cap:
            # 无摄像头，关闭线程
            return
        while True:
            try:
                self.q_get()
                now = time.time()
                sleep_time = self.time_fps - (now - pre_time)
                if sleep_time > 0:
                    time.sleep(sleep_time/3)
                ret, frame = self.cap.read()
                '''
                cv.imshow('cam', frame)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break
                # '''
                if sleep_time<=0 and self.__run_flag and ret:
                    # 发送视频
                    frame = cv.resize(frame, self.__img_quality[self.__img_quality_index])
                    encode_param = [int(cv.IMWRITE_JPEG_QUALITY), 25]
                    # img encode >> .jpg >> utf-8
                    img_encode = cv.imencode('.jpg', frame, encode_param)[1]
                    img_encode= img_encode.tostring()
                    dec_img = numpy.fromstring(img_encode, dtype='uint8')
                    dec_img = cv.imdecode(dec_img, 1)
                    t = time.time()
                    # step1 produce a header
                    self.header['size'] = len(img_encode)
                    header_bytes = json.dumps(self.header).encode('utf-8')
                    header = struct.pack('i', len(header_bytes))
                    # step2 send header size to server
                    self.sock.send(header)
                    # step3 send header to server
                    self.sock.send(header_bytes)
                    # send img
                    self.sock.send(img_encode)
                    # print(len(img_encode))
                    delta_time = time.time() - t
                    self.__img_quality_index = self.push_quality_index(delta_time)
                    # print(time.time()-now)
            except socket.error:
                self.sock = self.__conn_sock()
            except Exception as e:
                pass
        self.release()


    def release(self):
        #self.sock.shutdown(2)
        print('realse')
        try:
            self.__run_flag = False
            self.sock.close()
            for cap in self.caps:
                cap.release()
            cv.destroyAllWindows()
        except Exception as e:
            print(e)

    def push_quality_index(self, delta_time):
        sum_time = 0.0
        self.__push_time.pop(0)
        self.__push_time.append(delta_time)
        for item in self.__push_time:
            sum_time += item
        average_time = sum_time / len(self.__push_time)
        if average_time < self.__img_threshold / 2:
            return 0
        else:
            return 1

if __name__=='__main__':
    push_img = pushImg()
    push_img.run()
