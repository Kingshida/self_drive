#-*- coding:utf-8 -*-

import time
import datetime
import json
import sys
import Queue as queue
import requests
import threading
import socket
import base64
from SocketServer import ThreadingTCPServer, StreamRequestHandler
from config import config
from redisHandler import redisHandler


class tcp(redisHandler):
    def __init__(self):
        # 心跳时间
        self.timeout = 30
        self.conf = config()
        self.name = self.conf.robot['name']
        self.key = self.conf.robot['key']
        # 从主函数端接收消息的q
        self.recv_q = queue.Queue(5)
        self.address = ('117.184.129.18', 9000)
        self.robot_on()
        redisHandler.__init__(self)

    def recv_q_put(self, data):
        try:
            if self.recv_q.full():
                self.recv_q.get_nowait()
            self.recv_q.put_nowait(data)
        except Exception as e:
            pass

    def recv_q_get(self):
        try:
            return self.recv_q.get_nowait()
        except:
            return None

    def conn_server(self):
        while True:
            try:
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
                self.sock.connect(self.address)
                print('connected to server')
                break
            except Exception as e:
                time.sleep(5)
                print(e)
                print('connecting to server')

    def robot_on(self):
        self.conn_server()
        username = "AjYvN"  # username for RTCM correction service
        password = "dian123456"  # password for RTCM correction service
        pwd = base64.b64encode("{}:{}".format(username, password).decode('utf-8').encode('utf-8'))
        header = \
                "GET %s HTTP/1.1\r\n" % self.key + \
                "User-Agent: %s client.py/0.1\r\n" %self.name + \
                "Authorization: Basic {}\r\n\r\n".format(pwd)
        while True:
            try:
                self.sock.sendall(header.encode('utf-8'))
                # 从云端接收消息
                header_info = self.sock.recv(1024).strip()
                if header_info:
                    header_info = decode_ntrip_header(header_info)
                    if header_info:
                        self.name = header_info[0]
                        self.key = header_info[1]
                        self.conf.set_robot(self.name, self.key)
                        print('robot_on')
                        break
            except Exception as e:
                time.sleep(5)
                print(e)
                print('failed robot on')
                self.conn_server()
            

    def handle(self):
        self.pub_topics = ['tcp_out']
        self.sub_topics = ['tcp_in', 'move_base_out']
        self.start_sub()
        # 开启订阅 in 主题
        t = threading.Thread(target=self.run)
        t.setDaemon(True)
        t.start()
        while True:
            try:
                data = self.sock.recv(1024)
                if data:
                    # 来自云的消息, 发送至out topic
                    self.recv_q_put(data)
                else:
                    self.robot_on()
            except Exception as e:
                print(e)
                self.robot_on()
        self.sock.close()

    def t_sub(self):
        # 订阅redis topic 线程
        if len(self.sub_topics) == 0:
            return
        ps = self.rc.pubsub()
        ps.psubscribe(self.sub_topics)
        while True:
            msg = ps.parse_response(block=False, timeout=1)
            if msg:
                if b'pmessage' in msg:
                    self.q_put(msg[-1])

    def run(self):
        rc = self.rc
        heartbeat = {
                'header':'heartbeat',
                'data':''
                }
        heartbeat = json.dumps(heartbeat)
        pre_time = time.time()
        pre_server_t = time.time()
        while True:
            now = time.time()
            if now - pre_server_t > self.timeout:
                # 心跳超时
                self.robot_on()
                now = time.time()
                pre_server_t = now
            if now - pre_time >= self.timeout / 3.0:
                # 向服务器发送心跳，保持连接
                # print('tcp heart beat')
                pre_time = now
                try:
                    t = self.sock.sendall(heartbeat)
                except Exception as e:
                    print(e)

            data = self.q_get_nowait()
            data_recv = self.recv_q_get()
            if data_recv:
                pre_server_t = now
                # 来自tcp server 云的消息，publish 至tcp_out topic
                for topic in self.pub_topics:
                    try:
                        rc.publish(topic, data_recv)
                    except Exception as e:
                        print('tcp err')
                        print(e)

            if data:
                # tcp_in topic 发送至云server
                pre_time = now
                try:
                    self.sock.sendall(data)
                except Exception as e:
                    print('tcp err')
                    print(e)
                    self.robot_on()




def decode_ntrip_header(buff):
    '''
    "ICY 200 OK\r\n" + \
    "Server: {} {}\r\n".format(code, name) + \
    "Via: n4_2\r\n" + \
    "Date: {}\r\n".format(timephr) + \
    "Connection: keep-alive\r\n\r\n"
    '''
    try:
        buff = buff.decode('utf-8')
        if 'OK' in buff:
            role = 'client'
            request = buff.split('\r\n')
            info = request[1].split(' ')
            key = info[1]
            name = info[2]
            return name, key
    except Exception as e:
        print(e)
    return None


if __name__ == "__main__":
    tcp = tcp()
    tcp.handle()
    
