#coding=utf-8
#!/usr/bin/env python

import redis
import Queue
import threading
import json
import time

class redisHandler():
    '''
    # Redis通信模块, 共两个线程
    # 一个线程订阅目标topic list 并推入queue
    # 另一个线程为主要功能实现线程，通过Queue获取redis订阅消息，并发布相关topic
    '''
    def __init__(self, sub_topics=[], pub_topics=[]):
        self.q = Queue.Queue(2)
        self.pub_topics = pub_topics
        self.sub_topics = sub_topics
        self.host = '127.0.0.1'
        self.port = '6379'
        self.db = 0
        self.rc = redis.StrictRedis(host=self.host, port=self.port, db=self.db)

    def start_sub(self):
        # 开启订阅线程
        self.t = threading.Thread(target=self.t_sub)
        self.t.setDaemon(True)
        self.t.start()
        
    def q_put(self, data):
        # put queue 消息
        try:
            if self.q.full():
                self.q.get_nowait()
            self.q.put_nowait(data)
        except Exception as e:
            pass

    def q_get(self):
        # 获取queue消息
        try:
            return self.q.get()
        except:
            return None

    def q_get_nowait(self):
        # 获取queue消息
        try:
            return self.q.get_nowait()
        except:
            return None

    def t_sub(self):
        # 订阅redis topic 线程
        print(self.sub_topics)
        if len(self.sub_topics) == 0:
            return
        ps = self.rc.pubsub()
        ps.psubscribe(self.sub_topics)
        while True:
            msg = ps.parse_response(block=False, timeout=1)
            # print(msg)
            if msg:
                if b'pmessage' in msg:
                    channel = msg[2]
                    try:
                        msg = json.loads(msg[-1])
                        msg['channel'] = channel
                        self.q_put(msg)
                    except:
                        pass

    def pub_all(self, data):
        for topic in self.pub_topics:
            self.rc.publish(topic, json.dumps(data))

    def run(self):
        rc = self.rc
        pub_data = {
                'header':'camera_on',
                'data':''
                }
        while True:
            data = self.q_get_nowait()
            if data:
                pass
                # print(data)
            for topic in self.pub_topics:
                topic = self.pub_topics[topic]
                rc.publish(topic, json.dumps(pub_data))
                time.sleep(10)
            


class track_cmd(redisHandler):
    pass

class rtk(redisHandler):
    pass

class cmd(redisHandler):
    pass


if __name__ == '__main__':
    sub_topics = ['move_base_out', 'rtk_out']
    pub_topics = {'key1':'rtk_sub', 'key2':'puhimg_in'}
    track = track_cmd(sub_topics, pub_topics)
    track.start_sub()
    track.run()
    '''
    sub_topics = ['cmd_sub']
    pub_topics = {'key1':'cmd_pub'}
    cmd = track_cmd(sub_topics, pub_topics)

    sub_topics = ['rtk_sub']
    pub_topics = {'key1':'rtk_pub'}
    rtk = track_cmd(sub_topics, pub_topics)
    '''
