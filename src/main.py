# _*_ coding: UTF-8 _*_

import sys
import json
import time
import systemCtrl
from redisHandler import redisHandler

class main(redisHandler):
    """
    机器人主函数
    """
    def init(self):
        self.sub_topics = ['tcp_out', 'move_base_out', 'pushimg_out', 'ctrl_out', 'tracking_out']
        self.move_base_header = ['init', 'speed', 'heartbeat', 'get_base_info']
        self.ctrl_header = ['ctrl']
        self.pushimg_header = ['camera_on', 'camera_off']
        self.tracking_header = ['auto_on', 'auto_off', 'get_pos', 'auto_continue', 'speed']

        self.sys_ctrl = systemCtrl.systemCtrl()
        self.start_sub()
   
    def run(self):
        self.init()
        auto_flag = False
        # auto_flag = True
        while True:
            try:
                data = self.q_get()
                if data:
                    channel = data['channel']
                    del data['channel']
                    header = data['header']
                    data = json.dumps(data)
                    print('channel:%s, data: %s'%(channel, data))
                    if channel == 'tcp_out':
                        if header in self.move_base_header:
                            if not auto_flag or header == 'get_base_info':
                                self.rc.publish('move_base_in', data)
                        elif header in self.ctrl_header:
                            if not auto_flag:
                                self.rc.publish('ctrl_in', data)
                        elif header in self.pushimg_header:
                            self.rc.publish('pushimg_in', data)
                        elif header in self.tracking_header:
                            self.rc.publish('tracking_in', data)
                            self.rc.publish('track_cmd_in', data)
                            if header == 'auto_on' or header == 'auto_continue':
                                # 自动开始
                                auto_flag = True
                            elif header == 'auto_off':
                                #　自动结束
                                auto_flag = False
                        elif header == 'shutdown':
                            # 关机
                            self.sys_ctrl.shut_down()
                        elif header == 'reboot':
                            # 关机
                            self.sys_ctrl.reboot()
            except Exception as e:
                print('main err')
                print(e)


if __name__ == '__main__':
    main_p = main()
    main_p.run()
