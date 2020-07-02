#coding=utf-8
#!/usr/bin/env python

import yaml
import os
import sys
import time

class config():
    def __init__(self, file_name='robot.yaml'):
        self.robot = {
                'name':'robot:' + time.strftime('%m-%d %H:%M',time.localtime(time.time())),
                'key':'null'
                }
        self.config = {
                'robot':self.robot
                }
        # 获取当前目录的上一层
        # self.file_name_path = os.path.split(os.path.realpath(__file__))[0]
        self.file_name_path = os.path.dirname(os.path.abspath(sys.argv[0])) + '/..'
        # 合成配置文件目录
        self.yaml_path = os.path.join(self.file_name_path, file_name)
        try:
            f = open(self.yaml_path, 'r')
            cont = f.read()
            data = yaml.load(cont, Loader=yaml.Loader)
            f.close()
        except:
            f = open(self.yaml_path, 'w+')
            f.close()
            f = open(self.yaml_path, 'r')
            cont = f.read()
            data = yaml.load(cont, Loader=yaml.Loader)
            f.close()
        if not data:
            data = self.config
        else:
            try:
                self.robot['name'] = data['robot']['name']
                self.robot['key'] = data['robot']['key']
                self.config = data
            except Exception as e:
                print(e)
                data.update({'robot':robot})
        # 装载数据
        f = open(self.yaml_path, 'w')
        yaml.dump(data, f, allow_unicode=True)
        f.close()
        # print('data is :', data)

    def set_robot(self, name, key):
        self.robot['name'] = name
        self.robot['key'] = key
        self.write_config({'robot':self.robot})

    def set_para(self, key, data):
        self.config[key] = data
        self.write_config(self.config)

    def write_config(self, write_data):
        f = open(self.yaml_path, 'r')
        cont = f.read()
        data = yaml.load(cont, Loader=yaml.Loader)
        data.update(write_data)
        f.close()
        f = open(self.yaml_path, 'w')
        yaml.dump(data, f, allow_unicode=True)
        f.close()
        # print('data is :', data)
        


if __name__ == '__main__':
    c = config('test.yaml')
    # c.set_robot('robot', 'keyuyyyyyyyyyyy')
    print(c.config)
    c.set_para('cx','[1,2,4]')
    print(c.config)
