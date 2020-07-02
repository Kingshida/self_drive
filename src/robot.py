# _*_ coding: UTF-8 _*_

import time
from multiprocessing import Process
from tcp import tcp
from moveBase import moveBase
from ctrl import ctrl
from main import main
from joy import joy
from rtk import rtk
from pushImg import pushImg
from ultrasound import ultrasound


if __name__ == '__main__':
    p_list = []
    node = tcp()
    p = Process(target=node.handle)
    p_list.append(p)

    node = moveBase()
    p = Process(target=node.run)
    p_list.append(p)

    node = ctrl()
    p = Process(target=node.run)
    p_list.append(p)

    node = main()
    p = Process(target=node.run)
    p_list.append(p)

    node = joy()
    p = Process(target=node.run)
    p_list.append(p)

    node = rtk()
    p = Process(target=node.run)
    p_list.append(p)

    node = pushImg()
    p = Process(target=node.run)
    p_list.append(p)
    '''
    node = ultrasound()
    p = Process(target=node.run)
    p_list.append(p)
    '''
    try: 
        for p in p_list:
            p.daemon = True
            p.start()
            time.sleep(1)
    except Exception as e:
        print(e)

    while True:
        time.sleep(1000)

