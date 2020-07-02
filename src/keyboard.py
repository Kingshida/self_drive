# _*_ coding: UTF-8 _*_

import sys
import select
import tty
import termios
import json
import time
import Queue
import threading
import serialCmd

class keyboard():
    def __init__(self):
        self.q = Queue.Queue(2)
        t = threading.Thread(target=self.run)
        t.setDaemon(True)
        t.start()

        t = threading.Thread(target=self.get_key)
        t.setDaemon(True)
        t.start()
        while True:
            time.sleep(30)
    
    def q_put(self, data):
        try:
            if self.q.full():
                self.q.get_nowait()
            self.q.put_nowait(data)
        except:
            pass

    def q_get(self):
        try:
            return self.q.get_nowait()
        except:
            return None

    def run(self):
        timeout = 2
        pre_time = time.time()
        ser = serialCmd.serialCmd()
        while True:
            now = time.time()
            data = self.q_get()
            if now - pre_time >= timeout:
                data = '*STOP#'
            if data:
                try:
                    if not ser.serial.isOpen():
                        ser.open_port()
                    ser.send_cmd(cmd)
                except Exception as e:
                    pass
                    


    def get_key(self):
        cmd = ''
        pre_cmd = ''
        print "Reading form keybord"
        print """      i
        j k l
          m"""
        print 'press Q to quit'
        while True:
            fd=sys.stdin.fileno()
            old_settings=termios.tcgetattr(fd)
            #old_settings[3]= old_settings[3] & ~termios.ICANON & ~termios.ECHO
            try:
              tty.setraw(fd)
              ch=sys.stdin.read(1)
            finally:
              termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
              #print 'error'
            if ch=='i':
              print 'move forward'
              cmd = '*AHEAD#'
            elif ch=='m':
              print 'move back'
              cmd = '*BACK#'
            elif ch=='j':
              print "turn left!"
              cmd = '*LEFT#'
            elif ch=='l':
              print "turn right!"
              cmd = '*RIGHT#'
              print "turn right!"
            elif ch=='k':
              print "stop motor!"
              cmd = '*STOP#'
            elif ch=='q':
              print "shutdown!"
              break
            elif ord(ch)==0x3:
              #这个是ctrl c
              print "shutdown"
              break
            print "Reading form keybord"
            print """            i
            j k l
              m"""
            print 'press Q or ctrl+c to quit'
            try:
                if cmd != pre_cmd or cmd == '*STOP#':
                    pre_cmd = cmd
                    print(cmd)
                    self.q_put(cmd)
            except Exception as e:
                print(e)



if __name__ == '__main__':
    t = keyboard()
