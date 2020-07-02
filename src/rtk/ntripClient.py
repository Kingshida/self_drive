import socket
import serial
import base64
import time
import datetime
from functools import reduce
import operator
import Queue as queue
import logging

# from generate_sol import SolGenerator
# from RTCMv3_decode import decode_rtcm3_from_net, set_generator

# dummyNMEA = "$GPGGA,100429.20,2232.1120000,N,11356.5668000,E,1,00,1.0,3.428,M,-3.428,M,0.0,*5C"
HOST = "117.184.129.18"
username = "sweet"  # username for RTCM correction service
password = "dian123456"  # password for RTCM correction service
# my_host = "rtk.qxwz.com"
# port = 8001  # port for the service
# my_host = "127.0.0.1"
port = 8061  # port for the service
MOUNT_PT = "sweet"

'''Generate an encoding of the username:password for the service.
The string must be first encoded in ascii to be correctly parsed by the
base64.b64encode function.'''
pwd = base64.b64encode("{}:{}".format(username, password).encode('ascii'))

# The following decoding is necessary in order to remove the b' character that
# the ascii encoding add. Othrewise said character will be sent to the net and misinterpreted.
pwd = pwd.decode('ascii')


# header = \
#     "GET /mountpoint HTTP/1.1\r\n" + \
#     "Host my_host\r\n" + \
#     "Ntrip-Version: Ntrip/1.0\r\n" + \
#     "User-Agent: ntrip.py/0.1\r\n" + \
#     "Accept: */*" + \
#     "Connection: close\r\n" + \
#     "Authorization: Basic {}\r\n\r\n".format(pwd)

header = \
    "GET /%s HTTP/1.1\r\n" % MOUNT_PT + \
    "User-Agent: NTRIP client.py/0.1\r\n" + \
    "Authorization: Basic {}\r\n\r\n".format(pwd)


def nmea_checksum(nmea_str):
    return reduce(operator.xor, map(ord, nmea_str), 0)


def generate_gga():
    # str1 = "GPGGA,133622.69,2232.1120000,N,11356.5668000,E,1,00,1.0,3.428,M,-3.428,M,0.0,"
    tst = datetime.datetime.utcnow().strftime("%H%M%S.00,")
    gga_str = "GPGGA," + tst + "2232.1120000,N,11356.5668000,E,1,00,1.0,3.428,M,-3.428,M,0.0,"
    # gga_str = "GPGGA,133622.69,2232.1120000,N,11356.5668000,E,1,00,1.0,3.428,M,-3.428,M,0.0,"
    gga_str += '*%02X' % nmea_checksum(gga_str)
    gga_str = '$' + gga_str
    # print gga_str
    return gga_str


# def recv_from_svr(s, callback):
def recv_from_svr(s):
    port = '/dev/ttyS6'
    bps = 115200
    ser=serial.Serial(port, bps)
    while True:
        pre_t =time.time()
        head_flag = False
        try:
            dat = s.recv(1056) 
        except socket.error:
            print('err')
            dat = []
        if len(dat) > 0:
            # print(hex(dat[0]))
            # print(len(dat))
            ser.write(dat)
            # print(time.time()-pre_t)


def timed_sending_gga(s):
    while True:
        s.send(generate_gga().encode('ascii'))
        s.send('\r\n\r\n'.encode('ascii'))
        time.sleep(5)

def start_gga_sending(s):
    import threading

    print("begin GGA thread...")
    t = threading.Thread(target=timed_sending_gga, args=(s,))
    t.start()


class NtripClient:
    def __init__(self, request_handle, mount_point=MOUNT_PT, pass_phrase=pwd):
        self.status = "init"
        self.connected = False
        self.source_caster = request_handle.server
        self.buf = bytes()
        self.mount_point = mount_point
        self.lat = 0.0
        self.lon = 0.0
        self.alt = 0.0
        self.pass_phrase = pass_phrase
        self.q = queue.Queue(10)

    def push_data(self, data):
        try:
            if self.q.full():
                self.q.get_nowait()
            self.q.put(data)
        except:
            pass
        # self.buf += data
        # if len(self.buf) > 20480:
        #     self.buf = self.buf[-20480:-1]

    def get_data(self):
        # send_buff = self.buf
        # self.flush()
        # return send_buff
        if not self.q.empty():
            return self.q.get()

    def flush(self):
        self.buf = bytes()

    def auth_check(self):

        return True


def init_logger():
    LOG_FILE = 'ntrip_client.log'

    handler = logging.FileHandler(LOG_FILE)
    # fmt = '%(asctime)s-%(filename)s:%(lineno)s-%(name)s-%(message)s'
    fmt = '%(message)s'

    formatter = logging.Formatter(fmt)
    handler.setFormatter(formatter)

    logger = logging.getLogger('ntrip_client')
    logger.addHandler(handler)
    logger.setLevel(logging.DEBUG)

    logger.info('>log for Nrrip client begin')
    # logger.debug('first debug message')

    return logger


def log_com_data(logger, data):
    logger.info(data)


def main_loop():
    logger = init_logger()
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # s.settimeout(0.5)
    s.connect((HOST, int(port)))
    while True:
        print("Header sending... \n")
        s.send(header.encode('ascii'))
        print("Waiting answer...\n")
        data = s.recv(12).decode('ascii')
        if len(data) == 0:
            continue
        if 'OK' in data: 
            break
        time.sleep(1)
    # start_gga_sending(s)
    recv_from_svr(s)
    # recv_from_svr(s, decode_rtcm_stream)
    s.close()


if __name__ == '__main__':
    main_loop()
