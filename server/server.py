'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 服务端主程序
'''

from socketserver import BaseRequestHandler, ThreadingTCPServer
from model_RK4 import FirstOrderInvertedPendulum
import time, struct
import threading
import numpy as np
from protocol import MyProtocol
from filter import AntiPeakFilter
from math import pi

class CmdReceiver(threading.Thread, MyProtocol):

    def __init__(self, tname, handler):
        super(CmdReceiver, self).__init__()
        self.tname = tname
        self.handler = handler
        self.receive_data = []
        self.command = b''
        self.is_receive_cmd = False

    def run(self) -> None:
        try:
            if not self.is_receive_cmd:
                while 1:
                    ret, length = self.mp_receive_head(self.handler.request)
                    if ret:
                        # 加锁
                        self.handler.lock.acquire()
                        self.receive_data = self.mp_receive_data(self.handler.request, length)
                        if self.receive_data[0] == self.receive_data[1]:
                            self.is_receive_cmd = True
                            self.command = self.receive_data[:2]
                        self.handler.lock.release()
        except OSError as ex:
            print(self.handler.client_address,"连接断开")
        finally:
            self.handler.request.close()

    def get_recv_detail(self):
        return self.receive_data[2:]


class Handler(BaseRequestHandler, MyProtocol):

    lock = threading.Lock()

    def handle(self) -> None:
        '''一个子线程
        '''
        print(threading.currentThread().getName())
        receiver = CmdReceiver('receiver', self)
        receiver.setDaemon(True)

        input_force = 0
        sample_time = 0.014
        surplus_time = 0.014
        ip_model = FirstOrderInvertedPendulum(M_car=1, 
                                    M_stick=1,
                                    stick_lenght=0.6,
                                    initial_theta=pi,
                                    sample_time = sample_time)

        input_force_filter = AntiPeakFilter(80, sample_time, 1)

        start_flag = False

        receiver.start()

        while 1:

            if start_flag and not receiver.is_receive_cmd:
                start_tick = time.perf_counter()
                ip_model.forward(input_force)
                end_tick = time.perf_counter()

                start_tick = time.perf_counter()
                surplus_time = sample_time - (end_tick - start_tick)
                surplus_time = round(surplus_time, 8)

                # end_tick = time.perf_counter()
                # print(threading.currentThread().getName(), surplus_time, end_tick-start_tick)

                output_posi = ip_model.get_position()
                output_theta = ip_model.get_theta()

                float_buf = struct.pack('<2f', output_posi, output_theta)
                send_buf = self.mp_send_buf_pack(self.CMD_MODEL_CONTROL, float_buf)

                try:
                    self.request.sendall(send_buf)
                except OSError:
                    break

                if surplus_time > 0:
                    time.sleep(0.01)

            elif receiver.is_receive_cmd:
                command = receiver.command
                if command == self.CMD_MODEL_CONTROL:
                    recv_data = receiver.get_recv_detail()
                    input_force = struct.unpack('<f', recv_data)[0]
                    input_force = input_force_filter.get_value(input_force)

                else:
                    if command == self.CMD_START:
                        start_flag = True

                    elif command == self.CMD_PAUSE:
                        start_flag = False

                    elif command == self.CMD_MODEL_PARAM_READ:
                        float_buf = struct.pack('<3f', ip_model.M_car, ip_model.M_stick, 
                                                ip_model.l_stick * 2)
                        send_buf = self.mp_send_buf_pack(self.CMD_MODEL_PARAM_READ, float_buf)
                        try:
                            self.request.sendall(send_buf)
                        except OSError:
                            break

                    elif command == self.CMD_MODEL_PARAM_WRITE:
                        recv_data = receiver.get_recv_detail()
                        float_buf = struct.unpack('<3f', recv_data)
                        ip_model.reset_param(float_buf[0], float_buf[1], float_buf[2])
                        start_flag = False

                    time.sleep(0.01)

                receiver.is_receive_cmd = False
                #time.sleep(surplus_time)


    def setup(self) -> None:
        print("before handle,连接建立：",self.client_address)

    def finish(self) -> None:
        print("finish run  after handle")


if __name__ == '__main__':
    HOST, PORT = 'localhost', 9999
    
    server = ThreadingTCPServer((HOST,PORT), Handler)
    print('服务器开启')
    server.serve_forever()
