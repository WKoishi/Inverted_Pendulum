'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 服务端主程序
'''

from socketserver import BaseRequestHandler, ThreadingTCPServer
from control import InvertedPendulumSimulator
import time, struct
import threading
import numpy as np
from protocol import MyProtocol

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
                        with self.handler.lock:
                            self.receive_data = self.mp_receive_data(self.handler.request, length)
                            if self.receive_data[0] == self.receive_data[1]:
                                self.is_receive_cmd = True
                                self.command = self.receive_data[:2]
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

        simulator_posi_target = 0
        sample_time = 0.013
        surplus_time = 0.013
        simulator = InvertedPendulumSimulator(sample_time)

        start_flag = False

        receiver.start()

        while 1:

            if start_flag and not receiver.is_receive_cmd:
                start_tick = time.perf_counter()
                simulator.runtime(simulator_posi_target)
                end_tick = time.perf_counter()

                start_tick = time.perf_counter()
                surplus_time = sample_time - (end_tick - start_tick)
                surplus_time = round(surplus_time, 8)
                if surplus_time > 0:
                    time.sleep(surplus_time)

                #end_tick = time.perf_counter()
                #print(threading.currentThread().getName(), surplus_time, end_tick-start_tick)

                output_posi = simulator.get_position()
                output_theta = simulator.get_theta()

                float_buf = struct.pack('<2f', *[output_posi, output_theta])
                send_buf = self.mp_send_buf_pack(self.CMD_MODEL_STATE, float_buf)

                try:
                    self.request.sendall(send_buf)
                except OSError:
                    break

            else:
                command = receiver.command
                if command == self.CMD_START:
                    start_flag = True
                    receiver.is_receive_cmd = False

                elif command == self.CMD_CHANGE_TARGET:
                    recv_data = receiver.get_recv_detail()
                    simulator_posi_target = struct.unpack('<f', recv_data)[0]
                    receiver.is_receive_cmd = False

                elif command == self.CMD_READ_C_PARAM:
                    lqr_param = list(simulator.feedback_gain_fast) + \
                                list(simulator.feedback_gain_slow)
                    float_buf = struct.pack('<8f', *lqr_param)
                    send_buf = self.mp_send_buf_pack(self.CMD_READ_C_PARAM, float_buf)
                    try:
                        self.request.sendall(send_buf)
                    except OSError:
                        break
                    receiver.is_receive_cmd = False

                elif command == self.CMD_CHANGE_C_PARAM:
                    recv_data = receiver.get_recv_detail()
                    lqr_param = struct.unpack('<8f', recv_data)
                    for i in range(4):
                        simulator.feedback_gain_fast[i] = lqr_param[i]
                    for i in range(4):
                        simulator.feedback_gain_slow[i] = lqr_param[i+4]

                    receiver.is_receive_cmd = False

                elif command == self.CMD_DISCONNECT:
                    receiver.is_receive_cmd = False

                else:
                    receiver.is_receive_cmd = False

                time.sleep(surplus_time)


    def setup(self) -> None:
        print("before handle,连接建立：",self.client_address)

    def finish(self) -> None:
        print("finish run  after handle")


if __name__ == '__main__':
    HOST, PORT = 'localhost', 9999
    
    server = ThreadingTCPServer((HOST,PORT), Handler)
    print('服务器开启')
    server.serve_forever()
