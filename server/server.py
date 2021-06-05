from socketserver import BaseRequestHandler, ThreadingTCPServer
from control import InvertedPendulumSimulator
import time, struct
import threading
import numpy as np
from protocol import MyProtocol

BUF_SIZE = 1024


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


# class ControlRuntime(threading.Thread, MyProtocol):

#     def __init__(self, tname, handler):
#         super(ControlRuntime, self).__init__()
#         self.tname = tname
#         self.handler = handler
#         self.sample_time = 0.013
#         self.simulator = InvertedPendulumSimulator(self.sample_time)
#         self.target = 0

#     def run(self) -> None:
#         while 1:

#             start_tick = time.perf_counter()
#             self.simulator.runtime(self.target)
#             end_tick = time.perf_counter()

#             start_tick = time.perf_counter()
#             surplus_time = self.sample_time - (end_tick - start_tick)
#             surplus_time = round(surplus_time, 8)
#             if surplus_time > 0:
#                 time.sleep(surplus_time)

#             end_tick = time.perf_counter()
#             #print(threading.currentThread().getName(), surplus_time, end_tick-start_tick)

#             output_posi = self.simulator.get_position()
#             output_theta = self.simulator.get_theta()

#             float_buf = struct.pack('<2f', *[output_posi, output_theta])

#             send_buf = self.mp_send_buf_pack(float_buf)

#             with self.handler.lock:
#                 if self.handler.stop_flag:
#                     break

#             self.handler.request.sendall(send_buf)




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

                end_tick = time.perf_counter()
                #print(threading.currentThread().getName(), surplus_time, end_tick-start_tick)

                output_posi = simulator.get_position()
                output_theta = simulator.get_theta()

                float_buf = struct.pack('<2f', *[output_posi, output_theta])

                send_buf = self.mp_send_buf_pack(float_buf)

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

                elif command == self.CMD_CHANGE_PARAM:
                    receiver.is_receive_cmd = False

                elif command == self.CMD_DISCONNNECT:
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
    server.serve_forever()
