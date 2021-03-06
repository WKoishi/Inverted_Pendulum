'''
Author: WKoishi \\
Creation date: 2021-05-21 \\
Description: 数据传输协议
'''

import struct

class MyProtocol:

    __buf_HEAD = b'\xac\xac\xac\xac'

    CMD_START = b'\x01\x01'
    CMD_PAUSE = b'\x02\x02'
    CMD_MODEL_PARAM_READ = b'\x03\x03'
    CMD_MODEL_PARAM_WRITE = b'\x04\x04'
    CMD_DISCONNECT = b'\x05\x05'

    CMD_MODEL_CONTROL = b'\x99\x99'

    def mp_send_buf_pack(self, command, data_bytes_buf) -> bytes:
        if data_bytes_buf == None:
            bytes_buf = command
        else:
            bytes_buf = command + data_bytes_buf
        buf_len = len(bytes_buf)
        buf_len = struct.pack('<h', buf_len)
        send_buf = self.__buf_HEAD + buf_len + bytes_buf
        #example = int.from_bytes(buf_len, byteorder='little')

        return send_buf

    def mp_receive_data(self, sock, length):
        buf = bytearray()
        while length > 0:
            newbuf = sock.recv(length)
            if not newbuf: return None
            buf += newbuf
            length -= len(newbuf)

        return buf

    def mp_receive_head(self, sock):
        head = sock.recv(6)
        head = bytearray(head)
        headIndex = head.find(self.__buf_HEAD)
        if headIndex == 0 and len(head) == 6:
            length = struct.unpack('<h', head[headIndex+4: headIndex+6])[0]
            return True, length
        else:
            return False, 0