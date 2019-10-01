from mock import patch
from struct import pack, unpack
import socket
import time
import copy

from ..drivers.misc.udp import UdpConnector
import src.drivers.misc.status

udp = None
seqs = []
current_seq = 0
handler_called = False

def mock_status():
    assert True

def handler(r):
    assert True

def handler_2(r):
    global handler_called
    handler_called = True

def mock_connect(cls, addr):
    assert True

def mock_bind(cls, addr):
    assert True

def mock_sendto(cls, *args, **kwargs):
    assert True

def mock_recv(cls, size):
    global udp
    global seqs
    global current_seq

    msg = bytearray()
    msg.extend(pack('<i', 0)[::-1])
    msg.extend(pack('<i', seqs[current_seq])[::-1])

    current_seq += 1

    return msg

def mock_send(cls, msg):
    # send called
    assert True

class TestUdp(object):
    def test_init(self):
        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(UdpConnector, "consumeQueue", return_value=None) as mock_consume:
            
            #test if connect is called with the right parameters
            udp = UdpConnector("localhost", 8000)
            time.sleep(2)
            #test if the consuming of the queue starts
            assert mock_consume.called

            udp.stopConsumeThread()

    def test_add_queue(self):
        msg = bytearray()
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 0)[::-1])

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(UdpConnector, 'receive', mock_recv), \
            patch.object(UdpConnector, 'send', mock_send):

            udp = UdpConnector("localhost", 8000)
            udp.appendToQueue(msg, "test", handler)

            assert len(udp.sock_queue) == 1

            udp.stopConsumeThread()

    def test_is_queued(self):
        msg = bytearray()
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 0)[::-1])

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(UdpConnector, 'receive', mock_recv), \
            patch.object(UdpConnector, 'send', mock_send):

            udp = UdpConnector("localhost", 8000)
            udp.appendToQueue(msg, "test", handler)
            assert udp.isQueued("test")
            assert not udp.isQueued("something_else")
            assert len(udp.sock_queue) == 1

            udp.stopConsumeThread()

    def test_handler_called(self):
        global udp
        global seqs

        msg = bytearray()
        msg.extend(pack('<i', 0)[::-1])

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(UdpConnector, 'receive', mock_recv), \
            patch.object(UdpConnector, 'send', mock_send):

            udp = UdpConnector("localhost", 8000)
            seqs.append(udp.appendToQueue(msg, "test", handler_2))

            assert len(udp.sock_queue) == 1

            while len(udp.sock_queue) > 0:
                time.sleep(0.1)

            assert handler_called
            udp.stopConsumeThread()

    def test_consume_queue(self): 
        global udp
        global seqs
        
        msg = bytearray()
        msg.extend(pack('<i', 0)[::-1])

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind, \
            patch.object(UdpConnector, 'receive', mock_recv), \
            patch.object(UdpConnector, 'send', mock_send):

            udp = UdpConnector("localhost", 8000)
            seqs.append(udp.appendToQueue(copy.deepcopy(msg), "test", handler))
            seqs.append(udp.appendToQueue(copy.deepcopy(msg), "test", handler))
            seqs.append(udp.appendToQueue(copy.deepcopy(msg), "test", handler))
            seqs.append(udp.appendToQueue(copy.deepcopy(msg), "test", handler))

            while len(udp.sock_queue) > 0:
                time.sleep(0.1)

            assert True

            udp.stopConsumeThread()

    

