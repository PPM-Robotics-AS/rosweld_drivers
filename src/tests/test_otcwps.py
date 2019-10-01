from ..drivers.misc.udp import UdpConnector, MessageQueueItem
import rospy
import ftplib
from mock import patch
from struct import pack, unpack
import socket
import configparser
import src.drivers.otc_wps
import os
import time
from threading import Thread, currentThread
from mocks import *
from rosweld_drivers.srv import SetJobNumber, SetJobNumberRequest, SetWeldingParametersRequest, InputRequest
from rosweld_drivers.msg import WeldingState

class MockPublisher():
    def __init__(self, *args,**kwargs):
        pass

    def publish(self, *args,**kwargs):
        assert True

class MockRate():
    def __init__(self, hz):
        self.timeout = 1 / hz

    def sleep(self):
        time.sleep(self.timeout)

def mock_connect(cls, addr):
    assert True

def mock_bind(cls, addr):
    assert True

def check_command(cmd):
    valid = False
    for property, value in vars(src.drivers.otc_wps.Commands).iteritems():
        if value == cmd:
            valid = True
        
    assert valid

def mock_err_nlst():
    raise ftplib.error_perm

def mock_appendToQueue_sendstore(self, msg, label, handler = None):
    #Store and arc update called
    command = unpack('>i',msg[0:4])[0]
    check_command(command)

    assert label == "send_store"
    assert command == src.drivers.otc_wps.Commands.arc_update

def mock_appendToQueue_sendupdate(self, msg, label, handler = None):
    #Store and arc update called
    command = unpack('>i',msg[0:4])[0]
    check_command(command)

    assert label == "send_update"
    assert command == src.drivers.otc_wps.Commands.update

def mock_appendToQueue_setjobnr(self, msg, label, handler = None):
    command = unpack('>i',msg[0:4])[0]
    check_command(command)

    assert label == "send_set_job_number"
    assert command == src.drivers.otc_wps.Commands.set_job_number

    src.drivers.otc_wps.current_job_idx = unpack('>i',msg[4:8])[0]

def mock_appendToQueue_arcstart(self, msg, label, handler = None):
    command = unpack('>i',msg[0:4])[0]
    check_command(command)

    assert label == "send_arc_start" 
    assert command == src.drivers.otc_wps.Commands.arc_start 

    src.drivers.otc_wps.current_job_idx = unpack('>i',msg[4:8])[0]

def mock_appendToQueue_arcstop(self, msg, label, handler = None):
    command = unpack('>i',msg[0:4])[0]
    check_command(command)

    assert label == "send_arc_stop" 
    assert command == src.drivers.otc_wps.Commands.arc_stop 

def mock_download_config(f):
    config = configparser.ConfigParser()
    config.optionxform = str
    config['AST_CONDITION'] = {
         'A_CURRENT': '1',
         'A_SPEED': '1',
         'A_CURRENT_OUTPUT': '1',
         'A_FILLER_SPEED': '1'
    }
    return config

src.drivers.otc_wps.jobs["1"] = WeldingState(job_number=1)
src.drivers.otc_wps.jobs["2"] = WeldingState(job_number=2)
src.drivers.otc_wps.jobs["3"] = WeldingState(job_number=3)

checkInputParam = None
checkInputBig = None

class TestOtcWps(object):

    def test_set_job_nr(self):

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect, \
            patch.object(rospy, 'get_param', mock_get_param), \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_setjobnr):

            src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)

            req = SetJobNumberRequest()
            req.value = 1

            src.drivers.otc_wps.set_job_number(req)
            src.drivers.otc_wps.udp.stopConsumeThread()

            assert src.drivers.otc_wps.current_job_idx == req.value

    def test_set_job_nr2(self):
        src.drivers.otc_wps.current_job_idx = 1

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect, \
            patch.object(rospy, 'get_param', mock_get_param), \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_setjobnr):

            src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)
            req = SetJobNumberRequest()
            req.value = 5

            src.drivers.otc_wps.set_job_number(req)
            src.drivers.otc_wps.udp.stopConsumeThread()

            assert src.drivers.otc_wps.current_job_idx != req.value
            assert src.drivers.otc_wps.current_job_idx == 1

    def test_set_params(self):
        src.drivers.otc_wps._download_config = mock_download_config
        src.drivers.otc_wps.ftp = ftplib.FTP()

        with patch.object(src.drivers.otc_wps.ftp, "storlines", return_value=None) as mock_store ,\
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendstore):
            req = SetWeldingParametersRequest()
            req.params = WeldingState()

            src.drivers.otc_wps.set_params(req)
            assert mock_store.called

    def test_send_update(self):
        src.drivers.otc_wps.ftp = ftplib.FTP()

        with patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate):
            src.drivers.otc_wps.update_asynch()

        with patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect, \
            patch.object(rospy, 'get_param', mock_get_param), \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate): 

            src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)
            src.drivers.otc_wps.udp.sock_queue.append(MessageQueueItem(0, bytearray(), "send_update"))
            src.drivers.otc_wps.update_asynch()

            src.drivers.otc_wps.udp.stopConsumeThread()
     

    def test_edit_config(self):
        src.drivers.otc_wps._download_config = mock_download_config
        src.drivers.otc_wps.ftp = ftplib.FTP()

        with patch.object(src.drivers.otc_wps.ftp, "storlines", return_value=None) as mock_store:
            #test default
            src.drivers.otc_wps.set_config(SetWeldingParametersRequest(params = WeldingState()))
            assert mock_store.called

            #test non zeros
            src.drivers.otc_wps.set_config(SetWeldingParametersRequest(params = WeldingState(
                amperage = 220.0,
                voltage = 12.0,
                filler_speed = 1,
                speed = 13.3,
                mode = 1   
            )))

    def test_update_jobs(self):
        src.drivers.otc_wps.jobs = {}
        src.drivers.otc_wps._download_config = mock_download_config
        src.drivers.otc_wps.ftp = ftplib.FTP()

        with patch.object(src.drivers.otc_wps.ftp, "nlst", return_value=["ASDA1ARCW.000"]) as mock_nlst:
            jobs = src.drivers.otc_wps.update_jobs()

            assert jobs["0"].amperage == 1

        with patch.object(src.drivers.otc_wps.ftp, "nlst", mock_err_nlst):
            try:
                jobs = src.drivers.otc_wps.update_jobs()
            except:
                assert True

    def test_lines_to_config(self):
        with open(os.path.dirname(os.path.realpath(__file__)) + "/ASDA1ARCW.000") as f:
            lines = f.readlines()
            cfg = src.drivers.otc_wps.lines_to_config(lines)

            assert cfg is not None
            assert cfg["AST_CONDITION"]["A_CURRENT_OUTPUT"] == "1"

    def test_parse_job(self):   
        with open(os.path.dirname(os.path.realpath(__file__)) + "/ASDA1ARCW.000") as f:
            lines = f.readlines()
            cfg = src.drivers.otc_wps.lines_to_config(lines)
            job = src.drivers.otc_wps.parse_job(cfg, 0)

            assert job is not None
            assert job.amperage == 1

    def test_arc_startstop(self):
        src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)
        
        with patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_arcstart):
            #test with valid job number
            src.drivers.otc_wps.arc_start(SetJobNumberRequest( value = 1 ))
            assert src.drivers.otc_wps.current_job_idx == 1

            #test with invalid job number
            src.drivers.otc_wps.arc_start(SetJobNumberRequest( value = 10 ))
            assert src.drivers.otc_wps.current_job_idx == 1
            assert src.drivers.otc_wps.current_job_idx != 10
            

        with patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_arcstop):
            src.drivers.otc_wps.arc_stop()

        src.drivers.otc_wps.udp.stopConsumeThread()

    def test_handleupdate(self):

        # build up a test message
        msg = bytearray()

        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<f', 1.0)[::-1])
        msg.extend(pack('<f', 2.0)[::-1])
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 0)[::-1])
        msg.extend(pack('<i', 999)[::-1])

        # mock some objects with default value
        src.drivers.otc_wps.t_current_params = MockPublisher()
        src.drivers.otc_wps.t_sp_params = MockPublisher()
        src.drivers.otc_wps.jobs = {}
        src.drivers.otc_wps.jobs["999"] = WeldingState()

        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish:
            src.drivers.otc_wps.handleUpdateResponse(msg)

            assert src.drivers.otc_wps.current_job == src.drivers.otc_wps.jobs["999"]

    def test_update_thread(self):
        rospy.Rate = MockRate
        src.drivers.otc_wps.t_current_params = rospy.Publisher("test", WeldingState)
        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect:

            src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)

            thread_state_update = Thread(target = src.drivers.otc_wps.wps_state_publisher)
            thread_state_update.do_run = True
            thread_state_update.start()

            #run the thread for 4s, then stop, feed in every 0.1s
            i = 0.
            while i < 4:
                # build up a test message
                msg = bytearray()

                msg.extend(pack('<i', 0)[::-1])
                msg.extend(pack('<i', 0)[::-1])
                msg.extend(pack('<f', 1.0 + i)[::-1])
                msg.extend(pack('<f', 2.0)[::-1])
                msg.extend(pack('<i', 0)[::-1])
                msg.extend(pack('<i', 0)[::-1])
                msg.extend(pack('<i', 0)[::-1])
                msg.extend(pack('<i', 999)[::-1])

                print "call update"
                # call handler
                src.drivers.otc_wps.handleUpdateResponse(msg)

                # go to sleep
                time.sleep(0.0005)

                i = i + 0.05

            # stop the thread
            thread_state_update.do_run = False
            thread_state_update.join()

            try:
                # check if there is no new connection lost error
                for call in mock_publish.mock_calls:
                    assert call[1][0].error == ""
            
            finally:
                src.drivers.otc_wps.udp.stopConsumeThread()

    def test_init(self):
        """Test the driver initalization

        Requirements: 
        - all of the services are advertised
        - all of the topics are published
        - the UDP connection is initalized
        """

        with patch.object(rospy, "wait_for_service", return_value=True), \
            patch.object(rospy, "get_param", mock_get_param), \
            patch.object(rospy, "init_node", return_value=None), \
            patch.object(rospy, 'spin', return_value=None), \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service_init, \
            patch.object(rospy.Publisher, '__init__', return_value=None) as mock_publisher_init, \
            patch.object(Thread, 'start', return_value=None) as mock_start_thread, \
            patch.object(Thread, 'join', return_value=None), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_bind:

            src.drivers.otc_wps.ftp_connect = mock_connect
            src.drivers.otc_wps.init()

            for sn in ["edit_config", "set_job_number", "set_params", "arc_start", "arc_stop"]:
                # both required services are advertised
                assert len([call for call in mock_service_init.mock_calls if call[1][0] == sn]) == 1

            for tn in ["jobs", "current_set_points", "current_params"]:
                # both required topics are published
                assert len([call for call in mock_publisher_init.mock_calls if call[1][0] == tn]) == 1

    def test_update_thread_conn_lost(self):
        rospy.Rate = MockRate
        src.drivers.otc_wps.t_current_params = rospy.Publisher("test", WeldingState)
        with patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(UdpConnector, 'appendToQueue', mock_appendToQueue_sendupdate), \
            patch.object(socket.socket, 'connect', return_value=True) as mock_connect, \
            patch.object(socket.socket, 'bind', return_value=True) as mock_connect:

            src.drivers.otc_wps.udp = UdpConnector("localhost", 8000)

            thread_state_update = Thread(target = src.drivers.otc_wps.wps_state_publisher)
            thread_state_update.do_run = True
            thread_state_update.start()

            #run the thread for 4s, then stop
            time.sleep(4)
            thread_state_update.do_run = False
            thread_state_update.join()
            
            try:
                #test no data received
                assert mock_publish.called
                # check if there is no new connection lost error
                for call in mock_publish.mock_calls:
                    assert call[1][0].error != ""
            
            finally:
                src.drivers.otc_wps.udp.stopConsumeThread()

            

            



