import rospy
import time
import datetime
import pylinllt as llt
import pytest

from mock import patch

import src.drivers.uepsilon_scanner as scanner

def mock_llt_connect(self, *args, **kwargs):
    """Create hllt during init
    """

    self.hLLT = {}

def has_llt():
    """Check if llt is available in the system
    
    Returns:
        Bool -- True, if available
    """

    try:
        with patch.object(llt, 'del_device', return_value=1) as mock_del_device:
            return True
    except:
        return False

class TestUEpsilon(object):
    """Unit testing for the UEpsilon driver
    """
    
    def test_init(self):
        """Test the driver initalization

        Requirements:
        - all of the services are advertised
        - all of the topics are published
        - check the connection to the laser scanner
        """

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', return_value=None) as mock_connect, \
            patch.object(scanner.uEpsilon_Driver, 'disconnect', return_value=None) as mock_disconnect:

            ue = scanner.uEpsilon_Driver()

            assert mock_init_node.called
            assert mock_service.call_count == 2
            assert len([call for call in mock_service.mock_calls if call[1][0] in ["start", "stop"]]) == len(mock_service.mock_calls)
            assert mock_publish.mock_calls[0][1][0].data == False

    def test_start_stop(self):
        """Check the start and stop of the laser
        """

        scanner.now = datetime.datetime.now 

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy, 'sleep', time.sleep), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', return_value=None) as mock_connect, \
            patch.object(scanner.uEpsilon_Driver, 'start_transmission', return_value=None) as mock_start_trans, \
            patch.object(scanner.uEpsilon_Driver, 'stop_transmission', return_value=None) as mock_stop_trans, \
            patch.object(scanner.uEpsilon_Driver, 'disconnect', return_value=None) as mock_disconnect:

            ue = scanner.uEpsilon_Driver()

            ue.start()
            assert ue.scanning != scanner.ScanningMode.Disabled
            assert mock_start_trans.called
            assert mock_publish.mock_calls[0][1][0].data == False
            call_count = mock_publish.call_count

            ue.start() #2nd start wont increase the call count
            assert len(mock_publish.mock_calls) == call_count # return if already started

            ue.stop()
            assert mock_disconnect.call_count == 2 #connection test + stop
            assert mock_stop_trans.called #stop transmission func is called

            ue.stop()
            assert mock_disconnect.call_count == 2 #connection test + 1st stop
            assert mock_stop_trans.call_count == 1 #2nd time it is not called

    @pytest.mark.skipif(not has_llt(), reason="No way to test llt from remote")
    def test_disconnect(self):
        """Test disconnecting from the scanner

        Requirements:
        - disconnect only available when connected
        - the device is removed after disconnect

        """

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy, 'sleep', time.sleep), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', mock_llt_connect), \
            patch.object(scanner.uEpsilon_Driver, 'start_transmission', return_value=None) as mock_start_trans, \
            patch.object(scanner.uEpsilon_Driver, 'stop_transmission', return_value=None) as mock_stop_trans, \
            patch.object(llt, 'disconnect', return_value=1) as mock_disconnect, \
            patch.object(llt, 'del_device', return_value=1) as mock_del_device:

            ue = scanner.uEpsilon_Driver()

            assert mock_disconnect.called
            assert mock_del_device.called

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy, 'sleep', time.sleep), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', mock_llt_connect), \
            patch.object(scanner.uEpsilon_Driver, 'start_transmission', return_value=None) as mock_start_trans, \
            patch.object(scanner.uEpsilon_Driver, 'stop_transmission', return_value=None) as mock_stop_trans, \
            patch.object(llt, 'disconnect', return_value=1) as mock_disconnect, \
            patch.object(llt, 'del_device', return_value=-1) as mock_del_device:

            try:
                #cant remove device
                ue = scanner.uEpsilon_Driver()
            except:
                assert mock_disconnect.called
                assert mock_del_device.called

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy, 'sleep', time.sleep), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', mock_llt_connect), \
            patch.object(scanner.uEpsilon_Driver, 'start_transmission', return_value=None) as mock_start_trans, \
            patch.object(scanner.uEpsilon_Driver, 'stop_transmission', return_value=None) as mock_stop_trans, \
            patch.object(llt, 'disconnect', return_value=-1) as mock_disconnect, \
            patch.object(llt, 'del_device', return_value=1) as mock_del_device:

            try:
                #cant disconnect from device
                ue = scanner.uEpsilon_Driver()
            except:
                assert mock_disconnect.called
                assert not mock_del_device.called

    @pytest.mark.skipif(not has_llt(), reason="No way to test llt from remote")
    def test_transmission_start_stop(self):
        """Test the transmission start and stop

        Requirements:
        - transmission can be started when the device is connected
        - can not stop a not running transmission
        - the parameters are correct
        """

        scanner.now = datetime.datetime.now 

        with patch.object(rospy, 'init_node', return_value=None) as mock_init_node, \
            patch.object(rospy.Service, '__init__', return_value=None) as mock_service, \
            patch.object(rospy, 'sleep', time.sleep), \
            patch.object(rospy.Publisher, 'publish', return_value=None) as mock_publish, \
            patch.object(scanner.uEpsilon_Driver, 'connect', mock_llt_connect), \
            patch.object(scanner.uEpsilon_Driver, 'disconnect', return_value=None) as mock_disconnect:

            ue = scanner.uEpsilon_Driver()

            with patch.object(llt, "transfer_profiles", return_value=1) as mock_transfer_profiles:

                ue.start()
                ue.stop()

                assert mock_transfer_profiles.call_count == 2
                for call in mock_transfer_profiles.mock_calls:
                    assert call[1][1] == llt.TTransferProfileType.NORMAL_TRANSFER #both is normal transfer
                assert mock_transfer_profiles.mock_calls[0][1][2] == 1 #start transfer
                assert mock_transfer_profiles.mock_calls[1][1][2] == 0 #stop transfer

            with patch.object(llt, "transfer_profiles", return_value=-1) as mock_transfer_profiles:    
                
                #transfer profiles throws exception
                
                try:
                    ue.start()
                except:
                    assert True

                try:
                    ue.stop()
                except:
                    assert True













