"""
ROSWELD
Version 0.0.1, March 2019
http://rosin-project.eu/ftp/rosweld

Copyright (c) 2019 PPM Robotics AS

This library is part of ROSWELD project,
the Focused Technical Project ROSWELD - ROS based framework
for planning, monitoring and control of multi-pass robot welding
is co-financed by the EU project ROSIN (www.rosin-project.eu)
and ROS-Industrial initiative (www.rosindustrial.eu).

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

import socket
import sys
import time
from datetime import datetime
from struct import pack, unpack
from threading import Thread, currentThread

from src.drivers.misc.status import STATE, status


class UdpConnector(object):
    """UDP Connector class to handle UDP connections
    """

    def __init__(self, host, port, binary_mode=True, local_port=None, name=""):
        """UdpConnector init function
        """

        # The ip address of the other party
        self.host = host
        # The TCP port of the other party
        self.port = port
        # The address of the other party
        self.addr = (self.host, self.port)
        # The first message identifier, for matching requests and responses
        self.seq = 1
        # The message queue
        self.sock_queue = []
        # True, if the queue consumer is processing a request
        self.queue_waiting = False
        # param to stop the consuming thread
        self.consume = True
        # param to set the encoding mode
        self.binary_mode = binary_mode
        # name of the parent
        self.name = "UDP %s:%d" % (name, port)

        # connect
        try:
            # Create the socket
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Setting the timeout to 3s
            self.sock.settimeout(3)
            # Setting the local port
            if local_port is not None:
                self.sock.bind(("0.0.0.0", local_port))
            # Try to initiate the connection
            self.sock.connect(self.addr)
        except Exception as e:
            print e
            status(
                self.name,
                'ERROR: Failed to create socket: %s' %
                (str(e)),
                STATE.ERROR)
            sys.exit()

        # Starts the network message queue consumer
        self.consumeThread = Thread(target=self.consumeQueue)
        self.startConsumeThread()

    def stopConsumeThread(self):
        """Stop the consuming thread
        """

        self.consumeThread.do_run = False
        self.consumeThread.join()

    def startConsumeThread(self):
        """Start the consuming thread
        """

        self.consumeThread.do_run = True
        self.consumeThread.start()

    def isQueued(self, desc):
        """Checks, if a specific item is already in the queue

           Goes through the queue, checking the list for items with the desc in the param

        Arguments:
            desc {string} -- description

        Returns:
            bool -- is queued
        """

        for item in self.sock_queue:
            if (item.desc == desc):
                return True

        return False

    def appendToQueue(self, msg, desc="Unknown", handler=None):
        """Appends a new network message to the queue

           Takes the message from the parameters, and places it into the queue.
           After this, it calls the consumeQueue, to start processing

        Arguments:
            msg {bytearray} -- message

        Keyword Arguments:
            desc {str} -- description (default: {"Unknown"})
            handler {function} -- handler (default: {None})

        Returns:
            int -- sequence number
        """

        _msg = bytearray()
        # add a new seq to message
        self.seq += 1
        if self.binary_mode:
            _msg.extend(pack('<i', self.seq)[::-1])
            _msg.extend(msg)
        else:
            _msg = '{:+05d}'.format(self.seq) + msg

        self.sock_queue.append(MessageQueueItem(self.seq, _msg, desc, handler))
        return self.seq

    def receive(self, size=1024):
        """Receive the message from the socket

        Keyword Arguments:
            size {int} -- buffer size (default: {1024})

        Returns:
            bytearray -- response
        """

        response = self.sock.recv(size)

        return response

    # Send message to the socket
    def send(self, msg):
        """Send message to the socket

        Arguments:
            msg {bytearray} -- message
        """

        self.sock.sendto(msg, self.addr)

    def consumeQueue(self):
        """Consumes a network message from the queue

           If there are messages in the queue, it takes one, and sends the message
           If the response function pointer is defined, it also waits for the response
           Runs in a new thread, to avoid blocks on the main thread
        """

        t = currentThread()
        while getattr(t, "do_run", True):
            #print "consumeQueue", len(sock_queue)

            # If there is no message in the queue, waits for 10ms
            if len(self.sock_queue) == 0:
                time.sleep(0.01)
                continue

            item = self.sock_queue[0]

            #rospy.loginfo("Sending item %s %d..."%(item.desc, item.seq))
            try:
                # Send the message
                item.tryStamps.append(datetime.now().microsecond)
                #if item.desc != "send_update":
                #    print "sending: ", item.desc
                #rospy.loginfo("Sent: %s" % item.msg)
                self.send(item.msg)
                r = self.receive(1024)
                # If we are expecting a response
                if item.handler is not None:
                    #rospy.loginfo("answer received")

                    # read the seq in the response
                    seq_back = None
                    if self.binary_mode:
                        #rospy.loginfo("Sent: %s %s" % (binascii.hexlify(item.msg), len(item.msg)))
                        #rospy.loginfo("Received: %s %s" % (binascii.hexlify(r), len(r)))
                        seq_back = unpack('>i', r[4:8])[0]
                    else:
                        #rospy.loginfo("Received: %s" % r[:-1])
                        seq_back = int(r[:-1].split(" ")[1])

                    # if the seq is not correct, we need to discard the
                    # response
                    if seq_back != item.seq:
                        status(
                            self.name, "Error: invalid seq. The response is discarded. %d != %d" %
                            (seq_back, item.seq), STATE.ERROR)

                        # receive any additional invalid messages
                        self.receive(1024)
                        self.receive(1024)
                        self.receive(1024)
                        continue

                    # call the handler function pointer
                    item.answered = datetime.now().microsecond
                    
                    if item.desc != "send_update":
                        status(self.name, "Handled: %s, time: %dms, tries: %d" % (item.desc, abs(
                            item.answered - item.requested) / 1000, len(item.tryStamps)), STATE.INFO)

                    item.handler(r)
                    #print "handled",

                # remove the first msg from the queue
                self.sock_queue.pop(0)
                #print "done"
            except socket.timeout:
                status(
                    self.name, "timeout: %d - %s" %
                    (item.seq, item.desc), STATE.ERROR)
            except Exception as e:
                status(
                    self.name, "%s UDP Error: %s" %
                    (self.addr, str(e)), STATE.ERROR)

        status(self.name, "UDP socket closed: %s:%d" % (self.host, self.port))


class MessageQueueItem(object):
    """Message queue item class

       Stores information about a message in the queue

    Arguments:
        object {object} -- parent class
    """

    def __init__(self, _seq, _msg, _desc="Unknown", _handler=None):
        self.seq = _seq
        self.msg = _msg
        self.desc = _desc
        self.handler = _handler
        self.requested = datetime.now().microsecond
        self.tryStamps = []
