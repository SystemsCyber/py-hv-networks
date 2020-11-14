#!/usr/bin/env python2.7
""" ecm_driver.py: loads the appropriate PRU binary into the appropriate PRU core, then
    mediates non-ecm J1708 traffic between the PRU and the J1708 driver.
"""

from __future__ import print_function

import pypruss
import mmap
import struct
import time
import threading
import socket
import sys
import select
import signal
import os


ECM = (6969, 6970)
DPA = (6971, 6972)

class PRUWriteThread(threading.Thread):
    def __init__(self, stopped, socket, ddr_mem):
        super(PRUWriteThread, self).__init__()
        self.stopped = stopped
        ddr_offset = ddr_addr - 0x10000000
        ddr_filelen = ddr_size + 0x10000000
        ddr_start = 0x10000000
        ddr_end = 0x10000000 + ddr_size
        self.ddr_mem = ddr_mem
        self.struct_start = ddr_start + 704#offset of send_buf in pru mem
        self.message_base = self.struct_start + 8
        self.message_ptr = self.message_base
        self.socket = socket

    def killme(self):
        self.stopped.set()

    def join(self, timeout=None):
        super(PRUWriteThread, self).join(timeout)

    def run(self):
        while not self.stopped.is_set():
            ready = select.select([self.socket],[],[],0.5)[0]
            if ready == []:
                continue

            message = self.socket.recv(256)
            (produce, consume) = struct.unpack('LL', self.ddr_mem[self.struct_start:self.struct_start+8])
            if (produce + 1) % 16 == consume:#buffer is full, drop message
                continue
            if len(message) > 42:
                message = message[:42]
                
            len_byte = struct.pack('B', len(message))
            self.ddr_mem[self.message_ptr] = len_byte
            self.ddr_mem[self.message_ptr+1:self.message_ptr+1+len(message)] = message
            produce = (produce + 1) % 16
            self.message_ptr = self.message_base + (produce * 43)
            self.ddr_mem[self.struct_start:self.struct_start+4] = struct.pack('L', produce)
            
            

class PRUReadThread(threading.Thread):
    def __init__(self, stopped, socket, ddr_mem):
        super(PRUReadThread, self).__init__()
        self.stopped = stopped
        ddr_addr = 0x4a300000
        ddr_size = pypruss.ddr_size()

        ddr_offset = ddr_addr - 0x10000000
        ddr_filelen = ddr_size + 0x10000000
        ddr_start = 0x10000000
        ddr_end = 0x10000000 + ddr_size

        self.messages_base = ddr_start+8
        self.messages_ptr = self.messages_base
        self.calls = 0
        self.socket = socket
        self.ddr_mem = ddr_mem

    def killme(self):
        self.stopped.set()


    def join(self, timeout=None):
        ddr_start = 0x10000000
        super(PRUReadThread, self).join(timeout)
        data = self.ddr_mem[ddr_start:ddr_start+696]
        msg = map(lambda x: "{:02x}".format(ord(x)), data)
        for i in range(8, len(msg), 43):
            print(",".join(msg[i:i+43]))

        produce = struct.unpack('L', data[:4])[0]
        consume = struct.unpack('L', data[4:8])[0]
        print("Produce: %d, Consume: %d" % (produce, consume))
        print("Interrupts received: %d" % self.calls)
        

    def run(self):
        ddr_start = 0x10000000
        old_consume = 0
        while not self.stopped.is_set():
            pypruss.wait_for_event(0)                                           # Wait for event 0 which is connected to PRU0_ARM_INTERRUPT
            pypruss.clear_event(0, pypruss.PRU0_ARM_INTERRUPT)                                              # Clear the event
            self.calls += 1
            (produce, consume) = struct.unpack("LL", self.ddr_mem[ddr_start:ddr_start+8])
            while consume != produce:
                length = struct.unpack("B", self.ddr_mem[self.messages_ptr])[0]
                message = struct.unpack("B"*length, self.ddr_mem[self.messages_ptr+1:self.messages_ptr+1+length])
                self.socket.sendto(''.join(map(chr,message)), ('localhost', 6970))

                consume = (consume + 1) % 16
                self.messages_ptr = self.messages_base + (consume * 43)
            if old_consume != consume:
                self.ddr_mem[ddr_start+4:ddr_start+8] = struct.pack('L', consume)
                old_consume = consume


            

#pypruss.modprobe()






my_socket = socket.socket(family=socket.AF_INET,
                          type=socket.SOCK_DGRAM)

(serveport, clientport) = ECM

try:
    my_socket.bind(('localhost', serveport))
except OSError as e:
    print(e)
    sys.exit(-1)

ddr_addr = 0x4a300000
ddr_size = pypruss.ddr_size()

ddr_offset = ddr_addr - 0x10000000
ddr_filelen = ddr_size + 0x10000000
ddr_start = 0x10000000
ddr_end = 0x10000000 + ddr_size


f = open("/dev/mem", "r+b")
ddr_mem = mmap.mmap(f.fileno(), ddr_filelen, offset=ddr_offset)  # mmap the right area


pypruss.init()                                                      # Init the PRU
pypruss.open(0)                                                     # Open PRU event 0 which is PRU0_ARM_INTERRUPT
pypruss.pruintc_init()                                              # Init the interrupt controller

stopped = threading.Event()
stopped.clear()
pru_stop_thread = PRUReadThread(stopped, my_socket, ddr_mem)
pru_send_thread = PRUWriteThread(stopped, my_socket, ddr_mem)
pru_stop_thread.start()
pru_send_thread.start()
pypruss.exec_program(0, "/opt/bbonePRU/text1.bin")                          # Load firmware "ddr_write.bin" on PRU 0

def signal_handler(signal, frame):
    print("Program termination requested")
    pru_stop_thread.killme()
    pru_send_thread.killme()
    pru_stop_thread.join()
    pru_send_thread.join()
    

signal.signal(signal.SIGINT, signal_handler)

pru_stop_thread.join()
pru_send_thread.join()


pypruss.exit()                                                      # Exit PRU
