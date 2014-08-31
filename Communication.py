#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# comunication abstraction layer
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

import socket
import string
import time

import serial         
import sys

from pybot import pybot_client

SIM_HOST = 'localhost'
SIM_PORT = 7777

class Communication(object):

    def __init__(self):
        self.client = None

    def connect(self):
        pass

    def send_msg(self, msg):
        pass
    
    def recv_msg(self):
        pass
        
        
#for the bioloidcontrol simulator
class CommSimulator(Communication):

    def __init__(self, simulator_port = SIM_PORT, simulator_address = SIM_HOST):
        Communication.__init__(self)	
        self.port = simulator_port
        self.address = simulator_address

    def connect(self):
        client = socket.socket()
        client.connect((SIM_HOST, SIM_PORT))  
        try:
            self.client = socket.socket()
            self.client.connect((SIM_HOST, SIM_PORT))  
        except:
            print "error stablishing network connection"

    def send_msg(self, msg):
        #print "mando sim"
        try:
            for val in msg:    
                self.client.send(chr(val))
        except:
            print "problems sending package"


class CommSerial(Communication):

    def __init__(self, tty_node = "/dev/ttyUSB0", baudrate=1000000):
        Communication.__init__(self)
        self.tty_node = tty_node
        self.baudrate = baudrate

    def connect(self):
        print "conectado"
        try:
            self.client = serial.Serial()         # create a serial port object
            self.client.baudrate = self.baudrate  # baud rate, in bits/second
            self.client.port = self.tty_node      # this is whatever port your are using
            self.client.open()
        except:
            print "error stablishing serial connection"

    def send_msg(self, msg):
        #print "mando serial"
        try:
            for val in msg:    
                self.client.write(chr(val))
        except:
            print "problems sending package"
            
    def recv_msg(self):
        checksum = 0;
        dato = chr(0)
        packet = []     

        while (dato != chr(0xFF) and dato != ""):        
            dato = self.client.read(1)       
            print "primer read"

        if (dato == ""):
            return 0

        dato = self.client.read(1)
        print "segundo read"
        if (dato == chr(0xFF)):
            id_motor = ord(self.client.read(1))
            print "tercer read"
            packet.append(id_motor)

            length = ord(self.client.read(1))
            print "cuarto read"
            packet.append(length)

            error = ord(self.client.read(1))
            print "quinto read"
            packet.append(error)

            for i in range(0, length - 2):
                dato = self.client.read(1)
                print "sexto read"
                packet.append(ord(dato))

            dato = self.client.read(1)        
            print "septimo read"
            checksum = ord(dato)

        packet.append(checksum)
        return packet
    
            

class CommButia(Communication):
    
    def __init__(self):
        Communication.__init__(self)
       
    def connect(self):
        try:
            self.client = pybot_client.robot()
        except:
            print "problems connecting butiá"
    
    def send_msg(self, msg):
        try:
            #self.client.callModule('butia', str(0), '0', 'getVolt', msg)
            msg2 = [str(i) for i in msg]
            res = self.client.callModule('ax', str('0'), '0', 'sendPacket', msg2)
            print res
        except:
            print "problems sending ax msg"
