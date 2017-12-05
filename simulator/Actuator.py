#! /usr/bin/env python
# -*- coding: utf-8 -*-
# Andrés Aguirre Dorelo
# MINA/INCO/UDELAR
# basic ax12 actuator control
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

import Communication

BROADCAST_ID      = 254

from defs       import Instruction
from defs       import Register
from AXAngle    import AXAngle

class Actuator:

    def __init__(self, communication):
        self.communication = communication

    def checksum_check(self, msg):
        checksum = 0
        for i in range(2, len(msg)):
            checksum = (checksum + msg[i])%256
        checksum = 255 - checksum
        return checksum

    def make_msg(self, id, instruction, parameters=[]):
        msg = []
        length_field = len(parameters) + 2
        msg = [0xff, 0xff, id, length_field, instruction] + parameters
        checksum = self.checksum_check(msg)
        msg.append(checksum)
        return msg

    def sync_write(self, joints):
        goal_position_low = goal_position & 0xff
        goal_position_high = (goal_position & 0xff00) >> 8
        angular_speed_low = angular_speed & 0xff
        angular_speed_high = (angular_speed & 0xff00) >> 8
        msg = self.make_msg(0xfe, Instruction.SYNC_WRITE, [Register.GOAL_POSITION, goal_position_low, goal_position_high, angular_speed_low, angular_speed_high])
        self.communication.send_msg(msg)

    def move_actuator(self, id, goal_position, angular_speed):
        goal_position_low = goal_position & 0xff
        goal_position_high = (goal_position & 0xff00) >> 8
        angular_speed_low = angular_speed & 0xff
        angular_speed_high = (angular_speed & 0xff00) >> 8
        msg = self.make_msg(id, Instruction.WRITE_DATA, [Register.GOAL_POSITION, goal_position_low, goal_position_high, angular_speed_low, angular_speed_high])
        self.communication.send_msg(msg)

    def set_speed_actuator(self, id, angular_speed):
        angular_speed_low = angular_speed & 0xff
        angular_speed_high = (angular_speed & 0xff00) >> 8
        msg = self.make_msg(id, Instruction.WRITE_DATA, [Register.MOVING_SPEED, angular_speed_low, angular_speed_high])
        self.communication.send_msg(msg)

    def led_state_change(self, id, led_state):
        msg = self.make_msg(id, Instruction.WRITE_DATA, [Register.LED, led_state])
        #print msg
        self.communication.send_msg(msg)

    def factory_reset(self, id=BROADCAST_ID):
        msg = self.make_msg(id, Instruction.RESET, [])
        self.communication.send_msg(msg)

    def ping(self, id=BROADCAST_ID):
        msg = self.make_msg(id, Instruction.PING, [])
        self.communication.send_msg(msg)

    def setear_id(self, newID):
        msg = self.make_msg(BROADCAST_ID, Instruction.WRITE_DATA, [Register.ID, newID])
        self.communication.send_msg(msg)

    def get_position(self, id):
        AXposition = AXAngle()
        self.communication.flushInput()  # to empty the data buffer
        bytesToRead = 0x02
        positionRequestMsg = self.make_msg(id, Instruction.READ_DATA, [Register.CURRENT_POSITION, bytesToRead])
        self.communication.send_msg(positionRequestMsg)
        ret = self.communication.recv_msg()
        print ("*******")
        print (ret)
        print ("*******")
        positionHighByte = ret[4]
        positionHighByte = positionHighByte << 8
        positionLowByte  = ret[3]
        position = positionHighByte | positionLowByte
        AXposition.setValue(position)
        return AXposition
