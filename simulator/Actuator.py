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
import time
import struct

BROADCAST_ID      = 254

# The number of times to attempt to send a packet before raising an Exception.
NUM_ERROR_ATTEMPTS = 10
VERBOSE = True

from defs       import Instruction
from defs       import Register
from AXAngle    import AXAngle


def get_error_string(error):
    """
    Get a string to describe a Dynamixel error.

    :param error: A string that represents the error returned in a response packet.

    :returns: A string describing an error, or ``None`` if the error is undefined.

    """
    errors = []

    if error & registers.Error_Status.INPUT_VOLTAGE > 0:
        errors.append('input voltage error')
    elif error & registers.Error_Status.ANGLE_LIMIT > 0:
        errors.append('angle limit error')
    elif error & registers.Error_Status.OVERHEATING > 0:
        errors.append('motor overheating')
    elif error & registers.Error_Status.RANGE > 0:
        errors.append('range error')
    elif error & registers.Error_Status.CHECKSUM > 0:
        errors.append('checksum mismatch')
    elif error & registers.Error_Status.OVERLOAD > 0:
        errors.append('motor overloaded')
    elif error & registers.Error_Status.INSTRUCTION > 0:
        errors.append('instruction error')

    if len(errors) == 0:
        return None
    elif len(errors) == 1:
        return errors[0][0].upper() + errors[0][1:]
    else:
        s = errors[0][0].upper() + errors[0][1:]

        for i in range(1, len(errors) - 1):
            s += ', ' + errors[i][0].upper() + errors[i][1:]

        s += ' and ' + errors[-1][0].upper() + errors[-1][1:]

        return s


class DynamixelFatalError(Exception):
    def __init__(self, value):
        self.value = value

    def __str__(self):
        return self.value

def get_exception(error_code):
    if error_code == registers.Error_Status.CHECKSUM:
        return Exception('Send checksum mismatch.')
    else:
        return DynamixelFatalError(get_error_string(error_code))


class Response:
    """
    Holds the response received from the dynamixel
    """
    def __init__(self, servo_id, error, data, checksum_match):
        self.servo_id = servo_id
        self.error = error
        self.data = data
        self.checksum_match = checksum_match

    def get_error(self):
        """
        Determines whether the packet indicates an error.

        :returns: A Boolean value indicating ``True`` if an error occured,
            or ``False`` otherwise.
        """
        return self.error > 0 or self.checksum_match == False

    def get_error_str(self):
        """
        Get a text string describing one of the errors that the packet indicates.

        :returns: A string describing an error specified by the packet, or ``None``
            if no error occured.

        """

        if self.error > 0:
            return get_error_string(self.error)
        elif self.checksum_match == False:
            return 'Checksum mismatch.'
        else:
            return None



def get_response(ser):
    """
    Attempt to read a response packet.

    Throws an exception if a ``serial`` timeout occurs.

    :param ser: The ``serial`` object to use.
    :raises: ``Exception`` if a ``serial`` timeout occurs.
    :returns: A ``Response`` object.

    """

    data = []

    byte_sum = 0

    last_byte = None
    while True:
        data = ser.read()
        if data == '':
            raise Exception('Unable to read response header.')
        data_byte = struct.unpack('B', data)[0]
        if data_byte == 0xFF and last_byte == 0xFF:
            break
        last_byte = data_byte

    id_str = ser.read()
    if id_str == '':
        raise Exception('Unable to read response id.')
    servo_id = struct.unpack('B', id_str)[0]
    byte_sum += servo_id

    length_str = ser.read()
    if length_str == '':
        raise Exception('Unable to read length.')
    length = struct.unpack('B', length_str)[0]
    byte_sum += length

    error_str = ser.read()
    if error_str == '':
        raise Exception('Unable to read error.')
    error = struct.unpack('B', error_str)[0]
    byte_sum += error

    data = None

    if length > 2:
        data_str = ser.read(length-2)
        if data_str == None or len(data_str) < length-2:
            raise Exception('Unable to read response data.')
        data = []
        for d in data_str:
            b = struct.unpack('B', d)[0]
            data.append(b)
            byte_sum += b

    calc_checksum = (~byte_sum) & 0xFF

    checksum_str = ser.read()
    if checksum_str == None:
        raise Exception('Unable to read response checksum.')
    checksum = struct.unpack('B', checksum_str)[0]

    if checksum != calc_checksum:
        raise Exception('Checksum mismatch ({0} vs {1}).'.format(checksum, calc_checksum))

    return Response(servo_id, error, data, calc_checksum == checksum)


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

    '''
    def get_bulk_position(self, ids):


        AXposition = AXAngle()
        self.communication.flushInput()  # to empty the data buffer
        bytesToRead = 0x02

        p = []
        for id in ids:
            p += [bytesToRead, id, Register.CURRENT_POSITION]

        positionRequestMsg = self.make_msg(0xfe, Instruction.BULK_READ, [0x00] + p)
        print positionRequestMsg
        self.communication.send_msg(positionRequestMsg)
        ret = self.communication.recv_msg()
        print ("*******")
        print (ret)
        print ("*******")
    '''


    def sync_move(self, angles, speeds, verbose = VERBOSE, attempts = NUM_ERROR_ATTEMPTS):

        p = []
        for k in angles:

            angleAX = AXAngle()
            angleAX.setDegreeValue(angles[k])

            goal_position = int(angleAX.getValue())
            angular_speed = speeds[k]

            goal_position_low = goal_position & 0xff
            goal_position_high = (goal_position & 0xff00) >> 8
            angular_speed_low = angular_speed & 0xff
            angular_speed_high = (angular_speed & 0xff00) >> 8
            p += [k, goal_position_low, goal_position_high, angular_speed_low, angular_speed_high]
            #print([k, goal_position_low, goal_position_high, angular_speed_low, angular_speed_high])

        msg = self.make_msg(0xfe, Instruction.SYNC_WRITE, [Register.GOAL_POSITION, 0X04] + p)
        #self.communication.write_serial(msg)
        Communication.write_serial(self.communication, msg)


    def move_actuator(self, servo_id, goal_position, angular_speed, verbose = VERBOSE, attempts = NUM_ERROR_ATTEMPTS):

        for i in range(attempts):
            try:
                Communication.flush_serial(self.communication)

                goal_position_low = goal_position & 0xff
                goal_position_high = (goal_position & 0xff00) >> 8
                angular_speed_low = angular_speed & 0xff
                angular_speed_high = (angular_speed & 0xff00) >> 8
                msg = self.make_msg(servo_id, Instruction.WRITE_DATA, [Register.GOAL_POSITION, goal_position_low, goal_position_high, angular_speed_low, angular_speed_high])
                #self.communication.write_serial(msg)

                Communication.write_serial(self.communication, msg)

                response = get_response(self.communication)

                if servo_id != None and response.servo_id != servo_id:
                    raise Exception('Got packet from {0}, expected {1}.'.format(response.servo_id, servo_id))

                if response.checksum_match == False:
                    raise Exception('Checksum mismatch.')

                if response.error > 0:
                    raise get_exception(response.error)

                return response

            except DynamixelFatalError as d:
                raise d

            except Exception as e:
                if verbose:
                    print('Got exception when waiting for response from {0} on attempt {1}: {2}'.format(servo_id, i, e))

        raise Exception('Unable to read response for servo')


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
        print positionRequestMsg
        start = time.time()
        self.communication.send_msg(positionRequestMsg)
        print('espero')
        ret = self.communication.recv_msg()
        print ("*******")
        print (ret)
        print ("*******")
        positionHighByte = ret[4]
        positionHighByte = positionHighByte << 8
        positionLowByte  = ret[3]
        position = positionHighByte | positionLowByte
        AXposition.setValue(position)
        end = time.time()
        print (end-start)
        return AXposition
