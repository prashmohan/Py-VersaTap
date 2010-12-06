#!/usr/bin/env python
"""VersaTap interfacing driver

Author: Prashanth Mohan <prashmohan@gmail.com>
        http://www.cs.berkeley.edu/~prmohan
"""

import sys
import os
import serial
import array


def bit_shift_right(byte, no_of_bits):
    return byte >> no_of_bits

def bit_shift_left(byte, no_of_bits):
    return int(get_binary(byte << no_of_bits)[-8:], 2)

def get_type_from_class(klass, content):
    for attrib in dir(klass):
        if attrib.startswith('__'):
            continue # built in function
        
        if klass.__dict__[attrib] == content:
            return attrib
    return None

def generate_binary_string(bytes):
    return array.array('B', bytes).tostring()

def get_binary(byte):
    return bin(byte)[2:]

class VersaTap:
    class VersaTapException(Exception):
        def __init__(self, message):
            self.message = message

        def __repr__(self):
            return '\n'.join([self.message, self.message.msg_type])
            
    class Message:
        MESSAGE_LENGTH = {VersaTap.MsgType.RMT_REAL_TIME_CLOCK_MSBS_CHANGE : 3,
                          VersaTap.MsgType.RMT_DCE_DATA_RECEIVED = 4,
                          VersaTap.MsgType.RMT_DCE_STATUS_CHANGED = 4,
                          VersaTap.MsgType.RMT_DTE_DATA_RECEIVED = 4,
                          VersaTap.MsgType.RMT_DTE_STATUS_CHANGE = 4,
                          VersaTap.MsgType.RMT_FLASH_MEMORY_STATUS_RESPONSE = 2,
                          VersaTap.MsgType.RMT_FLASH_MEMORY_READ_RESPONSE = 3, # incorrect
                          VersaTap.MsgType.RMT_VERSA_TAP_STATUS_RESPONSE = 6,
                          VersaTap.MsgType.RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE = 3,
                          VersaTap.MsgType.RMT_READ_CONTROL_SIGNAL_STATES_RESPONSE = 3,
                          VersaTap.MsgType.RMT_VERSA_TAP_ERROR_MESSAGE = 2,
                          VersaTap.MsgType.RMT_VERSA_TAP_DTE_HDLC_FRAME_RECEIVED = 8, #incorrect
                          VersaTap.MsgType.RMT_VERSA_TAP_DCE_HDLC_FRAME_RECEIVED = 8, #incorrect
                          }

        def set_msg_type(self, msg_type):
            self.msg_type = msg_type

        def get_type(self):
            return self.msg_type

        def __repr__(self):
            return get_type_from_class(VersaTap.MsgType, self.msg_type)
        
        def __init__(self, msg):
            self.msg_type = VersaTap.Message.get_message_type(msg[0])

            if self.msg_type == VersaTap.MsgType.RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE:
                self.orig_msg_type = ord(msg[1])
                self.status = ord(msg[2])
            elif self.msg_type == VersaTap.MsgType.RMT_DCE_DATA_RECEIVED:
                self.data = msg[3]
                real_time_clock_lsb = msg[1]
                real_time_clock_msb = msg[2]
                self.real_time_clock = ord(msg[2] + msg[1])
                self.ferror = ord(msg[0]) >> 7
                self.perr = (ord(msg[0]) << 1) >> 7
                self.breakerr = (ord(msg[0]) << 2) >> 7
            elif self.msg_type == VersaTap.MsgType.RMT_VERSA_TAP_ERROR_MESSAGE:
                self.err_code = ord(msg[1])
            elif self.msg_type == VersaTap.MsgType.RMT_REAL_TIME_CLOCK_MSBS_CHANGE:
                pass

        @staticmethod
        def get_message_length(message_type):
            return VersaTap.Message.MESSAGE_LENGTH.get(message_type)

        @staticmethod
        def get_message_type(byte):
            return bit_shift_right(bit_shift_left(ord(byte), 3), 3) # unset first 3 bytes
        
    class Status:
        SUCCESS = 0
        FAILURE = 1
    
    class MsgType:
        CMT_DATA_ACQUISITION_CONTROL = 0x01
        CMT_SET_COM_PARAMETERS = 0x2
        CMT_VERSA_TAP_STATUS_REQUEST = 0x4
        CMT_FLASH_MEMORY_STATUS_REQUEST = 0x5
        CMT_FLASH_WRITE_REQUEST = 0x6
        CMT_FLASH_MEMORY_READ_REQUEST = 0x7
        CMT_READ_CONTROL_SIGNAL_STATES_REQUEST = 0x8
        CMT_SET_DCE_COM_PARAMETERS = 0x9
        
        RMT_REAL_TIME_CLOCK_MSBS_CHANGE = 0x1
        RMT_DCE_DATA_RECEIVED = 0x2
        RMT_DCE_STATUS_CHANGED = 0x3
        RMT_DTE_DATA_RECEIVED = 0x4
        RMT_DTE_STATUS_CHANGE = 0x5
        RMT_FLASH_MEMORY_STATUS_RESPONSE = 0x6
        RMT_FLASH_MEMORY_READ_RESPONSE = 0x7
        RMT_VERSA_TAP_STATUS_RESPONSE = 0x8
        RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE = 0x9
        RMT_READ_CONTROL_SIGNAL_STATES_RESPONSE = 10
        RMT_VERSA_TAP_ERROR_MESSAGE = 11
        RMT_VERSA_TAP_DCE_HDLC_FRAME_RECEIVED = 12
        RMT_VERSA_TAP_DTE_HDLC_FRAME_RECEIVED = 13

    def __init__(self, dev_name):
        self.serial_dev = serial.Serial(dev_name, baudrate=9600, bytesize=8, \
                                        parity='N', stopbits=1, timeout=1, \
                                        xonxoff=0, rtscts=0, dsrdtr=0)


    def __data_acquisition_control(self, signalling_level):
        """signalling_level:
        0 -> RS232/TTL
        1 -> RS422/485
        2 -> Inverted RS232/TTL
        """
        bytes = [VersaTap.MsgType.CMT_DATA_ACQUISITION_CONTROL,
                 int('00' + get_binary(signalling_level) + '0101',
                     2)]
        return generate_binary_string(bytes)

    def data_acquisition_control(self, signalling_level):
        self.serial_dev.write(self.__data_acquisition_control(signalling_level))
        return self.check_for_ack(VersaTap.MsgType.RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE)
    
    def __gen_set_com_parameters(self, msg_type, baud_rate):
        if baud_rate > 100000 and baud_rate not in [115200, 230400, 460800, 921600]:
            raise Exception('Not yet implemented')
        brd_div = 44 * (10**6) / 16 / baud_rate - 1
        brd_div = get_binary(brd_div)
        if len(brd_div) <= 8:
            brd_div_lsb = '0' * (len(brd_div) - 8) + brd_div
            brd_div_msb = '0' * 8
        else:
            brd_div_lsb = brd_div[8:]
            brd_div_msb = '0' * (len(brd_div) - 16) + brd_div[:len(brd_div) - 8]
        
        bytes = [msg_type,
                 int('0' + '000' + '00' + '11', 2),
                 brd_div_lsb,
                 brd_div_msb]
        return generate_binary_string(bytes)

    def check_for_ack(self, msg_type):
        """Check if a RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE was received"""
        while True:
            msg = self.get_message()
            if msg.msg_type == VersaTap.MsgType.RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE and \
                   msg.orig_msg_type == msg_type and \
                   msg.status == VersaTap.Status.SUCCESS:
                return True
            if msg.msg_type != VersaTap.MsgType.RMT_BASIC_ACKNOWLEDGEMENT_MESSAGE:
                continue
            raise VersaTap.VersaTapException(msg)

    def gen_set_com_parameters(self, msg_type, baud_rate):
        self.serial_dev.write(self.__gen_set_com_parameters(msg_type, baud_rate))
        return self.check_for_ack(msg_type)

    def set_com_paramters(self, baud_rate):
        return self.gen_set_com_parameters(VersaTap.MsgType.CMT_SET_COM_PARAMETERS, baud_rate)

    def set_dce_com_parameters(self, baud_rate):
        return self.gen_set_com_parameters(VersaTap.MsgType.CMT_SET_DCE_COM_PARAMETERS, baud_rate)

    def __reset_device(self):
        bytes = [VersaTap.MsgType.CMT_DATA_ACQUISITION_CONTROL,
                 0]
        return generate_binary_string(bytes)

    def reset_device(self):
        """Reset the device back to normal state - as it would be if the device
        was unplugged and plugged back in"""
        self.serial_dev.write(self.__reset_device())
    
    def get_message(self):
        msg_byte1 = self.serial_dev.read(1) # read first byte to understand how many more bytes to read
        msg_bytes_remaining = self.serial_dev.read(VersaTap.Message.get_message_length(VersaTap.Message.get_message_type(msg_byte1)))
        return VersaTap.Message(''.join([msg_byte1, msg_bytes_remaining]))
    

# def main():
#     s = serial.Serial('/dev/ttyUSB2', baudrate=9600, bytesize=8, parity='N', stopbits=1, timeout=1, xonxoff=0, rtscts=0, dsrdtr=0)

# if __name__ == '__main__':
#     main()
    
