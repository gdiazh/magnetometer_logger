#!/usr/bin/python

__author__ = 'gdiaz'

import serial 
import time
from threading import Timer
import struct

class serialReceiver(object):
    def __init__(self, port, baud = 115200, debug = False):
        # self.btSocket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        self.arduino = serial.Serial(port, baud)
        self.packet = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # self.packet = [0,0,0,0]
        self.debug = debug
        self.struct = None

    def initialize(self):
        #Buetooth
        # bt_addr = "00:13:04:03:00:02" #CUBE
        # bt_addr = "30:14:11:10:08:12" #SATBOT(one axis)
        # bt_addr = "00:12:06:05:04:51" #SEGWAY
        # local_port = 1
        # TO DO: check if actually succeed
        # self.btSocket.connect((bt_addr, local_port))
        # self.btSocket.connect(("00:13:04:03:00:02", 1))
        print "Conection succed! starting comunication ..."

    def stop(self):
        print "Conection Finish. Closing ports ..."
        # self.btSocket.close()
        self.arduino.close()

    def reset(self):
        self.packet = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        # self.packet = [0,0,0,0]

    def DEBUG_PRINT(self, msg_type, msg):
        if not(self.debug): return
        if msg_type == "info":
            print chr(27)+"[0;32m"+"[INFO]: "+chr(27)+"[0m" + msg
        elif msg_type == "warn":
            print chr(27)+"[0;33m"+"[WARN]: "+chr(27)+"[0m" + msg
        elif msg_type == "error":
            print chr(27)+"[0;31m"+"[ERROR]: "+chr(27)+"[0m" + msg
        elif msg_type == "alert":
            print chr(27)+"[0;34m"+"[ALERT]: "+chr(27)+"[0m" + msg
        else:
            print "NON implemented Debug print type"

    def checksum(self, packet, sz):
        sum = 0
        for j in range(0,sz-1): sum += packet[j]
        return sum

    def read_test(self):
        data = self.arduino.read(1)
        print(ord(data))

    def read(self):
        i = 0
        k = 0
        sz = 32
        self.reset()
        while (k < 2*sz):
            # print(k)
            # byte = self.btSocket.recv(1)
            byte = self.arduino.read(1)
            # print("byte = "+str(byte))
            self.packet[i] = ord(byte)
            i+=1
            if (i==sz):
                # print(self.packet)
                chksm = self.checksum(self.packet, sz) & 0x00FF #Low byte of data checksum
                if (chksm == self.packet[sz-1] and chksm !=0 and self.packet[30] == 253 and self.packet[29] == 254):
                    self.DEBUG_PRINT("info", "frame received = "+str(self.packet))
                    p = struct.pack('BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB', self.packet[0],self.packet[1],self.packet[2],self.packet[3],self.packet[4],self.packet[5],self.packet[6],self.packet[7],self.packet[8],self.packet[9],self.packet[10],self.packet[11],self.packet[12],self.packet[13],self.packet[14],self.packet[15],self.packet[16],self.packet[17],self.packet[18],self.packet[19],self.packet[20],self.packet[21],self.packet[22],self.packet[23],self.packet[24],self.packet[25],self.packet[26],self.packet[27],self.packet[28],self.packet[29],self.packet[30],self.packet[31])
                    # p = struct.pack('BBBB', self.packet[0],self.packet[1],self.packet[2],self.packet[3])
                    self.packet = struct.unpack("fffffffBBBB", p)
                    self.struct = struct.unpack("fffffffBBBB", p)
                    # self.packet = struct.unpack("HBB", p)
                    # self.struct = struct.unpack("HBB", p)
                    print(self.struct)
                    return True
                else:
                    for j in range(0,sz-1): self.packet[j] = self.packet[j+1] #Shift Left packet
                    self.packet[sz-1] = 0 #Clean last byte to receive other packet
                    i = sz-1
                    self.DEBUG_PRINT("warn", "Bad checksum = "+str(chksm))
            k+=1
        # Packet not received Correctly
        self.DEBUG_PRINT("error", "Frame lost")
        self.reset()
        # for j in range(0,sz): self.packet[j] = 0 #Reset packet
        return False
        #try:
         #   print struct.unpack("fffi", byte)
        #except:
         #   pass

    def write(self, data):
        p = struct.pack('H', data)
        p2 = struct.unpack('BB', p)
        frame = list(p2)
        # frame.reverse()
        frame.append(0)
        frame[-1] = self.checksum(frame, len(frame)) & 0x00FF #Low byte of data checksum
        #N+1 byte frame: D1 D2 D3 D4 ... DN CS
        self.DEBUG_PRINT("info", "write = "+str(frame))
        for i in xrange(0,len(frame)):
            self.btSocket.send(chr(frame[i]))

if __name__ == '__main__':
    # ser_receiver = serialReceiver(debug = True)
    port = '/dev/ttyACM0'
    baud = 115200
    ser_receiver = serialReceiver(port, baud, debug = True)
    ser_receiver.initialize()
    # test read
    while True:
        ser_receiver.read()
    ser_receiver.stop()