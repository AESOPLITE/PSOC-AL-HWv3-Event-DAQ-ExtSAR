import serial
import time
import binascii
from bitstring import BitArray
import math
import numpy as np
import random

# Address = 8 for the event PSOC, 10 for the main PSOC
addrMain = 10
addrEvnt = 8

CALMASK = 1
DATAMASK = 2
TRIGMASK = 3

def openCOM(portName):
  global ser
  ser = serial.Serial(portName, 115200, timeout=.2)
  
def closeCOM():
  ser.close()
  
# Convert a string of bytes into a decimal integer
def bytes2int(str):
   if str == b'':
       return 0
   return int(str.hex(), 16)

def getBinaryString(byteList):
  strReturn = ""
  for byte in byteList:
    if byte == b'': continue
    strByte = bin(int(binascii.hexlify(byte),16))[2:]
    L2 = len(strByte)
    for i in range(8-L2):
      strByte = '0' + strByte      
    strReturn = strReturn + strByte
  return strReturn

# Put together the command header for transmission 
def mkCmdHdr(nData, cmdCode, address):
   dataByte = cmdCode
   addressNib = (address & 0x00F) << 2
   add11 = nData & 0x003
   #print("mkCmdHdr: dataByte = " + hex(dataByte))
   addressByte = (((nData & 0x00C)<<4) | addressNib) | add11
   #print("mkCmdHdr: addressByte = " + hex(addressByte))
   nib0 = ord('S').to_bytes(1,'big')
   nib1 = (dataByte & 0xF0) >> 4
   nib1 = ord(hex(nib1)[2]).to_bytes(1,'big')
   nib2 = (dataByte & 0x0F)
   nib2 = ord(hex(nib2)[2]).to_bytes(1,'big')
   nib3 = (addressByte & 0xF0) >> 4
   nib3 = ord(hex(nib3)[2]).to_bytes(1,'big')
   nib4 = (addressByte & 0x0F)
   nib4 = ord(hex(nib4)[2]).to_bytes(1,'big')
   nib5 = ord(' ').to_bytes(1,'big')
   nib6 = ord('x').to_bytes(1,'big')
   nib7 = ord('y').to_bytes(1,'big')
   nib8 = ord('W').to_bytes(1,'big')
   cmd1 = nib0 + nib1 + nib2 + nib3 + nib4 + nib5 + nib6 + nib7 + nib8
   CR = 13
   end1 = CR.to_bytes(1,'big')   # CR
   LF = 10
   end2 = LF.to_bytes(1,'big')   # LF
   return cmd1 + cmd1 + cmd1 + end1 + end2

# Assemble one data byte for transmission to the main PSOC
def mkDataByte(dataByte, address, byteID):
   addressByte = ((address & 0x00F) << 2) | ((byteID & 0x00C)<<4) | (byteID & 0x003)
   #print("  mkDataByte: dataByte= " + hex(dataByte))
   #print("  mkDataByte: addressByte= " + hex(addressByte))
   nib0 = ord('S').to_bytes(1,'big')
   nib1 = (dataByte & 0xF0) >> 4
   nib1 = ord(hex(nib1)[2]).to_bytes(1,'big')
   nib2 = (dataByte & 0x0F)
   nib2 = ord(hex(nib2)[2]).to_bytes(1,'big')
   nib3 = (addressByte & 0xF0) >> 4
   nib3 = ord(hex(nib3)[2]).to_bytes(1,'big')
   nib4 = (addressByte & 0x0F)
   nib4 = ord(hex(nib4)[2]).to_bytes(1,'big')
   nib5 = ord(' ').to_bytes(1,'big')
   nib6 = ord('x').to_bytes(1,'big')
   nib7 = ord('y').to_bytes(1,'big')
   nib8 = ord('W').to_bytes(1,'big')
   cmd1 = nib0 + nib1 + nib2 + nib3 + nib4 + nib5 + nib6 + nib7 + nib8
   CR = 13
   end1 = CR.to_bytes(1,'big')   # CR
   LF = 10
   end2 = LF.to_bytes(1,'big')   # LF
   return cmd1 + cmd1 + cmd1 + end1 + end2

def getData(address, debug = False): 
    ret = b''
    if debug: print("Entering getData for PSOC address " + str(address))
    success = False
    while not success:
        for i in range(360):  #Sometimes the read starts with blank bytes, so search for the header
            ret = ser.read(1)
            if debug: print("getData: i= " + str(i) + " ret = " + str(ret))
            if ret == b'\xDC': break
            if ret != b'':
                print("getData: expected 'DC' but received " + str(ret))
        if ret != b'\xDC':
            print("getData: failed to find the header 'DC'")
            if address > 0: readErrors(address)
            raise IOError("getData: failed to find the header 'DC'")
            return

        ret = ser.read(2)
        if debug: print("getData: remainder of header = " + str(ret))
        if ret != b'\x00\xFF':
            print("getData: invalid header returned: b'\\xDC' " + str(ret))
            return
        ret = ser.read(1)
        dataLength = bytes2int(ret)
        command = ser.read(1)
        if command == b'\xDE' or command == b'\xDF' or command == b'\xDB' or command == b'\xDD' or command == b'\xDA' or command == b'\xde' or command == b'\xdf' or command == b'\xdb' or command == b'\xdd' or command == b'\xda':
            nCmdData = bytes2int(ser.read(1))
            if nCmdData != 0: print("getData: # command bytes " + str(nCmdData) + " != 0 for packet " + str(command))
            dataList = []
            byteList = []
            for m in range(dataLength):
                byte = ser.read()
                dataList.append(bytes2int(byte))
                byteList.append(byte)
            numPad = 3 - dataLength%3
            if numPad == 3: numPad = 0
            #print("getData: dataLength = " + str(dataLength) + " # padding bytes=" + str(numPad))
            for m in range(numPad):
                byte = ser.read()
                #print("getData: padding byte = " + str(byte))
            trailer = ser.read(3)
            if trailer != b'\xff\x00\xff':
                print("getData: bad trailer = " + str(trailer))
            if command == b'\xDE' or command == b'\xde':    # housekeeping packet
                print("getData: housekeeping packet received with " + str(dataLength) + " bytes")
                printHousekeeping(dataList, byteList)
            elif command == b'\xDF' or command == b'\xdf':  # tracker housekeeping packet
                print("getData: tracker housekeeping packet received with " + str(dataLength) + " bytes")
                printTkrHousekeeping(dataList)
            elif command == b'\xDB' or command == b'\xdb':
                print("getData: TOF debug event data packet received with " + str(dataLength) + " bytes") 
            elif command == b'\xDD' or command == b'\xdd':  # data packet
                print("getData: event data packet received with " + str(dataLength) + " bytes")
            elif command == b'\xDA' or command == b'\xda':  # error record
                print("getData: error record received with " + str(dataLength) + " bytes")
                for i in range(dataLength):
                    ret = byteList[i]
                    print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex")
        else: 
            success = True
            #print("getData: received data for command " + str(command))
    if debug: print("getData: command = " + str(command) + " " + str(bytes2int(command)) + " " + str(command.hex()) + " data length = " + str(dataLength))
    nCmdBytes = bytes2int(ser.read(1))
    if debug: print("getData: number of command data bytes = " + str(nCmdBytes)) 
    nPadding = 3 - dataLength%3
    if nPadding == 3: nPadding = 0
    if debug: print("getData: number of padding bytes = " + str(nPadding))
    dataLength = dataLength - nCmdBytes
    cmdDataBytes = []
    for i in range(nCmdBytes):
        ret = ser.read(1)
        cmdDataBytes.append(ret)
        if debug: print("getData: command data byte " + str(i) + ": " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex")
    dataBytes = []
    for i in range(dataLength):
        ret = ser.read(1)
        dataBytes.append(ret)
        if debug: print("getData: output data byte " + str(i) + ": " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex")
    for i in range(nPadding):
        ret = ser.read(1)
        if debug: print("getData: padding byte " + str(i) + ": " + str(ret))
    trailer = ser.read(3)
    if debug: print("getData: trailing bytes: " + str(trailer))
    if trailer != b'\xFF\x00\xFF':
        print("getData: invalid trailer returned: " + str(trailer))
    if debug: print("getData: command=" + str(command) + " # cmdDataBytes=" + str(len(cmdDataBytes)) + " # dataBytes=" + str(len(dataBytes)))
    return command,cmdDataBytes,dataBytes

# Retrieve the data for a short data packet coming from the PSOC (<= 3 bytes)
# This is no longer used, because of the addition of the command echo to the returning data
def getShortData(address, nBytes):
    ret = b''
    for i in range(36):  #Sometimes the read starts with blank bytes, so search for the header
        ret = ser.read(1)
        #print("getShortData: i= " + str(i) + " ret = " + str(ret))
        if ret == b'\xDC':
            print("getShortData: wrong header 'DC' found, expected 'DB'")
            break
        if ret == b'\xDB': break
    if ret != b'\xDB':
        print("getShortData: failed to find the header 'DB' for a short data return")
        return b''
    ret = ser.read(2)
    #print("getShortData: remainder of header = " + str(ret))
    if ret != b'\x00\xFF':
        print("getShortData: invalid header returned: b'\\xDB' " + str(ret))
    if nBytes >= 3:
        ret = ser.read(3)
    elif nBytes == 2:
        ret = ser.read(2)
        #print("getShortData: two bytes of data = " + str(ret))
        extra = ser.read(1)
        #print("getShortData: extra byte = " + str(extra))
    else:
        ret = ser.read(1)
        ser.read(2)
    trailer = ser.read(3)
    if trailer != b'\xFF\x00\xFF':
        print("getShortData: invalid trailer returned: " + str(trailer))
    return ret

def startHouseKeeping(interval, tkrRates):
    print("startHouseKeeping: interval between housekeeping packets set to " + str(interval) + " seconds.")
    cmdHeader = mkCmdHdr(2, 0x57, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(interval, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(tkrRates, addrEvnt, 2)
    ser.write(data2)
    
def startTkrHouseKeeping(interval):
    print("startTkrHouseKeeping: interval between tracker housekeeping packets set to " + str(interval) + " minutes.")
    cmdHeader = mkCmdHdr(1, 0x5C, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(interval, addrEvnt, 1)
    ser.write(data1)
    
def stopHouseKeeping():
    print("stopHouseKeeping: no more housekeeping packets will be sent")
    cmdHeader = mkCmdHdr(1, 0x58, addrEvnt)
    ser.write(cmdHeader)
    
def stopTkrHouseKeeping():
    print("stopTkrHouseKeeping: no more tracker housekeeping packets will be sent")
    cmdHeader = mkCmdHdr(1, 0x5D, addrEvnt)
    ser.write(cmdHeader)

def getPmtRates():
    print("getPmtRates: getting singles rates from all PMTs")
    cmdHeader = mkCmdHdr(0, 0x53, addrEvnt)
    ser.write(cmdHeader)    
    time.sleep(0.2)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    timeInterval = float(bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1]))/200.
    print("getPmtRates: time interval = " + str(timeInterval) + " seconds")
    if timeInterval == 0:
        print("getPmtRates error: zero time interval")
        return
    Ch1 = bytes2int(dataBytes[2])*256 + bytes2int(dataBytes[3])
    Ch2 = bytes2int(dataBytes[4])*256 + bytes2int(dataBytes[5])
    Ch3 = bytes2int(dataBytes[6])*256 + bytes2int(dataBytes[7])
    Ch4 = bytes2int(dataBytes[8])*256 + bytes2int(dataBytes[9])
    Ch5 = bytes2int(dataBytes[10])*256 + bytes2int(dataBytes[11])
    #print("getPmtRates: Ch1= " + str(Ch1))
    #print("getPmtRates: Ch2= " + str(Ch2))
    #print("getPmtRates: Ch3= " + str(Ch3))
    #print("getPmtRates: Ch4= " + str(Ch4))
    #print("getPmtRates: Ch5= " + str(Ch5))
    Guard = float(Ch1)/(timeInterval)
    T3 = float(Ch2)/(timeInterval)
    T1 = float(Ch3)/(timeInterval)
    T4 = float(Ch4)/(timeInterval)
    T2 = float(Ch5)/(timeInterval)
    print("Guard rate = " + str(Guard) + " Hz")
    print("T3 rate =    " + str(T3) + " Hz")
    print("T1 rate =    " + str(T1) + " Hz")
    print("T4 rate =    " + str(T4) + " Hz")
    print("T2 rate =    " + str(T2) + " Hz")
    
def getTkrLyrRates():
    print("getTkrLyrRates: getting rates from all tracker layers")
    cmdHeader = mkCmdHdr(0, 0x49, addrEvnt)
    ser.write(cmdHeader)    
    time.sleep(2)
    dataReturned = getTkrHousekeeping()
    if dataReturned[0] != b'\x6D':
        print("getTkrLyrRates: byte code 0x6D not returned! Got instead " + str(dataReturned[0]))
        return
    numLayers = bytes2int(dataReturned[1])
    if numLayers < 1 or numLayers > 8:
        print("getTkrLyrRates: bad number of layers " + str(numLayers) + " returned!")
        return
    for lyr in range(numLayers):
        byte2 = bytes2int(dataReturned[2+lyr])
        byte1 = bytes2int(dataReturned[3+lyr])
        print("getTkrLyrRates: byte1 = " + str(byte1) + " and byte2 = " + str(byte2))
        print("getTkrLyerRates: layer " + str(lyr+1) + ": rate = " + str(byte2*256 + byte1) + " Hz")
        
def getLyrTrgCnt(FPGA):
    cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x6C, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0, addrEvnt, 3)
    ser.write(data3)
    time.sleep(1.2)
    rate = []
    for brd in FPGA:
        if brd < 0 or brd > 7: continue
        cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
        ser.write(cmdHeader)
        data1 = mkDataByte(brd, addrEvnt, 1)
        ser.write(data1)
        data2 = mkDataByte(0x6D, addrEvnt, 2)
        ser.write(data2)
        data3 = mkDataByte(0, addrEvnt, 3)
        ser.write(data3)
        time.sleep(0.1)
        dataReturned = getTkrHousekeeping()
        rate.append(bytes2int(dataReturned[7])*256 + bytes2int(dataReturned[8]))
    return rate

def loadEventPSOCrtc():
    print("Loading the RTC setting from the main PSOC to the event PSOC")
    cmdHeader = mkCmdHdr(0, 0x32, addrMain)
    ser.write(cmdHeader)
    #time.sleep(0.1)
    #ret = ser.read(9)
    #print("loadEventPSOCrtc: command = " + str(ret))
    #for i in range(10):
    #    ret = ser.read(9)
    #    print("loadEvenPSOCrtc: data " + str(i) + " is " + str(ret))

def hardResetEventPSOC():
    print("Sending a hard reset pulse to the Event PSOC")
    cmdHeader = mkCmdHdr(0, 0x48, addrMain)
    ser.write(cmdHeader)
    time.sleep(0.1)
    
def softResetEventPSOC():
    print("Sending a software reset to the Event PSOC")
    cmdHeader = mkCmdHdr(0, 0x49, addrMain)
    ser.write(cmdHeader)
    time.sleep(0.1)

def LED2(onOFF, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x06, PSOCaddress)
    ser.write(cmdHeader)
    if onOFF == "on": data1 = mkDataByte(1, PSOCaddress, 1)
    else: data1 = mkDataByte(0, PSOCaddress, 1)
    ser.write(data1)
    print("LED2: turning LED number 2 " + onOFF + " for PSOC address " + str(PSOCaddress))
    #ret = ser.read(9)
    #print("LED2: command sent to PSOC " + str(PSOCaddress) + " is " + str(cmdHeader)) 
    #if PSOCaddress == 10: print("LED2: command made in main PSOC = " + str(ret))
    
# Reset the logic and counters
def logicReset(PSOCaddress):
    cmdHeader = mkCmdHdr(0, 0x38, PSOCaddress)
    ser.write(cmdHeader)
    cmd,cmdData,dataBytes = getData(PSOCaddress)
    count = bytes2int(dataBytes[0])*65536 + bytes2int(dataBytes[1])*256 + bytes2int(dataBytes[2])
    print("logicReset: the logic of PSOC " + str(PSOCaddress) + " was reset at clkCnt= " + str(count))
        
# Set the trigger prescale registers
def setTriggerPrescale(whichOne, count):
    if (whichOne == "Tracker"):
        ID = 1
    elif (whichOne == "PMT"):
        ID = 2
    else:
        print("setTriggerPrescale: unrecognized trigger type " + whichOne)
        return
    cmdHeader = mkCmdHdr(2, 0x39, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(ID, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(count, addrEvnt, 2)
    ser.write(data2)
    
def getTriggerPrescale(whichOne):
    if (whichOne == "Tracker"):
        ID = 1
    elif (whichOne == "PMT"):
        ID = 2
    else:
        print("getTriggerPrescale: unrecognized trigger type " + whichOne)
        return 0
    cmdHeader = mkCmdHdr(1, 0x62, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(ID, addrEvnt, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    return bytes2int(dataBytes[0])
        
def tkrSetCRCcheck(choice):
    if choice == "yes":
        ch=1
    else: 
        ch=0
    cmdHeader = mkCmdHdr(1, 0x4E, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(ch, addrEvnt, 1)
    ser.write(data1)
    print("tkrSetCRCcheck: will CRC checks be made on Tracker hit lists?: " + choice)

def setSettlingWindowAll(count):
    if count > 126:
        print("setSettlingWindow: input count of " + str(count) + " is too large. Must be < 127")
        return
    cmdHeader = mkCmdHdr(1, 0x3A, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(count, addrEvnt, 1)
    ser.write(data1)
    print("setSettlingWindow: setting the comparator settling time window for all channels to " + str(count) + " counts")
    
def setSettlingWindow(chan, count):
    if count > 126:
        print("setSettlingWindow: input count of " + str(count) + " is too large. Must be < 127")
        return
    cmdHeader = mkCmdHdr(2, 0x3A, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(chan, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(count, addrEvnt, 2)
    ser.write(data2)
    print("setSettlingWindow: setting the comparator settling time window for channel " + str(chan) + " to " + str(count) + " counts")
    
def setPeakDetResetWait(count):
    if count > 126:
        print("setPeakDetResetWait: input count of " + str(count) + " is too large. Must be < 127")
        return
    cmdHeader = mkCmdHdr(1, 0x4B, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(count, addrEvnt, 1)
    ser.write(data1)
    print("setPeakDetResetWait: setting the peak detector settling wait time to " + str(count) + " counts")

def setTkrRatesMuliplier(setting):
    if setting < 1 or setting > 255:
        print("setTkrRatesMuliplier: invalid setting " + str(setting))
        return
    cmdHeader = mkCmdHdr(1, 0x60, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(setting, addrEvnt, 1)
    ser.write(data1)
    print("setTkrRatesMuliplier: multiplying the tracker monitoring period by " + str(setting))
    
def enableTrigger():
    cmdHeader = mkCmdHdr(1, 0x3B, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(1, addrEvnt, 1)
    ser.write(data1)
    print("Enabling the trigger now")
    
def disableTrigger():
    cmdHeader = mkCmdHdr(1, 0x3B, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, addrEvnt, 1)
    ser.write(data1)
    print("Disabling the trigger now")
    
def resetTrackerLogic():
    cmdHeader = mkCmdHdr(0, 0x47, addrEvnt)
    ser.write(cmdHeader)
    print("Resetting the state machines on all Tracker boards")

def tkrCalibrateFPGAstart(FPGAaddress):
    print("tkrCalibrateFPGAstart: starting the input timing delay calibration for FPGA " + str(FPGAaddress))
    cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x81, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, addrEvnt, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'81'"):
        print("tkrCalibrateFPGAstart: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'81'")   

def tkrFPGAtimingReset(FPGAaddress):
    print("tkrFPGAtimingReset: resetting the input timing delay to center for FPGA " + str(FPGAaddress))
    cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x82, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, addrEvnt, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'82'"):
        print("tkrFPGAtimingReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'82'")  

def tkrFPGAtimingIncrement(sense):
    inc = 99
    if sense == "up":
        inc = 1
    elif sense == "down":
        inc = 0
    else:
        print("tkrFPGAtimingIncrement: the argument must be 'up' or 'down'")
        return
    print("tkrFPGAtimingIncrement: incrementing " + sense + " the input timing delay for FPGA " + str(FPGAaddress))
    cmdHeader = mkCmdHdr(4, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x83, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, addrEvnt, 3)
    ser.write(data3)
    data4 = mkDataByte(inc, addrEvnt, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'83'"):
        print("tkrFPGAtimingIncrement: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'83'")  

def calibrateAllFPGAinputTiming():
    print("Start calibrating the FPGA input timing delays on all boards")
    cmdHeader = mkCmdHdr(1, 0x48, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(8, addrEvnt, 1)
    ser.write(data1)
    time.sleep(2)
    print("Done with calibrating the FPGA input timing delays.")
    
def calibrateFPGAinputTiming(FPGAaddress):
    print("Start calibrating the FPGA input timing delays on FPGA " + str(FPGAaddress))
    cmdHeader = mkCmdHdr(1, 0x48, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, addrEvnt, 1)
    ser.write(data1)
    time.sleep(.2)
    #tkrCalibrateFPGAstart(FPGAaddress)
    #nIterations = 2
    #for chip in range(12):
    #    if FPGAaddress == 7 and chip == 4: continue
    #    print("    read the configuration of chip " + str(chip) + " " + str(nIterations) + " times, to calibrate the input timing")
    #    for iter in range(nIterations): tkrGetASICconfig(FPGAaddress, chip, True)
    #tkrFPGAtimingReset(FPGAaddress)
    print("Done with calibrating the FPGA input timing delays on FPGA " + str(FPGAaddress))

# -------------------------------------- ASIC Header --------------------------------

def parseASICHeader(bitString):
  return {
  "overflowBit": getChipOverflowBit(bitString), 
  "numberOfClusters": getChipNumberOfClusters(bitString),
  "errorBit": getChipErrorBit(bitString),
  "parityErrorBit": getChipParityErrorBit(bitString),
  "address": getChipDataAddress(bitString)
  }

def getChipOverflowBit(bitString):
  return bitString[0]
  
def getChipNumberOfClusters(bitString):
  return int(bitString[2:6], 2)
  
def getChipErrorBit(bitString): # takes ASIC header
  return bitString[6]
  
def getChipParityErrorBit(bitString):
  return bitString[7]
  
def getChipDataAddress(bitString): #returns a string address, map to strips
  return bitString[8:12]
  
# ------------------------------------------------------------------------------------

def CRC6(bitString): # The key is 1'100101
  #print("bitString= " + bitString)
  divisor = "1100101"
  result = bitString 
  for i in range(len(result) - len(divisor)+1):
    #print result#result[0:(len(result)-len(divisor)+1)] , result[(len(result)-len(divisor)+1):]
    if (result[i] == "1"):
      for j in range(len(divisor)):
          if (result[i+j] == divisor[j]):   # Exclusive OR
            result = result[0:i+j] + "0" + result[i+j+1:len(result)]
          else:
            result = result[0:i+j] + "1" + result[i+j+1:len(result)]
    #print " "*i+divisor
  return result[len(result)-6:]
  
def ParseASIChitList(bitString, verbose):
  pointer = 0    
  firstStripChip = [] 
  Hits = []
  if bitString == "":
    print("    Error: ASIC hit list is empty")
    return [-1,0]
  if bitString[pointer:pointer+8] != "11100111": 
    print("    Error: ASIC hit list " + bitString[pointer:pointer+8] + "... does not begin with 11100111")
    return [-2,0]
  else:
    rc = 0
    pointer = pointer + 9
    FPGAaddress = int(bitString[pointer:pointer+7],2)
    if verbose: print("      ASIC hit list for FPGA address " + str(FPGAaddress))
    pointer = pointer + 7
    FPGAheader = bitString[pointer:pointer+12]
    pointer = pointer + 12
    numberOfChips = int(FPGAheader[8:12],2)
    if verbose: print("          Event tag= " + str(int(FPGAheader[0:7],2)) + " Error flag= " + FPGAheader[7] + " Number of chips= " + str(numberOfChips))
    nHits = 0
    if (numberOfChips != 0):
      for chip in range(numberOfChips):
        if len(bitString) < pointer + 12: continue
        asicHead=parseASICHeader(bitString[pointer:pointer+12])
        numberOfClusters = asicHead["numberOfClusters"]
        nHits = nHits + numberOfClusters
        chipNum= int(asicHead["address"],2)
        if chipNum>12:
            rc = rc + 1
            print("    Error: Chip number " + str(chipNum) + " > 12")
        if verbose:
            print("          Chip {:d}: {:d} clusters, Overflow={:s}, Error={:s}, Parity error={:s}".format(chipNum,numberOfClusters,asicHead["overflowBit"],asicHead["errorBit"],asicHead["parityErrorBit"]))
        firstStripList = printChipClusters(bitString[pointer:(pointer+12+numberOfClusters*12)], verbose)
        if firstStripList == []: rc = rc + 1
        for item in firstStripList:
            firstStripChip.append(64*(chipNum+1) - item)                 #append all the first strip hit, for each cluster, for each chip 
        pointer = pointer + 12 + numberOfClusters*12
    CRC = bitString[pointer:pointer+6]
    #if verbose: print("      CRC= " + CRC)
    newCRC = CRC6('1'+bitString[0:pointer])    # The FPGA CRC calculation included the start bit that was later suppressed.
    #if verbose: print("      CRC= " + newCRC)
    if CRC != newCRC: 
      rc = rc + 1
      print("    CRC mismatch: received " + CRC + " and calculated " + newCRC)
    if bitString[pointer+6:pointer+8] != "11": 
      rc = rc + 1
      print("    Error: the trailing '11' bits are missing")
    return [rc,FPGAaddress,firstStripChip]

def printChipClusters(bitString, verbose): # takes a chip header and following clusters
  firstStripList = []                        #list of all the first strip hit list, to be appended over each cluster and each chip hit
  if bitString == "":
    if verbose: print("    Error:      Empty cluster list. . .")
    return firstStripList
  pointer = 12
  nclust = getChipNumberOfClusters(bitString)
  if len(bitString) != 12 + 12*nclust:
    print("    Error:       Wrong length cluster list. " + str(len(bitString)) + " bits for " + str(nclust) + " clusters.")
    return firstStripList
  for cluster in range(nclust):
    firstStrip = getChipFirstStrip(bitString[(pointer):(pointer+12)])
    ClusterWidth = getChipClusterWidth(bitString[pointer:pointer+12])
    clusterLoc = firstStrip + 0.5 + (ClusterWidth-1.0)/2.
    if verbose: print("              Cluster width={:d}   First strip={:d}".format(ClusterWidth,firstStrip))
    firstStripList.append(clusterLoc)
    pointer = 12 + pointer
  return  firstStripList

def getChipClusterWidth(bitString): #
  return int(bitString[0:6],2) + 1
  
def getChipFirstStrip(bitString):
  return int(bitString[6:12], 2)

# Load 2 bytes into an i2c register
def tkrLoadi2cReg(FPGA,i2cAddress,regID,byte1,byte2):
  address = addrEvnt
  cmdHeader = mkCmdHdr(7, 0x10, address)
  ser.write(cmdHeader)
  data1 = mkDataByte(FPGA, address, 1)
  ser.write(data1)
  data2 = mkDataByte(0x45, address, 2)
  ser.write(data2)
  data3 = mkDataByte(0x04, address, 3)
  ser.write(data3)
  data4 = mkDataByte(i2cAddress, address, 4)
  ser.write(data4)
  data5 = mkDataByte(regID, address, 5)
  ser.write(data5)  
  data6 = mkDataByte(byte1, address, 6)
  ser.write(data6) 
  data7 = mkDataByte(byte2, address, 7)
  ser.write(data7) 
  time.sleep(0.1)
  echo = getTkrEcho()
  if (str(binascii.hexlify(echo)) != "b'45'"):
      print("tkrLoadi2cReg: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'45'") 

# Read 2 bytes from an i2c register. In general the pointer register has to be set first
def tkrReadi2cReg(FPGA,i2cAddress):
  address = addrEvnt
  cmdHeader = mkCmdHdr(4, 0x10, address)
  ser.write(cmdHeader)
  data1 = mkDataByte(FPGA, address, 1)
  ser.write(data1)
  data2 = mkDataByte(0x46, address, 2)
  ser.write(data2)
  data3 = mkDataByte(0x01, address, 3)
  ser.write(data3)
  data4 = mkDataByte(i2cAddress, address, 4)
  ser.write(data4)
  time.sleep(0.1)
  cmd,cmdData,dataBytes = getData(addrEvnt)  
  result = getBinaryString(dataBytes[1:3])
  return result

def tkrGetBusVoltage(FPGA, item):
  #print("Reading the bus voltage for supply " + item + " on board " + str(FPGA))

  i2cAddress = i2cAddresses[item]
  tkrLoadi2cReg(FPGA, i2cAddress, 0x02, 0x00, 0x00) #Setting the pointer register to 2 for bus voltage
 
  response = tkrReadi2cReg(FPGA, i2cAddress)

  intResponse = int(response,2)
  busVoltage = 1.25*intResponse/1000.
  print("  Bus voltage for " + item + " is " + str(busVoltage) + " volts on board " + str(FPGA))

  return busVoltage

def tkrGetShuntCurrent(FPGA, item):
  #print("Reading the shunt voltage for supply " + item + " on board " + str(FPGA))
  i2cAddress = i2cAddresses[item]
  tkrLoadi2cReg(FPGA, i2cAddress, 0x01, 0x00, 0x00) #Setting the pointer register to 1 for shunt voltage
  
  response = tkrReadi2cReg(FPGA, i2cAddress)
  intResponse = int(response,2)

  shuntVoltage = 2.5*intResponse/1000000.
  if item == 'bias100': R=100.
  else: R=0.03
  shuntCurrent = shuntVoltage*1000./R
  if item == 'bias100':
    V = shuntVoltage * 1000.
    I = shuntCurrent * 1000.
    print("  Shunt voltage for " + item + " = " + str(V) + " mV; Shunt current = " + str(I) + " microamps for board " + str(FPGA))
  else: 
    print("  Shunt voltage for " + item + " = " + str(shuntVoltage) + " mV; Shunt current = " + str(shuntCurrent) + " milliamps for board " + str(FPGA))
  return shuntVoltage  

def tkrGetTemperature(FPGA):
  #print("tkrGetTemperature: reading the temperature on board " + str(FPGA))
  address = addrEvnt
  
  i2cAddress = i2cAddresses['temp']
  # Set config reg to 01100000,  the two 1's set the resolution to high
  tkrLoadi2cReg(FPGA,i2cAddress,0x01,0x60,0x00)

  # Set the i2c pointer register to x00 for temperature
  tkrLoadi2cReg(FPGA,i2cAddress,0x00,0x00,0x00)

  # Read the register
  result = tkrReadi2cReg(FPGA,i2cAddress)
  #print("tkrGetTemperature: result= " + result)
  Temperature = BitArray(bin=result)
  Tcelsius = (0.25/4.0)*(Temperature.int/16.)
  print("tkrGetTemperature: temperature for tracker board " + str(FPGA) + " is " + str(Tcelsius) + " degrees Celsius")
  
  # Reset the chip
  tkrLoadi2cReg(FPGA,i2cAddress,0x01,0x61,0x00)
  return Tcelsius

def sendTkrCalStrobe(FPGA, trgDelay, trgTag, verbose):
    cmdHeader = mkCmdHdr(3, 0x42, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(trgDelay, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(trgTag, addrEvnt, 3)
    ser.write(data3)
    if verbose: print("sendTkrCalStrobe: calibration strobe sent to FPGA " + str(FPGA) + " for trigger tag " + str(trgTag) + " and delay " + str(trgDelay))
    time.sleep(0.01)
    nBytes = 9
    if verbose: print("sendTkrCalStrobe: expecting " + str(nBytes) + " bytes coming back from the tracker.")

    command,cmdDataBytes,byteList = getData(addrEvnt)
    
    if (len(byteList) != 9): print("sendTkrCalStrobe: wrong number " + str(len(bytesList)) + " of bytes returned")
    i=0
    if verbose:
        for byte in byteList:   
            print("  Byte " + str(i) + " is " + str(byte.hex()))
            i=i+1
    theStuff = getBinaryString(byteList[0:9])
    if verbose: print("Bits returned = " + theStuff)
    fpgaBack = (bytes2int(byteList[0]) & 0x38)>>3
    if fpgaBack != FPGA: print("sendTkrCalStrobe: wrong FPGA address returned: " + str(fpgaBack))
    if theStuff[5:17] != "111111100001": print("sendTkrCalStrobe: wrong first pattern received")
    if theStuff[17:29] != "010011001111": print("sendTkrCalStrobe: wrong second pattern received")
    if verbose:
        print("Pat1= " + theStuff[5:17])
        print("Pat2= " + theStuff[17:29])
        print("t0  = " + theStuff[29:41])
        print("ToT = " + theStuff[41:53])
        print("hits= " + theStuff[53:65])
    return theStuff[53:65]

# Read back tracker calibration data after executing a calibration strobe
# The trigger tag should match that given in the strobe command.
def readCalEvent(trgTag, verbose):
    cmdHeader = mkCmdHdr(1, 0x43, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(trgTag, addrEvnt, 1)
    ser.write(data1)
    if verbose: print("readCalEvent: reading back calibration strobe data for tag " + str(trgTag))
    # Wait for an event to show up
 
    command,cmdDataBytes,dataList = getData(addrEvnt)
    
    iPtr = 4
    numBoards = bytes2int(dataList[iPtr])
    if numBoards != 1: print("readCalEvent: wrong number of boards " + str(numBoards) + " in calibration event")
    #iPtr = iPtr + 1
    #brdNum = bytes2int(dataList[iPtr])
    iPtr = iPtr + 1
    nBytes = bytes2int(dataList[iPtr])
    iPtr = iPtr + 1
    print(" Hit list from tracker board with " + str(nBytes) + " bytes:")
    hitList = []
    for hit in range(nBytes):
        #print("                      " + str(hit) + "  " + hex(dataList[iPtr]))
        hitList.append(dataList[iPtr])
        iPtr = iPtr + 1
    print("           Hit list= " + getBinaryString(hitList))
    if verbose: rc = ParseASIChitList(getBinaryString(hitList),True)[0]
    return hitList
            
def setTKRlayers(lyr0, lyr1, lyr2, lyr3, lyr4, lyr5, lyr6, lyr7):
    print("setTKRlayers: setting the configuration of tracker layers:")
    cmdHeader = mkCmdHdr(8, 0x59, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(lyr0, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(lyr1, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(lyr2, addrEvnt, 3)
    ser.write(data3)
    data4 = mkDataByte(lyr3, addrEvnt, 4)
    ser.write(data4)
    data5 = mkDataByte(lyr4, addrEvnt, 5)
    ser.write(data5)
    data6 = mkDataByte(lyr5, addrEvnt, 6)
    ser.write(data6)
    data7 = mkDataByte(lyr6, addrEvnt, 7)
    ser.write(data7)
    data8 = mkDataByte(lyr7, addrEvnt, 8)
    ser.write(data8)

def bumpTKRthreshold(d0,d1,d2,d3,d4,d5,d6,d7):
    cmdHeader = mkCmdHdr(8, 0x5B, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(d0, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(d1, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(d2, addrEvnt, 3)
    ser.write(data3)
    data4 = mkDataByte(d3, addrEvnt, 4)
    ser.write(data4)
    data5 = mkDataByte(d4, addrEvnt, 5)
    ser.write(data5)
    data6 = mkDataByte(d5, addrEvnt, 6)
    ser.write(data6)
    data7 = mkDataByte(d6, addrEvnt, 7)
    ser.write(data7)
    data8 = mkDataByte(d7, addrEvnt, 8)
    ser.write(data8)
    print("bumpTKRthreshold: increasing tracker thresholds by")
    print("    Layer 0: " + str(d0))
    print("    Layer 1: " + str(d1))
    print("    Layer 2: " + str(d2))
    print("    Layer 3: " + str(d3))
    print("    Layer 4: " + str(d4))
    print("    Layer 5: " + str(d5))
    print("    Layer 6: " + str(d6))
    print("    Layer 7: " + str(d7))

def getTKRlayers():
    print("getTKRlayers: current map of tracker layers:")
    cmdHeader = mkCmdHdr(0, 0x5A, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.1)
    command,cmdDataBytes,byteList = getData(addrEvnt)
    for lyr in range(8):
        print("   Layer " + str(lyr) + " is board " + str(bytes2int(byteList[lyr])))

def getTkrASICerrors():
    print("getTkrASICerrors: list of all DAQ error codes from ASIC configuration registers:")
    cmdHeader = mkCmdHdr(0, 0x61, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.2)
    command,cmdDataBytes,byteList = getData(addrEvnt)
    for lyr in range(8):
        print("   Layer " + str(lyr) + " chips 0-3: " + str(byteList[3*lyr]))
        print("   Layer " + str(lyr) + " chips 4-7: " + str(byteList[3*lyr+1]))
        print("   Layer " + str(lyr) + " chips 8-11: " + str(byteList[3*lyr+2]))



def printHousekeeping(dataList, byteList):
    run = dataList[4]*256 + dataList[5]
    print("Housekeeping packet for run " + str(run))
    timeDate = dataList[6]*16777216 + dataList[7]*65536 + dataList[8]*256 + dataList[9]
    year = ((timeDate & 0x7C000000) >> 26) + 2000
    month = (timeDate & 0x03C00000) >> 22
    day = (timeDate & 0x003E0000) >> 17
    hour = (timeDate & 0x0001F000) >> 12
    minute = (timeDate & 0x00000FC0) >> 6
    second = (timeDate & 0x0000003F)
    print("   Housekeeping time = " + str(hour) + ":" + str(minute) + ":" + str(second) + " on " + months[month] + " " + str(day) + ", " + str(year))
    print("   Last command = " + str(byteList[10].hex() + str(byteList[11].hex())))
    cmdCnt = dataList[12]*255 + dataList[13]
    print("   Command count = " + str(cmdCnt))
    print("   Number of bad commands = " + str(dataList[14]))
    print("   Number of errors = " + str(dataList[15]))
    cntGO = dataList[16]*16777216 + dataList[17]*65536 + dataList[18]*256 + dataList[19]
    cntGO1 = dataList[20]*16777216 + dataList[21]*65536 + dataList[22]*256 + dataList[23]
    print("   GO count = " + str(cntGO) + "     GO1 count = " + str(cntGO1))
    avgReadTime = dataList[24]*256 + dataList[25]
    print("   Average readout time = " + str(avgReadTime) + " microseconds")
    Grate = dataList[34]*256 + dataList[35]
    print("   Guard rate = " + str(Grate) + " Hz")
    T1rate = dataList[26]*256 + dataList[27]
    print("   T1 rate = " + str(T1rate) + " Hz")
    T2rate = dataList[28]*256 + dataList[29]
    print("   T2 rate = " + str(T2rate) + " Hz")
    T3rate = dataList[30]*256 + dataList[31]
    print("   T3 rate = " + str(T3rate) + " Hz")
    T4rate = dataList[32]*256 + dataList[33]
    print("   T4 rate = " + str(T4rate) + " Hz")
    tkrCmdCnt = dataList[36]*256 + dataList[37]
    print("   Tracker command count = " + str(tkrCmdCnt))
    print("   Fraction of triggers with Tracker 1 bit set = " + str(dataList[38]) + " percent")
    print("   Fraction of triggers with Tracker 2 bit set = " + str(dataList[39]) + " percent")
    print("   Number of tracker data errors = " + str(dataList[40]))
    print("   Number of tracker time-outs = " + str(dataList[41]))
    for brd in range(8):
        numChipsHit = dataList[42+brd]/10.0
        print("    Tracker board " + str(brd) + " has " + str(numChipsHit) + " chips hit per event")
    for brd in range(8):
        layerRate = dataList[50 + brd*2]*256 + dataList[50 + brd*2 + 1]
        print("    Tracker board " + str(brd) + " rate = " + str(layerRate) + " Hz")
    dieTemp = dataList[66]*256 + dataList[67]
    print("   Event PSOC die temperature = " + str(dieTemp) + " Celsius")
    tkrTemp = dataList[68]*256 + dataList[69]
    Tcelsius = (0.25/4.0)*(tkrTemp/16.)
    print("   Tracker layer 0 temperature = " + str(Tcelsius) + " Celsius")
    tkrTemp = dataList[70]*256 + dataList[71]
    Tcelsius = (0.25/4.0)*(tkrTemp/16.)
    print("   Tracker layer 7 temperature = " + str(Tcelsius) + " Celsius")
    print("   Average number of TOF-A stops per event = " + str(dataList[72]))
    print("   Average number of TOF-B stops per event = " + str(dataList[73]))
    print("   Maximum number of TOF-A stops per event = " + str(dataList[74]))
    print("   Maximum number of TOF-B stops per event = " + str(dataList[75]))
    print("   Percent SPI busy = " + str(dataList[76]))
    print("   Percent live = " + str(dataList[77]))
    numLiveSamples = dataList[78]*256 + dataList[79]
    print("   Number of samples for the ADC state-machine live-time = " + str(numLiveSamples))
    print("   ADC state-machine live-time = " + str(dataList[80]) + "%")

def printTkrHousekeeping(dataList):
    run = dataList[4]*256 + dataList[5]
    print("Tracker housekeeping packet for run " + str(run) + ":")
    timeDate = dataList[6]*16777216 + dataList[7]*65536 + dataList[8]*256 + dataList[9]
    year = ((timeDate & 0x7C000000) >> 26) + 2000
    month = (timeDate & 0x03C00000) >> 22
    day = (timeDate & 0x003E0000) >> 17
    hour = (timeDate & 0x0001F000) >> 12
    minute = (timeDate & 0x00000FC0) >> 6
    second = (timeDate & 0x0000003F)
    print("   Time = " + str(hour) + ":" + str(minute) + ":" + str(second) + " on " + months[month] + " " + str(day) + ", " + str(year))
    offset = 9
    for brd in range(8):
        if dataList[offset+1] == 0 and dataList[offset+2] == 0: break
        print("   Housekeeping data for Tracker board " + str(brd) + ":")
        tkrTemp = dataList[offset+1]*256 + dataList[offset+2]
        Tcelsius = (0.25/4.0)*(tkrTemp/16.)
        print("      Temperature = " + str(Tcelsius) + " Celsius")
        shuntVoltage = 2.5*(dataList[offset+3]*256 + dataList[offset+4])/1000000.
        R = 100.0
        shuntCurrent = shuntVoltage*1000000./R
        print("      Bias current = " + str(shuntCurrent) + " microamps")
        busVoltage = 1.25*(dataList[offset+5]*256 + dataList[offset+6])/1000.
        print("      Digital 1.2V bus voltage reading = " + str(busVoltage) + " V")
        shuntVoltage = 2.5*(dataList[offset+7]*256 + dataList[offset+8])/1000000.
        R = 0.03
        shuntCurrent = shuntVoltage*1000./R
        print("      Digital 1.2V shunt current reading = " + str(shuntCurrent) + " milliamps")
        busVoltage = 1.25*(dataList[offset+9]*256 + dataList[offset+10])/1000.
        print("      Digital 2.5V bus voltage reading = " + str(busVoltage) + " V")
        shuntVoltage = 2.5*(dataList[offset+11]*256 + dataList[offset+12])/1000000.
        shuntCurrent = shuntVoltage*1000./R
        print("      Digital 2.5V shunt current reading = " + str(shuntCurrent) + " milliamps")
        busVoltage = 1.25*(dataList[offset+13]*256 + dataList[offset+14])/1000.
        print("      Digital 3.3V bus voltage reading = " + str(busVoltage) + " V")
        shuntVoltage = 2.5*(dataList[offset+15]*256 + dataList[offset+16])/1000000.
        shuntCurrent = shuntVoltage*1000./R
        print("      Digital 3.3V shunt current reading = " + str(shuntCurrent) + " milliamps")
        busVoltage = 1.25*(dataList[offset+17]*256 + dataList[offset+18])/1000.
        print("      Analog 2.1V bus voltage reading = " + str(busVoltage) + " V")
        shuntVoltage = 2.5*(dataList[offset+19]*256 + dataList[offset+20])/1000000.
        shuntCurrent = shuntVoltage*1000./R
        print("      Analog 2.1V shunt current reading = " + str(shuntCurrent) + " milliamps")
        busVoltage = 1.25*(dataList[offset+21]*256 + dataList[offset+22])/1000.
        print("      Analog 3.3V bus voltage reading = " + str(busVoltage) + " V")
        shuntVoltage = 2.5*(dataList[offset+23]*256 + dataList[offset+24])/1000000.
        shuntCurrent = shuntVoltage*1000./R
        print("      Analog 3.3V shunt current reading = " + str(shuntCurrent) + " milliamps")
        offset = offset + 24

# Print out the BOR record
def printBOR(dataBytes):
    print("printBOR: begin-of-run record:")
    dataList = []
    for item in dataBytes: dataList.append(bytes2int(item))
    runNumber = dataList[4]*256 + dataList[5]
    print("  Run number " + str(runNumber))
    timeDate = dataList[6]*16777216 + dataList[7]*65536 + dataList[8]*256 + dataList[9]
    year = ((timeDate & 0x7C000000) >> 26) + 2000
    month = (timeDate & 0x03C00000) >> 22
    day = (timeDate & 0x003E0000) >> 17
    hour = (timeDate & 0x0001F000) >> 12
    minute = (timeDate & 0x00000FC0) >> 6
    second = (timeDate & 0x0000003F)    
    print("  Time = " + str(hour) + ":" + str(minute) + ":" + str(second) + " on " + months[month] + " " + str(day) + ", " + str(year))
    print("  Event PSOC code version = " + str(dataList[10]) + "." + str(dataList[11]))
    print("  DAC settings: ")
    print("        G: " + str(dataList[12]))
    print("       T3: " + str(dataList[13]))
    print("       T1: " + str(dataList[14]))
    print("       T4: " + str(dataList[15]))
    print("       T2: " + str(dataList[16]*256 + dataList[17]))
    print("     TOFA: " + str(dataList[18]*256 + dataList[19]))
    print("     TOFB: " + str(dataList[20]*256 + dataList[21]))
    print("   TOFA-control shift register delay period = " + str(dataList[22]))
    print("   TOFB-control shift register delay period = " + str(dataList[23]))
    print("   ADC-control state machine delay = " + str(dataList[24]))
    print("   T3 comparator settling time period = " + str(dataList[25]))
    print("   T1 comparator settling time period = " + str(dataList[26]))
    print("   T4 comparator settling time period = " + str(dataList[27]))
    print("   T2 comparator settling time period = " + str(dataList[28]))
    print("   PMT-trigger tracker trigger delay time = " + str(dataList[29]))
    print("   PMT secondary trigger prescale value = " + str(dataList[30]))
    print("   Tracker trigger prescale value = " + str(dataList[31]))
    print("   Primary trigger mask = " + str(dataList[32]))
    print("   Secondary trigger mask = " + str(dataList[33]))
    print("   Tracker threshold forced offsets:")
    for lyr in range(8):
        print("       Layer " + str(lyr) + " = " + str(dataList[34+lyr]))
    print("   Tracker master trigger delay = " + str(dataList[42]))
    print("   Tracker Master trigger source setting = " + str(dataList[43]))
    for lyr in range(8):
        print("   Tracker settings for board " + str(lyr))
        print("       Tracker firmware code version = " + str(dataList[44+lyr*5]))
        print("       Tracker firmware configuration register = " + str(dataBytes[44+lyr*5+1].hex()))
        print("       Number of readout layers = " + str(dataList[44+lyr*5+2]))
        print("       Trigger output length = " + str(dataList[44+lyr*5+3]))
        print("       Trigger output delay = " + str(dataList[44+lyr*5+4]))
           
# Execute a run for a specified number of events to be acquired
def limitedRun(runNumber, numEvnts, readTracker = True, outputEvents = False, debugTOF = False):
    cmdHeader = mkCmdHdr(4, 0x3C, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(runNumber>>8, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(runNumber & 0x00FF, addrEvnt, 2)
    ser.write(data2)   
    doTkr = 0
    if readTracker: doTkr = 1
    data3 = mkDataByte(doTkr, addrEvnt, 3)
    ser.write(data3)
    doDebug = 0
    if debugTOF: doDebug = 1
    data4 = mkDataByte(doDebug, addrEvnt, 4)
    ser.write(data4)

    time.sleep(1)
    # Catch the BOR record
    cmd,cmdData,dataBytes = getData(addrEvnt)
    #for i in range(84):
    #    ret = dataBytes[i]
    #    print("   Byte " + str(i) + ":" + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex  = " + str(ret))  
    printBOR(dataBytes)
    
    nToPlot = 0
    verbose = True
    
    print("limitedRun: starting run number " + str(runNumber) + " for " + str(numEvnts) + " events")
    if not readTracker: print("            The tracking detector will not be read out")
    f = open("nTuple_run" + str(runNumber) + ".txt", "w")
    #f.write("N  T1  T2  T3  T4  G   NH  TOFA    TOFB    DTMIN   NTOFA   NTOFB   DELTAT  RC\n")
    if outputEvents:
        f2 = open("dataOutput_run" + str(runNumber) + ".txt", "w")
        f2.write("Starting run " + str(runNumber) + " on " + time.strftime("%c") + "\n") 
    #print("limitedRun: trigger enable status = " + str(triggerEnableStatus()))
    time.sleep(0.1)
    pmtTrg1 = 0
    pmtTrg2 = 0
    tkrTrg0 = 0
    tkrTrg1 = 0
    pmtGrd = 0
    nAvg = 0
    ADCavg = [0.,0.,0.,0.,0.]
    ADCavg2 = [0.,0.,0.,0.,0.]
    TOFavg = 0.
    TOFavg2 = 0.
    startTime = time.time()
    nBadTkr = 0
    lastTime = 0
    timeSum = 0
    numHits = 0
    nPlotted = 0
    for event in range(numEvnts):
        # Wait for an event to show up
        cnt = 0
        while True:
            ret = ser.read(1)
            if cnt%1 == 0:
                print("limitedRun " + str(cnt) + ": looking for start of event. Received byte " + str(ret))
            if ret == b'\xDC': break
            time.sleep(0.1)
            cnt = cnt + 1
            if cnt%30 == 0: readErrors(addrEvnt);
        ret = ser.read(2)
        if ret != b'\x00\xFF':
            print("limitedRun: bad header found: b'\\xdc' " + str(ret))
        print("limitedRun: reading packet " + str(event) + " of run " + str(runNumber))
        ret = ser.read(1)
        nData = bytes2int(ret)
        ret = ser.read(1)
        dataID = str(ret.hex())
        print("   Data type ID is " + dataID)
        ret = ser.read(1)
        if bytes2int(ret) != 0: print("   Bad number of command data bytes = " + str(bytes2int(ret)))
        R = nData % 3
        nPackets = int(nData/3)
        if (R != 0): nPackets = nPackets + 1
        dataList = []
        byteList = []
        if verbose: print("   Reading " + str(nData) + " data bytes in " + str(nPackets) + " packets")
        #if event == 0:   # Read the command echo
        #    ret = ser.read(2)
        #    print("limitedRun: echo of the run number = " + str(bytes2int(ret)))
        #    ret = ser.read(1)
        #    print("limitedRun: echo of the readTracker flag = " + str(ret))
        for i in range(nPackets):
            byte1 = ser.read()
            #if verbose: print("   Packet " + str(i) + ", byte 1 = " + str(bytes2int(byte1)) + " decimal, " + str(byte1.hex()) + " hex")
            dataList.append(bytes2int(byte1))
            byteList.append(byte1)
            byte2 = ser.read()
            #if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(byte2)) + " decimal, " + str(byte2.hex()) + " hex")
            dataList.append(bytes2int(byte2))
            byteList.append(byte2)
            byte3 = ser.read()
            #if verbose: print("   Packet " + str(i) + ", byte 3 = " + str(bytes2int(byte3)) + " decimal, " + str(byte3.hex()) + " hex")
            dataList.append(bytes2int(byte3))
            byteList.append(byte3)
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("limitedRun: invalid trailer returned: " + str(ret))
        if dataID == "DE" or dataID == "de":   # Parse the housekeeping packet
            printHousekeeping(dataList, byteList)
        elif dataID == "DF" or dataID == "df":
            printTkrHousekeeping(dataList)
        else:
            run = dataList[4]*256 + dataList[5]
            trigger = dataList[6]*16777216 + dataList[7]*65536 + dataList[8]*256 + dataList[9]
            cntGo1 = dataList[14]*16777216 + dataList[15]*65536 + dataList[16]*256 + dataList[17]
            timeDate = dataList[18]*16777216 + dataList[19]*65536 + dataList[20]*256 + dataList[21]
            if verbose: print("   Trigger: " + str(trigger) + " accepted, " + str(trigger+cntGo1) + " generated.  Data List length = " + str(len(dataList)))
            timeStamp = dataList[10]*16777216 + dataList[11]*65536 + dataList[12]*256 + dataList[13]
            year = ((timeDate & 0x7C000000) >> 26) + 2000
            month = (timeDate & 0x03C00000) >> 22
            day = (timeDate & 0x003E0000) >> 17
            hour = (timeDate & 0x0001F000) >> 12
            minute = (timeDate & 0x00000FC0) >> 6
            second = (timeDate & 0x0000003F)

            T1 = dataList[23]*256 + dataList[24]
            T2 = dataList[25]*256 + dataList[26]
            T3 = dataList[27]*256 + dataList[28]
            T4 = dataList[29]*256 + dataList[30]
            G =  dataList[31]*256 + dataList[32]
            if verbose:
                print("        T1 ADC=" + str(T1))
                print("        T2 ADC=" + str(T2))
                print("        T3 ADC=" + str(T3))
                print("        T4 ADC=" + str(T4))
                print("         G ADC=" + str(G))
            dtmin = 10*np.int16(dataList[33]*256 + dataList[34])
            trgCount = dataList[35]*256 + dataList[36]
            if debugTOF:
                nTOFA = dataList[39]
                nTOFB = dataList[40]
                tofA = 10*(dataList[41]*256 + dataList[42])
                tofB = 10*(dataList[43]*256 + dataList[44])
                clkA = dataList[45]*256 + dataList[46]
                clkB = dataList[47]*256 + dataList[48]
                nTkrLyrs = dataList[49]
                iPtr = 50
            else: 
                nTOFA = 0
                nTOFB = 0
                tofA = 9999
                tofB = 9999
                clkA = 9999
                clkB = 9999
                nTkrLyrs = dataList[39]
                iPtr = 40
            if verbose:
                print("        TimeStamp = " + str(timeStamp))
                print("        TOF=" + str(dtmin) + " Number A=" + str(nTOFA) + " Number B=" + str(nTOFB))
                print("        run=" + str(run) + "  trigger " + str(trigger) + " Tkr Trig Cnt = " + str(trgCount))
            trgStatus = dataList[22]
            rc = 0
            numHitEvt = 0
            if verbose: 
                if month > 12 or month < 1: month = 1
                print("        Event time = " + str(hour) + ":" + str(minute) + ":" + str(second) + " on " + months[month] + " " + str(day) + ", " + str(year))
                if debugTOF: print("        REF-A=" + str(tofA) + "  REF-B=" + str(tofB))
                if debugTOF: print("        TOF clkA=" + str(clkA) + "  TOF clkB=" + str(clkB))
                print("        Trigger status = " + str(hex(trgStatus)))
                print("        Number of tracker layers read out = " + str(nTkrLyrs))          
            deltaTime = timeStamp - lastTime
            deltaTimeSec = deltaTime * (1./200.)
            if event > 0:
                if verbose: print("    Time since the previous event = " + str(deltaTime) + " counts, or " + str(deltaTimeSec) + " seconds")
                timeSum += deltaTime
            lastTime = timeStamp
            if verbose:             
                FPGAs = []
                stripHits = []
                for brd in range(nTkrLyrs):
                    ##brdNum = dataList[iPtr]
                    ##iPtr = iPtr + 1
                    nBytes = dataList[iPtr]
                    iPtr = iPtr + 1
                    hitList = []
                    for hit in range(nBytes):
                        #print("                      " + str(hit) + "  " + hex(dataList[iPtr]))
                        hitList.append(byteList[iPtr])
                        iPtr = iPtr + 1
                    #print("           Hit list= " + getBinaryString(hitList))
                    rc, FPGA, strips = ParseASIChitList(getBinaryString(hitList),True)
                    if rc != 0: nBadTkr = nBadTkr + 1
                    numHitEvt = len(strips)
                    numHits = numHits + numHitEvt
                    FPGAs.append(FPGA)
                    stripHits.append(strips)
                if nPlotted < nToPlot: 
                    plotTkrEvnt(run, trigger, FPGAs, stripHits)
                    nPlotted = nPlotted + 1
            if trgStatus & 0x01: pmtTrg1 = pmtTrg1 + 1
            if trgStatus & 0x02: pmtTrg2 = pmtTrg2 + 1
            if trgStatus & 0x04: tkrTrg0 = tkrTrg0 + 1
            if trgStatus & 0x08: tkrTrg1 = tkrTrg1 + 1
            if trgStatus & 0x10: pmtGrd = pmtGrd + 1
            strOut = '{} {} {} {} {} {} {} {} {} {} {} {} {} {}\n'.format(trigger, T1, T2, T3, T4, G, numHitEvt, tofA, tofB, dtmin, nTOFA, nTOFB, deltaTime, rc)
            f.write(strOut)   
            TOFavg = TOFavg + float(dtmin)
            TOFavg2 = TOFavg2 + float(dtmin)*float(dtmin)
            if event != 0:
                nAvg = nAvg + 1
                ADCavg[0] = ADCavg[0] + T1
                ADCavg[1] = ADCavg[1] + T2
                ADCavg[2] = ADCavg[2] + T3
                ADCavg[3] = ADCavg[3] + T4
                ADCavg[4] = ADCavg[4] + G
                ADCavg2[0] = ADCavg2[0] + T1*T1
                ADCavg2[1] = ADCavg2[1] + T2*T2
                ADCavg2[2] = ADCavg2[2] + T3*T3
                ADCavg2[3] = ADCavg2[3] + T4*T4
                ADCavg2[4] = ADCavg2[4] + G*G
            # Output all of the data to an ASCII file
            if outputEvents:
                timeStr = str(hour) + ":" + str(minute) + ":" + str(second) + " on " + months[month] + " " + str(day) + ", " + str(year)
                f2.write("Event {:d}: {} {:s}   rc={}\n".format(trigger, timeStamp, timeStr, rc))
                f2.write("  ADC: {}, {}, {}, {}, {}\n".format(T1, T2, T3, T4, G))
                f2.write("  TOF: {}  nA={}  nB={}  refA={}  refB={}  clkA={}  clkB={} \n".format(dtmin, nTOFA, nTOFB, tofA, tofB, clkA, clkB))
                for lyr, hits in zip(FPGAs,stripHits):
                    f2.write("    Lyr {}:".format(lyr))
                    for strip in hits:
                        f2.write(" {} ".format(strip))
                    f2.write("\n")

    endTime = time.time()
    runTime = endTime - startTime
    print("Elapsed time for the run = " + str(runTime) + " seconds")
    if numEvnts > 1: timeSum = timeSum/float(numEvnts - 1)
    timeSumSec = timeSum*(1./200.)
    print("Average time between event time stamps = " + str(timeSum) + " counts = " + str(timeSumSec) + " seconds")
    
    # Tell the Event PSOC to stop the run
    cmdHeader = mkCmdHdr(0, 0x44, addrEvnt)
    ser.write(cmdHeader)
    print("limitedRun: expecting an EOR record.")
    cnt = 0
    byteList = []
    while True:
        if cnt > 20:
            print("Cannot find an EOR record. Print accumulated errors instead")
            readErrors(addrEvnt)
            break
        ret = ser.read(1)
        if cnt%1 == 0:
            print("limitedRun " + str(cnt) + ": looking for start of EOR record. Received byte " + str(ret))
        if ret == b'\xDC': 
            ret = ser.read(2)
            if ret != b'\x00\xFF':
                print("limitedRun: bad header found: b'\\xdc' " + str(ret)) 
            nBytes = bytes2int(ser.read(1))
            print("limitedRun: number of header bytes = " + str(nBytes))
            ret = ser.read(1)
            print("return = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex  = " + str(ret))
            if ret == b'\xDB' or ret == b'\xDD' or ret == b'\xDE' or ret == b'\xDF':
                ret = ser.read(1)
                if ret != b'\x00': print("limitedRun: invalid command data bytes " + str(ret) + " received for EOR header")
                print("limitedRun: dumping the bytes for an extra event trigger that came in while ending the run:")
                for i in range(nBytes):
                    ret = ser.read(1);      
                    if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex  = " + str(ret))
                nPadding = 3 - nBytes%3
                if nPadding == 3: nPadding = 0
                for i in range(nPadding): ser.read(1)
                ret = ser.read(3)
                if ret != b'\xFF\x00\xFF':
                    print("limitedRun: invalid trailer returned: " + str(ret))     
                continue
            print("limitedRun: command code = " + str(ret.hex()))
            print("limitedRun: found EOR record, number of header bytes = " + str(nBytes))
            ret = ser.read(1)
            if ret != b'\x00': print("limitedRun: invalid command data bytes " + str(ret) + " received for EOR header")
            for i in range(3):
                ret = ser.read(1)
                if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex  = " + str(ret))
                byteList.append(ret)
            if (byteList[0] == b'\x45' and  byteList[1] == b'\x52' and byteList[2] == b'\x52'):
                print("limitedRun: printing out an error record:")
                for i in range(3,nBytes):
                    ret = ser.read(1)
                    if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex")
                ret = ser.read(1)   # padding byte
                ret = ser.read(3)
                if ret != b'\xFF\x00\xFF':
                    print("limitedRun: invalid trailer returned: " + str(ret))    
                byteList = []
                continue                
            break               
        time.sleep(0.1)
        cnt = cnt + 1       

    for i in range(2,nBytes):
        ret = ser.read(1)
        if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(ret)) + " decimal, " + str(ret.hex()) + " hex")
        byteList.append(ret)
    ret = ser.read(1)   # padding byte
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("limitedRun: invalid trailer returned: " + str(ret))         
    go0 = bytes2int(byteList[5])
    go1 = bytes2int(byteList[6])
    go2 = bytes2int(byteList[7])
    go3 = bytes2int(byteList[8])
    cntGo1 = go0*16777216 + go1*65536 + go2*256 + go3

    go0 = bytes2int(byteList[9])
    go1 = bytes2int(byteList[10])
    go2 = bytes2int(byteList[11])
    go3 = bytes2int(byteList[12])
    cntGo = go0*16777216 + go1*65536 + go2*256 + go3
    
    nBadCRC = bytes2int(byteList[13])
    
    nTkrReadReady = bytes2int(byteList[14])*16777216 + bytes2int(byteList[15])*65536 + bytes2int(byteList[16])*256 + bytes2int(byteList[17])
    nTkrReadNotReady = bytes2int(byteList[18])*256 + bytes2int(byteList[19])
    
    print("Run number = " + str(bytes2int(byteList[3])*256 + bytes2int(byteList[4])))
    print("Number of triggers generated = " + str(cntGo1+cntGo))
    print("Number of triggers accepted = " + str(cntGo))
    print("Number of bad Tracker CRC = " + str(nBadCRC))
    print("Number of tracker reads when status is ready = " + str(nTkrReadReady))
    print("Number of tracker reads when status is not ready = " + str(nTkrReadNotReady))
    print("Average number of TOF-A stops per event = " + str(bytes2int(byteList[20])))
    print("Average number of TOF-B stops per event = " + str(bytes2int(byteList[21])))
    print("Maximum number of TOF-A stops in an event = " + str(bytes2int(byteList[22])))
    print("Maximum number of TOF-B stops in an event = " + str(bytes2int(byteList[23])))
    nBusy = bytes2int(byteList[24])*16777216 + bytes2int(byteList[25])*65536 + bytes2int(byteList[26])*256 + bytes2int(byteList[27])
    print("Number of events created when the SPI link is BUSY = " + str(nBusy))
    nTkrMasterGo = bytes2int(byteList[28])*256 + bytes2int(byteList[29])
    print("Number of GO signals received by the Tracker master = " + str(nTkrMasterGo))
    for lyr in range(8):
        nTrigs = bytes2int(byteList[30+9*lyr])*256 + bytes2int(byteList[30+9*lyr+1])
        print("  Layer " + str(lyr) + ": number of triggers received = " + str(nTrigs))
        nReads = bytes2int(byteList[30+9*lyr+2])*256 + bytes2int(byteList[30+9*lyr+3])
        print("  Layer " + str(lyr) + ": number of read commands received = " + str(nReads))
        print("  Layer " + str(lyr) + ": number of missed triggers = " + str(bytes2int(byteList[30+9*lyr+4])))
        print("  Layer " + str(lyr) + ": number of reads with no trigger = " + str(bytes2int(byteList[30+9*lyr+5])))
        print("  Layer " + str(lyr) + ": error codes = " + str(byteList[30+9*lyr+6].hex()))
        print("  Layer " + str(lyr) + ": ASIC error codes = " + str(byteList[30+9*lyr+7].hex()))
        print("  Layer " + str(lyr) + ": number of bad command addresses or codes received = " + str(byteList[30+9*lyr+8].hex()))
    cntBytes = []
    for i in range(47):
        cntBytes.append(byteList[102+i])
    #for item in cntBytes: print("  counter byte = " + str(item))
    printRunCounters(cntBytes)
    
    Sigma = [0.,0.,0.,0.,0.,0.]
    TOFavg = TOFavg/float(numEvnts)
    TOFavg2 = TOFavg2/float(numEvnts)
    numHitsAvg = numHits/float(numEvnts)
    print("Average number of hits per event = " + str(numHitsAvg))
    sigmaTOF = math.sqrt(TOFavg2 - TOFavg*TOFavg)
    for ch in range(5):
        if numEvnts > 1:
            ADCavg[ch] = ADCavg[ch]/float(nAvg)
            ADCavg2[ch] = ADCavg2[ch]/float(nAvg)
            Sigma[ch] = math.sqrt(ADCavg2[ch] - ADCavg[ch]*ADCavg[ch])
        else:
            Sigma[ch] = 0.
    f.close()
    if outputEvents: f2.close()
    if (cntGo+cntGo1 == 0):
        live = 0.
    else:
        live = cntGo/float(cntGo+cntGo1)
    print("Live time fraction = " + str(live))
    print("Number of primary PMT triggers captured = " + str(pmtTrg1))
    print("Number of secondary PMT triggers captured = " + str(pmtTrg2))
    print("Number of tracker-0 triggers captured = " + str(tkrTrg0))
    print("Number of tracker-1 triggers captured = " + str(tkrTrg1))
    print("Number of triggers with guard fired = " + str(pmtGrd))
    print("Number of bad tracker events = " + str(nBadTkr))
    print("Number of events included in the ADC averages = " + str(nAvg))
    return ADCavg, Sigma, TOFavg, sigmaTOF

def plotTkrEvnt(run, event, layers, hitList):
    f = open("plot_run" + str(run) + "_evt_" + str(event) + ".plt", "w")
    f.write("set title 'Event Number {:d}\n".format(event))
    f.write("set multiplot layout 1,2 rowsfirst\n")
    f.write("set xrange [0.:20.]\n")
    f.write("set yrange [0.:35.]\n")
    f.write("set xlabel 'X'\n")
    f.write("set ylabel 'Z'\n")
    f.write("$bending << EOD\n")
    for lyr, hits in zip(layers,hitList):
        if view[lyr] == "bending":
            for strip in hits:
                x = (strip-1)*0.0228
                f.write(" {:f} {:f}\n".format(x, lyrZ[lyr]+30.))
    f.write("EOD\n")
    f.write("plot $bending u 1:2 with points pt 7 ps 1\n")
    
    f.write("set xlabel 'Y'\n")
    f.write("$nonbending << EOD\n")
    for lyr, hits in zip(layers,hitList):
        if view[lyr] == "nonbending":
            for strip in hits:
                x = (strip-1)*0.0228
                f.write(" {:f} {:f}\n".format(x, lyrZ[lyr]+30.))
    f.write("EOD\n")
    f.write("plot $nonbending u 1:2 with points pt 7 ps 1\n")
    
    f.write("unset multiplot")
    f.close()
    
# Read the PMT channel counts
def getChannelCount(channel):
    PSOCaddress = addrEvnt;
    cmdHeader = mkCmdHdr(1, 0x37, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    # The hardware counter period is 255, not 256, hence the 255 in this expression
    count = (bytes2int(dataBytes[0])*16777216 + bytes2int(dataBytes[1])*65536 + bytes2int(dataBytes[2])*256 + bytes2int(dataBytes[3]))*255 + bytes2int(dataBytes[4])
    return count
    
# Read the PMT channel counts from end of run
def getEndOfRunChannelCount(channel):
    PSOCaddress = addrEvnt;
    cmdHeader = mkCmdHdr(1, 0x33, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    # The hardware counter period is 255, not 256, hence the 255 in this expression
    count = (bytes2int(dataBytes[0])*16777216 + bytes2int(dataBytes[1])*65536 + bytes2int(dataBytes[2])*256 + bytes2int(dataBytes[3]))*255 + bytes2int(dataBytes[4])
    return count

# Set up the Event PSOC trigger masks
def setTriggerMask(maskNumber, mask):
    PSOCaddress = addrEvnt
    if maskNumber != 1 and maskNumber != 2:
        print("setTriggerMask: the mask number must be 1 or 2")
        return
    cmdHeader = mkCmdHdr(2, 0x36, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(maskNumber, PSOCaddress, 1)
    ser.write(data1)
    data2 = mkDataByte(mask, PSOCaddress, 2)
    ser.write(data2)
    
def getTriggerMask(maskNumber):
    PSOCaddress = addrEvnt
    if maskNumber != 1 and maskNumber != 2:
        print("getTriggerMask: the mask number must be 1 or 2")
        return
    cmdHeader = mkCmdHdr(1, 0x3E, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(maskNumber, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    mask = bytes2int(dataBytes[0])
    return mask
    
# Set where the event PSOC will send its data
def setOutputMode(mode):
    PSOCaddress = addrEvnt
    cmdHeader = mkCmdHdr(1, 0x30, PSOCaddress)
    ser.write(cmdHeader)
    if mode == "UART":
        imode = 1
    else: 
        imode = 0
    data1 = mkDataByte(imode, PSOCaddress, 1)
    ser.write(data1)    

# Set the threshold for the DAC of a PMT channel
def setPmtDAC(channel, value, address):
    if channel > 5 or channel < 1: return 1
    print("setPmtDAC: setting PMT DAC to " + str(value) + " for channel " + str(channel))
    byte2 = (value & 0x0f00) >> 8
    byte1 = (value & 0x00FF) 
    if (channel < 5): nData = 2
    else: nData = 3  # Channel 5 has a 12-bit DAC
    cmdHeader = mkCmdHdr(nData, 0x01, address)
    #print("command =" + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, address, 1)
    #print("1st data byte =" + str(data1))
    ser.write(data1)
    if (channel == 5):
        data2 = mkDataByte(byte2, address, 2)
        #print("2nd data byte =" + str(data2))
        ser.write(data2)
        data3 = mkDataByte(byte1, address, 3)
        #print("3rd data byte =" + str(data3))
    else:
        data3 = mkDataByte(byte1, address, 2)
        #print("2rd data byte =" + str(data3))
    ser.write(data3)   
    return 0

# Read back the current setting of a PMT DAC
def readPmtDAC(channel, address):
    if channel > 5 or channel < 1: return 999
    cmdHeader = mkCmdHdr(1, 0x02, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, address, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(address)
    if (channel == 5):
        value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    else:
        value = bytes2int(dataBytes[0])
    return value

def triggerEnableStatus():
    cmdHeader = mkCmdHdr(0, 0x3D, addrEvnt)
    ser.write(cmdHeader)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    return bytes2int(dataBytes[0])        

# Load a 12-bit DAC for the TOF threshold
def setTofDAC(channel, value, address):
    if channel > 2 or channel < 1: return 1
    byte2 = (value & 0x0F00) >> 8
    byte1 = (value & 0x00FF) 
    nData = 3  
    cmdHeader = mkCmdHdr(nData, 0x04, address)
    #print("command =" + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, address, 1)
    #print("1st data byte =" + str(data1))
    ser.write(data1)
    data2 = mkDataByte(byte2, address, 2)
    #print("2nd data byte =" + str(data2))
    ser.write(data2)
    data3 = mkDataByte(byte1, address, 3)
    #print("3rd data byte =" + str(data3))
    ser.write(data3)   
    return 0

# Read back the current setting of a TOF 12-bit DAC
def readTofDAC(channel, address):
    if channel > 2 or channel < 1: return 999
    cmdHeader = mkCmdHdr(1, 0x05, address)
    #print("readTofDAC: cmdHeader = " + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, address, 1)
    #print("readTofDAC: data = " + str(data1))
    ser.write(data1)
    time.sleep(0.1)
    cmd,cmdBytes,dataBytes = getData(address)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    #print("value="+str(value))
    return value

# Read the bus voltage from an INA226 chip
def readBusVoltage(i2cAddress, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x20, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(i2cAddress, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(PSOCaddress)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    #print("value="+str(value))
    return (value * 1.25)/1000.

# Read the shunt voltage from an INA226 chip and calculate the current
def readCurrent(i2cAddress, address):
    cmdHeader = mkCmdHdr(1, 0x21, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(i2cAddress, address, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(address)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    #print("value="+str(value))
    return (value * 0.03)
    
# Read the board temperature
def readTemperature(address):
    cmdHeader = mkCmdHdr(0, 0x22, address)
    ser.write(cmdHeader)
    cmd,cmdData,dataBytes = getData(address)
    value = bytes2int(dataBytes[0] + dataBytes[1]) >> 4
    #print("value="+str(value))
    return (value*0.0625)

# Load a byte into any register of the Real-Time Clock
def loadRTCregister(regAddress, regValue, address):
    cmdHeader = mkCmdHdr(2, 0x24, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(regAddress, address, 1)
    ser.write(data1)
    data2 = mkDataByte(regValue, address, 2)
    ser.write(data2)
    
# Load a byte into any register of the barometer chip
def loadBarometerReg(regAddress, regValue, PSOCaddress):    
    cmdHeader = mkCmdHdr(2, 0x27, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(regAddress, PSOCaddress, 1)
    ser.write(data1)
    data2 = mkDataByte(regValue, PSOCaddress, 2)
    ser.write(data2)
    
# Read a byte from any register of the barometer chip
def readBarometerReg(regAddress, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x26, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(regAddress, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(PSOCaddress)
    #print("readBarometerReg: byte = " + str(binascii.hexlify(byte1)))
    return bytes2int(dataBytes[0])
    
# Read a byte from any register of the Real-Time Clock
def readRTCregister(regAddress, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x23, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(regAddress, PSOCaddress, 1)
    ser.write(data1)
    cmd,cmdData,dataBytes = getData(PSOCaddress)
    #print("readRTCregister: byte = " + str(binascii.hexlify(dataBytes[0])))
    return dataBytes[0]

# Turn on or off the 3.6 MHz oscillator
def ctrlOsc(setting):
    if setting == 1:
        print("Turning the external clock oscillator on")
    elif setting == 0:
        print("Turning the external clock oscillator off")
      
    PSOCaddress = addrMain
    cmdHeader = mkCmdHdr(1, 0x30, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(setting, PSOCaddress, 1)
    ser.write(data1)

# Pack two decimal digits into a byte
def createDec(number):
    ones = number%10
    tens = int(number/10)
    return (tens<<4 | ones)

# Set the i2c Real-Time Clock according to the current time and date
def setRTCtime(address):
    now = time.localtime()
    #print(now)
    c = 1<<7
    b = createDec(now.tm_sec) | c
    loadRTCregister(0x00, b, address)
    b = createDec(now.tm_min)
    loadRTCregister(0x01, b, address)
    b = createDec(now.tm_hour)
    loadRTCregister(0x02, b, address)
    batteryEnable = 1
    b = (batteryEnable<<3 | now.tm_wday) 
    loadRTCregister(0x03, b, address)
    b =  createDec(now.tm_mday)
    loadRTCregister(0x04, b, address)
    b = createDec(now.tm_mon)
    loadRTCregister(0x05, b, address)
    yr = now.tm_year%100
    b = createDec(yr)
    loadRTCregister(0x06, b, address)
    time.sleep(2)

# Set the PSOC internal Real-Time-Clock according to the current time and date
def setInternalRTC(address):
    now = time.localtime()
    print("setInternalRTC for PSOC " + str(address) + ": " + str(now))
    cmdHeader = mkCmdHdr(10, 0x45, address)
    #print("command header x45 10 for address=" + str(address) + "  is " + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(now.tm_sec, address, 1)
    #print("data1= " + str(now.tm_sec))
    ser.write(data1)
    data2 = mkDataByte(now.tm_min, address, 2)
    ser.write(data2)
    #print("data2= " + str(now.tm_min))
    data3 = mkDataByte(now.tm_hour, address, 3)
    #print("data3= " + str(now.tm_hour))
    ser.write(data3)
    data4 = mkDataByte(now.tm_wday+1, address, 4)
    #print("data4= " + str(now.tm_wday+1))
    ser.write(data4)
    data5 = mkDataByte(now.tm_mday, address, 5)
    ser.write(data5)
    #print("data5= " + str(now.tm_mday))
    dayOfYear = now.tm_yday
    data6 = mkDataByte(dayOfYear//256, address, 6)
    #print("data6= " + str(dayOfYear//256))
    ser.write(data6)
    data7 = mkDataByte(dayOfYear%256, address, 7)
    ser.write(data7)
    #print("data7= " + str(dayOfYear%256))
    data8 = mkDataByte(now.tm_mon, address, 8)
    #print("data8= " + str(now.tm_mon))
    ser.write(data8)
    year = now.tm_year
    yearHigh = year//256
    #print("data9= " + str(yearHigh))
    data9 = mkDataByte(yearHigh, address, 9)
    ser.write(data9)
    yearLow = year%256
    #print("data10= " + str(yearLow))
    data10 = mkDataByte(yearLow, address, 10)
    ser.write(data10)
    time.sleep(1)
    
def getInternalRTC(address):
    print("getInternalRTC: reading the internal real-time-clock for PSOC address " + str(address))
    cmdHeader = mkCmdHdr(0, 0x46, address)
    ser.write(cmdHeader)
    time.sleep(0.5)
    cmd,cmdData,byteList = getData(address)
    #print("getInternalRTC: command = " + str(bytes2int(cmd)) + " decimal, " + str(cmd.hex()) + " hex")

    for i in range(10):
        #print("getInternalRTC:    Byte " + str(i) + " is " +  str(bytes2int(byteList[i])) + " decimal, " + str(byteList[i].hex()) + " hex")   
        byteList[i] = bytes2int(byteList[i])
    year = byteList[8]*256 + byteList[9]
    month = byteList[7]
    if month > 12: month = 1
    date = byteList[4]
    weekday = byteList[3] - 1  # internal RTC counts from 1 to 7
    if weekday > 6: weekday = 0
    hour = byteList[2]
    minute = byteList[1]
    second = byteList[0]
    print("The current RTC time is " + str(hour) + ":" + str(minute) + ":" + str(second) + ", " + days[weekday] + " " + months[month] + " " + str(date) + ", " + str(year))         
    
# Unpack 2 decimal digits from a byte
def dec2int(byte):
    ones = byte & 0x0F
    tens = (byte & 0xF0) >> 4
    return tens*10 + ones

# Set the main PSOC internal real time clock from the i2c RTC
def setInternalRTCfromI2C():
    cmdHeader = mkCmdHdr(0, 0x47, addrMain)
    ser.write(cmdHeader)
    print("setInternalRTCfromI2C: setting the internal Main PSOC RTC from the i2c RTC")
    time.sleep(1)

# Read and print out the current time date, from the onboard i2c-bus Real-Time-Clock
def readRTCtime(address):
    byte = readRTCregister(0x00, address)
    second = dec2int(bytes2int(byte) & 0x7F)
    #print("readRTCtime: seconds = " + str(second))
    byte = readRTCregister(0x01, address)
    minute = dec2int(bytes2int(byte) & 0x7F)
    #print("readRTCtime: minutes = " + str(minute))
    byte = readRTCregister(0x02, address)
    hour = dec2int(bytes2int(byte) & 0x3F)
    #print("readRTCtime: hours = " + str(hour))
    byte = readRTCregister(0x03, address)
    weekday = bytes2int(byte) & 0x07
    #print("readRTCtime: weekday = " + str(weekday))
    byte = readRTCregister(0x04, address)
    date = dec2int(bytes2int(byte) & 0x3F)
    #print("readRTCtime: date = " + str(date))
    byte = readRTCregister(0x05, address)
    month = dec2int(bytes2int(byte) & 0x1F)
    #print("readRTCtime: month = " + str(month))
    byte = readRTCregister(0x06, address)
    year = 2000 + dec2int(bytes2int(byte))
    #print("readRTCtime: year = " + str(year))
    print("The current i2c RTC time is " + str(hour) + ":" + str(minute) + ":" + str(second) + ", " + days[weekday] + " " + months[month] + " " + str(date) + ", " + str(year))

# Read back and print out all of the error codes stored up in the PSOC
def readErrors(address):
    cmdHeader = mkCmdHdr(0, 0x03, address)
    ser.write(cmdHeader)
    cmd,cmdData,dataBytes = getData(0)
    nData = len(dataBytes)
    if nData == 3 and bytes2int(dataBytes[0])==0:
        print("readErrors for PSOC address " + str(address) + ": no errors encountered.")
        return
    print("readErrors for PSOC address " + str(address) + ": number of data bytes = " + str(nData))           
    nPackets = int((nData-1)/3) + 1;
    for packet in range(nPackets):
        ret = dataBytes[packet*3]
        print("Error code returned to readErrors is " + str(bytes2int(ret)))  
        ret = dataBytes[packet*3+1]
        print("    First information byte = " + str(binascii.hexlify(ret)))
        ret = dataBytes[packet*3+2]
        print("    Second information byte = " + str(binascii.hexlify(ret)))

# Read back the voltage of the 5V supply on the backplane, digitized by the main PSOC Sigma-Delta ADC
def readBackplaneVoltage():
    address = addrMain
    cmdHeader = mkCmdHdr(0, 0x25, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    cmd,cmdData,dataBytes = getData(address)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    return value/1000.

# Read back the voltage of the watch battery (used by the RTC), digitized by the event PSOC Sigma-Delta ADC
# Don't use this. It is being eliminated in the new boards, as it drained the battery.
def readBatteryVoltage():
    cmdHeader = mkCmdHdr(0, 0x25, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.1)
    cmd,cmdData,dataBytes = getData(addrEvnt)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    print("readBatteryVoltage: result = " + str(value) + " mV")
    return value/1000.

def readNumTOF(address):
    cmdHeader = mkCmdHdr(0, 0x34, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    cmd,cmdData,dataBytes = getData(address)
    valueA = bytes2int(dataBytes[0])
    print("readNumTOF: channel-A TOF pointer = " + str(valueA))
    valueB = bytes2int(dataBytes[1])
    print("readNumTOF: channel-B TOF pointer = " + str(valueB))
    return [valueA, valueB]

def readSAR_ADC(address):
    cmdHeader = mkCmdHdr(0, 0x33, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    cmd,cmdData,dataBytes = getData(address)
    value = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    return value*3.3/4096.

def TOFselectDAQ(address, mode):
    if mode == "DMA": 
        arg = 1
        print("TOFselecDAQ: use DMA mode for the TOF data acquisition")
    else: 
        arg = 0
        print("TOFselecDAQ: use interrupt mode for the TOF data acquisition")
    cmdHeader = mkCmdHdr(1, 0x4D, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(arg, address, 1)
    ser.write(data1)    

def TOFenable(address, onOFF):
    if onOFF == 1: print("TOFenable, enable the TOF")
    else: print("TOFenable, disable the TOF")
    cmdHeader = mkCmdHdr(1, 0x4C, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(onOFF, address, 1)
    ser.write(data1)

def readAllTOFdata(address):
    print("readAllTOFdata: read back all accumulated time-of-flight data")
    cmdHeader = mkCmdHdr(0, 0x40, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    cmd,cmdData,dataBytes = getData(address)
    nA = bytes2int(dataBytes[0])
    print("readAllTOFdata: number of A-channel hits = " + str(nA))
    nB = bytes2int(dataBytes[1])     
    print("readAllTOFdata: number of B-channel hits = " + str(nB))
    b3 = bytes2int(dataBytes[2])
    print("readAllTOFdata: 3rd byte = " + str(b3))
    if (b3 == 2):
        print("    readAllTOFdata: the TOF data were truncated")
    ptr = 3
    refA= []
    refB= []
    clkCntA= []
    clkCntB= []
    timeA=[]
    timeB=[]
    for i in range(nA):
        refIdx = bytes2int(dataBytes[ptr])*256 + bytes2int(dataBytes[ptr+1])
        stp0 = bytes2int(dataBytes[ptr+2])       
        stop = stp0*256 + bytes2int(dataBytes[ptr+3])
        clkCnt = bytes2int(dataBytes[ptr+4])*256 + bytes2int(dataBytes[ptr+5])             
        timeStop = refIdx*8333 + stop;
        print("    Channel A hit " + str(i) + ": ref=" + str(refIdx) + ", stop=" + str(stop) + ", time=" + str(timeStop) + ",  clock count = " + str(clkCnt))
        refA.append(refIdx)
        clkCntA.append(clkCnt)
        timeA.append(timeStop)
        ptr = ptr + 6
    for i in range(nB):
        refIdx = bytes2int(dataBytes[ptr])*256 + bytes2int(dataBytes[ptr+1])
        stp0 = bytes2int(dataBytes[ptr+2])       
        stop = stp0*256 + bytes2int(dataBytes[ptr+3])
        clkCnt = bytes2int(dataBytes[ptr+4])*256 + bytes2int(dataBytes[ptr+5])                 
        timeStop = refIdx*8333 + stop;
        print("    Channel B hit " + str(i) + ": ref=" + str(refIdx) + ", stop=" + str(stop) + ": time=" + str(timeStop) + ",  clock count = " + str(clkCnt)) 
        refB.append(refIdx)
        clkCntB.append(clkCnt)
        timeB.append(timeStop)      
        ptr = ptr + 6
    TOFs = []
    for i in range(nA):
        for j in range(nB):
            if clkCntA[i] == clkCntB[j]:
                if refA[i] == refB[j]:
                    TOF = (timeB[j] - timeA[i])/100.
                    print(str(i) + " " + str(j) + " tA=" + str(timeA[i]) +
                            " tB=" + str(timeB[j]) + "   TOF = " + str(TOF))
                    TOFs.append(TOF)
    print("     ")
    return TOFs
            
def readTOFevent(address, channel):
    cmdHeader = mkCmdHdr(1, 0x35, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, address, 1)
    ser.write(data1)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("readTOFevent: invalid header returned: " + str(ret))
    ret = ser.read(1)
    print("    readTOFevent: number of data bytes = " + str(bytes2int(ret)))
    ret = ser.read(2)       
    #ret = ser.read(3)
    #if ret != b'\xFF\x00\xFF':
    #    print("readTOFevent: invalid trailer returned: " + str(ret))    
    #ret = ser.read(3)
    #if ret != b'\xDC\x00\xFF':
    #    print("readTOFevent: invalid header returned: " + str(ret))
    ref = bytes2int(ser.read(2))
    print("    readTOFevent: reference index = " + str(ref))
    ret = ser.read(1)       
    #ret = ser.read(3)
    #if ret != b'\xFF\x00\xFF':
    #    print("readTOFevent: invalid trailer returned: " + str(ret))
    #ret = ser.read(3)
    #if ret != b'\xDC\x00\xFF':
    #    print("readTOFevent: invalid header returned: " + str(ret))
    tim = bytes2int(ser.read(2))
    print("    readTOFevent: stop time = " + str(tim))
    ret = ser.read(1)       
    #ret = ser.read(3)
    #if ret != b'\xFF\x00\xFF':
    #    print("readTOFevent: invalid trailer returned: " + str(ret))
    #ret = ser.read(3)
    #if ret != b'\xDC\x00\xFF':
    #    print("readTOFevent: invalid header returned: " + str(ret))
    clk = bytes2int(ser.read(2))
    print("    readTOFevent: clock count = " + str(clk))
    idx = bytes2int(ser.read(1))
    print("    readTOFevent: number = " + str(idx)) 
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTOFevent: invalid trailer returned: " + str(ret))   
    return 10*(ref*8333 + tim)      

# Read back the full configuration of the time-of-flight chip
def readTofConfig():
    address = addrEvnt
    cmdHeader = mkCmdHdr(0, 0x0E, address)
    ser.write(cmdHeader)
    time.sleep(0.4)
    cmd,cmdData,dataBytes = getData(address)
    for i in range(len(dataBytes)):
        ret = dataBytes[i]
        print("Byte "+str(i)+" returned by SPI from the TOF chip config reg: " + str(binascii.hexlify(ret)))  

# Receive and check the echo from a tracker command
def getTkrEcho():
    cmd,cmdData,dataBytes = getData(addrEvnt)
    cmdCount = bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])
    #print("getTkrEcho: command count = " + str(cmdCount))
    cmdCode = dataBytes[2]
    #print("getTkrEcho: command code = " + str(binascii.hexlify(cmdCode)))
    return cmdCode

def tkrFPGAreset(FPGA):
    address = addrEvnt
    print("Resetting FPGA " + str(FPGA))
    cmdHeader = mkCmdHdr(3, 0x10, address)
    #print("cmdHeader = " + str(binascii.hexlify(cmdHeader)))
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    #print("data1 = " + str(binascii.hexlify(data1)))
    ser.write(data1)
    data2 = mkDataByte(0x04, address, 2)
    #print("data2 = " + str(binascii.hexlify(data2)))
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'04'"):
        print("tkrFPGAreset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'04'")   
        
# Reset to default the tracker configuration in a given FPGA
def tkrConfigReset(FPGA):
    address = addrEvnt
    print("Resetting the configuration of FPGA " + str(FPGA))
    cmdHeader = mkCmdHdr(3, 0x10, address)
    #print("cmdHeader = " + str(binascii.hexlify(cmdHeader)))
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    #print("data1 = " + str(binascii.hexlify(data1)))
    ser.write(data1)
    data2 = mkDataByte(0x03, address, 2)
    #print("data2 = " + str(binascii.hexlify(data2)))
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'03'"):
        print("tkrConfigReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'03'")   

# Set the trigger delay for the tracker, for PMT triggered events
def tkrSetPMTtrgDly(value):
    print("tkrSetPMTtrgDly: setting to trigger delay to " + str(value))
    cmdHeader = mkCmdHdr(1, 0x4F, addrEvnt)
    ser.write(cmdHeader)    
    data1 = mkDataByte(value, addrEvnt, 1)
    ser.write(data1)    

# Set the ASIC mask of a tracker board
def tkrSetASICmask(FPGA, maskUp, maskDn):
    address = addrEvnt
    print("Setting the ASIC mask of Tracker FPGA " + str(FPGA) + " to " 
                                  + str(maskUp) + str(maskDn))
    cmdHeader = mkCmdHdr(5, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0E, address, 2)
    ser.write(data2)
    data3 = mkDataByte(2, address, 3)
    ser.write(data3)   
    data4 = mkDataByte(maskUp, address, 4)
    ser.write(data4)
    data5 = mkDataByte(maskDn, address, 5)
    ser.write(data5)

def setTkrTrigOutputTiming(FPGA, delay, length):
    print("setTkrTrigOutputTiming: for board " + str(FPGA) + " setting the delay to " + str(delay) + " and the length to " + str(length))
    cmdHeader = mkCmdHdr(5, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x61, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(2, addrEvnt, 3)
    ser.write(data3)
    data4 = mkDataByte(length, addrEvnt, 4)
    ser.write(data4)
    data5 = mkDataByte(delay, addrEvnt, 5)
    ser.write(data5)
    
    
def NOOP():
    print("NOOP: sending a NOOP command to the event PSOC")
    cmdHeader= mkCmdHdr(0, 0x7A, addrEvnt)
    ser.write(cmdHeader)
    
def getTkrTrigOutputTiming(FPGA):
    cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x71, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0, addrEvnt, 3)
    ser.write(data3)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    print("getTkrTrigOutputTiming: for board " + str(FPGA) + " the delay setting is " + str(bytes2int(byteList[8])) + " and the length is " + str(bytes2int(byteList[7])))
    
# Dump all the bytes accumulated in the Tracker output RAM buffer (for debugging)
def dmpTkrRAMbuffer():
    cmdHeader = mkCmdHdr(2, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0D, addrEvnt, 2)
    ser.write(data2)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    print("dmpTkrRAMbuffer: " + str(bytes2int(byteList[0])) + " bytes returned")
    for byte in byteList:
        print("dmpTkrRAMbuffer: byte = " + str(binascii.hexlify(byte)))

# Hard reset of ASICs on all tracker boards. Use address 31 = 0x1F to reset all ASICs.
def tkrAsicHardReset(ASICaddress):
    address = addrEvnt
    print("Hard resetting the ASICs for address " + str(ASICaddress))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x05, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(ASICaddress, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'05'"):
        print("tkrConfigReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'05'")   

def tkrLogicReset(FPGAaddress):
    print("tkrLogicReset: resetting the state machines in FPGA " + str(FPGAaddress))
    cmdHeader = mkCmdHdr(3, 0x10, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(0x04, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, addrEvnt, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'04'"):
        print("tkrLogicReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'04'")   

# Hard reset of ASICs on all tracker boards. Use address 31 = 0x1F to reset all ASICs.
def tkrAsicSoftReset(ASICaddress):
    address = addrEvnt
    print("Soft resetting the ASICs for address " + str(ASICaddress))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0c, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(ASICaddress, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'0c'"):
        print("tkrConfigReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'0c'")   

def tkrSetDualTrig(FPGAaddress,value):
    if value != 0 and value != 1:
        print("tkrSetDualTrig: invalid value " + str(value) + " supplied")
        return
    address = addrEvnt
    print("Setting the dual trigger mode for board " + str(FPGAaddress) + " to " + str(value))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x5b, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(value, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'5b'"):
        print("tkrSetDualTrig: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'5b'")   

# Set the tracker trigger end status
def tkrTrigEndStat(FPGAaddress,value):
    if value != 0 and value != 1:
        print("tkrTrigEndStat: invalid value " + str(value) + " supplied")
        return
    address = addrEvnt
    print("Setting the trigger end status for board " + str(FPGAaddress) + " to " + str(value))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGAaddress, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x5a, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(value, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'5a'"):
        print("tkrTrigEndStat: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'5a'")   

# Turn on the ASIC analog power
def tkrAsicPowerOn():
    address = addrEvnt
    print("Turning on the ASIC analog power")
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x08, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'08'"):
        print("tkrConfigReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'08'")   

# Get housekeeping data sent back from the tracker
def getTkrHousekeeping():
    command,cmdDataBytes,dataBytes = getData(addrEvnt)
    return dataBytes

def getEvtVersionNumber():
    cmdHeader = mkCmdHdr(0, 0x07, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.2)
    command,cmdDataBytes,dataBytes = getData(addrEvnt)
    #print("getEvtVersionNumber: command = " + str(binascii.hexlify(command)))
    print("getEvtVersionNumber: Event PSOC code Version Number = " + str(bytes2int(dataBytes[0])) + "." + str(bytes2int(dataBytes[1])))

def getAvgReadoutTime():
    cmdHeader = mkCmdHdr(0, 0x51, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.2)
    command,cmdDataBytes,dataBytes = getData(addrEvnt)
    totalTime = bytes2int(dataBytes[0])*16777216+bytes2int(dataBytes[1])*65536+bytes2int(dataBytes[2])*256+bytes2int(dataBytes[3])
    numReadouts = bytes2int(dataBytes[4])*16777216+bytes2int(dataBytes[5])*65536+bytes2int(dataBytes[6])*256+bytes2int(dataBytes[7])
    if numReadouts == 0: 
        print("getAvgReadoutTime: no events were read out")
        return
    avgTime = 5.0*float(totalTime)/float(numReadouts)
    print("getAvgReadoutTime: for " + str(numReadouts) + " events, the average readout time is " + str(avgTime) + " ms")

def printRunCounters(dataBytes):
    print("getRunCounters: Global command count = " + str(bytes2int(dataBytes[0])*256 + bytes2int(dataBytes[1])))
    print("                Command count = " + str(bytes2int(dataBytes[2])*256 + bytes2int(dataBytes[3])))
    print("                Number of command timeouts = " + str(bytes2int(dataBytes[4])))
    print("                Number of Tracker resets = " + str(bytes2int(dataBytes[5])))
    print("                Number of events with ASIC error flag set = " + str(bytes2int(dataBytes[6])))
    print("                Number of events with ASIC parity errors = " + str(bytes2int(dataBytes[7])))
    print("                Number of bad ASIC headers = " + str(bytes2int(dataBytes[8])))
    print("                Number of bad ASIC clusters = " + str(bytes2int(dataBytes[9])))
    print("                Number of bad commands = " + str(bytes2int(dataBytes[10])))
    print("                Number of tracker chips with > 10 clusters = " + str(bytes2int(dataBytes[11])))
    print("                Number of tracker hit-list overruns while parsing = " + str(bytes2int(dataBytes[12])))
    print("                Number of tracker tag mismatches = " + str(bytes2int(dataBytes[13])))
    print("                Number of events too big to output = " + str(bytes2int(dataBytes[14])))
    print("                Number of tracker data errors = " + str(bytes2int(dataBytes[15])))
    print("                Number of tracker records with bad N-Data = " + str(bytes2int(dataBytes[16])))
    numTkrTimeOuts = bytes2int(dataBytes[17])*256 + bytes2int(dataBytes[18])
    print("                Number of tracker time-outs = " + str(numTkrTimeOuts))
    nTkTrg1 = bytes2int(dataBytes[19])*16777216+bytes2int(dataBytes[20])*65536+bytes2int(dataBytes[21])*256+bytes2int(dataBytes[22])
    nTkTrg2 = bytes2int(dataBytes[23])*16777216+bytes2int(dataBytes[24])*65536+bytes2int(dataBytes[25])*256+bytes2int(dataBytes[26])
    nPMTonly = bytes2int(dataBytes[27])*16777216+bytes2int(dataBytes[28])*65536+bytes2int(dataBytes[29])*256+bytes2int(dataBytes[30])
    nTkrOnly = bytes2int(dataBytes[31])*16777216+bytes2int(dataBytes[32])*65536+bytes2int(dataBytes[33])*256+bytes2int(dataBytes[34])
    nAllTrg = bytes2int(dataBytes[35])*16777216+bytes2int(dataBytes[36])*65536+bytes2int(dataBytes[37])*256+bytes2int(dataBytes[38])
    nNoCk = bytes2int(dataBytes[39])*16777216+bytes2int(dataBytes[40])*65536+bytes2int(dataBytes[41])*256+bytes2int(dataBytes[42])
    print("                Number of events with tracker trigger 1 =  " + str(nTkTrg1))
    print("                Number of events with tracker trigger 2 =  " + str(nTkTrg2))
    print("                Number of events with only PMT triggers =  " + str(nPMTonly))
    print("                Number of events with only TKR triggers =  " + str(nTkrOnly))
    print("                Number of events with all 3 triggers =     " + str(nAllTrg))
    print("                Number of events with no primary trigger = " + str(nNoCk))
    liveTime = (bytes2int(dataBytes[43])*256 + bytes2int(dataBytes[44]))/100.
    print("                ADC control state machine live time = " + str(liveTime))
    nNOOP = bytes2int(dataBytes[45])*256 + bytes2int(dataBytes[46])
    print("                Number of NOOP commands received = " + str(nNOOP))
    
def getRunCounters():
    cmdHeader = mkCmdHdr(0, 0x50, addrEvnt)
    ser.write(cmdHeader)
    time.sleep(0.1)
    command,cmdDataBytes,dataBytes = getData(addrEvnt)
    printRunCounters(dataBytes)
    
def tkrGetCodeVersion(FPGA):
    address = addrEvnt
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0A, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    return bytes2int(getTkrHousekeeping()[7])

def tkrSetCalMask(FPGA, chip, list):
    nItems = len(list)
    if nItems==0 or nItems>5:
        print("tkrSetCalMask: bad number of channel clusters to calibrate" + str(nItems) + ", must be from 1 to 5")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(5+2*nItems, 0x41, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(chip, address, 2)
    ser.write(data2)
    data3 = mkDataByte(CALMASK, address, 3)
    ser.write(data3)
    data4 = mkDataByte(0, address, 4)
    ser.write(data4)
    data5 = mkDataByte(nItems, address, 5)
    ser.write(data5)
    byteNum = 6
    for item in list:
        #print("tkrSetCalMask: calibrate channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
        data1 = mkDataByte(item[0], address, byteNum)
        ser.write(data1)
        data2 = mkDataByte(item[1], address, byteNum+1)
        ser.write(data2)
        byteNum = byteNum + 2       
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'15'"):
        print("tkrSetCalMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be b'15'")
        
def tkrSetDataMask(FPGA, chip, sense, list):
    if sense != "mask" and sense != "unmask":
        print("tkrSetDataMask: the argument " + sense + " should be 'mask' or 'unmask'")
        return
    nItems = len(list)
    if nItems<0 or nItems>5:
        print("tkrSetDataMask: bad number of clusters of bad channels " + str(nItems) + ", must be from 0 to 5")
        return
    if chip > 11 and chip != 31: 
        print("tkrSetDataMask: the chip number " + str(chip) + " is out of range")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(5+2*nItems, 0x41, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(chip, address, 2)
    ser.write(data2)
    data3 = mkDataByte(DATAMASK, address, 3)
    ser.write(data3)
    if sense == "mask": data4 = mkDataByte(0, address, 4)
    else: data4 = mkDataByte(1, address, 4)
    ser.write(data4)
    data5 = mkDataByte(nItems, address, 5)
    ser.write(data5)
    byteNum = 6
    for item in list:
        #print("tkrSetDataMask: " + sense + " channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
        data1 = mkDataByte(item[0], address, byteNum)
        ser.write(data1)
        data2 = mkDataByte(item[1], address, byteNum+1)
        ser.write(data2)
        byteNum = byteNum + 2       
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'13'"):
        print("tkrSetDataMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be b'13'")
        
def tkrSetDAC(FPGA, chip, select, value, range):
    if select != "threshold" and select != "calibration":
        print("tkrSetDAC: the argument " + select + " should be 'threshold' or 'calibration'")
        return
    if range != "high" and range != "low":
        print("tkrSetDAC: the range " + range + " should be 'high' or 'low'")
        return
    if chip > 11 and chip != 31: 
        print("tkrSetDAC: the chip number " + str(chip) + " is out of range")
        return
    if select == "threshold": cmdCode = 0x11
    else: cmdCode = 0x10
    if value > 127:
        print("tkrSetDAC: the DAC setting " + str(value) + " is out of range for the " + select + " DAC")
        return
    print("tkrSetDAC: setting " + select + " DAC in chip " + str(chip) + " of FPGA " + str(FPGA) + " to " + str(value) + " and range " + range)
    if range == "high": value = value + 128
    address = addrEvnt
    if select == "threshold":
        cmdHeader = mkCmdHdr(3, 0x55, address)
        ser.write(cmdHeader)
        data1 = mkDataByte(FPGA, address, 1)
        ser.write(data1)
        data2 = mkDataByte(chip, address, 2)
        ser.write(data2)
        data3 = mkDataByte(value, address, 3)
        ser.write(data3)
    else:
        cmdHeader = mkCmdHdr(5, 0x10, address)
        ser.write(cmdHeader)
        data1 = mkDataByte(FPGA, address, 1)
        ser.write(data1)
        data2 = mkDataByte(cmdCode, address, 2)
        ser.write(data2)
        data3 = mkDataByte(0x02, address, 3)
        ser.write(data3)
        data4 = mkDataByte(chip, address, 4)
        ser.write(data4)
        data5 = mkDataByte(value, address, 5)
        ser.write(data5)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (bytes2int(echo) != cmdCode):
        print("tkrSetDAC: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be " + str(cmdCode))

def tkrGetDAC(FPGA, chip, select):
    if select != "threshold" and select != "calibration":
        print("tkrGetDAC: the argument " + select + " should be 'threshold' or 'calibration'")
        return
    if chip > 11: 
        print("tkrGetDAC: the chip number " + str(chip) + " is out of range")
        return
    if select == "threshold": cmdCode = 0x21
    else: cmdCode = 0x20
    address = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(cmdCode, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(chip, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    if (bytes2int(byteList[0]) != 9):
        print("tkrGetDAC: wrong number " + str(bytes2int(byteList[0])) + " of bytes returned")
        for byte in byteList:
            print("tkrGetDAC for FPGA " + str(FPGA) + " chip " + str(chip) + " byte = " + str(binascii.hexlify(byte)))
        return
    stuff = getBinaryString(byteList[1:3])
    if select == "threshold": 
        if stuff[1:4] != "010": print("tkrGetDAC: wrong register ID " + stuff[1:4])
    else:
        if stuff[1:4] != "001": print("tkrGetDAC: wrong register ID " + stuff[1:4])
    print(select + " DAC for chip " + str(chip) + " of board " + str(FPGA) + ": " + stuff[5:13])
    
def tkrSetTriggerMask(FPGA, chip, sense, list):
    if sense != "mask" and sense != "unmask":
        print("tkrSetTriggerMask: the argument " + sense + " should be 'mask' or 'unmask'")
        return
    nItems = len(list)
    if nItems<0 or nItems>5:
        print("tkrSetTriggerMask: bad number of clusters of bad channels " + str(nItems) + ", must be from 1 to 5")
        return
    if chip > 11 and chip != 31: 
        print("tkrSetTriggerMask: the chip number " + str(chip) + " is out of range")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(5+2*nItems, 0x41, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(chip, address, 2)
    ser.write(data2)
    data3 = mkDataByte(TRIGMASK, address, 3)
    ser.write(data3)
    if sense == "mask": data4 = mkDataByte(0, address, 4)
    else: data4 = mkDataByte(1, address, 4)
    ser.write(data4)
    data5 = mkDataByte(nItems, address, 5)
    ser.write(data5)
    byteNum = 6
    for item in list:
        #print("tkrSetTriggerMask: " + sense + " channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
        data1 = mkDataByte(item[0], address, byteNum)
        ser.write(data1)
        data2 = mkDataByte(item[1], address, byteNum+1)
        ser.write(data2)
        byteNum = byteNum + 2       
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'14'"):
        print("tkrSetTriggerMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be b'14'")

def tkrRandomCalMask(FPGA, chip):
    address = addrEvnt
    cmdHeader = mkCmdHdr(12, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x15, address, 2)
    ser.write(data2)
    data3 = mkDataByte(9, address, 3)
    ser.write(data3)
    data4 = mkDataByte(chip, address, 4)
    #print("data4= " + str(data4))
    ser.write(data4)
    data5 = mkDataByte(random.randint(0,255), address, 5)
    ser.write(data5)
    #print("data5= " + str(data5))
    data6 = mkDataByte(random.randint(0,255), address, 6)
    ser.write(data6)
    data7 = mkDataByte(random.randint(0,255), address, 7)
    ser.write(data7)
    data8 = mkDataByte(random.randint(0,255), address, 8)
    ser.write(data8)
    data9 = mkDataByte(random.randint(0,255), address, 9)
    ser.write(data9)
    data10 = mkDataByte(random.randint(0,255), address, 10)
    ser.write(data10)
    data11 = mkDataByte(random.randint(0,255), address, 11)
    ser.write(data11)
    data12 = mkDataByte(random.randint(0,255), address, 12)
    ser.write(data12)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'15'"):
        print("tkrRandomCalMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be b'15'")

def tkrGetCalMask(FPGA, chip, verbose = True):
    if chip > 11: 
        print("tkrGetCalMask: the chip number " + str(chip) + " is out of range")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x25, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data3 = mkDataByte(chip, address, 4)
    ser.write(data3)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    #for byte in byteList:
    #    print("tkrGetCalMask for FPGA " + str(FPGA) + " chip " + str(chip) + " byte = " + str(binascii.hexlify(byte)))
    if (bytes2int(byteList[0]) != 9):
        print("tkrGetCalMask: wrong number " + str(bytes2int(byteList[0])) + " of bytes returned")
        return
    stuff = getBinaryString(byteList[1:10])
    if stuff[1:4] != "110": print("tkrGetCalMask: wrong register ID " + stuff[1:4])
    if verbose: print("Calibration mask for chip " + str(chip) + " of board " + str(FPGA) + ": " + stuff[5:69])
    return stuff[5:69]

def tkrGetDataMask(FPGA, chip):
    if chip > 11: 
        print("tkrGetDataMask: the chip number " + str(chip) + " is out of range")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x23, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data3 = mkDataByte(chip, address, 4)
    ser.write(data3)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    #for byte in byteList:
    #    print("tkrGetDataMask for FPGA " + str(FPGA) + " chip " + str(chip) + " byte = " + str(binascii.hexlify(byte)))
    if (bytes2int(byteList[0]) != 9):
        print("tkrGetDataMask: wrong number " + str(bytes2int(byteList[0])) + " of bytes returned")
        return
    stuff = getBinaryString(byteList[1:10])
    if stuff[1:4] != "100": print("tkrGetDataMask: wrong register ID " + stuff[1:4])
    print("Data mask for chip " + str(chip) + " of board " + str(FPGA) + ": " + stuff[5:69])
    return stuff[5:69]
    
def tkrGetTriggerMask(FPGA, chip):
    if chip > 11: 
        print("tkrGetTriggerMask: the chip number " + str(chip) + " is out of range")
        return
    address = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x24, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data3 = mkDataByte(chip, address, 4)
    ser.write(data3)
    time.sleep(0.1)
    byteList = getTkrHousekeeping()
    #for byte in byteList:
    #    print("tkrGetTriggerMask for FPGA " + str(FPGA) + " chip " + str(chip) + " byte = " + str(binascii.hexlify(byte)))
    if (bytes2int(byteList[0]) != 9):
        print("tkrGetTriggerMask: wrong number " + str(bytes2int(byteList[0])) + " of bytes returned")
        return
    stuff = getBinaryString(byteList[1:10])
    if stuff[1:4] != "101": print("tkrGetTriggerMask: wrong register ID " + stuff[1:4])
    print("Trigger mask for chip " + str(chip) + " of board " + str(FPGA) + ": " + stuff[5:69])
    return stuff[5:69]

def tkrSetTriggerSource(N):
    address = addrEvnt
    print("Set the tracker trigger source to " + str(N))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x64, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(N, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'64'"):
        print("tkrSetTriggerSource: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'64'")

def tkrGetTriggerSource(FPGA):
    address = addrEvnt
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x74, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    return getTkrHousekeeping()[7]    
    
def tkrSetNumLyrs(N):
    address = addrEvnt
    print("Set the number of tracker readout layers to " + str(N))
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0F, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x01, address, 3)
    ser.write(data3)
    data4 = mkDataByte(N, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'0f'"):
        print("tkrSetNumLyrs: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'0f'")  

# Turn off the ASIC analog power
def tkrAsicPowerOff():
    address = addrEvnt
    print("Turning off the ASIC analog power")
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x09, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'09'"):
        print("tkrAsicPowerOff: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'09'")   

def tkrSetTrgMask(FPGA,Value):
    address = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x62, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    data4 = mkDataByte(Value, address, 4)
    ser.write(data4)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'62'"):
        print("tkrSetTrgMask: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'62'")  
        
def tkrTriggerEnable():
    address = addrEvnt
    print("Enable the tracker trigger.")
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x65, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'65'"):
        print("tkrTriggerEnable: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'65'")  
        
def tkrTriggerDisable():
    address = addrEvnt
    print("Disable the tracker trigger.")
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(0, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x66, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'66'"):
        print("tkrTriggerDisable: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'66'")  

def tkrGetFPGAconfig(FPGA):
    address = addrEvnt
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x0B, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    houseStuff = getTkrHousekeeping()[7]
    return houseStuff.hex()
    
def tkrGetNumLyrs(FPGA):
    address = addrEvnt
    cmdHeader = mkCmdHdr(3, 0x10, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, address, 1)
    ser.write(data1)
    data2 = mkDataByte(0x1F, address, 2)
    ser.write(data2)
    data3 = mkDataByte(0x00, address, 3)
    ser.write(data3)
    time.sleep(0.1)
    return getTkrHousekeeping()[7]
    
def tkrGetASICconfig(FPGA, chipAddress, quiet=False):
    if not quiet: print("tkrGetASICconfig: the configuration for ASIC " + str(chipAddress) + 
          " of FPGA " + str(FPGA) + " is as follows:")
    PSOCaddress = addrEvnt
    cmdHeader = mkCmdHdr(4, 0x10, PSOCaddress)
    #print("tkrGetASICconfig: cmdHeader = " + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, PSOCaddress, 1)
    #print("tkrGetASICconfig: data1 = " +str(data1))
    ser.write(data1)
    data2 = mkDataByte(0x22, PSOCaddress, 2)
    #print("tkrGetASICconfig: data2 = " +str(data2))
    ser.write(data2)
    data3 = mkDataByte(0x01, PSOCaddress, 3)
    #print("tkrGetASICconfig: data3 = " +str(data3))
    ser.write(data3)
    data4 = mkDataByte(chipAddress, PSOCaddress, 4)
    #print("tkrGetASICconfig: data4 = " +str(data4))
    ser.write(data4)
    time.sleep(0.1)
    bytes = getTkrHousekeeping()
    #for byte in bytes:
    #    print("tkrGetASICconfig for FPGA " + str(FPGA) + " chip " + str(chipAddress) + " byte = " + str(binascii.hexlify(byte)))
    if quiet: return
    if bytes2int(bytes[0]) != 9:
        print("tkrGetASICconfig: wrong number bytes returned. Got " + str(bytes2int(bytes[0])) + " and expected 9")
        if bytes2int(bytes[0]) < 9: return      
    if bytes[9] != b'\xfe':
        print("tkrGetASICconfig, trailing byte " + str(bytes[8]) + " is not the expected 0xFE")
    word1 = bytes[1]+bytes[2]+bytes[3]+bytes[4]
    #print("tkrGetASICconfig: binary data returned = " + str(binascii.hexlify(word1)))
    intWord = bytes2int(word1)
    regType = (intWord & 0x70000000) >> 28
    if regType != 3:
        print("tkrGetASICconfig: register type returned = " + str(regType) + " should be 3!")
    errCodes = (intWord & 0x07000000) >> 24
    print("tkrGetASICconfig: ASIC error codes = " + str(errCodes))
    if errCodes & 4 != 0:
        print("tkrGetASICconfig: a command parity error was detected by the ASIC.")
    if errCodes & 2 != 0:
        print("tkrGetASICconfig: a trigger was received by the ASIC while a previous one was in progress.")
    if errCodes & 1 != 0:
        print("tkrGetASICconfig: a read command was sent to the ASIC with no prior trigger.")
    maxClusts = (intWord & 0x00F00000) >> 20
    print("tkrGetASICconfig: the maximum number of clusters is " + str(maxClusts))
    ioCurrent = (intWord & 0x000C0000) >> 18
    print("tkrGetASICconfig: the I/O current setting is " + str(ioCurrent))
    trigWindow = (intWord & 0x00020000) >> 17
    print("tkrGetASICconfig: the trigger window is " + str(trigWindow+2) + " clock cycles long.")
    trigDelay = (intWord & 0x0001F000) >> 12
    print("tkrGetASICconfig: the trigger delay setting is " + str(trigDelay))
    bufSpeed = (intWord & 0x00000E00) >> 9
    print("tkrGetASICconfig: the buffer speed setting is " + str(bufSpeed))
    shapTim = (intWord & 0x00000100) >> 8
    if shapTim == 0:
        print("tkrGetASICconfig: the shaping time setting is short.")
    else:
        print("tkrGetASICconfig: the shaping time setting is long.")
    gain = (intWord & 0x00000080) >> 7
    if gain == 0:
        print("tkrGetASICconfig: the gain setting is high.")
    else:
        print("tkrGetASICconfig: the gain setting is low.")
    oneShot = (intWord & 0x00000040) >> 6
    if oneShot == 0:
        print("tkrGetASICconfig: the 1-shot feature is on.")
    else:
        print("tkrGetASICconfig: the 1-shot feature is off.")
    polarity = (intWord & 0x00000020) >> 5
    if polarity == 0:
        print("tkrGetASICconfig: the polarity setting is normal.")
    else:
        print("tkrGetASICconfig: the polarity setting is negative.")

def configureTkrASICs(numTKRlyrs):
    print("configureTkrASICs: configure the tracker ASICs for " + str(numTKRlyrs) + " layers, after turning on the ASIC power.")
    cmdHeader = mkCmdHdr(1, 0x56, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(numTKRlyrs, addrEvnt, 1)
    ser.write(data1)
    time.sleep(0.1)

def tkrLoadASICconfig(FPGA, address, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust):
    gain = gain & 0x01
    oneShot = oneShot & 0x01
    shaping = shaping & 0x01
    bufSpeed = bufSpeed & 0x07
    trigDelay = trigDelay & 0x1F
    trigWindow = trigWindow & 0x01
    ioCurrent = ioCurrent & 0x03
    maxClust = maxClust & 0x0F
    if maxClust > 10: maxClust = 10
    nib1 = 0x0F;
    nib2 = (oneShot << 2) | (gain << 3) 
    nib3 = shaping | (bufSpeed << 1)
    nib4 = trigDelay & 0x0F
    nib5 = (trigDelay & 0x10)>>4 | trigWindow<<1 | ioCurrent<<2
    nib6 = maxClust
    all = nib6<<20 | nib5<<16 | nib4<<12 | nib3<<8 | nib2<<4 | nib1
    print("tkrLoadASICconfig: for FPGA " + str(FPGA) + ", chip " + str(address) + 
             ", the register setting will be " + str(hex(all)))
    PSOCaddress = addrEvnt
    cmdHeader = mkCmdHdr(3, 0x54, PSOCaddress)
    #print("tkrLoadASICconfig: cmdHeader = " + str(cmdHeader))
    ser.write(cmdHeader)
    byte5 = (nib6<<4) | nib5
    data1 = mkDataByte(byte5, PSOCaddress, 1)
    ser.write(data1)
    byte6 = (nib4<<4) | nib3
    data2 = mkDataByte(byte6, PSOCaddress, 2)
    ser.write(data2)
    byte7 = (nib2<<4) | nib1
    data3 = mkDataByte(byte7, PSOCaddress, 3)
    ser.write(data3)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'12'"):
        print("tkrLoadASICconfig: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'12'")  
        
# Re-initialize the event PSOC SPI interface        
def initSPI():
    address = addrEvnt
    cmdHeader = mkCmdHdr(0, 0x31, address)
    ser.write(cmdHeader)
    print("initSPI: initializing the SPI communication on the Event PSOC")
    
# Re-initialize the UART
def initUART():
    address = addrMain
    cmdHeader = mkCmdHdr(0, 0x31, address)
    ser.write(cmdHeader)
    print("initUART: initializing the UART for sending commands to the Event PSOC")

def stopTOF():
    address = addrEvnt
    cmdHeader = mkCmdHdr(0, 0x3F, address)
    ser.write(cmdHeader)    

# Send TOF info directly to USB-UART at each interrupt, for testing
def startTOF(numEvt):
    address = addrEvnt
    cmdHeader = mkCmdHdr(0, 0x32, address)
    ser.write(cmdHeader)
    for event in range(numEvt):
        # Wait for an event to show up
        channel1 = b'  '
        while True:
            ret = ser.read(1)
            #print("startTOF: looking for start of packet. Received bytes " + str(ret.hex()))
            channel1 = binascii.hexlify(ret)
            if channel1 == b'aa' or channel1 == b'cc': break
            time.sleep(0.01)
        if channel1 == b'aa': print("startTOF: reading event " + str(event) + " for channel " + str(channel1))
        value = bytes2int(ser.read(2))
        if channel1 == b'aa': print("  Channel " + str(channel1) +  " stop result=" + str(value) + " " + str(hex(value))) 
        ref = bytes2int(ser.read(2))
        if channel1 == b'aa': print("  Channel " + str(channel1) + " reference index=" + str(ref) + " " + str(hex(ref)))
        timev1 = (ref*8333 + value)*10
        if channel1 == b'aa': print("  Channel " + str(channel1) + " time = " + str(timev1))   
        clkcnt = bytes2int(ser.read(2))
        if channel1 == b'aa': print("  Channel " + str(channel1) + " clock count = " + str(clkcnt)) 
        cntr = bytes2int(ser.read(1))
        if channel1 == b'aa': print("  Channel " + str(channel1) + " FIFO length = " + str(cntr))
        ret = ser.read(1)
        channel2 = binascii.hexlify(ret)
        if channel2 == b'bb': print("startTOF: reading event " + str(event) + " for channel " + str(channel2))
        value = bytes2int(ser.read(2))
        if channel2 == b'bb': print("  Channel " + str(channel2) +  " stop result=" + str(value) + " " + str(hex(value))) 
        ref = bytes2int(ser.read(2))
        if channel2 == b'bb': print("  Channel " + str(channel2) + " reference index=" + str(ref) + " " + str(hex(ref)))
        timev2 = (ref*8333 + value)*10
        if channel2 == b'bb': print("  Channel " + str(channel2) + " time = " + str(timev2))   
        clkcnt = bytes2int(ser.read(2))
        if channel2 == b'bb': print("  Channel " + str(channel2) + " clock count = " + str(clkcnt)) 
        cntr = bytes2int(ser.read(1))
        if channel2 == b'bb': print("  Channel " + str(channel2) + " FIFO length = " + str(cntr))
        if channel1 == b'aa' and channel2 == b'bb':
            tof = timev1 - timev2
            print("    Time-of-Flight = " + str(tof))

# Lots of barometer calibration stuff
def Kfactor(A, S, OTP):
    return A + (S*OTP)/32767.

def aa_barometer(address):
    COE_PTAT32 = readBarometerReg(0xB4, address)
    COE_PTAT31 = readBarometerReg(0xB3, address)
    COE_PTAT = toDec16((COE_PTAT31<<8)  + COE_PTAT32)
    return Kfactor(0., 4.2e-4, COE_PTAT)

def ba_barometer(address):
    COE_PTAT21 = readBarometerReg(0xB1, address)
    COE_PTAT22 = readBarometerReg(0xB2, address)
    COE_PTAT = toDec16((COE_PTAT21<<8)  + COE_PTAT22)
    return Kfactor(-160., 8.0, COE_PTAT)
    
def ca_barometer(address):
    COE_PTAT11 = readBarometerReg(0xAD, address)
    COE_PTAT12 = readBarometerReg(0xAE, address)
    COE_PTAT13 = readBarometerReg(0xAF, address)
    COE_PTAT = toDec24((COE_PTAT11<<16) + (COE_PTAT12<<8) + COE_PTAT13)
    return COE_PTAT

def ap_barometer(address):
    COE_PR31 = readBarometerReg(0xA5, address)
    COE_PR32 = readBarometerReg(0xA6, address)
    COE_PR = toDec16((COE_PR31<<8) + COE_PR32)
    #print("  COE_PR31=" + str(COE_PR31) + "  COE_PR32=" + str(COE_PR32) + "  COE_PR=" + str(COE_PR))
    return Kfactor(0.0, 3.0e-5, COE_PR)
    
def bp_barometer(address):
    COE_PR21 = readBarometerReg(0xA3, address)
    COE_PR22 = readBarometerReg(0xA4, address)
    COE_PR = toDec16((COE_PR21<<8) + COE_PR22)
    return Kfactor(30., 10., COE_PR)
    
def cp_barometer(address):
    COE_PR11 = readBarometerReg(0xA0, address)
    COE_PR12 = readBarometerReg(0xA1, address)
    COE_PR13 = readBarometerReg(0xA2, address)
    COE_PR = toDec24((COE_PR11<<16) + (COE_PR12<<8) + COE_PR13)
    return COE_PR

def at_barometer(address):
    COE_TEMP31 = readBarometerReg(0xAB, address)
    COE_TEMP32 = readBarometerReg(0xAC, address)
    COE_TEMP = toDec16((COE_TEMP31<<8) + COE_TEMP32)
    return Kfactor(0., 8.0e-11, COE_TEMP)
    
def bt_barometer(address):
    COE_TEMP21 = readBarometerReg(0xA9, address)
    COE_TEMP22 = readBarometerReg(0xAA, address)
    COE_TEMP = toDec16((COE_TEMP21<<8) + COE_TEMP22)
    return Kfactor(-6.6e-6, 1.6e-6, COE_TEMP)
    
def ct_barometer(address):
    COE_TEMP11 = readBarometerReg(0xA7, address)
    COE_TEMP12 = readBarometerReg(0xA8, address)
    COE_TEMP = toDec16((COE_TEMP11<<8) + COE_TEMP12)
    return Kfactor(0.04, 0.0085, COE_TEMP)

# Read the temperature from the barometer chip
def readBarometerTemp(PSOCaddress):
    while ((readBarometerReg(0xF3, PSOCaddress) & 0x08) != 0):
        pass
    TXD0 = readBarometerReg(0xFC, PSOCaddress)
    #print("readBarometerTemp: TXD0 = " + str(TXD0))
    TXD1 = readBarometerReg(0xFB, PSOCaddress)
    #print("readBarometerTemp: TXD1 = " + str(TXD1))
    TXD2 = readBarometerReg(0xFA, PSOCaddress)
    #print("readBarometerTemp: TXD2 = " + str(TXD2))
    # Apparently the 2SMPB-02B barometer chip uses 2's complement negative numbers for the calibration constants 
    # but not for the raw temperature and pressure readings!
    Dt = (TXD2<<16) + (TXD1<<8) + TXD0 - pow(2,23)
    #print("Barometer Dt = " + str(Dt))
    aa = aa_barometer(PSOCaddress)
    ba = ba_barometer(PSOCaddress)
    ca = ca_barometer(PSOCaddress)
    #print("Barometer aa= " + str(aa) + "   ba= " + str(ba) + "    ca= " + str(ca))
    Tr = (-ba - math.sqrt(ba*ba - 4.0*aa*(ca-Dt)))/(2.0*aa)
    return Tr/256.0

def readBarometer(PSOCaddress):
    while ((readBarometerReg(0xF3, PSOCaddress) & 0x08) != 0):
        pass
    TXD0 = readBarometerReg(0xF9, PSOCaddress)
    #print("readBarometer: TXD0 = " + str(TXD0))
    TXD1 = readBarometerReg(0xF8, PSOCaddress)
    #print("readBarometer: TXD1 = " + str(TXD1))
    TXD2 = readBarometerReg(0xF7, PSOCaddress)
    #print("readBarometer: TXD2 = " + str(TXD2))
    # Apparently the 2SMPB-02B barometer chip uses 2's complement negative numbers for the calibration constants 
    # but not for the raw temperature and pressure readings!
    Dp = (TXD2<<16) + (TXD1<<8) + TXD0 - pow(2,23)
    #print("Barometer Dp = " + str(Dp))
    ap = ap_barometer(PSOCaddress)
    bp = bp_barometer(PSOCaddress)
    cp = cp_barometer(PSOCaddress)
    #print("Barometer ap= " + str(ap) + "   bp= " + str(bp) + "    cp= " + str(cp))
    #print(str(cp-Dp))
    arg = bp*bp - 4.0*ap*(cp-Dp)
    #print("Argument of sqrt = " + str(arg))
    Pl = (-bp + math.sqrt(arg))/(2.0*ap)
    return Pl

def compensateBarometer(Pl, T, PSOCaddress):
    at = at_barometer(PSOCaddress)
    bt = bt_barometer(PSOCaddress)
    ct = ct_barometer(PSOCaddress)
    return Pl/((ct+1.0) + (bt + at*T)*T)
    
def getPressure(PSOCaddress):
    loadBarometerReg(0xF5, 0xA0, PSOCaddress)
    loadBarometerReg(0xF4, 0x6F, PSOCaddress)
    time.sleep(1.0)
    T = readBarometerTemp(PSOCaddress)
    pRaw = readBarometer(PSOCaddress)
    pressure = compensateBarometer(pRaw, T, PSOCaddress)
    print("2SMPB-02B leak-detector barometer pressure = " + str(pressure) + " Pa at a temperature of " + str(T) + " C")
    print("    p= " + str(pressure/100000.) + " bar, or " + str((pressure/100000.)*14.50377) + " psi")
    print("    p= " + str((pressure/100000.)*750.062) + " mm of Hg, or " + str((pressure/100000.)*29.53) + " inches of Hg")
    print("    p= " + str(pressure/101325) + " standard atmospheres")
    return (pressure, T)

def toDec16(a):   # 16-bit 2's complement to signed decimal
    if (a & pow(2,15)):
        twosComp = pow(2,16) - a
        return (-twosComp)
    else: 
        return a
    
def toDec24(a):   # 24-bit 2's complement to signed decimal
    if (a & pow(2,23)):
        twosComp = pow(2,24) - a
        return (-twosComp)
    else: 
        return a

view = {
0: "nonbending",
1: "bending",
2: "bending",
3: "bending",
4: "bending",
5: "nonbending",
6: "bending",
7: "nonbending",
8: "nonbending"
}

lyrZ = {
0: -1.50067,
1: -3.50727,
2: -5.99012,
3: -12.61952,
4: -19.22352,
5: -21.23012,
6: -23.23672,
7: -25.24332,
8: -1.50067,
}

lyrX = {
0: 0.,
1: 0.,
2: 0.,
3: 0.,
4: 0.,
5: 0.,
6: 0.,
7: 0.,
8: 0.
}

days = {
0: "Monday",
1: "Tuesday",
2: "Wednesday",
3: "Thursday",
4: "Friday",
5: "Saturday",
6: "Sunday",
7: "Monday"
}

months = {
0 : "unknown",
1 : "January",
2 : "February",
3 : "March",
4 : "April",
5 : "May",
6 : "June",
7 : "July",
8 : "August",
9 : "September",
10 : "October",
11 : "November",
12 : "December"
}

INA226_Address = {
'DVDD5' : 0x41,
'DVDD33' : 0x44,
'AVDD5' : 0x45,
'AVDD33' : 0x43,
'Back15' : 0x42,
'TkrBias' : 0x46,
'TKR' : 0x40
}

i2cAddresses = {
'flash18' : 0x45,
'fpga12'  : 0x40,
'digi25'  : 0x41,
'i2c33'   : 0x42,
'analog21': 0x44, 
'analog33': 0x43,
'bias100' : 0x46,
'temp'    : 0x48
}
