import serial
import time
import numpy
import binascii
from bitstring import BitArray
import math
import numpy as np

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

def LED2(onOFF, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x06, PSOCaddress)
    ser.write(cmdHeader)
    if onOFF == "on": data1 = mkDataByte(1, PSOCaddress, 1)
    else: data1 = mkDataByte(0, PSOCaddress, 1)
    ser.write(data1)
    print("LED2: turning LED number 2 " + onOFF + " for PSOC address " + str(PSOCaddress))
    
# Reset the logic and counters
def logicReset(PSOCaddress):
    cmdHeader = mkCmdHdr(0, 0x38, PSOCaddress)
    ser.write(cmdHeader)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("logicReset: invalid header returned: " + str(ret))
    count = bytes2int(ser.read(3))
    print("logicReset: the logic of PSOC " + str(PSOCaddress) + " was reset at clkCnt= " + str(count))
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("logicReset: invalid trailer returned: " + str(ret))  
        
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

def setTriggerWindow(count):
    cmdHeader = mkCmdHdr(1, 0x3A, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(count, addrEvnt, 1)
    ser.write(data1)
    print("setTriggerWindow: setting the trigger window to " + str(count) + " counts")

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
  divisor = "1100101"
  result = bitString 
  for i in range(len(result) - len(divisor)+1):
    #print result#result[0:(len(result)-len(divisor)+1)] , result[(len(result)-len(divisor)+1):]
    if (result[i] == "1"):
      for j in range(len(divisor)):
          if (result[i+j] == divisor[j]):
            result = result[0:i+j] + "0" + result[i+j+1:len(result)]
          else:
            result = result[0:i+j] + "1" + result[i+j+1:len(result)]
    #print " "*i+divisor
  return result[len(result)-6:]
  
def ParseASIChitList(bitString, verbose):
  pointer = 0    
  firstStripChip = [] 
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
    return [rc,nHits]

def printChipClusters(bitString, verbose): # takes a chip header and following clusters
  firstStripList = []                        #list of all the first strip hit list, to be appended over each cluster and each chip hit
  if bitString == "":
    if verbose: print("    Error:      Empty cluster list. . .")
    return firstStripList
  pointer = 12
  nclust = getChipNumberOfClusters(bitString)
  if len(bitString) != 12 + 12*nclust:
    print("    Error:       Wrong length cluster list. . .")
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
  
  ret = ser.read(3)
  if ret != b'\xDC\x00\xFF':
    print("tkrReadi2cReg: invalid header returned: " + str(ret))
  nBytes = bytes2int(ser.read(1))
  #print("tkrReadi2cReg: number of bytes returned = " + str(nBytes))
  ser.read(2)
  ret = ser.read(3)
  if ret != b'\xFF\x00\xFF':
    print("tkrReadi2cReg: invalid trailer returned: " + str(ret))      
  results = []
  for packet in range(2):
      ret = ser.read(3)
      if ret != b'\xDC\x00\xFF':
          print("tkrReadi2cReg: invalid header returned: " + str(ret))
      for chunk in range(3):
          ret = ser.read(1)
          results.append(ret)
          #print("tkrReadi2cReg: byte " + str(chunk) + " in packet " + str(packet) + " = " + str(ret))
      ret = ser.read(3)
      if ret != b'\xFF\x00\xFF':
          print("tkrReadi2cReg: invalid trailer returned: " + str(ret))    
  result = getBinaryString(results[1:3])
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
    print("  Shunt voltage for " + item + " = " + str(shuntVoltage) + " mV; Shunt current = " + str(shuntCurrent) + " microamps for board " + str(FPGA))
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

def sendTkrCalStrobe(FPGA, trgDelay, trgTag):
    cmdHeader = mkCmdHdr(3, 0x42, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(trgDelay, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(trgTag, addrEvnt, 3)
    ser.write(data3)
    print("sendTkrCalStrobe: calibration strobe sent to FPGA " + str(FPGA) + " for trigger tag " + str(trgTag) + " and delay " + str(trgDelay))
    time.sleep(0.1)
    nBytes = 9
    nPackets = 4
    print("sendTkrCalStrobe: expecting " + str(nBytes) + " bytes coming back from the tracker.")
    byteList = []
    for i in range(nPackets):
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("sendTkrCalStrobe: invalid header returned: " + str(ret))
        byteList.append(ser.read())
        byteList.append(ser.read())
        byteList.append(ser.read())
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("sendTkrCalStrobe: invalid trailer returned: " + str(ret))        
    if (bytes2int(byteList[0]) != 9): print("sendTkrCalStrobe: wrong number " + str(byteList[0].hex()) + " of bytes returned")
    i=0
    for byte in byteList:   
        print("  Byte " + str(i) + " is " + str(byte.hex()))
        i=i+1
    theStuff = getBinaryString(byteList[3:12])
    print("Bits returned = " + theStuff)
    fpgaBack = (bytes2int(byteList[3]) & 0x38)>>3
    if fpgaBack != FPGA: print("sendTkrCalStrobe: wrong FPGA address returned: " + str(fpgaBack))
    if theStuff[5:17] != "111111100001": print("sendTkrCalStrobe: wrong first pattern received")
    if theStuff[17:29] != "010011001111": print("sendTkrCalStrobe: wrong second pattern received")
    print("Pat1= " + theStuff[5:17])
    print("Pat2= " + theStuff[17:29])
    print("t0  = " + theStuff[29:41])
    print("ToT = " + theStuff[41:53])
    print("hits= " + theStuff[53:65])

# Read back tracker calibration data after executing a calibration strobe
# The trigger tag should match that given in the strobe command.
def readCalEvent(trgTag):
    cmdHeader = mkCmdHdr(1, 0x43, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(trgTag, addrEvnt, 1)
    ser.write(data1)
    print("readCalEvent: reading back calibration strobe data for tag " + str(trgTag))
    # Wait for an event to show up
    while True:
        ret = ser.read(3)
        print("readCalEvent: looking for start of event. Received bytes " + str(ret.hex()))
        if ret == b'\xDC\x00\xFF': break
        time.sleep(0.1) 
    ret = ser.read(1)
    nData = bytes2int(ret)
    print("ReadCalEvent: expecting " + str(nData) + " bytes of data for this event.")
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("ReadCalEvent: invalid trailer returned: " + str(ret))  
    R = nData % 3
    nPackets = int(nData/3)
    if (R != 0): nPackets = nPackets + 1
    dataList = []
    byteList = []
    verbose = False
    if verbose: print("ReadCalEvent: reading " + str(nData) + " data bytes in " + str(nPackets) + " packets")
    for i in range(nPackets):
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("ReadCalEvent: invalid header returned: " + str(ret))
        byte1 = ser.read()
        if verbose: print("   Packet " + str(i) + ", byte 1 = " + str(bytes2int(byte1)) + " decimal, " + str(byte1.hex()) + " hex")
        dataList.append(bytes2int(byte1))
        byteList.append(byte1)
        byte2 = ser.read()
        if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(byte2)) + " decimal, " + str(byte2.hex()) + " hex")
        dataList.append(bytes2int(byte2))
        byteList.append(byte2)
        byte3 = ser.read()
        if verbose: print("   Packet " + str(i) + ", byte 3 = " + str(bytes2int(byte3)) + " decimal, " + str(byte3.hex()) + " hex")
        dataList.append(bytes2int(byte3))
        byteList.append(byte3)
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readCalEvent: invalid trailer returned: " + str(ret)) 

    iPtr = 4
    numBoards = dataList[iPtr]
    if numBoards != 1: print("readCalEvent: wrong number of boards " + str(numBoards) + " in calibration event")
    iPtr = iPtr + 1
    brdNum = dataList[iPtr]
    iPtr = iPtr + 1
    nBytes = dataList[iPtr]
    iPtr = iPtr + 1
    #print(" Hit list from tracker board " + str(brdNum) + " with " + str(nBytes) + " bytes:")
    hitList = []
    for hit in range(nBytes):
        #print("                      " + str(hit) + "  " + hex(dataList[iPtr]))
        hitList.append(byteList[iPtr])
        iPtr = iPtr + 1
    #print("           Hit list= " + getBinaryString(hitList))
    rc = ParseASIChitList(getBinaryString(hitList),True)
            
# Execute a run for a specified number of events to be acquired
def limitedRun(runNumber, numEvnts):
    cmdHeader = mkCmdHdr(4, 0x3C, addrEvnt)
    ser.write(cmdHeader)
    data1 = mkDataByte(runNumber>>8, addrEvnt, 1)
    ser.write(data1)
    data2 = mkDataByte(runNumber & 0x00FF, addrEvnt, 2)
    ser.write(data2)
    data3 = mkDataByte(numEvnts>>8, addrEvnt, 3)
    ser.write(data3)
    data4 = mkDataByte(numEvnts & 0x00FF, addrEvnt, 4)
    ser.write(data4)
    print("limitedRun: starting run number " + str(runNumber) + " for " + str(numEvnts) + " events")
    f = open("dataFile_run" + str(runNumber) + ".txt", "w")
    print("limitedRun: trigger enable status = " + str(triggerEnableStatus()))
    time.sleep(0.1)
    pmtTrg1 = 0
    pmtTrg2 = 0
    tkrTrg0 = 0
    tkrTrg1 = 0
    pmtGrd = 0
    ADCavg = [0.,0.,0.,0.,0.,0.]
    ADCavg2 = [0.,0.,0.,0.,0.,0.]
    TOFavg = 0.
    TOFavg2 = 0.
    startTime = time.time()
    nBadTkr = 0
    lastTime = 0
    timeSum = 0
    numHits = 0
    for event in range(numEvnts):
        # Wait for an event to show up
        while True:
            ret = ser.read(3)
            print("limitedRun: looking for start of event. Received bytes " + str(ret.hex()))
            if ret == b'\xDC\x00\xFF': break
            time.sleep(0.1)
        verbose = True
        print("limitedRun: reading event " + str(event) + " of run " + str(runNumber))
        ret = ser.read(1)
        nData = bytes2int(ret)
        ser.read(2)
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("limitedRun: invalid trailer returned: " + str(ret))  
        R = nData % 3
        nPackets = int(nData/3)
        if (R != 0): nPackets = nPackets + 1
        dataList = []
        byteList = []
        if verbose: print("limitedRun: reading " + str(nData) + " data bytes in " + str(nPackets) + " packets")
        for i in range(nPackets):
            ret = ser.read(3)
            if ret != b'\xDC\x00\xFF':
                print("limitedRun: invalid header returned: " + str(ret))
            byte1 = ser.read()
            if verbose: print("   Packet " + str(i) + ", byte 1 = " + str(bytes2int(byte1)) + " decimal, " + str(byte1.hex()) + " hex")
            dataList.append(bytes2int(byte1))
            byteList.append(byte1)
            byte2 = ser.read()
            if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(byte2)) + " decimal, " + str(byte2.hex()) + " hex")
            dataList.append(bytes2int(byte2))
            byteList.append(byte2)
            byte3 = ser.read()
            if verbose: print("   Packet " + str(i) + ", byte 3 = " + str(bytes2int(byte3)) + " decimal, " + str(byte3.hex()) + " hex")
            dataList.append(bytes2int(byte3))
            byteList.append(byte3)
            ret = ser.read(3)
            if ret != b'\xFF\x00\xFF':
                print("limitedRun: invalid trailer returned: " + str(ret)) 
        run = dataList[4]*256 + dataList[5]
        trigger = dataList[6]*16777216 + dataList[7]*65536 + dataList[8]*256 + dataList[9]
        if verbose: print("   Trigger " + str(trigger) + ", Data List length = " + str(len(dataList)))            
        timeStamp = dataList[10]*16777216 + dataList[11]*65536 + dataList[12]*256 + dataList[13]
        deltaTime = timeStamp - lastTime
        if event > 0:
            if verbose: print("    Time since the previous event = " + str(deltaTime))
            timeSum += deltaTime
        lastTime = timeStamp
        T1 = dataList[19]*256 + dataList[20]
        T2 = dataList[21]*256 + dataList[22]
        T3 = dataList[23]*256 + dataList[24]
        T4 = dataList[25]*256 + dataList[26]
        G =  dataList[27]*256 + dataList[28]
        Ex = dataList[29]*256 + dataList[30]
        cntGo = dataList[14]*16777216 + dataList[15]*65536 + dataList[16]*256 + dataList[17]
        if verbose:
            print("        T1 ADC=" + str(T1))
            print("        T2 ADC=" + str(T2))
            print("        T3 ADC=" + str(T3))
            print("        T4 ADC=" + str(T4))
            print("         G ADC=" + str(G))
            print("        Ex ADC=" + str(Ex))
        nTOFA = dataList[37]
        nTOFB = dataList[38]
        dtmin = 10*np.int16(dataList[31]*256 + dataList[32])
        if verbose:
            print("        TimeStamp = " + str(timeStamp))
            print("        TOF=" + str(dtmin) + " Number A=" + str(nTOFA) + " Number B=" + str(nTOFB))
            print("        run=" + str(run) + "  trigger " + str(trigger))
        tofA = 10*(dataList[39]*256 + dataList[40])
        tofB = 10*(dataList[41]*256 + dataList[42])
        clkA = dataList[43]*256 + dataList[44]
        clkB = dataList[45]*256 + dataList[46]
        trgStatus = dataList[18]
        nTkrLyrs = dataList[47]
        if verbose: 
            print("        REF-A=" + str(tofA) + "  REF-B=" + str(tofB))
            print("        TOF clkA=" + str(clkA) + "  TOF clkB=" + str(clkB))
            print("        Trigger status = " + str(hex(trgStatus)))
            print("      Number of tracker layers read out = " + str(nTkrLyrs))
            iPtr = 48
            for brd in range(nTkrLyrs):
                brdNum = dataList[iPtr]
                iPtr = iPtr + 1
                nBytes = dataList[iPtr]
                iPtr = iPtr + 1
                #print("           " + str(brd) + " Hit list from tracker board " + str(brdNum) + " with " + str(nBytes) + " bytes:")
                hitList = []
                for hit in range(nBytes):
                    #print("                      " + str(hit) + "  " + hex(dataList[iPtr]))
                    hitList.append(byteList[iPtr])
                    iPtr = iPtr + 1
            #print("           Hit list= " + getBinaryString(hitList))
            rc = ParseASIChitList(getBinaryString(hitList),True)
            if rc[0] != 0: nBadTkr = nBadTkr + 1
            numHits = numHits + rc[1]
        if trgStatus & 0x01: pmtTrg1 = pmtTrg1 + 1
        if trgStatus & 0x02: pmtTrg2 = pmtTrg2 + 1
        if trgStatus & 0x04: tkrTrg0 = tkrTrg0 + 1
        if trgStatus & 0x08: tkrTrg1 = tkrTrg1 + 1
        if trgStatus & 0x10: pmtGrd = pmtGrd + 1
        if abs(dtmin) < 10000.:
            strOut = '{}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}\n'.format(trigger, T1, T2, T3, T4, G, Ex, tofA, tofB, dtmin, nTOFA, nTOFB, deltaTime)
            f.write(strOut)   
        TOFavg = TOFavg + float(dtmin)
        TOFavg2 = TOFavg2 + float(dtmin)*float(dtmin)
        ADCavg[0] = ADCavg[0] + T1
        ADCavg[1] = ADCavg[1] + T2
        ADCavg[2] = ADCavg[2] + T3
        ADCavg[3] = ADCavg[3] + T4
        ADCavg[4] = ADCavg[4] + G
        ADCavg[5] = ADCavg[5] + Ex
        ADCavg2[0] = ADCavg2[0] + T1*T1
        ADCavg2[1] = ADCavg2[1] + T2*T2
        ADCavg2[2] = ADCavg2[2] + T3*T3
        ADCavg2[3] = ADCavg2[3] + T4*T4
        ADCavg2[4] = ADCavg2[4] + G*G
        ADCavg2[5] = ADCavg2[5] + Ex*Ex
    endTime = time.time()
    runTime = endTime - startTime
    print("Elapsed time for the run = " + str(runTime) + " seconds")
    timeSum = timeSum/float(numEvnts - 1)
    print("Average time between event time stamps = " + str(timeSum))
    Sigma = [0.,0.,0.,0.,0.,0.]
    TOFavg = TOFavg/float(numEvnts)
    TOFavg2 = TOFavg2/float(numEvnts)
    numHitsAvg = numHits/float(numEvnts)
    print("Average number of hits per event = " + str(numHitsAvg))
    sigmaTOF = math.sqrt(TOFavg2 - TOFavg*TOFavg)
    for ch in range(6):
        ADCavg[ch] = ADCavg[ch]/float(numEvnts)
        ADCavg2[ch] = ADCavg2[ch]/float(numEvnts)
        Sigma[ch] = math.sqrt(ADCavg2[ch] - ADCavg[ch]*ADCavg[ch])
    f.close()
    print("Number of triggers generated = " + str(cntGo))
    print("Number of triggers accepted = " + str(trigger))
    live = trigger/float(cntGo)
    print("Live time fraction = " + str(live))
    print("Number of primary PMT triggers captured = " + str(pmtTrg1))
    print("Number of secondary PMT triggers captured = " + str(pmtTrg2))
    print("Number of tracker-0 triggers captured = " + str(tkrTrg0))
    print("Number of tracker-1 triggers captured = " + str(tkrTrg1))
    print("Number of triggers with guard fired = " + str(pmtGrd))
    print("Number of bad tracker events = " + str(nBadTkr))
    return ADCavg, Sigma, TOFavg, sigmaTOF
# Read the channel counts
def getChannelCount(channel):
    PSOCaddress = addrEvnt;
    cmdHeader = mkCmdHdr(1, 0x37, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, PSOCaddress, 1)
    ser.write(data1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("getChannelCount: invalid header returned: " + str(ret))
    count = bytes2int(ser.read(3))
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("getChannelCount: invalid trailer returned: " + str(ret))  
    return count
# Read the channel counts from end of run
def getEndOfRunChannelCount(channel):
    PSOCaddress = addrEvnt;
    cmdHeader = mkCmdHdr(1, 0x33, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(channel, PSOCaddress, 1)
    ser.write(data1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("getEndOfRunChannelCount: invalid header returned: " + str(ret))
    count = bytes2int(ser.read(3))
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("getEndOfRunChannelCount: invalid trailer returned: " + str(ret))  
    return count

# Set up the trigger masks
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
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("getTriggerMask: invalid header returned: " + str(ret))
    mask = bytes2int(ser.read(1))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("getTriggerMask: invalid trailer returned: " + str(ret))  
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
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readPmtDAC: invalid header returned: " + str(ret))
    if (channel == 5):
        value = bytes2int(ser.read(2))
    else:
        value = bytes2int(ser.read(1))
        ser.read(1)
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readPmtDAC: invalid trailer returned: " + str(ret))
    return value

def triggerEnableStatus():
    cmdHeader = mkCmdHdr(0, 0x3D, addrEvnt)
    ser.write(cmdHeader)
    ret = ser.read(3);
    if ret != b'\xDB\x00\xFF':
        print("triggerEnableStatus: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(1))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readPmtDAC: invalid trailer returned: " + str(ret))
    return value        

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
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readTofDAC: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    #print("value="+str(value))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTofDAC: invalid trailer returned: " + str(ret))
    return value

# Read the bus voltage from an INA226 chip
def readBusVoltage(i2cAddress, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x20, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(i2cAddress, PSOCaddress, 1)
    ser.write(data1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readBusVoltage: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    #print("value="+str(value))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readBusVoltage: invalid trailer returned: " + str(ret))
    return (value * 1.25)/1000.

# Read the shunt voltage from an INA226 chip and calculate the current
def readCurrent(i2cAddress, address):
    cmdHeader = mkCmdHdr(1, 0x21, address)
    ser.write(cmdHeader)
    data1 = mkDataByte(i2cAddress, address, 1)
    ser.write(data1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readCurrent: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    #print("value="+str(value))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readCurrent: invalid trailer returned: " + str(ret))
    return (value * 0.03)
    
# Read the shunt voltage from an INA226 chip and calculate the current
def readTemperature(address):
    cmdHeader = mkCmdHdr(0, 0x22, address)
    ser.write(cmdHeader)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readCurrent: invalid header returned: " + str(ret))
    byte1 = ser.read(1)
    byte2 = ser.read(1)
    #print("readTemperature: byte1 = " + str(binascii.hexlify(byte1)) + " byte2 = " + str(binascii.hexlify(byte2)))
    value = bytes2int(byte1 + byte2) >> 4
    #print("value="+str(value))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTemperature: invalid trailer returned: " + str(ret))
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
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readBarometerReg: invalid header returned: " + str(ret))
    byte1 = ser.read(1)
    #print("readBarometerReg: byte = " + str(binascii.hexlify(byte1)))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readBarometerReg: invalid trailer returned: " + str(ret))
    return bytes2int(byte1)
    
# Read a byte from any register of the Real-Time Clock
def readRTCregister(regAddress, PSOCaddress):
    cmdHeader = mkCmdHdr(1, 0x23, PSOCaddress)
    ser.write(cmdHeader)
    data1 = mkDataByte(regAddress, PSOCaddress, 1)
    ser.write(data1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readRTCregister: invalid header returned: " + str(ret))
    byte1 = ser.read(1)
    #print("readRTCregister: byte = " + str(binascii.hexlify(byte1)))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readRTCregister: invalid trailer returned: " + str(ret))
    return byte1

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

# Set the Real-Time Clock according to the current time and date
def setRTCtime(address):
    now = time.localtime()
    print(now)
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

# Unpack 2 decimal digits from a byte
def dec2int(byte):
    ones = byte & 0x0F
    tens = (byte & 0xF0) >> 4
    return tens*10 + ones

# Read and print out the current time date, from the onboard Real-Time-Clock
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
    print("The current RTC time is " + str(hour) + ":" + str(minute) + ":" + str(second) + ", " + days[weekday] + " " + months[month] + " " + str(date) + ", " + str(year))

# Read back and print out all of the error codes stored up in the main PSOC
def readErrors(address):
    cmdHeader = mkCmdHdr(0, 0x03, address)
    ser.write(cmdHeader)
    ret = ser.read(3)
    if ret == b'\xDB\x00\xFF':
        byte1 = ser.read(1)
        if bytes2int(byte1) == 0x00:
            print("No errors have been logged from PSOC address " + str(address))
            byte2 = ser.read(1)
            print("    Byte 2 returned to readErrors: " + str(binascii.hexlify(byte2)))
            byte3 = ser.read(1)
            print("    Byte 3 returned to readErrors: " + str(binascii.hexlify(byte3)))
        else:
            print("For PSOC address " + str(address) + ", Error code returned to readErrors: " + str(bytes2int(byte1)))
            byte2 = ser.read(1)
            print("    First information byte = " + str(binascii.hexlify(byte2)))
            byte3 = ser.read(1)
            print("    Second information byte = " + str(binascii.hexlify(byte3)))
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readErrors: invalid trailer returned: " + str(ret))
    elif ret == b'\xDC\x00\xFF':
        for i in range(3):
            ret = ser.read(1)
            print("Byte "+str(i)+" returned to readErrors: " + str(binascii.hexlify(ret)))
            if i == 0: nData = bytes2int(ret)
        print("readErrors for PSOC address " + str(address) + ": number of data bytes = " + str(nData))           
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readErrors: invalid trailer returned: " + str(ret))
        nPackets = int((nData-1)/3) + 1;
        for packet in range(nPackets):
            ret = ser.read(3)
            if ret != b'\xDC\x00\xFF':
                print("readErrors: invalid header returned: " + str(ret))
            ret = ser.read(1)
            print("Error code returned to readErrors is " + str(bytes2int(ret)))  
            ret = ser.read(1)
            print("    First information byte = " + str(binascii.hexlify(ret)))
            ret = ser.read(1)
            print("    Second information byte = " + str(binascii.hexlify(ret)))
            ret = ser.read(3)
            if ret != b'\xFF\x00\xFF':
                print("readErrors: invalid trailer returned: " + str(ret))
    else:
        print("readErrors: invalid header returned: " + str(ret))


# Read back the voltage of the 5V supply on the backplane, digitized by the main PSOC Sigma-Delta ADC
def readBackplaneVoltage():
    address = addrMain
    cmdHeader = mkCmdHdr(0, 0x25, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readBackplaneVoltage: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readBackplaneVoltage: invalid trailer returned: " + str(ret))
    return value/1000.

# Read back the voltage of the watch battery (used by the RTC), digitized by the event PSOC Sigma-Delta ADC
def readBatteryVoltage():
    address = addrEvnt
    cmdHeader = mkCmdHdr(0, 0x25, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readBatteryVoltage: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readBatteryVoltage: invalid trailer returned: " + str(ret))
    return value/1000.

def readNumTOF(address):
    cmdHeader = mkCmdHdr(0, 0x34, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readNumTOF: invalid header returned: " + str(ret))
    valueA = bytes2int(ser.read(1))
    print("readNumTOF: channel-A TOF pointer = " + str(valueA))
    valueB = bytes2int(ser.read(1))
    print("readNumTOF: channel-B TOF pointer = " + str(valueB))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readNumTOF: invalid trailer returned: " + str(ret))  
    return [valueA, valueB]

def readSAR_ADC(address):
    cmdHeader = mkCmdHdr(0, 0x33, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret != b'\xDB\x00\xFF':
        print("readSAR_ADC: invalid header returned: " + str(ret))
    value = bytes2int(ser.read(2))
    ser.read(1)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readSAR_ADC: invalid trailer returned: " + str(ret))
    return value*3.3/4096.

def readAllTOFdata(address):
    cmdHeader = mkCmdHdr(0, 0x40, address)
    ser.write(cmdHeader)
    time.sleep(0.1)
    ret = ser.read(3)
    if ret == b'\xDB\x00\xFF':
        nA = bytes2int(ser.read(1))
        print("readAllTOFdata: number of A-channel hits = " + str(nA))
        nB = bytes2int(ser.read(1))     
        print("readAllTOFdata: number of B-channel hits = " + str(nB))
        b3 = bytes2int(ser.read(1))
        print("readAllTOFdata: 3rd byte = " + str(b3))
        return
    if ret != b'\xDC\x00\xFF':
        print("readAllTOFdata: invalid header returned: " + str(ret))
    nData = bytes2int(ser.read(1))
    print("readAllTOFdata: number of bytes in event = " + str(nData))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readAllTOFdata: invalid trailer returned: " + str(ret))
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("readAllTOFdata: invalid header returned: " + str(ret))
    nA = bytes2int(ser.read(1))
    print("readAllTOFdata: number of A-channel hits = " + str(nA))
    nB = bytes2int(ser.read(1))     
    print("readAllTOFdata: number of B-channel hits = " + str(nB))
    b3 = bytes2int(ser.read(1))
    print("readAllTOFdata: 3rd byte = " + str(b3))
    if (b3 == 2):
        print("    readAllTOFdata: the TOF data were truncated")
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readAllTOFdata: invalid trailer returned: " + str(ret))
    for i in range(nA):
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("    readAllTOFdata: invalid header returned: " + str(ret))
        refIdx = bytes2int(ser.read(2))
        stp0 = bytes2int(ser.read(1))       
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readAllTOFdata: invalid trailer returned: " + str(ret))
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("    readAllTOFdata: invalid header returned: " + str(ret))
        stop = stp0*256 + bytes2int(ser.read(1))
        clkCnt = bytes2int(ser.read(2))             
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readAllTOFdata: invalid trailer returned: " + str(ret))
        timeA = refIdx*8333 + stop;
        print("    Channel A hit " + str(i) + ": ref=" + str(refIdx) + ", stop=" + str(stop) + ", time=" + str(timeA) + ",  clock count = " + str(clkCnt))
    for i in range(nB):
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("    readAllTOFdata: invalid header returned: " + str(ret))
        refIdx = bytes2int(ser.read(2))
        stp0 = bytes2int(ser.read(1))       
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readAllTOFdata: invalid trailer returned: " + str(ret))
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("    readAllTOFdata: invalid header returned: " + str(ret))
        stop = stp0*256 + bytes2int(ser.read(1))
        clkCnt = bytes2int(ser.read(2))             
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readAllTOFdata: invalid trailer returned: " + str(ret))
        timeB = refIdx*8333 + stop;
        print("    Channel B hit " + str(i) + ": ref=" + str(refIdx) + ", stop=" + str(stop) + ": time=" + str(timeB) + ",  clock count = " + str(clkCnt))        
    print("     ")
            
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
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTOFevent: invalid trailer returned: " + str(ret))    
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("readTOFevent: invalid header returned: " + str(ret))
    ref = bytes2int(ser.read(2))
    print("    readTOFevent: reference index = " + str(ref))
    ret = ser.read(1)       
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTOFevent: invalid trailer returned: " + str(ret))
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("readTOFevent: invalid header returned: " + str(ret))
    tim = bytes2int(ser.read(2))
    print("    readTOFevent: stop time = " + str(tim))
    ret = ser.read(1)       
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTOFevent: invalid trailer returned: " + str(ret))
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("readTOFevent: invalid header returned: " + str(ret))
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
    time.sleep(0.1)
    ret = ser.read(3)
    #print("readTofConfig: ret = " + str(ret))
    if ret != b'\xDC\x00\xFF':
        print("readTofConfig: invalid header returned: " + str(ret))
    ret = ser.read(1)
    #print("readTofConfig: ret = " + str(ret))
    print("readTofConfig: number of data bytes = " + str(bytes2int(ret)))
    ret = ser.read(2)
    #print("readTofConfig: ret = " + str(ret))
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("readTofConfig: invalid trailer returned: " + str(ret)) 
    for packet in range(6):
        ret = ser.read(3)
        #print("readTofConfig: ret = " + str(ret))
        if ret != b'\xDC\x00\xFF':
            print("readTofConfig: invalid header returned: " + str(ret))
        for i in range(3):
            ret = ser.read(1)
            print("Byte "+str(i)+" returned by SPI from the TOF chip config reg: " + str(binascii.hexlify(ret)))  
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("readTofConfig: invalid trailer returned: " + str(ret)) 

# Receive and check the echo from a tracker command
def getTkrEcho():
    ret = ser.read(3)
    #print("getTkrEcho: returned header = ", str(ret))
    if ret != b'\xDB\x00\xFF':
        print("getTkrEcho: invalid header returned: " + str(ret))
    cmdCount = bytes2int(ser.read(2))
    #print("getTkrEcho: command count = " + str(cmdCount))
    cmdCode = ser.read(1)
    #print("getTkrEcho: command code = " + str(binascii.hexlify(cmdCode)))
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("getTkrEcho: invalid trailer returned: " + str(ret))
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

# Set the tracker trigger end status
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
    ret = ser.read(3)
    if ret != b'\xDC\x00\xFF':
        print("getTkrHousekeeping: invalid header returned: " + str(ret))    
        if ret == b'\xdb\x00\xff':
            ret = ser.read(1)
            print("   next byte = " + str(ret))
            ret = ser.read(1)
            print("   next byte = " + str(ret))
            ret = ser.read(1)
            print("   next byte = " + str(ret))
            ret = ser.read(3)
            if ret != b'\xFF\x00\xFF':
                print("getTkrHousekeeping: invalid trailer returned: " + str(ret))
        readErrors(0)
        return [0,1,2,3,4,5,6,7,8,9]            
    nData = bytes2int(ser.read(1))
    #print("getTkrHousekeeping: nData= " + str(nData))
    ser.read(2)
    ret = ser.read(3)
    if ret != b'\xFF\x00\xFF':
        print("getTkrHousekeeping: invalid trailer returned: " + str(ret))  
    nPackets = int((nData - 1)/3) + 1
    returnStuff = []
    for packet in range(nPackets):
        #print("getTkrHousekeeping: read packet " + str(packet))
        ret = ser.read(3)
        if ret != b'\xDC\x00\xFF':
            print("getTkrHousekeeping, packet " + str(packet) + ": invalid header returned: " + str(ret))
        returnStuff.append(ser.read(1))
        returnStuff.append(ser.read(1))
        returnStuff.append(ser.read(1))
        ret = ser.read(3)
        if ret != b'\xFF\x00\xFF':
            print("getTkrHousekeeping, packet " + str(packet) + ": invalid trailer returned: " + str(ret))
    #for junk in returnStuff:
    #    print("getTkrHousekeeping: byte = " + str(binascii.hexlify(junk)))
    return returnStuff

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
    return bytes2int(getTkrHousekeeping()[6])

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
        print("tkrSetCalMask: calibrate channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
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
        print("tkrSetDataMask: bad number of clusters of bad channels " + str(nItems) + ", must be from 1 to 5")
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
        print("tkrSetDataMask: " + sense + " channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
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
        print("tkrSetDataMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be " + str(cmdCode))

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
    #for byte in byteList:
    #    print("tkrGetDAC for FPGA " + str(FPGA) + " chip " + str(chip) + " byte = " + str(binascii.hexlify(byte)))
    if (bytes2int(byteList[0]) != 9):
        print("tkrGetDAC: wrong number " + str(bytes2int(byteList[0])) + " of bytes returned")
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
    if nItems==0 or nItems>5:
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
        print("tkrSetTriggerMask: " + sense + " channels in range " + str(item[1]) + " through " + str(item[1]+item[0]-1) + " of chip " + str(chip) + " on FPGA " + str(FPGA))
        data1 = mkDataByte(item[0], address, byteNum)
        ser.write(data1)
        data2 = mkDataByte(item[1], address, byteNum+1)
        ser.write(data2)
        byteNum = byteNum + 2       
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'14'"):
        print("tkrSetTriggerMask: incorrect Tracker echo received (" + str(binascii.hexlify(echo)) + "), should be b'14'")

def tkrGetCalMask(FPGA, chip):
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
    print("Calibration mask for chip " + str(chip) + " of board " + str(FPGA) + ": " + stuff[5:69])

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
    return getTkrHousekeeping()[6]    
    
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
        print("tkrConfigReset: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'09'")   

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
    houseStuff = getTkrHousekeeping()[6]
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
    return getTkrHousekeeping()[6]
    
def tkrGetASICconfig(FPGA, chipAddress):
    print("tkrGetASICconfig: the configuration for ASIC " + str(chipAddress) + 
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
    if bytes2int(bytes[0]) != 9:
        print("tkrGetASICconfig: wrong number bytes returned.")
        return      
    if bytes[9] != b'\xfe':
        print("tkrGetASICconfig, trailing byte " + str(bytes[8]) + " is not the expected 0xFE")
    word1 = bytes[1]+bytes[2]+bytes[3]+bytes[4]
    #print("tkrGetASICconfig: binary data returned = " + str(binascii.hexlify(word1)))
    intWord = bytes2int(word1)
    regType = (intWord & 0x70000000) >> 28
    if regType != 3:
        print("tkrGetASICconfig: register type returned = " + str(regType) + " should be 3!")
    errCodes = (intWord & 0x07000000) >> 24
    #print("tkrGetASICconfig: ASIC error codes = " + str(errCodes))
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
    nib2 = 0x01 | (oneShot << 2) | (gain << 3) 
    nib3 = shaping | (bufSpeed << 1)
    nib4 = trigDelay & 0x0F
    nib5 = (trigDelay & 0x10)>>4 | trigWindow<<1 | ioCurrent<<2
    nib6 = maxClust
    all = nib6<<20 | nib5<<16 | nib4<<12 | nib3<<8 | nib2<<4 | nib1
    print("tkrLoadASICconfig: for FPGA " + str(FPGA) + ", chip " + str(address) + 
             ", the register setting will be " + str(hex(all)))
    PSOCaddress = addrEvnt
    cmdHeader = mkCmdHdr(7, 0x10, PSOCaddress)
    #print("tkrLoadASICconfig: cmdHeader = " + str(cmdHeader))
    ser.write(cmdHeader)
    data1 = mkDataByte(FPGA, PSOCaddress, 1)
    #print("tkrLoadASICconfig: data1 = " +str(data1))
    ser.write(data1)
    data2 = mkDataByte(0x12, PSOCaddress, 2)
    #print("tkrLoadASICconfig: data2 = " +str(data2))
    ser.write(data2)
    data3 = mkDataByte(0x04, PSOCaddress, 3)
    #print("tkrLoadASICconfig: data3 = " +str(data3))
    ser.write(data3)
    data4 = mkDataByte(address, PSOCaddress, 4)
    #print("tkrLoadASICconfig: data4 = " +str(data4))
    ser.write(data4)    
    byte5 = (nib6<<4) | nib5
    data5 = mkDataByte(byte5, PSOCaddress, 5)
    #print("tkrLoadASICconfig: byte5 = " +str(hex(byte5)))
    ser.write(data5)
    byte6 = (nib4<<4) | nib3
    data6 = mkDataByte(byte6, PSOCaddress, 6)
    #print("tkrLoadASICconfig: byte6 = " +str(hex(byte6)))
    ser.write(data6)
    byte7 = (nib2<<4) | nib1
    data7 = mkDataByte(byte7, PSOCaddress, 7)
    #print("tkrLoadASICconfig: byte7 = " +str(hex(byte7)))
    ser.write(data7)
    time.sleep(0.1)
    echo = getTkrEcho()
    if (str(binascii.hexlify(echo)) != "b'12'"):
        print("tkrLoadASICconfig: incorrect echo received (" + str(binascii.hexlify(echo)) + "), should be b'0f'")  
        
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
