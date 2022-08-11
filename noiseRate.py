import serial
import time
import numpy
import binascii
import math
import sys
import random

from PSOC_cmd import *

asicReset = True

portName = "COM11"
openCOM(portName)

print("Entering noiseRate.py")

address = 8   # Address of the event PSOC

print("Set up the Event PSOC to send its output over the UART")
setOutputMode("UART")

getEvtVersionNumber()

setInternalRTC(address)
#time.sleep(1)
getInternalRTC(address)

#
#GDEF
setTKRlayers(8, 3, 4, 1, 6, 2, 7, 5)
getTKRlayers()

boards = [0]
nBoards = len(boards)
if nBoards > 0:
    tkrFPGAreset(0x00)
    
    readErrors(address)
    #sys.exit("abort")

    for board in boards: tkrConfigReset(board)

    tkrAsicPowerOn()
    #time.sleep(1)

    if asicReset: tkrAsicHardReset(0x1F)

    #tkrSetNumLyrs(1)
    print("The number of tracker readout layers is set to " + str(bytes2int(tkrGetNumLyrs(0))))
    #bumpTKRthreshold(6,6,6,6,6,6,6,6)
    configureTkrASICs(nBoards)
    readErrors(address) 
    for brd in boards:
        print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(brd))) + " for board " + str(brd))
        readErrors(address) 
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(brd))
 
    tkrTrigEndStat(0, 1)
    tkrSetTrgMask(0,1)

    tkrSetTriggerSource(0)    # We want the external trigger
    trgsrc = bytes2int(tkrGetTriggerSource(0))
    print("The tracker trigger source is set to " + str(trgsrc))

    oneShot = 0
    gain = 0
    shaping = 0
    bufSpeed = 3
    trigDelay = 4
    trigWindow = 1
    ioCurrent = 3
    maxClust = 10
    chips = [3]
    for brd in boards:
        #tkrLoadASICconfig(brd, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
        for chip in chips: tkrGetASICconfig(brd, chip)
    readErrors(address)

    for brd in boards:
        tkrGetTemperature(brd)
        tkrGetShuntCurrent(brd, "bias100")

    # Test setting of the calibration mask
    nullList = []
    #newChan = random.randint(0,5)
    #newSize = random.randint(1,6)

    f = open("noiseRates.txt", "w")
    for brd in boards:
        tkrSetTriggerMask(brd, 31, "mask", nullList)
        tkrGetTriggerMask(brd, 3)
        for chip in range(12):
            for chan in range(64):
                hitList = []
                hit = [1, chan]
                hitList.append(hit)
                tkrSetTriggerMask(brd, chip, "mask", hitList)
                #tkrGetTriggerMask(brd, chip)
                rate = getLyrTrgCnt(brd)
                print("board " + str(brd) + " chip " + str(chip) + " channel " + str(chan) + ": rate = " + str(rate))
                strOut = 'Board {} chip {} channel {}: rate={}\n'.format(brd,chip,chan,rate)
                f.write(strOut) 
            tkrSetTriggerMask(brd,chip,"mask",nullList)				
                
f.close()               
                
if asicReset and nBoards>0: tkrAsicPowerOff()

readErrors(address) 

closeCOM()


