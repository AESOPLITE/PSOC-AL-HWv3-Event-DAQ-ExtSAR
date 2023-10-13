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

print("Entering testTKRcom.py")

address = 8   # Address of the event PSOC

print("Set up the Event PSOC to send its output over the UART")
setOutputMode("UART")

#LED2("on", address)
#time.sleep(1)
#LED2("off", address)

getEvtVersionNumber()

setInternalRTC(address)
#time.sleep(1)
getInternalRTC(address)

setTofDAC(1, 32, address) 
readErrors(address)  
setTofDAC(2, 32, address)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, address)) + " counts.")

pmtThr = 8
ch5Thresh = 40
pmtThresh = [pmtThr,pmtThr,pmtThr,pmtThr,ch5Thresh]
for chan in range(1,5):
    setPmtDAC(chan, pmtThresh[chan-1], addrEvnt)
    time.sleep(0.1)
    print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts")

print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
setPmtDAC(5, ch5Thresh, address)
print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
#sys.exit("abort")
readTofConfig()

#sys.exit("abort")

#ret = ser.read()
#print(ret)

#startPmtRateMonitor(4, 10)
#time.sleep(5)
#getPmtRates()
#time.sleep(1)

readErrors(address)
tkrSetCRCcheck("yes")

setTKRlayers(0, 3, 4, 5, 6, 8, 7, 2)
getTKRlayers()

boards = [0]
nBoards = len(boards)
if nBoards > 0:
    tkrFPGAreset(0x00)
    
    readErrors(address)
    #sys.exit("abort")

    tkrConfigReset(0x00)

    tkrAsicPowerOn()
    #time.sleep(1)

    if asicReset: tkrAsicHardReset(0x1F)

    #tkrSetNumLyrs(1)
    print("The number of tracker readout layers is set to " + str(bytes2int(tkrGetNumLyrs(0))))
    bumpTKRthreshold(6,0,0,0,0,0,0,0)
    configureTkrASICs(nBoards)
    readErrors(address) 
    for brd in boards:
        print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(brd))) + " for board " + str(brd))
        readErrors(address) 
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(brd))
 
    #calibrateAllFPGAinputTiming()
 
    tkrTrigEndStat(0, 1)
    tkrSetDualTrig(0, 0)

    tkrSetTriggerSource(0)    # We want the external trigger
    trgsrc = bytes2int(tkrGetTriggerSource(0))
    print("The tracker trigger source is set to " + str(trgsrc))

    readErrors(address)

    oneShot = 0
    gain = 0
    shaping = 0
    bufSpeed = 3
    trigDelay = 4
    trigWindow = 1
    ioCurrent = 3
    maxClust = 10
    chips = [0,1,2,3,4,5,6,7,8,9,10,11]
    for brd in boards:
        tkrLoadASICconfig(brd, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
        for chip in chips: tkrGetASICconfig(brd, chip)
    readErrors(address)

    for brd in boards:
        tkrGetTemperature(brd)
        tkrGetBusVoltage(brd, "flash18")
        tkrGetBusVoltage(brd, "fpga12")
        tkrGetBusVoltage(brd, "digi25")
        tkrGetBusVoltage(brd, "i2c33")
        tkrGetBusVoltage(brd, "analog21")
        tkrGetBusVoltage(brd, "analog33")
        tkrGetShuntCurrent(brd, "flash18")
        tkrGetShuntCurrent(brd, "fpga12")
        tkrGetShuntCurrent(brd, "digi25")
        tkrGetShuntCurrent(brd, "i2c33")
        tkrGetShuntCurrent(brd, "analog21")
        tkrGetShuntCurrent(brd, "analog33")
        tkrGetShuntCurrent(brd, "bias100")

    # Test setting of the calibration mask
    hitList = []
    #newChan = random.randint(0,5)
    #newSize = random.randint(1,6)
    hit = [2, 7]
    hitList.append(hit)
    #newChan = random.randint(12,16)
    #newSize = random.randint(1,6)
    hit = [3,15]
    hitList.append(hit)
    #newChan = random.randint(23,28)
    #newSize = random.randint(1,6)
    hit = [1,23]
    hitList.append(hit)
    #newChan = random.randint(35,44)
    #newSize = random.randint(1,6)
    hit = [4,50]
    hitList.append(hit)
    #newChan = random.randint(51,57)
    #newSize = random.randint(1,6)
    hit = [2,61]
    hitList.append(hit)

    for brd in boards:
        print("Check for bit errors by reading back the calibration mask, for board " + str(brd))
        for iter in range(0):
            maskGen = tkrRandomCalMask(brd, 0x1F)
            print("  Iteration " + str(iter) + ": generated mask = " + str(hex(maskGen))
            mask1 = tkrGetCalMask(brd, 3, False)
            if iter%10 == 0: print("  Iteration " + str(iter) + " mask = " + mask1)
            for chip in chips: 
                mask = tkrGetCalMask(brd, chip, False)
                if mask != mask1: 
                    print("Mask from chip " + str(chip) + " is incorrect: " + mask)
                    print("      it should be " + mask1)
                else: print("Success in chip " + str(chip) + " of iteration " + str(iter))               

time.sleep(0.1)
if asicReset and nBoards>0: tkrAsicPowerOff()

readErrors(address)

#time.sleep(.1)

#num = readNumTOF(address)[0]

#for i in range(num):
#    readTOFevent(address, 0)    
#   readTOFevent(address, 0)     

closeCOM()


