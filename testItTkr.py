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

print("Entering testItEvt.py")

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
#
#GDEF
setTKRlayers(5, 3, 4, 1, 6, 7, 2, 8)
getTKRlayers()

boards = [0,1,2,3,4,5,6]
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
    bumpTKRthreshold(6,6,6,6,6,6,6,6)
    configureTkrASICs(nBoards)
    readErrors(address) 
    for brd in boards:
        print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(brd))) + " for board " + str(brd))
        readErrors(address) 
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(brd))
 
    tkrTrigEndStat(3, 1)
    tkrTrigEndStat(6, 1)
    tkrSetTrgMask(0,1)
    tkrSetTrgMask(1,1)
    tkrSetTrgMask(2,0)
    tkrSetTrgMask(3,0)
    tkrSetTrgMask(4,0)
    tkrSetTrgMask(5,1)
    tkrSetTrgMask(6,1)

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
    chips = [6]
    for brd in boards:
        #tkrLoadASICconfig(brd, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
        for chip in chips: tkrGetASICconfig(brd, chip)
    readErrors(address)

    for brd in boards:
        #tkrGetTemperature(brd)
        #tkrGetBusVoltage(brd, "flash18")
        #tkrGetBusVoltage(brd, "fpga12")
        #tkrGetBusVoltage(brd, "digi25")
        #tkrGetBusVoltage(brd, "i2c33")
        #tkrGetBusVoltage(brd, "analog21")
        #tkrGetBusVoltage(brd, "analog33")
        #tkrGetShuntCurrent(brd, "flash18")
        #tkrGetShuntCurrent(brd, "fpga12")
        #tkrGetShuntCurrent(brd, "digi25")
        #tkrGetShuntCurrent(brd, "i2c33")
        #tkrGetShuntCurrent(brd, "analog21")
        #tkrGetShuntCurrent(brd, "analog33")
        tkrGetShuntCurrent(brd, "bias100")

    rates = getLyrTrgCnt(boards)
    for brd, rate in zip(boards, rates):
        print("Trigger rate in board " + str(brd) + " = " + str(rate))


    # Test setting of the calibration mask
    hitList = []
    #newChan = random.randint(0,5)
    #newSize = random.randint(1,6)
    hit = [1, 53]
    hitList.append(hit)

    for brd in boards:
        #print("Check for bit errors by reading back the calibration mask")
        #for iter in range(0):
        #    tkrRandomCalMask(brd, 0x1F)
        #    mask1 = tkrGetCalMask(brd, 3, False)
        #    if iter%10 == 0: print("  Iteration " + str(iter) + " mask = " + mask1)
        #    for chip in chips: 
        #        mask = tkrGetCalMask(brd, chip, False)
        #        if mask != mask1: 
        #            print("Mask from chip " + str(chip) + " is incorrect: " + mask)
        #            print("      it should be " + mask1)
        #        #else: print("Success in chip " + str(chip) + " of iteration " + str(iter)) 
                
        #sys.exit("abort")
        
        #tkrSetDataMask(brd, 31, "mask", hitList)
        #time.sleep(0.1)
        #for chip in chips: tkrGetDataMask(brd, chip)

        #tkrSetTriggerMask(brd, 31, "unmask", hitList)
        #time.sleep(0.1)
        for chip in chips: tkrGetTriggerMask(brd, chip)

        tkrSetDAC(brd, 31, "calibration", 40 , "high")
        for chip in chips: tkrGetDAC(brd, chip, "calibration")

        #tkrSetDAC(brd, 31, "threshold", 25 , "low")
        for chip in chips: tkrGetDAC(brd, chip, "threshold")

        #tkrSetDataMask(brd, 31, "unmask", [])
        #time.sleep(0.1)
        for chip in chips: tkrGetDataMask(brd, chip)

        tkrSetCalMask(brd, 31, hitList)
        time.sleep(0.1)
        for chip in chips: mask1 = tkrGetCalMask(brd, chip, True)
        
        print("Board " + str(brd) + " layer trigger-OR count = " + str(rate) + " Hz")
        delay = 0
        length = 3
        #setTkrTrigOutputTiming(brd, delay, length)
        getTkrTrigOutputTiming(brd)

#for i in range(1):
#    print("send reset pulse")
#    logicReset(addrEvnt)
#    time.sleep(0.3)

#sys.exit("abort")

TOFselectDAQ(address,"DMA")

TOFs = []
for iter in range(0):
    TOFenable(address, 1)
    time.sleep(1)
    TOFenable(address, 0)
    TOFs.append(readAllTOFdata(address))
N=0
Sum=0.
Sum2=0.
for TOF1 in TOFs:
    for TOF in TOF1:
        N=N+1
        print(str(N) + "  TOF= " + str(TOF) + " ns")
        Sum=Sum + TOF
        Sum2=Sum2 + TOF*TOF
if N>1:
    tAvg = Sum/N
    t2Avg = Sum2/N
    Var = t2Avg - tAvg*tAvg
    print("Mean TOF = " + str(tAvg) + " ns    Std. Dev. = " + str(math.sqrt(Var)) + " ns")

for brd in boards:
    #tkrSetDAC(brd, 31, "threshold", 64 , "low")
    tkrGetDAC(brd, 0, "threshold")
    for chip in chips: tkrGetDAC(brd, chip, "threshold")

#getLyrTrgCnt(0)

NOOP()

mask = 0x01    # T1&T2&T3
print("Setting the first trigger mask to " + str(mask))
setTriggerMask(1, mask)
mask = 0x05    # T1 & T3 prescaled
print("Setting the second trigger mask to " + str(mask))
setTriggerMask(2, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

prescale = 2
print("Setting the PMT trigger prescale to " + str(prescale))
setTriggerPrescale("PMT", prescale)
print("The PMT trigger prescale is set to " + str(getTriggerPrescale("PMT")))
print("Setting the TKR trigger prescale to " + str(prescale))
setTriggerPrescale("Tracker",prescale)
print("The Tracker trigger prescale is set to " + str(getTriggerPrescale("Tracker")))
#sys.exit("abort")
setSettlingWindowAll(16)
setPeakDetResetWait(28)

tkrSetPMTtrgDly(12)

print("Count on channel 2 = " + str(getChannelCount(2)))
print("Before run, trigger enable status is " + str(triggerEnableStatus()))

setTkrRatesMuliplier(2)
startHouseKeeping(4, 1)
#startTkrHouseKeeping(1)

setTkrLogic("AND")
getTkrLogic()

#readErrors(address)

#response = input("Enter something")
ADC, Sigma, TOF, sigmaTOF = limitedRun(89, 100, True, True, True)
readErrors(address)

getRunCounters()
#response = input("Enter something")
getAvgReadoutTime()
#response = input("Enter something")
print("Average ADC values:")
print("    T1 = " + str(ADC[0]) + " +- " + str(Sigma[0]))
print("    T2 = " + str(ADC[1]) + " +- " + str(Sigma[1]))
print("    T3 = " + str(ADC[2]) + " +- " + str(Sigma[2]))
print("    T4 = " + str(ADC[3]) + " +- " + str(Sigma[3]))
print("     G = " + str(ADC[4]) + " +- " + str(Sigma[4]))
print("    TOF = " + str(TOF) + " +- " + str(sigmaTOF))
chName = ["G","T3","T1","T4","T2"]
for ch in range(5):
    cnt = getEndOfRunChannelCount(ch+1)
    print("Counter for channel " + chName[ch] + " = " + str(cnt))

readErrors(address)
#getPmtRates()
#getTkrLyrRates()

#getLyrTrgCnt(0)   # This goes directly to the tracker to get the rate

#for brd in boards:
#    for chip in chips: tkrGetASICconfig(brd, chip)

readErrors(address)
time.sleep(0.1)
if asicReset and nBoards>0: tkrAsicPowerOff()

readErrors(address)

#time.sleep(.1)

#num = readNumTOF(address)[0]

#for i in range(num):
#    readTOFevent(address, 0)    
#   readTOFevent(address, 0)     

closeCOM()


