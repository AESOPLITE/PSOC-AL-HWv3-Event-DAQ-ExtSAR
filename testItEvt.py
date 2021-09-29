import serial
import time
import numpy
import binascii
import math
import sys

from PSOC_cmd import *

asicReset = True

portName = "COM7"
openCOM(portName)

print("Entering testItEvt.py")

address = 8   # Address of the event PSOC

#LED2("on", address)
#time.sleep(1)
#LED2("off", address)

#setInternalRTC(address)
#time.sleep(0.1)
#getInternalRTC(address)

setTofDAC(1, 64, address)   
setTofDAC(2, 64, address)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, address)) + " counts.")

pmtThresh = [4,4,4,4,20]
ch5Thresh = pmtThresh[4]
print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
setPmtDAC(5, ch5Thresh, address)

print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
time.sleep(0.2)
print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
for chan in range(1,5):
    setPmtDAC(chan, pmtThresh[chan-1], addrEvnt)
    time.sleep(0.1)
    print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts")

print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")
setPmtDAC(5, ch5Thresh, address)
print("Channel 5 PMT DAC was set to " + str(readPmtDAC(5, address)) + " counts.")

readTofConfig()
#sys.exit("abort")

#ret = ser.read()
#print(ret)

boards = [0]
nBoards = len(boards)
if nBoards > 0:
    tkrFPGAreset(0x00)
    
    readErrors(address)
    #sys.exit("abort")

    tkrConfigReset(0x00)

    tkrSetNumLyrs(nBoards)

    for brd in boards:
        print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(brd))) + " for board " + str(brd))
        print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(brd))

    if asicReset: tkrAsicPowerOn()

    if asicReset: tkrAsicHardReset(0x1F)

    if asicReset: tkrAsicSoftReset(0x1F)

    calibrateAllFPGAinputTiming()
		
    tkrTrigEndStat(1, 1)
    tkrTrigEndStat(0, 1)
    tkrSetDualTrig(0, 0)

    tkrSetTriggerSource(0)    # We want the external trigger
    trgsrc = bytes2int(tkrGetTriggerSource(0))
    print("The tracker trigger source is set to " + str(trgsrc))

    oneShot = 0
    gain = 0
    shaping = 1
    bufSpeed = 3
    trigDelay = 1
    trigWindow = 1
    ioCurrent = 2
    maxClust = 10
    for brd in boards:
        tkrGetASICconfig(brd,3)
        tkrLoadASICconfig(brd, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
        tkrGetASICconfig(brd, 3)

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

    readErrors(address)

    # Test setting of the calibration mask
    hitList = []
    hit = [2, 5]
    hitList.append(hit)
    hit = [1, 18]
    hitList.append(hit)
    hit = [1, 13]
    hitList.append(hit)
    hit = [2, 61]
    hitList.append(hit)
    for brd in boards:
        tkrSetCalMask(brd, 31, hitList)
        time.sleep(0.1)
        tkrGetCalMask(brd, 3)
        tkrGetCalMask(brd, 2)

        tkrSetDataMask(brd, 31, "mask", hitList)
        time.sleep(0.1)
        tkrGetDataMask(brd, 3)
        tkrGetDataMask(brd, 7)

        tkrSetTriggerMask(brd, 31, "mask", hitList)
        time.sleep(0.1)
        tkrGetTriggerMask(brd, 3)
        tkrGetTriggerMask(brd, 9)

        tkrSetDAC(brd, 31, "calibration", 20 , "high")
        tkrGetDAC(brd, 3, "calibration")

        tkrSetDAC(brd, 31, "threshold", 21 , "low")
        tkrGetDAC(brd, 3, "threshold")

        tkrSetDataMask(brd, 31, "unmask", [])
        time.sleep(0.1)
        tkrGetDataMask(brd, 3)

        #tkrSetTriggerMask(brd, 31, "unmask", [])
        #time.sleep(0.1)
        #tkrGetTriggerMask(brd, 3)

    if len(boards) == 1:
        triggerDelay = 6    # 6?
        triggerTag = 0
        sendTkrCalStrobe(0, triggerDelay, triggerTag, True)

        time.sleep(0.01)
        readCalEvent(triggerTag, True)

        tkrGetASICconfig(0, 3)
        readErrors(address)

    for brd in boards:
        # Measure the trigger noise count
        getLyrTrgCnt(brd)
        getLyrTrgCnt(brd)

#sys.exit("abort")

mask = 0x01    # T1, T2, T3
print("Setting the first trigger mask to " + str(mask))
setTriggerMask(1, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

prescale = 4
print("Setting the PMT trigger prescale to " + str(prescale))
setTriggerPrescale("PMT", prescale)

setTriggerWindow(24)

mask = 0x00    # T1, T3 prescaled
print("Setting the second trigger mask to " + str(mask))
setTriggerMask(2, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

print("Count on channel 2 = " + str(getChannelCount(2)))

for brd in boards:
    tkrSetDAC(brd, 31, "threshold", 9 , "low")
    tkrGetDAC(brd, 3, "threshold")

for i in range(1):
    print("send reset pulse")
    logicReset(addrEvnt)
    time.sleep(0.3)

print("Count on channel 2 = " + str(getChannelCount(2)))
print("Before run, trigger enable status is " + str(triggerEnableStatus()))

readErrors(address)
ADC, Sigma, TOF, sigmaTOF = limitedRun(47, 4, True)
print("Average ADC values:")
print("    T1 = " + str(ADC[0]) + " +- " + str(Sigma[0]))
print("    T2 = " + str(ADC[1]) + " +- " + str(Sigma[1]))
print("    T3 = " + str(ADC[2]) + " +- " + str(Sigma[2]))
print("    T4 = " + str(ADC[3]) + " +- " + str(Sigma[3]))
print("     G = " + str(ADC[4]) + " +- " + str(Sigma[4]))
print("    Ex = " + str(ADC[5]) + " +- " + str(Sigma[5]))
print("    TOF = " + str(TOF) + " +- " + str(sigmaTOF))
chName = ["G","T3","T1","T4","T2"]
for ch in range(5):
    cnt = getEndOfRunChannelCount(ch+1)
    print("Counter for channel " + chName[ch] + " = " + str(cnt))

if asicReset and nBoards>0: tkrAsicPowerOff()

readErrors(address)

#time.sleep(.1)

#num = readNumTOF(address)[0]

#for i in range(num):
#    readTOFevent(address, 0)    
#   readTOFevent(address, 0)     

closeCOM()


