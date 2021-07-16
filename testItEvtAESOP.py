import serial
import time
import numpy
import binascii
import math
import sys

from PSOC_cmd import *

# Script for testing the AESOP-Lite system via the Event PSOC

asicReset = True

portName = "COM6"
openCOM(portName)

print("Entering testItEvtAESOP.py at " + time.strftime("%c"))

address = 8   # Address of the event PSOC

#LED2("on", address)
#time.sleep(1)
#LED2("off", address)

setInternalRTC(address)   #Why doesn't this work the first time?
time.sleep(0.1)
setInternalRTC(address)
time.sleep(0.1)
getInternalRTC(address)

    
print("The watch battery voltage is " + str(readBatteryVoltage()) + " V")

setTofDAC(1, 49, address)   
setTofDAC(2, 49, address)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, address)) + " counts.")

pmtThresh = [3,4,4,4,60]
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

# Communication with the TOF chip is messed up due to conflict with the Main PSOC, I think.  Needs some work.
#readTofConfig()

#ret = ser.read()
#print(ret)

boards = [0,1,2,3,4,5,6,7]
nTkrBoards = len(boards)
if nTkrBoards > 0:
    tkrFPGAreset(0x00)
    tkrConfigReset(0x00)

    resetTrackerLogic()
    tkrSetNumLyrs(nTkrBoards)

    readErrors(address)

    print("Trigger enable status = " + str(triggerEnableStatus()))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)) + " for board " + str(0))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(1)) + " for board " + str(1))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(2)) + " for board " + str(2))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(3)) + " for board " + str(3))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(4)) + " for board " + str(4))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(5)) + " for board " + str(5))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(6)) + " for board " + str(6))
    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(7)) + " for board " + str(7))
    #for brd in boards:
    #    print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(brd)) + " for board " + str(brd))
    readErrors(address)

    #sys.exit("abort")

    tkrSetTriggerSource(0)    # We want the external trigger
    trgsrc = bytes2int(tkrGetTriggerSource(0))
    print("The tracker trigger source is set to " + str(trgsrc))

    if asicReset: tkrAsicPowerOn()

    if asicReset: tkrAsicHardReset(0x1F)
    time.sleep(0.1)
    if asicReset: tkrAsicSoftReset(0x1F)

    calibrateFPGAinputTiming(0)
    calibrateFPGAinputTiming(1)
    calibrateFPGAinputTiming(2)
    calibrateFPGAinputTiming(3)
    calibrateFPGAinputTiming(4)
    calibrateFPGAinputTiming(5)
    calibrateFPGAinputTiming(6)
    calibrateFPGAinputTiming(7)
 

    #tkrTrigEndStat(1, 1)
    #tkrTrigEndStat(0, 1)
    tkrSetDualTrig(0, 0)

    oneShot = 0
    gain = 0
    shaping = 0
    bufSpeed = 3
    trigDelay = 6
    trigWindow = 1
    ioCurrent = 2
    maxClust = 10
    tkrLoadASICconfig(0, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
    readErrors(address)
    print("The tracker trigger source is set to " + str(bytes2int(tkrGetTriggerSource(0))))
    readErrors(address)
    for brd in boards:
        tkrGetASICconfig(brd, 3)
        
    #sys.exit("abort")

    #for brd in boards:
    #    tkrGetTemperature(brd)
    #    #tkrGetBusVoltage(brd, "flash18")
    #    tkrGetBusVoltage(brd, "fpga12")
    #    tkrGetBusVoltage(brd, "digi25")
    #    #tkrGetBusVoltage(brd, "i2c33")
    #    tkrGetBusVoltage(brd, "analog21")
    #    tkrGetBusVoltage(brd, "analog33")
    #    #tkrGetShuntCurrent(brd, "flash18")
    #    tkrGetShuntCurrent(brd, "fpga12")
    #    tkrGetShuntCurrent(brd, "digi25")
    #    #tkrGetShuntCurrent(brd, "i2c33")
    #    tkrGetShuntCurrent(brd, "analog21")
    #    tkrGetShuntCurrent(brd, "analog33")
    #    tkrGetShuntCurrent(brd, "bias100")

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
        #tkrSetCalMask(brd, 31, hitList)
        #time.sleep(0.1)
        #tkrGetCalMask(brd, 3)

        #tkrSetDataMask(brd, 31, "mask", hitList)
        #time.sleep(0.1)
        #tkrGetDataMask(brd, 3)

        #tkrSetTriggerMask(brd, 31, "mask", hitList)
        #time.sleep(0.1)
        #tkrGetTriggerMask(brd, 3)

        #tkrSetDAC(brd, 31, "calibration", 60 , "high")
        #tkrGetDAC(brd, 3, "calibration")

        tkrSetDAC(brd, 31, "threshold", 21 , "low")
        tkrGetDAC(brd, 3, "threshold")

        tkrSetDataMask(brd, 31, "unmask", [])
        tkrGetDataMask(brd, 1)

        tkrSetTriggerMask(brd, 31, "unmask", [])
        tkrGetTriggerMask(brd, 3)

    #tkrSetDataMask(6, 4, "mask", [])       
    #tkrSetDataMask(3, 1, "mask", [])

    #tkrGetDataMask(3, 1)
    #tkrGetDataMask(6, 4)

    if len(boards) == 1:
        triggerDelay = 6
        triggerTag = 0
        sendTkrCalStrobe(0, triggerDelay, triggerTag, True)

        time.sleep(0.01)
        readCalEvent(triggerTag, True)

        tkrGetASICconfig(0, 3)
        readErrors(address)

    #for brd in boards:
        # Measure the trigger noise count
    getLyrTrgCnt(4)

mask = 0x04    # T1 & T3 & T4
print("Setting the first trigger mask to " + str(mask))
setTriggerMask(1, mask)
print("Get the trigger mask")
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

prescale = 4
print("Setting the PMT trigger prescale to " + str(prescale))
setTriggerPrescale("PMT", prescale)

setTriggerWindow(16)

mask = 0x0A    # T1, T3 prescaled
print("Setting the second trigger mask to " + str(mask))
setTriggerMask(2, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

print("Count on channel 2 = " + str(getChannelCount(2)))

for i in range(2):
    print("send reset pulse")
    logicReset(addrEvnt)
    time.sleep(0.3)

print("Count on channel 2 = " + str(getChannelCount(2)))
print("Before run, trigger enable status is " + str(triggerEnableStatus()))

#readNumTOF(addrEvnt)
#for trial in range(10):
#    print("TOF event " + str(trial))
#    readAllTOFdata(addrEvnt)
    #time0 = readTOFevent(addrEvnt, 0)
    #time1 = readTOFevent(addrEvnt, 1)
    #print("    T0= " + str(time0) + "   T1= " + str(time1) + "    TOF= " + str(time1-time0))
#    time.sleep(1.0)

#startTOF(10)

#stopTOF()

readErrors(address)

# Run a fixed number of events
ADC, Sigma, TOF, sigmaTOF = limitedRun(35, 2)
print("Ending the run at " + time.strftime("%c"))
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

readErrors(address)

if nTkrBoards > 0:
    print("Tracker FPGA configuration = " + str(tkrGetFPGAconfig(0)))
    if asicReset: tkrAsicPowerOff()
    tkrTriggerDisable()

readErrors(address)



closeCOM()


