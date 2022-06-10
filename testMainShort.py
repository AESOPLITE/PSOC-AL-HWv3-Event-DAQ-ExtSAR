import serial
import time
import numpy
import binascii
import sys

# Address = 8 for the event PSOC, 10 for the main PSOC
addrMain = 10
addrEvnt = 8

from PSOC_cmd import *

address = addrMain

portName = "COM12"
openCOM(portName)

print("Entering testMain.py")

hardResetEventPSOC()
#softResetEventPSOC()

initSPI()
initUART()

print("Set up the Event PSOC to send its output over the SPI bus")
setOutputMode("SPI")

time.sleep(0.1)

#LED2("on", addrMain)
#time.sleep(1)
LED2("off", addrMain)

#time.sleep(1)
LED2("on", addrEvnt)
time.sleep(1)
LED2("off", addrEvnt)

print(" ")
print("Read the real-time clock over the i2c bus:")
loadRTCregister(0x07, 0x40, addrMain)
byte = readRTCregister(0x07, addrMain)
print("RTC register 0x07 = " + str(binascii.hexlify(byte)))
setRTCtime(addrMain)
time.sleep(1)
readRTCtime(addrMain)

#setInternalRTC(addrMain)
setInternalRTCfromI2C()
getInternalRTC(address)

time.sleep(0.1)
loadEventPSOCrtc()
#setInternalRTC(addrEvnt)
time.sleep(1)
getInternalRTC(addrEvnt)

#print(" ")
#print("Set up the thresholds for the PMT channels:")
#setTofDAC(1, 30, addrEvnt)
#setTofDAC(2, 30, addrEvnt)
#for channel in range(1,3):
#    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, addrEvnt)) + " counts.")


print(" ")
print("Set up the thresholds for the PMT channels:")
setTofDAC(1, 48, addrEvnt)
setTofDAC(2, 48, addrEvnt)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, addrEvnt)) + " counts.")

pmtThr = 8
ch5Thresh = 40
pmtThresh = [pmtThr,pmtThr,pmtThr,pmtThr,ch5Thresh]
for chan in range(1,6):
    setPmtDAC(chan, pmtThresh[chan-1], addrEvnt)
    time.sleep(0.1)
    print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts")

print(" ")
print("Check the configuration of the TOF chip:")
readTofConfig()

#sys.exit("abort")

print(" ")
print("Start the Tracker setup:")
tkrFPGAreset(0x00)
tkrSetCRCcheck("yes")

#calibrateAllFPGAinputTiming()

#tkrConfigReset(0x00)

#readErrors(addrEvnt)

print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)))

bumpTKRthreshold(4)
configureTkrASICs(1)

print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(0))))

tkrTrigEndStat(0, 1)
tkrSetDualTrig(0, 0)

tkrSetTriggerSource(0)    # We want the external trigger
trgsrc = bytes2int(tkrGetTriggerSource(0))
print("The tracker trigger source is set to " + str(trgsrc))

print(" ")
print("Set up the Tracker for a regular run:")
oneShot = 0
gain = 0
shaping = 1
bufSpeed = 3
trigDelay = 4
trigWindow = 1
ioCurrent = 2
maxClust = 10
#tkrLoadASICconfig(0, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
tkrGetASICconfig(0, 3)
hitList = []
#tkrSetDataMask(0, 31, "unmask", hitList)
tkrGetDataMask(0, 3)

print(" ")
print("Set up the trigger for a test run:")
mask = 0x01    # T1, T2, T3
print("Setting the first trigger mask to " + str(mask))
setTriggerMask(1, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

prescale = 4
print("Setting the PMT trigger prescale to " + str(prescale))
setTriggerPrescale("PMT", prescale)

setSettlingWindow(98)

mask = 0x00    # T1, T3 prescaled
print("Setting the second trigger mask to " + str(mask))
setTriggerMask(2, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

print("Count on channel 2 = " + str(getChannelCount(2)))

#for i in range(2):
#    print("send reset pulse")
#    logicReset(addrEvnt)
#    time.sleep(0.3)

startHouseKeeping(4, 1)

print("Count on channel 2 = " + str(getChannelCount(2)))
getInternalRTC(address)
getInternalRTC(addrEvnt)
print("Before the run, the trigger enable status is " + str(triggerEnableStatus()))

print(" ")
numEvents = 100
runNumber = 69
ADC, Sigma, TOF, sigmaTOF = limitedRun(runNumber, numEvents, True, False, True)
time.sleep(1)
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
readErrors(addrEvnt)

closeCOM()


