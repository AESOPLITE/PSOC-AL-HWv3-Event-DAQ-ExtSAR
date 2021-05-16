import serial
import time
import numpy
import binascii
import math

from PSOC_cmd import *

portName = "COM3"
openCOM(portName)

print("Entering testIt.py")

address = 8   # Address of the event PSOC

setTofDAC(1, 64, address)   
setTofDAC(2, 64, address)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, address)) + " counts.")

pmtThresh = [20,20,20,20,60]
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
    
voltage = readBusVoltage(INA226_Address['DVDD5'], address)
print("Digital 5V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['AVDD5'], address)
print("Analog 5V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['DVDD33'], address)
print("Digital 3.3V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['AVDD33'], address)
print("Analog 3.3V voltage = " + str(voltage) + "V")

current = readCurrent(INA226_Address['DVDD5'], address)
print("Digital 5V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['DVDD33'], address)
print("Digital 3.3V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['AVDD5'], address)
print("Analog 5V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['AVDD33'], address)
print("Analog 3.3V current = " + str(current) + "mA")

voltage = readBusVoltage(INA226_Address['TKR'], address)
current = readCurrent(INA226_Address['TKR'], address)
print("Tracker main power = " +str(voltage) + " V   and   " + str(current) + " mA")

voltage = 11.0*readBusVoltage(INA226_Address['TkrBias'], address)
print("Tracker bias voltage = " + str(voltage) + " V")

temp = readTemperature(address)
print("Board temperature = " + str(temp) + " degrees Celsius")

Vbat = readBatteryVoltage()
print("Watch battery voltage = " + str(Vbat) + " V")

getPressure(address)
readBarometerTemp(address)

loadRTCregister(0x07, 0x40, address)
byte = readRTCregister(0x07, address)
print("RTC register 0x07 = " + str(binascii.hexlify(byte)))
#setRTCtime(address)
byte = readRTCregister(0x00, address)
print("RTC register 0x00 (seconds) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x01, address)
print("RTC register 0x01 (minutes) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x02, address)
print("RTC register 0x02 (hours) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x03, address)
print("RTC register 0x03 (weekday) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x04, address)
print("RTC register 0x04 (date) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x05, address)
print("RTC register 0x05 (month) = " + str(binascii.hexlify(byte)))
byte = readRTCregister(0x06, address)
print("RTC register 0x06 (year) = " + str(binascii.hexlify(byte)))

readRTCtime(address)

readTofConfig()

#ret = ser.read()
#print(ret)

tkrFPGAreset(0x00)

tkrConfigReset(0x00)

tkrSetNumLyrs(1)
print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(0))))

print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)))

tkrAsicPowerOn()

tkrAsicHardReset(0x1F)

tkrAsicSoftReset(0x1F)

tkrTrigEndStat(0, 1)
tkrSetDualTrig(0, 0)

tkrSetTriggerSource(0)    # We want the external trigger
trgsrc = bytes2int(tkrGetTriggerSource(0))
print("The tracker trigger source is set to " + str(trgsrc))


tkrGetASICconfig(0, 3)

tkrGetTemperature(0)

tkrGetBusVoltage(0, "flash18")
tkrGetBusVoltage(0, "fpga12")
tkrGetBusVoltage(0, "digi25")
tkrGetBusVoltage(0, "i2c33")
tkrGetBusVoltage(0, "analog21")
tkrGetBusVoltage(0, "analog33")
tkrGetShuntCurrent(0, "flash18")
tkrGetShuntCurrent(0, "fpga12")
tkrGetShuntCurrent(0, "digi25")
tkrGetShuntCurrent(0, "i2c33")
tkrGetShuntCurrent(0, "analog21")
tkrGetShuntCurrent(0, "analog33")
tkrGetShuntCurrent(0, "bias100")

readErrors(address)

# Test setting of the calibration mask
hitList = []
hit = [2, 5]
hitList.append(hit)
hit = [1, 18]
hitList.append(hit)
hit = [1, 0]
hitList.append(hit)
hit = [2, 62]
hitList.append(hit)
tkrSetCalMask(0, 31, hitList)
time.sleep(0.1)
tkrGetCalMask(0, 3)
tkrGetCalMask(0, 2)

tkrSetDataMask(0, 31, "mask", hitList)
time.sleep(0.1)
tkrGetDataMask(0, 3)
tkrGetDataMask(0, 7)

tkrSetTriggerMask(0, 31, "mask", hitList)
time.sleep(0.1)
tkrGetTriggerMask(0, 3)
tkrGetTriggerMask(0, 9)

tkrSetDAC(0, 31, "calibration", 96 , "high")
tkrGetDAC(0, 3, "calibration")

tkrSetDAC(0, 31, "threshold", 15 , "low")
tkrGetDAC(0, 3, "threshold")

triggerDelay = 6
triggerTag = 0
sendTkrCalStrobe(0, triggerDelay, triggerTag)

time.sleep(0.01)
readCalEvent(triggerTag)
readErrors(address)

mask = 0x01    # T1, T2, T3
print("Setting the first trigger mask to " + str(mask))
setTriggerMask(1, mask)
print("The first trigger mask is set to  " + str(hex(getTriggerMask(1))))
print("The second trigger mask is set to " + str(hex(getTriggerMask(2))))

prescale = 4
print("Setting the PMT trigger prescale to " + str(prescale))
setTriggerPrescale("PMT", prescale)

setTriggerWindow(72)

mask = 0x00    # T1, T3 prescaled
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
print("Tracker FPGA configuration = " + str(tkrGetFPGAconfig(0)))
readErrors(address)
ADC, Sigma, TOF, sigmaTOF = limitedRun(33, 2)
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

print("Tracker FPGA configuration = " + str(tkrGetFPGAconfig(0)))

readTofConfig()

tkrAsicPowerOff()
tkrTriggerDisable()

print("Tracker FPGA configuration = " + str(tkrGetFPGAconfig(0)))

readErrors(address)

#time.sleep(.1)

#num = readNumTOF(address)[0]

#for i in range(num):
#    readTOFevent(address, 0)    
#   readTOFevent(address, 0)     

closeCOM()


