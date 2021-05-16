import serial
import time
import numpy
import binascii

# Address = 8 for the event PSOC, 10 for the main PSOC
addrMain = 10
addrEvnt = 8

from PSOC_cmd import *

address = addrMain

portName = "COM5"
openCOM(portName)

print("Entering testMain.py")

initSPI()
initUART()

print("Set up the Event PSOC to send its output over the SPI bus")
setOutputMode("SPI")

time.sleep(0.1)

#LED2("on", addrMain)
#time.sleep(1)
#LED2("off", addrMain)
LED2("on", addrEvnt)
time.sleep(1)
LED2("off", addrEvnt)

ctrlOsc(1)   # turn on the external clock

print(" ")
print("Get the DAQ Board and Tracker Power Board housekeeping information over the Main PSOC i2c bus")
voltage = readBusVoltage(INA226_Address['DVDD5'], addrMain)
print("Digital 5V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['AVDD5'], addrMain)
print("Analog 5V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['DVDD33'], addrMain)
print("Digital 3.3V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['AVDD33'], addrMain)
print("Analog 3.3V voltage = " + str(voltage) + "V")

voltage = readBusVoltage(INA226_Address['Back15'], addrMain)
print("Backplane 15V voltage = " + str(voltage) + "V")

current = readCurrent(INA226_Address['DVDD5'], addrMain)
print("Digital 5V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['DVDD33'], addrMain)
print("Digital 3.3V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['AVDD5'], addrMain)
print("Analog 5V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['AVDD33'], addrMain)
print("Analog 3.3V current = " + str(current) + "mA")

current = readCurrent(INA226_Address['Back15'], addrMain)
print("Backplane 15V current = " + str(current) + "mA")

voltage = readBusVoltage(INA226_Address['TKR'], addrMain)
current = readCurrent(INA226_Address['TKR'], addrMain)
print("Tracker main power = " +str(voltage) + " V   and   " + str(current) + " mA")

voltage = 11.0*readBusVoltage(INA226_Address['TkrBias'], addrMain)
print("Tracker bias voltage = " + str(voltage) + " V")

temp = readTemperature(addrMain)
print("Board temperature = " + str(temp) + " degrees Celsius")

getPressure(addrMain)

Vback = readBackplaneVoltage()
print("Backplane battery voltage = " + str(Vback) + " V")

print(" ")
print("Read the real-time clock over the i2c bus:")
loadRTCregister(0x07, 0x40, addrMain)
#byte = readRTCregister(0x07, addrMain)
#print("RTC register 0x07 = " + str(binascii.hexlify(byte)))
#setRTCtime(addrMain)
readRTCtime(addrMain)

print(" ")
print("Set up the thresholds for the PMT channels:")
setTofDAC(1, 30, addrEvnt)
setTofDAC(2, 30, addrEvnt)
for channel in range(1,3):
    print("TOF DAC channel " + str(channel) + " was set to " + str(readTofDAC(channel, addrEvnt)) + " counts.")

chan = 5
setPmtDAC(chan, 30, addrEvnt)
time.sleep(0.1)
print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts.")

for chan in range(1,5):
    setPmtDAC(chan, 10 + chan, addrEvnt)
    time.sleep(0.1)
    print("Channel " + str(chan) + " PMT DAC was set to " + str(readPmtDAC(chan, addrEvnt)) + " counts")

print(" ")
print("Check the configuration of the TOF chip:")
readTofConfig()

print(" ")
print("Start the Tracker setup:")
tkrFPGAreset(0x00)

tkrConfigReset(0x00)

readErrors(addrEvnt)

print("The tracker FPGA firmware code version is " + str(tkrGetCodeVersion(0)))

tkrSetNumLyrs(1)

print("The number of tracker readout layers is " + str(bytes2int(tkrGetNumLyrs(0))))

tkrAsicPowerOn()

tkrAsicHardReset(0x1F)

tkrAsicSoftReset(0x1F)

tkrTrigEndStat(0, 1)
tkrSetDualTrig(0, 0)

tkrSetTriggerSource(0)    # We want the external trigger
trgsrc = bytes2int(tkrGetTriggerSource(0))
print("The tracker trigger source is set to " + str(trgsrc))

print(" ")
print("Set up the Tracker ASIC configuration for calibration:")
oneShot = 1
gain = 0
shaping = 1
bufSpeed = 3
trigDelay = 1
trigWindow = 1
ioCurrent = 2
maxClust = 10
tkrLoadASICconfig(0, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
tkrGetASICconfig(0, 3)

print(" ")
print("Get the Tracker housekeeping information:")
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

print(" ")
print("Set up the Tracker calibration and try taking a calibration event:")
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

tkrSetDAC(0, 31, "calibration", 70 , "high")
tkrGetDAC(0, 3, "calibration")

tkrSetDAC(0, 31, "threshold", 30 , "low")
tkrGetDAC(0, 3, "threshold")

triggerDelay = 6
triggerTag = 0
sendTkrCalStrobe(0, triggerDelay, triggerTag)

time.sleep(0.01)
readCalEvent(triggerTag)

tkrGetASICconfig(0, 3)
readErrors(address)

print(" ")
print("Set up the Tracker for a regular run:")
oneShot = 0
gain = 0
shaping = 1
bufSpeed = 3
trigDelay = 1
trigWindow = 1
ioCurrent = 2
maxClust = 10
tkrLoadASICconfig(0, 31, oneShot, gain, shaping, bufSpeed, trigDelay, trigWindow, ioCurrent, maxClust)
tkrGetASICconfig(0, 3)
hitList = []
tkrSetDataMask(0, 31, "unmask", hitList)
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
print("Before the run, the trigger enable status is " + str(triggerEnableStatus()))

print(" ")
numEvents = 2
runNumber = 33
ADC, Sigma, TOF, sigmaTOF = limitedRun(runNumber, numEvents)
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
readErrors(addrEvnt)

#startTOF()

closeCOM()


