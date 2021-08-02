import re
import sys

def bytes2int(str):
   if str == b'':
       return 0
   return int(str.hex(), 16)

captFileName = sys.argv[-1]
print(captFileName)
captFile = open(captFileName, "rb")

frameRe = re.compile(rb"(?P<num>[\x00-\xff]{2})\x55\xab\x55\xab(?P<data>[\x00-\xff]{27})")
print(frameRe)


captFileBuff = captFile.read(3300)
# print(captFileBuff)
frameBuff = frameRe.findall(captFileBuff)
# print(frameBuff)
evtRe = re.compile(rb"\x5a\x45\x52")
# evtRe = re.compile(rb"\xdc\x00\xff")
framesWritten = 0
framesToWrite = 10
tmpFileName = "./temp-evt.bin"
outTmpFile = open(tmpFileName, "wb")
#inTmpFile = captFile
while frameBuff:
    for curFrame in frameBuff:
        if 0 == framesWritten:
            findRes = evtRe.search(curFrame[1])
            if findRes:
                startHead = findRes.start() - 3
                if 0 <= startHead:
                    outTmpFile = open(tmpFileName, "wb")
                    outTmpFile.write(curFrame[1][startHead:27])
                    print(curFrame[1])
                    print(startHead)
                    print(curFrame[1][startHead:27])
                    framesWritten = framesWritten + 1
        else:
            outTmpFile.write(curFrame[1])
            framesWritten = framesWritten + 1
            if framesToWrite <= framesWritten:
                outTmpFile.close()
                framesWritten = 0
                ser = open(tmpFileName, "rb")
                # print("first ")
                # print(ser.read(1)) #debug alignment issue
                while True:
                    ret = ser.read(3)
                    # print(ret)
                    #print("limitedRun: looking for start of event. Received bytes " + str(ret.hex()))
                    if ret == b'\xDC\x00\xFF': break
                    # if ret == b'': break
                    #time.sleep(0.1)
                verbose = True
                print("limitedRun: reading event " )#+ str(event) + " of run " + str(runNumber))
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
                    byte1 = ser.read(1)
                    if verbose: print("   Packet " + str(i) + ", byte 1 = " + str(bytes2int(byte1)) + " decimal, " + str(byte1.hex()) + " hex")
                    dataList.append(bytes2int(byte1))
                    byteList.append(byte1)
                    byte2 = ser.read(1)
                    if verbose: print("   Packet " + str(i) + ", byte 2 = " + str(bytes2int(byte2)) + " decimal, " + str(byte2.hex()) + " hex")
                    dataList.append(bytes2int(byte2))
                    byteList.append(byte2)
                    byte3 = ser.read(1)
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



    captFileBuff = captFile.read(3300)
    frameBuff = frameRe.findall(captFileBuff)

