#Uses regular expression to parse T2 & other data from dataOutput_runX.txt from testItEvtAESOP.py
# Brian Lucas
import re
import sys


captFileName = sys.argv[-1]
# print(captFileName)
captFile = open(captFileName, "r")

adcRe = re.compile(r"  ADC: (?P<T1>[0-9]+), (?P<T2>[0-9]+), (?P<T3>[0-9]+), (?P<T4>[0-9]+), (?P<G>[0-9]+)")
# print(adcRe)


captFileBuff = captFile.read()
# print(captFileBuff)
adcBuff = adcRe.findall(captFileBuff)
# print(adcBuff)

adcT2 = list(map(lambda x: int(x[1]), adcBuff))
print("T2 Peak ADC Counts in ", captFileName)
print(adcT2)