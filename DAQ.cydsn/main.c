/* ========================================
 *
 * Robert P. Johnson
 * Copyright U.C. Santa Cruz, 2020
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF U.C. Santa Cruz.
 *
 * Code to run in the Event PSOC on the AESOP-Lite DAQ board.
 * This version uses the external SAR ADCs on the V3 board.
 *
 * V20.0: UART command stream is searched for <CR><LF> in order to identify command strings.
 * V21.1: Event PSOC does the tracker initialization, instead of the tracker FPGA Verilog code doing it.
 * V22.1: Added BUSY signal from the Main PSOC to prevent sending of events when it isn't ready. That also keeps the trigger suspended.
 * V22.2: Corrected error in the tracker initialization code.
 * ========================================
 */
#include "project.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define MAJOR_VERSION 22
#define MINOR_VERSION 2

/*=========================================================================
 * Calibration/PMT input connections, from left to right looking down at the end of the DAQ board:
 *              T3        G        T4        T1        T2       
 * Connector  J10/12    J2/11    J17/18    J15/16    J25/26
 * Preamp      p4[6]    p3[2]     p3[3]     p4[5]      p2[0]
 * Schem pin    9         16       15        14        19
 * Channel      2          1        4         3         5 
 * TOF                              2[B]      1[A] 
 * Trig bit     1         N/A       0         3         2
 * ADC SSN      3          1        6         7         4       
 *
 * TOF SSN = 2
 * 
 * The 4 PSOC DACs are labeled by channel number.
 * Note that T2 is the channel with the external 12-bit DAC for setting its threshold. The digital signal goes to p2[0].
 * T1 and T4 are connected to the two channels that go to the TOF chip.
 * The guard signal, G, does not participate in the trigger logic but is discriminated and registered in the data.
 *
 *  Event output format:
 *  Header "ZERO" in ASCII (5A, 45, 52, 4F)
 *  Event Header:
 *      - Run number 2 bytes
 *      - Event number 4 bytes (counts accepted triggers)
 *      - Trigger time stamp 4 bytes
 *      - Trigger count (including deadtime) 4 bytes
 *      - Real time and date 4 bytes
 *      - Trigger status word 1 byte               
 *  PHA Data:
 *      - T1 2 bytes
 *      - T2 2 bytes
 *      - T3 2 bytes
 *      - T4 2 bytes
 *      - Guard 2 bytes
 *  TOF Data:
 *      Time difference in units of 10 ps, 2 bytes, signed integer
 *  Tracker trigger count 2 bytes
 *  Tracker command count 1 byte
 *  Tracker trigger pattern and event status register 1 byte
 *  TOF debugging data 10 bytes (optional)
 *  Number of tracker boards 1 byte
 *  Tracker Data (variable length)
 *  Trailer "FINI" in ASCII (46, 49, 4e, 49)
 *
 *  The Event PSOC can take commands from the USB-UART or main PSOC UART.    
 *    Each command is formatted as "S1234<sp>xyW" repeated 3 times, followed by <cr><lf>
 *    where 1234 are 4 ASCII characters, each representing a nibble 
 *    12 gives us the data byte and 34 the address byte             
 *    data byte: {7:0} gives the command code                 
 *    address byte: {7:6} and {1:0] give the number of data-byte "commands" to follow, 0 to 15
 *                  {5:2} = 0x8 indicate the event PSOC        
 *    All data arrive in up to 15 subsequent data-byte "commands" For those,
 *    bits {7:0} of the command byte are the data for the command in progress
 *    bits {7:6} and {1:0} give the data-byte number, 1 through 15
 *    Subsequent commands must wait until after the correct number of data bytes has arrived
 *
 *    EEPROM Map:
 *    Each ASIC has 20 bytes of configuration to be saved
 *    - 8 bytes of data mask
 *    - 8 bytes of trigger mask
 *    - 3 byes of configuration, but this must always be the same for all ASICs
 *    - 1 byte of threshold DAC
 *    There are 9*12=108 ASICs, for 1633 bytes total
 *    The first 108 rows have the data mask followed by the trigger mask, for each ASIC, numbered by chip and then by board
 *    The next 9 rows have the threshold DAC settings, one row for each board, with only the first 12 bytes used per row
 *    The last row has the 3 bytes of the configuration register setting
 *    The board numbering is alphabetical: ABCDEFGHI = 012345678
 */

/* I2C mode */
#define ACK  (1u)
#define NACK (0u)
#define I2C_READ (1u)
#define I2C_WRITE (0u)

/* Default DAC threshold setting */
#define THRDEF (5u)

/* Timeout in 5 millisecond units when waiting for command completion */
#define TIMEOUT 1000u 

/* Packet IDs */
#define FIX_HEAD ('\xDB')  // This one is no longer used, because the command echo was added to the data return
#define VAR_HEAD ('\xDC')

#if defined (__GNUC__)
    /* Add an explicit reference to the floating point printf library */
    /* to allow the usage of floating point conversion specifiers. */
    /* This is not linked in by default with the newlib-nano library. */
    asm (".global _printf_float");
#endif
 
#define MXERR 64
#define MAX_CMD_DATA 16
#define TOFSIZE 17
#define TKRHOUSE_LEN 70
#define TOFMAX_EVT 64
#define MAX_TKR_BOARDS 8
#define MAX_TKR_PCB 9     // Including the spare board
#define MAX_TKR_ASIC 12
#define MAX_TKR_BOARD_BYTES 203     // Two leading bytes, 12 bit header, 12 chips * (12-bit header and up to 10 12-bit cluster words) + CRC byte
#define USBFS_DEVICE (0u)
#define BUFFER_LEN  32u
#define MAX_DATA_OUT 255
#define MXERR 64
#define SPI_OUTPUT 0u
#define USBUART_OUTPUT 1u
#define CALMASK 1u
#define DATAMASK 2u
#define TRIGMASK 3u
#define TKR_DATA_READY 0x59
#define TKR_DATA_NOT_READY 0x4E

// Error codes
#define ERR_DAC_LOAD 1u
#define ERR_DAC_READ 2u 
#define ERR_TOF_DAC_LOAD 3u
#define ERR_TOF_DAC_READ 4u
#define ERR_CMD_IGNORE 5u
#define ERR_TKR_READ_TIMEOUT 12u
#define ERR_TKR_BAD_ID 7u
#define ERR_TKR_BAD_LENGTH 8u
#define ERR_TKR_BAD_ECHO 9u
#define ERR_GET_TKR_DATA 10u
#define ERR_TKR_BAD_FPGA 11u
#define ERR_TKR_BAD_TRAILER 12u
#define ERR_TKR_BAD_NDATA 13u
#define ERR_PMT_DAQ_TIMEOUT 14u
#define ERR_TKR_NUM_BOARDS 15u
#define ERR_TKR_BAD_BOARD_ID 16u
#define ERR_TKR_BOARD_SHORT 17u
#define ERR_HEAP_NO_MEMORY 18u
#define ERR_TX_FAILED 19u
#define ERR_BAD_CMD 20u
#define ERR_EVT_TOO_BIG 21u
#define ERR_BAD_BYTE 22u
#define ERR_TKR_BAD_STATUS 23u
#define ERR_TKR_TRG_ENABLE 24u
#define ERR_TKR_BAD_TRGHEAD 25u
#define ERR_TKR_TOO_BIG 26u
#define ERR_TKR_LYR_ORDER 27u
#define ERR_TRK_WRONG_DATA_TYPE 28u
#define ERR_CMD_BUF_OVERFLOW 29u
#define ERR_CMD_TIMEOUT 30u
#define ERR_TRG_NOT_ENABLED 31u
#define ERR_MISSING_HOUSEKEEPING 32u
#define ERR_BAD_CMD_INPUT 33u
#define ERR_TKR_BUFFER_OVERFLOW 34u
#define ERR_TOF_ADC_CONFLICT 35u
#define ERR_TKR_FIFO_NOT_EMPTY 36u
#define ERR_BAD_CMD_FORMAT 37u
#define ERR_UART_CMD 38u
#define ERR_UART_TKR 39u
#define ERR_BAD_CRC 40u
#define ERR_FIFO_OVERFLOW 41u
#define ERR_GET_TKR_EVENT 42u

#define WRAPINC(a,b) ((a + 1) % (b))
#define ACTIVELEN(a,b,c) ((((c) - (a)) + (b)) % (c)) //Macro to calculate active length in a circular buffer.
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define WRAPDEC(a,b) ((a + ((b) - 1)) % (b))
#define CR	(0x0Du) //Carriage return in hex
#define LF	(0x0Au) //Line feed in hex

// Identifiers for types of Tracker data
#define TKR_EVT_DATA 0xD3
#define TKR_HOUSE_DATA 0xC7
#define TKR_ECHO_DATA 0xF1

#define TKR_READ_TIMEOUT 31u    // Length of time to wait on the tracker before giving a time-out error

int boardMAP[MAX_TKR_BOARDS];   // Map from tracker board FPGA address to hardware board number (alphabetical)
struct TkrConfig {
    uint8 trgMask[8];
    uint8 datMask[8];
    uint8 threshDAC;
} tkrConfig[MAX_TKR_BOARDS][MAX_TKR_ASIC];
uint8 tkrConfigReg[3];

uint8 nDataReady;                 // Number of bytes of data ready to send out to the Main PSOC or PC
uint8 dataOut[MAX_DATA_OUT];      // Buffer for output data
uint16 tkrCmdCount;               // Command count returned from the Tracker
uint8 tkrCmdCode;                 // Command code echoed from the Tracker
bool eventDataReady;
bool awaitingCommand;             // The system is ready to accept a new command when true
bool ADCsoftReset;                // Used to force a soft reset of the external SAR ADCs on the first event.
bool doCRCcheck;                  // Whether to check the Tracker hitlist CRC
uint16 cmdCountGLB = 0;           // Count of all command packets received
uint16 cmdCount = 0;              // Count of all event PSOC commands received
uint8 nCmdTimeOut = 0;            // Count the number of command timeouts
uint8 numTkrResets = 0;
uint32 readTimeAvg = 0;
uint32 nReadAvg = 0;

// Register pointers for the power monitoring chips
const uint8 INA226_Config_Reg = 0x00;
const uint8 INA226_ShuntV_Reg = 0x01;
const uint8 INA226_BusV_Reg = 0x02;
const uint8 INA226_Power_Reg = 0x03;
const uint8 INA226_Current_Reg = 0x04;
const uint8 INA226_Calib_Reg = 0x05;
const uint8 INA226_Mask_Reg = 0x06;
const uint8 INA226_Alert_Reg = 0x07;

// Some i2c addresses (temperature, barometer, and RTC are usually only accessible from the Main PSOC)
const uint8 I2C_Address_TMP100 = '\x48';
const uint8 TMP100_Temp_Reg = '\x00';
const uint8 I2C_Address_Barometer = '\x70';
const uint8 I2C_Address_RTC = '\x6F';
const uint8 I2C_Address_DAC_Ch5 = '\x0E';
const uint8 I2C_Address_TOF_DAC1 = '\x0C';
const uint8 I2C_Address_TOF_DAC2 = '\x0F';

uint8 thrDACsettings[] = {THRDEF, THRDEF, THRDEF, THRDEF};

// Bit locations for setting up the trigger, in the Control_Reg_Trg1/2. Not actually used in this code.
#define trgBit_T4 = 0x01;
#define trgBit_T3 = 0x02;
#define trgBit_T2 = 0x04;
#define trgBit_T1 = 0x08;

/* Bit definitions for the pulse control register Control_Reg_Pls */
#define PULSE_LOGIC_RST 0x01    // Resets the hardware logic in the PSOC
#define PULSE_CNTR_RST 0x02     // Resets the counter on each of the PMT channels

// 4-bit slave addresses for the SPI interface
// Bits 0,1,2 drive the 3-to-8 decoder and are active high
// Bit 3 goes directly to the Main PSOC SS and is active low
uint8 SSN_SAR[5];
const uint8 SSN_None = 0x08;
const uint8 SSN_Main = 0x00;
const uint8 SSN_TOF  = 0x0A;
const uint8 SSN_CH1  = 0x09;
const uint8 SSN_CH2  = 0x0B;
const uint8 SSN_CH3  = 0x0F;
const uint8 SSN_CH4  = 0x0E;
const uint8 SSN_CH5  = 0x0C;
uint8 outputMode;              // Data output mode (SPI or USB-UART)
bool debugTOF;

#define END_DATA_SIZE 8u
bool endingRun;                // Set true when the run is ending
uint8 endData[END_DATA_SIZE];

// Command codes for the TOF chip
const uint8 TOF_enable = 0x18;
const uint8 powerOnRESET = 0x30;
const uint8 writeConfig = 0x80;
const uint8 readConfig = 0x40;
const uint8 readResults =  0x60;   // Not used

// Pointer to the internal PSOC real-time-clock structure
RTC_1_TIME_DATE* timeDate;

// Data structures for tracker rate monitoring
uint8 tkrMonitorInterval;   // time interval in seconds between successive 1-second rate accumulations
uint8 tkrMonitorNumAvg;      // number of 1-second rate measurements to average
uint16 tkrMonitorRates[MAX_TKR_BOARDS];
uint tkrMonitorSums[MAX_TKR_BOARDS];
uint nTkrMonSamples;
bool monitorTkrRates;
uint32 tkrClkAtStart;
uint32 tkrClkCntStart;
bool waitingTkrRateCnt;

#define MAX_PMT_CHANNELS 5
// Data structures for PMT singles rate monitoring
uint8  pmtDeltaT = 10;       // rough time in seconds for each rate measurement
uint32 pmtMonitorInterval;   // time interval in seconds between successive rate accumulations
uint32 pmtCntInit[MAX_PMT_CHANNELS];
uint16 pmtMonitorSums[MAX_PMT_CHANNELS];
uint16 pmtMonitorTime;
bool monitorPmtRates;
uint32 pmtClkAtStart;
uint32 pmtClkCntStart;
bool waitingPmtRateCnt;

// Circular FIFO buffer of bytes received from the Main PSOC via the UART.
// This buffer gets searched for 29-byte commands, which then go into the cmd_buffer structure
#define MX_FIFO 57
volatile uint8 cmdFIFO[MX_FIFO];
volatile int fifoWritePtr, fifoReadPtr;

// Circular FIFO buffer of 29-byte UART commands from the Main PSOC
#define CMD_LENGTH 29
#define MX_CMDS 32u
struct MainPSOCcmds {
    uint8 buf[CMD_LENGTH];    // An actual command is made up of multiple buf elements
    uint8 nBytes;
} cmd_buffer[MX_CMDS];
uint8 cmdWritePtr, cmdReadPtr;

// Circular FIFO buffer for bytes coming from the Tracker UART
#define MAX_TKR 1024
volatile uint8 tkrBuf[MAX_TKR];
volatile int tkrWritePtr, tkrReadPtr;

// TOF circular data buffers to store information coming into the shift registers by LVDS from the TOF chip.
// In the case that the TOF shift register is read by DMA instead of interrupt, the data first get written
// to the sampleArray and clkArray by as many linked TDs as possible, and then transferred to the TOF struct
// by an ISR when the TD chain has terminated.
bool TOF_DMA;   // Set true to send TOF data to memory by DMA instead of interrupting the CPU for every hit
#define TOF_DMA_BYTES_PER_BURST 4
#define TOF_DMA_REQUEST_PER_BURST 1
#define TOF_DMA_MAX_NO_OF_SAMPLES 32
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
volatile struct TOF {
    uint32 shiftReg[TOFMAX_EVT];
    uint8 clkCnt[TOFMAX_EVT];  
    bool filled[TOFMAX_EVT];
    uint8 ptr;
} tofA, tofB;
uint8 nTOF_DMA_samples;
volatile uint8 DMA_TOFA_Chan;
uint8 DMA_TOFA_TD[2*TOF_DMA_MAX_NO_OF_SAMPLES];   
volatile uint8 DMA_TOFB_Chan;
uint8 DMA_TOFB_TD[2*TOF_DMA_MAX_NO_OF_SAMPLES]; 
volatile uint32 tofA_sampleArray[TOF_DMA_MAX_NO_OF_SAMPLES] __attribute__ ((aligned(32))) = {0};
volatile uint8 tofA_clkArray[TOF_DMA_MAX_NO_OF_SAMPLES];
volatile uint32 tofB_sampleArray[TOF_DMA_MAX_NO_OF_SAMPLES] __attribute__ ((aligned(32))) = {0};
volatile uint8 tofB_clkArray[TOF_DMA_MAX_NO_OF_SAMPLES];

bool outputTOF;  // Controls a special debugging mode to send TOF data out immediately each time it comes in

// Temporary storage of Tracker housekeeping data
uint8 nTkrHouseKeeping;
uint8 tkrHouseKeepingFPGA;
uint8 tkrHouseKeepingCMD;
uint8 tkrHouseKeeping[TKRHOUSE_LEN];

volatile uint32 timeStamp;     // Internal event time stamp, in 5 millisecond units
volatile uint8 timeStamp8;     // Place to store the timestamp counter reading
volatile uint32 cntGO1save;    // Word to save the trigger counter in each time there is an accepted trigger
volatile uint8 trgStatus;      // Contents read from trigger status register
volatile bool triggered;       // The system is triggered, so a readout is needed.

// Tracker data are buffered in this structure prior to sending event data out to the Main PSOC or PC
struct TkrData {
    uint16 triggerCount;
    uint8 cmdCount;
    uint8 trgPattern;              // bit 7 = non-bending; bit 6 = bending
    uint8 nTkrBoards;              // number of boards read out
    struct BoardHits {
        uint8 nBytes;              // number of bytes in the hit list
        uint8* hitList;            // pointer to the variable length hit list
    } boardHits[MAX_TKR_BOARDS];
} tkrData;
uint8 numTkrBrds;
bool readTracker;

volatile uint32 clkCnt;    // 5ms clock counter
uint32 time() {   // Returns the time in 5 millisecond units
    uint8 cnt200val = Cntr8_Timer_ReadCount();   // This counter turns over every 200 counts
    return clkCnt + cnt200val;                  
}

// Channel and trigger rate counters
volatile uint32 ch1Count;  
volatile uint32 ch2Count;
volatile uint32 ch3Count;
volatile uint32 ch4Count;
volatile uint32 ch5Count;
volatile uint32 cntGO;
volatile uint32 cntGO1;
uint16 runNumber;

// Saved counts for end of run, from the 8-bit hardware counters and the 16-bit software counters
uint8 ch1CtrSave, ch2CtrSave, ch3CtrSave, ch4CtrSave, ch5CtrSave;
uint32 ch1CountSave, ch2CountSave, ch3CountSave, ch4CountSave, ch5CountSave;

// Variables for commands received
uint8 command = 0;                 // Most recent command code
uint8 cmdData[MAX_CMD_DATA];       // Data sent with commands
uint8 nDataBytes = 0;              // Number of data bytes in the current command
bool cmdDone;

// Function to turn the LED on/off furthest from the SMA inputs, for debugging
void LED2_OnOff(bool on) {
    if (on) {
        Pin_LED2_Write(1);
    } else {
        Pin_LED2_Write(0);
    }
}

// Errors are logged into this structure pending reading them out by command
struct Error {
    uint8 errorCode;  // Use error codes defined above
    uint8 value0;     // First information byte
    uint8 value1;     // Second information byte
} errors[MXERR];
uint8 nErrors = 0;

// Function used to log internal errors
void addError(uint8 code, uint8 val0, uint8 val1) {
    if (nErrors < MXERR) {
        errors[nErrors].errorCode = code;
        errors[nErrors].value0 = val0;
        errors[nErrors].value1 = val1;
        nErrors++;
    }
}

// Function used to log an internal error only if that error is not already logged
void addErrorOnce(uint8 code, uint8 val0, uint8 val1) {
    for (int i=0; i<nErrors; ++i) {
        if (errors[i].errorCode == code) return;
    }
    if (nErrors < MXERR) {
        errors[nErrors].errorCode = code;
        errors[nErrors].value0 = val0;
        errors[nErrors].value1 = val1;
        nErrors++;
    }
}

// Load a single I2C byte register
uint8 loadI2Creg(uint8 I2C_Address, uint8 regAddress, uint8 regValue) {

    // Send the start byte, with address
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register address
    rc = I2C_2_MasterWriteByte(regAddress);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register value
    rc = I2C_2_MasterWriteByte(regValue);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;    
}

// Read bytes from an I2C chip register
uint8 readI2Creg(int nBytes, uint8 I2C_Address, uint8 regAddress, uint8 regValue[]) {
   
    // Send the start byte, with address and control set to write
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    // Send the register address
    rc = I2C_2_MasterWriteByte(regAddress);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;

    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;    

    // Wait for the data transfer to complete 
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }      
    
    // Send the start byte, with address and control set to read
    rc = I2C_2_MasterSendStart(I2C_Address,I2C_READ);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    for (int i=0; i<nBytes-1; ++i) {
        regValue[i] = I2C_2_MasterReadByte(ACK);
    }
    regValue[nBytes-1] = I2C_2_MasterReadByte(NACK);
        
    // Send the stop condition
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;    
}

// Load the AD5622 DAC via the i2c bus
uint8 loadDAC(uint8 I2C_Address, uint16 voltage) {
    
    uint8 nib0 = (voltage & 0x00FF);
    uint8 nib1 = (voltage & 0x0F00)>>8;
    
    uint8 rc;
    
    rc = I2C_2_MasterSendStart(I2C_Address,I2C_WRITE);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterWriteByte(nib1);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterWriteByte(nib0);
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    rc = I2C_2_MasterSendStop();
    if(rc != I2C_2_MSTR_NO_ERROR) return rc;
    
    /* Wait for the data transfer to complete */
    rc = I2C_2_MasterStatus();
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    }  
    
    return 0;
} // end of loadDAC

// Read back over the i2c bus the setting from the AD5622 DAC and return it
uint8 readDAC(uint8 I2C_Address, uint16* rvalue) {
           
    uint8 bytes[2];
    //I2C_2_MasterClearReadBuf();
    //uint8 rc = I2C_2_MasterReadBuf(I2C_Address, bytes, 2, I2C_2_MODE_COMPLETE_XFER);
    uint8 rc = I2C_2_MasterSendStart(I2C_Address,I2C_READ);
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;    

    bytes[0] = I2C_2_MasterReadByte(ACK);
    bytes[1] = I2C_2_MasterReadByte(NACK);

    rc = I2C_2_MasterSendStop();
    if (rc != I2C_2_MSTR_NO_ERROR) return rc;

    *rvalue = ((uint16)(bytes[0] & '\x3F'))<<6;
    *rvalue = *rvalue | ((uint16)(bytes[1] & '\xFC')>>2);
    
    rc = I2C_2_MasterStatus();    
    if (rc != I2C_2_MSTAT_CLEAR) {
        while(I2C_2_MasterStatus() == I2C_2_MSTAT_XFER_INP) CyDelay(100);
    } 
    return 0;
}

// Check whether the trigger is enabled
bool isTriggerEnabled() {
    uint8 regValue = Control_Reg_Trg_Read();
    return regValue==0x01;
}

// Set the time delays used for coordinating the peak detector readout:
// Time to wait for the peak detector to rise and settle before starting the ADC conversion
// Time to wait for the conversions to finish before signaling the CPU to start the readout
// Time to hold the reset active on the peak detectors
void setPeakDetResetWait(uint8 waitTime) {
    Count7_3_WritePeriod(waitTime);
}

// Set the mask to define the coincidence level of the two PHA triggers, 'e' for electrons and 'p' for protons.
// Note that the 'p' trigger is prescaled by Cntr8_V1_PMT, see command 0x39.
void setTriggerMask(char trigger, uint8 mask) {
    mask = mask & 0x0F;
    if (trigger == 'e') {
        Control_Reg_Trg1_Write(mask);
    } else if (trigger == 'p') {
        Control_Reg_Trg2_Write(mask);
    }
}

// Reads back the current settings of the trigger masks.
uint8 getTriggerMask(char trigger) {
    if (trigger == 'e') return Control_Reg_Trg1_Read();
    else if (trigger == 'p') return Control_Reg_Trg2_Read();
    return 0;
}

// The TOF chip and the Main PSOC are on the same SPI bus, but the SAR ADCs were moved to a separate bus.
// However, a single decoder decodes the TOF and SAR ADC chip selects. The Main PSOC chip select is independent.

// Set the chip select for the SPI bus used only by the external SAR ADCs.
// The TOF chip's SPI should not be in use simultaneuously.
void set_ADC_SSN(uint8 SSN) {
    Control_Reg_SSN_Write(SSN & 0x07);
}

// Control of the SPI slave address. The slave select is active low.
// **** Note that the TOF chip SSN needs to go high, for reset, before each SPI transaction. ****
void set_SPI_SSN(uint8 SSN, bool clearBuffer) {
    while (!(SPIM_ReadTxStatus() & SPIM_STS_SPI_IDLE));  // Wait for completion of any transaction in progress
    if (SSN == SSN_TOF) {
        Pin_SSN_Main_Write(1);
        uint8 regContent = Control_Reg_SSN_Read();
        if (regContent != (SSN_TOF & 0x07) && regContent != 0x00) addError(ERR_TOF_ADC_CONFLICT, SSN, regContent);
        Control_Reg_SSN_Write(0);
        CyDelay(1);    // The TOF SPI is not accessed during DAQ, so this delay doesn't hurt anything.
        Control_Reg_SSN_Write(SSN & 0x07);
    } else if (SSN == SSN_Main) {
        Control_Reg_SSN_Write(0);   // Sets just the 3-8 decoder.
        Pin_SSN_Main_Write(0);      // Sets the Main PSOC dedicated SSN.
    } else if (SSN == SSN_None) {
        Control_Reg_SSN_Write(0);
        Pin_SSN_Main_Write(1);
    }
    if (clearBuffer) SPIM_ClearTxBuffer();
}

// Enable or disable the TOF acquisition
void TOFenable(bool enable) {
    if (TOF_DMA) {
        if (enable) {
            // The DMA needs to start up with the shift register FIFO empty, because it always reads only the first
            // entry in the register.
            while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) ShiftReg_A_ReadData();
            while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) ShiftReg_B_ReadData();
            CyDmaChEnable(DMA_TOFA_Chan, 1);   
            CyDmaChEnable(DMA_TOFB_Chan, 1);  
        } else {
            CyDmaChSetRequest(DMA_TOFA_Chan, CY_DMA_CPU_TERM_CHAIN);
            CyDmaChSetRequest(DMA_TOFB_Chan, CY_DMA_CPU_TERM_CHAIN);
        }
    } else {
        if (enable) {
            // Clean out the shift register FIFOs before enabling the ISR.
            while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) ShiftReg_A_ReadData();
            while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) ShiftReg_B_ReadData();
            isr_Store_A_Enable();
            isr_Store_B_Enable();
        } else {
            isr_Store_A_Disable();
            isr_Store_B_Disable();
        }
    }
}

// Control of the trigger enable bit. 
void triggerEnable(bool enable) {
    if (enable) {
        Control_Reg_Trg_Write(1); 
    } else {
        // Disable the master trigger 
        Control_Reg_Trg_Write(0);
    }
}

// Extract 1 of 4 bytes from a 32-bit word and return it as uint8
uint8 byte32(uint32 word, int byte) {
    const uint32 mask[4] = {0xFF000000, 0x00FF0000, 0x0000FF00, 0x000000FF};
    return (uint8)((word & mask[byte]) >> (3-byte)*8);
}
// Extract 1 of 2 bytes from a 16-bit word and return it as uint8
uint8 byte16(uint16 word, int byte) {
    const uint16 mask[2] = {0xFF00, 0x00FF};
    return (uint8)((word & mask[byte]) >> (1-byte)*8);
}

// Reset all the logic and counters
void logicReset() {
    LED2_OnOff(true);
    int InterruptState = CyEnterCriticalSection();
    clkCnt = 0;             
    ch1Count = 0;
    ch2Count = 0;
    ch3Count = 0;
    ch4Count = 0;
    ch5Count = 0;
    set_SPI_SSN(SSN_None,true);
    Pin_LED1_Write(0);
    Pin_LED_TKR_Write(0);
    Pin_LED_DAT_Write(0);
    cntGO = 0;
    cntGO1 = 0;
    Control_Reg_Pls_Write(PULSE_LOGIC_RST);
    Control_Reg_Pls_Write(PULSE_CNTR_RST);
    CyDelay(20);
    
    for (int brd=0; brd<MAX_TKR_BOARDS; ++brd) {
        if (tkrData.boardHits[brd].nBytes > 0) {
            tkrData.boardHits[brd].nBytes = 0;
            free(tkrData.boardHits[brd].hitList);
        }
    }
    CyExitCriticalSection(InterruptState);
    LED2_OnOff(false);
}

// Get a byte of data from the Tracker UART software buffer, with a time-out in case nothing is coming.
// The second argument (flag) helps to identify where a timeout error originated.
uint8 tkr_getByte(uint32 startTime, uint8 flag) {
    while (tkrReadPtr < 0) {  // No buffered data are available
        uint32 timeElapsed = time() - startTime;
        if (timeElapsed > TKR_READ_TIMEOUT || (startTime > time() && time() > TKR_READ_TIMEOUT)) {
            uint8 temp = (uint8)(timeElapsed & 0x000000ff);
            addError(ERR_TKR_READ_TIMEOUT, temp, flag);
            return 0x00;
        }
    }
    isr_TKR_Disable();
    uint8 theByte = tkrBuf[tkrReadPtr];
    if (tkrReadPtr < MAX_TKR-1) tkrReadPtr++;
    else tkrReadPtr = 0;
    if (tkrReadPtr == tkrWritePtr) tkrReadPtr = -1;  // All buffered data have been read
    isr_TKR_Enable();
    return theByte;
}

// Function to receive ASIC register data from the Tracker
void getASICdata() {
    uint32 startTime = time();
    nDataReady = tkr_getByte(startTime, 69);
    dataOut[0] = nDataReady;
    nDataReady++;
    for (int i=1; i<nDataReady; ++i) {
        startTime = time();
        dataOut[i] = tkr_getByte(startTime, 70+i);
    }
}

// Function to receive i2c register data from the Tracker
void getTKRi2cData() {
    uint32 startTime = time();
    nDataReady = 4;
    dataOut[0] = tkr_getByte(startTime, 0x89);
    dataOut[1] = tkr_getByte(startTime, 0x90);
    dataOut[2] = tkr_getByte(startTime, 0x91);
    dataOut[3] = tkr_getByte(startTime, 0x92);
}

// Receive trigger-primitive and TOT data from the tracker, for calibration-pulse events only
int getTrackerBoardTriggerData(uint8 FPGA) {
    int rc = 0;
    uint32 startTime = time();
    // Ignore the first byte, which is rubbish (not sure why. . .)
    uint8 theByte = tkr_getByte(startTime, 0x44);
    // The first good byte received encodes the FPGA address, so we check it here:
    theByte = tkr_getByte(startTime, 0x45);
    uint8 fpgaRet = (theByte & 0x38)>>3;
    if (fpgaRet != FPGA) {
        addError(ERR_TKR_BAD_TRGHEAD, FPGA, fpgaRet);
        rc = 1;
    }
    nDataReady = 9;   
    dataOut[0] = theByte;
    // Read in the other 8 bytes
    for (int i=1; i<nDataReady; ++i) {
        dataOut[i] = tkr_getByte(startTime, 0x46);
    }
    return rc;
}

// Build a dummy tracker empty hit list for use when the hardware fails to send a good hit list
void makeDummyHitList(int brd, uint8 code) {
    tkrData.boardHits[brd].nBytes = 5;                      // Minimum length of a board hit list
    tkrData.boardHits[brd].hitList = (uint8*) malloc(5);
    if (tkrData.boardHits[brd].hitList != NULL) {
        tkrData.boardHits[brd].hitList[0] = 0xE7;           // Identifier
        tkrData.boardHits[brd].hitList[1] = brd;            // Layer number
        tkrData.boardHits[brd].hitList[2] = 1;              // Trigger count set to 0, error bit set to 1
        tkrData.boardHits[brd].hitList[3] = (0x0F & code);  // 4 bits for 0 ASICs plus 4 bits of the CRC
        tkrData.boardHits[brd].hitList[4] = 0x30;           // 2 bits of the CRC set to 0, then 11, then 0 filler nibble
                                                            // The CRC is wrong and will get flagged. The code that
                                                            // replaced the CRC indicates why this dummy was inserted.
    }
}

// Build an entire dummy tracker emtpy event
void makeDummyTkrEvent(uint8 trgCnt, uint8 cmdCnt, uint8 trgPtr, uint8 code) {
    tkrData.triggerCount = trgCnt;  // Send back a packet that won't cause a crash down the road
    tkrData.cmdCount = cmdCnt;
    tkrData.trgPattern = trgPtr;
    tkrData.nTkrBoards = numTkrBrds;
    for (int brd=0; brd<numTkrBrds; ++brd) {  // Make a default empty ASIC hit list
        makeDummyHitList(brd, code);
    }
}

// Calculate a 6-bit CRC of a set of sequential bytes.
uint8 CRC6(int nBits, uint8 theBytes[]) {
    uint8 divisor[7] = {1,1,0,0,1,0,1};  // The key used by the Tracker FPGA
    uint8 mask[8] = {0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x01};
    nBits++;  // Need to add one more bit, for the start bit
    int nBytes = nBits/8;
    if (nBits%8 != 0) nBytes++;
    // First, expand the bytes of the hitlist into a bitstring.
    uint8* A = (uint8*) malloc(nBits);
    if (A == NULL) {
        addErrorOnce(ERR_HEAP_NO_MEMORY, nBits, 0x55);
        return 0x00;
    }
    A[0]=1;  // The CRC was calculated in the FPGA with the start bit, so we add it back here.
    int ibit = 1;
    for (int iB=0; iB<nBytes; ++iB) {
        for (int i=0; i<8; ++i) {
            if (theBytes[iB] & mask[i]) {
                A[ibit++] = 1;
            } else {
                A[ibit++] = 0;
            }
            if (ibit == nBits) goto done;
        }
    }
    done: // Start here the CRC calculation
    for (int i=0; i<nBits-6; ++i) {
        if (A[i] == 1) {
            for (int j=0; j<7; ++j) {
                if (A[i+j] == divisor[j]) {
                    A[i+j] = 0;             // Exclusive OR
                } else {
                    A[i+j] = 1;
                }
            }
        }
    }
    //Extract the last 6 bits remaining in 'A', and pack into a uint8 as the CRC
    uint8 crc = 0;
    for (int i=0; i<6; ++i) {
        crc = crc<<1;
        if (A[nBits-6+i]) crc = crc | 0x01;        
    }
    free(A);
    return crc;
}

// Recalculate the 6-bit hitlist CRC and compare with the Tracker FPGA calculation.
bool checkCRC(int nBytes, uint8 hitList[]) {
    uint8 masks[7] = {0xC0,0x60,0x30,0x18,0x0C,0x06,0x03};
    uint8 crc, crcL, crcR;
    int nBits = nBytes*8 - 2;
    int nShift = 2;
    // Look for the '11' that indicates the end of the hitlist.
    // It should be in the last byte of the hitlist, otherwise something is screwed up.
    // Then extract the preceeding 6 bits as the FPGA CRC.
    for (int i=6; i>=0; --i) {
        if ((hitList[nBytes-1] & masks[i]) == masks[i]) goto foundIt;
        nBits--;
        nShift++;
    }
    return false;
    foundIt:
    crcL = (hitList[nBytes-2]<<(8-nShift));
    crcR = (hitList[nBytes-1]>>nShift);
    crc = (crcL | crcR) & 0x3F;
    uint8 crcNew = CRC6(nBits-6, hitList);  // Recalculate the 6-bit CRC
    return (crcNew == crc);                 // Compare with the FPGA value
}

// Function to get a full data packet from the Tracker.
int getTrackerData(uint8 idExpected) {
    int rc = 0;
    uint32 startTime = time();
    uint8 len = tkr_getByte(startTime, 1);
    uint8 IDcode = tkr_getByte(startTime, 2);
    if (IDcode != idExpected) {
        if (idExpected != 0) {
            addError(ERR_TRK_WRONG_DATA_TYPE, IDcode, idExpected);
            if (idExpected == TKR_EVT_DATA) {
                makeDummyTkrEvent(0, 0, 0, 0);
                return 54;
            }
        } else {
            if (IDcode == TKR_EVT_DATA) {
                addError(ERR_TRK_WRONG_DATA_TYPE, IDcode, idExpected);
                return 53;
            }
        }
    }
    if (IDcode == TKR_EVT_DATA) {         // Event data
        if (len != 5) {           // Formal check
            addError(ERR_TKR_BAD_LENGTH, IDcode, len);
            makeDummyTkrEvent(0, 0, 0, 1);  // Send back a packet that won't cause a crash down the road
            return 55;
        }
        uint8 trgCnt = ((uint16)tkr_getByte(startTime, 3) & 0x00FF) << 8;
        trgCnt = trgCnt | ((uint16)tkr_getByte(startTime, 4) & 0x00FF);
        uint8 cmdCnt = tkr_getByte(startTime, 5);
        uint8 nBoards = tkr_getByte(startTime, 6);
        uint8 trgPtr = nBoards & 0xC0;
        nBoards = nBoards & 0x3F;
        if (nBoards != numTkrBrds) {
            addError(ERR_TKR_NUM_BOARDS, nBoards, tkrData.trgPattern);
            makeDummyTkrEvent(trgCnt, cmdCnt, trgPtr, 2);
            return 56;
        }
        tkrData.triggerCount = trgCnt;
        tkrData.cmdCount = cmdCnt;
        tkrData.trgPattern = trgPtr;
        tkrData.nTkrBoards = nBoards;
        for (int brd=0; brd < nBoards; ++brd) {
            uint8 nBrdBytes = tkr_getByte(startTime, 7);  // Length of the hit list, in bytes
            if (nBrdBytes < 4) {
                addError(ERR_TKR_BOARD_SHORT, nBrdBytes, brd);
                makeDummyHitList(brd, 4);
                rc = 57;
                continue;
            }
            uint8 IDbyte = tkr_getByte(startTime, 8);     // Hit list identifier, should always be 11100111
            if (IDbyte != 0xE7) {
                addError(ERR_TKR_BAD_BOARD_ID, IDbyte, brd);
                makeDummyHitList(brd, 5);
                rc = 58;
                continue;
            }
            uint8 byte2 = tkr_getByte(startTime, 9);        // Byte containing the board address
            if (byte2 > 8) {   // Formal check. Note that 8 denotes the master board, which really is layer 0
                addError(ERR_TKR_BAD_FPGA, byte2, brd);
                rc = 59;
            }
            uint8 lyr = 0x7 & byte2;  // Get rid of the master bit, leaving just the layer number
            // Require the boards to be set up to read out in order:
            if (lyr != brd) {
                addError(ERR_TKR_LYR_ORDER, lyr, brd);
                lyr = brd;
            }
            if (nBrdBytes > MAX_TKR_BOARD_BYTES) {    // This really should never happen, due to ASIC 10-hit limit
                tkrData.boardHits[lyr].nBytes = MAX_TKR_BOARD_BYTES;
                addError(ERR_TKR_TOO_BIG, nBrdBytes, lyr);
            } else {
                tkrData.boardHits[lyr].nBytes = nBrdBytes;
            }
            tkrData.boardHits[lyr].hitList = (uint8*) malloc(nBrdBytes);
            if (tkrData.boardHits[lyr].hitList == NULL) {   // Ran out of heap memory. Not good!
                addError(ERR_HEAP_NO_MEMORY, nBrdBytes-2, brd);
                rc = 60;
                continue;
            }
            tkrData.boardHits[lyr].hitList[0] = IDbyte;
            tkrData.boardHits[lyr].hitList[1] = byte2;
            for (int i=2; i<nBrdBytes; ++i) {
                uint8 theByte = tkr_getByte(startTime, 10);
                if (i<MAX_TKR_BOARD_BYTES) {       
                    tkrData.boardHits[lyr].hitList[i] = theByte;
                }
            }
        }
    } else if (IDcode == TKR_HOUSE_DATA) {  // Housekeeping data
        uint8 nData = tkr_getByte(startTime, 11);
        if (len != nData+6) {   // Formal check
            addError(ERR_TKR_BAD_NDATA, len, nData);
            nData = len - 6;
        }
        tkrCmdCount = (uint16)(tkr_getByte(startTime, 12)) << 8;
        tkrCmdCount = (tkrCmdCount & 0xFF00) | (uint16)tkr_getByte(startTime, 13);
        tkrHouseKeepingFPGA = tkr_getByte(startTime, 14);
        if (tkrHouseKeepingFPGA > 8) {   // Formal check
            addError(ERR_TKR_BAD_FPGA, tkrCmdCode, tkrHouseKeepingFPGA);
        }
        uint8 tkrHouseKeepingCMD = tkr_getByte(startTime, 15);
        if (tkrHouseKeepingCMD != tkrCmdCode) {   // Formal check
            addError(ERR_TKR_BAD_ECHO, tkrHouseKeepingCMD, tkrCmdCode);
        }
        nTkrHouseKeeping = 0;      // Overwrite any old data, even if it was never sent out.
        for (int i=0; i<nData; ++i) {
            uint8 tmpData = tkr_getByte(startTime, 16);
            if (i < TKRHOUSE_LEN) {
                tkrHouseKeeping[i] = tmpData;
                nTkrHouseKeeping++;
            }
        }
        if (tkrHouseKeeping[nTkrHouseKeeping-1] != 0x0F) {    // Formal check
            addError(ERR_TKR_BAD_TRAILER, tkrCmdCode, tkrHouseKeeping[nTkrHouseKeeping-1]);           
        }
    } else if (IDcode == TKR_ECHO_DATA) {  // Command Echo
        if (len != 4) {           // Formal check
            addError(ERR_TKR_BAD_LENGTH, IDcode, len);
        }
        nDataReady = 3;
        dataOut[0] = tkr_getByte(startTime, 17);
        tkrCmdCount = (uint16)dataOut[0] << 8;
        dataOut[1] = tkr_getByte(startTime, 18);
        tkrCmdCount = (tkrCmdCount & 0xFF00) | dataOut[1];
        uint8 tkrCmdCodeEcho = tkr_getByte(startTime, 19);
        dataOut[2] = tkrCmdCodeEcho;
        if (tkrCmdCode != tkrCmdCodeEcho) {
            addError(ERR_TKR_BAD_ECHO, tkrCmdCodeEcho, tkrCmdCode);
            rc = 1;
        }
    } else {  // WTF?!?   Not sure what to do with this situation, besides flag it.
        if (nErrors < MXERR) {
            addError(ERR_TKR_BAD_ID, IDcode, len);
        }
        // Wait a short time on the UART and then empty the Tracker buffer
        // and send out whatever crap came in, hoping for the best. . .
        CyDelay(2);
        isr_TKR_Disable();
        nDataReady = 0;
        while (tkrReadPtr != tkrWritePtr) {
            dataOut[nDataReady++] = tkrBuf[tkrReadPtr];            
            if (tkrReadPtr < MAX_TKR-1) tkrReadPtr++;
            else tkrReadPtr = 0;
        }  
        tkrReadPtr = -1;
        isr_TKR_Enable(); 
    }
    return rc;
}

// Turn on the LED on the double RJ45 connector to indicate Tracker communication. It will turn off only
// after a delay long enough to make it visible.
void tkrLED(bool on) {
    isr_timer_Disable();
    if (on) {
        Pin_LED_TKR_Write(1);
    } else {
        Timer_1_Start();
    }
    isr_timer_Enable();
}

// Send a command to the tracker via the UART
uint8 sendTrackerCmd(uint8 FPGA, uint8 code, uint8 nData, uint8 cmdData[], uint rtnType) {
    tkrLED(true);
    tkrCmdCode = code;
    while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {
        addErrorOnce(ERR_TKR_FIFO_NOT_EMPTY, tkrCmdCode, 0x2F);
    }
    UART_TKR_WriteTxData(FPGA);         // FPGA address
    UART_TKR_WriteTxData(tkrCmdCode);   // Command code
    UART_TKR_WriteTxData(nData);        // Number of data bytes
    for (int i=0; i<nData; ++i) {
        if (i>0) {
            while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
        }
        UART_TKR_WriteTxData(cmdData[i]);
    }
 
    if (tkrCmdCode == 0x0F) {     // This command sets the number of tracker boards in the readout.
        numTkrBrds = cmdData[0];  // Make sure the PSOC also knows how many boards are used.
    }
    
    // Wait around for up to a second for all the data to transmit
    uint32 tStart = time();
    while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {                                           
        if (time() - tStart > 200) {
            addError(ERR_TX_FAILED, tkrCmdCode, 0);
            tkrLED(false);
            break;
        }
    }        
    uint8 rc = 0;
    if (tkrCmdCode == 0x67 || tkrCmdCode == 0x6C) {
        tkrLED(false);
        rc = 0xCF;
        return rc;    // These commands have no echo or data to return
    }
    // Now look for the bytes coming back from the Tracker.
    if (tkrCmdCode >= 0x20 && tkrCmdCode <= 0x25) {
        getASICdata();
    } else if (tkrCmdCode == 0x46) {
        getTKRi2cData();
    } else {
        rc = getTrackerData(rtnType);
        if (rc != 0) {
            addError(ERR_GET_TKR_DATA, rc, tkrCmdCode);
        }
    }
    tkrLED(false);
    return rc;
}

// Function to send a command to the tracker that has no data bytes sent or returned
void sendSimpleTrackerCmd(uint8 FPGA, uint8 code) {
    tkrLED(true);
    tkrCmdCode = code;
    while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {
        addErrorOnce(ERR_TKR_FIFO_NOT_EMPTY, tkrCmdCode, 0x1F);
    }
    UART_TKR_WriteTxData(FPGA);           // FPGA address
    UART_TKR_WriteTxData(tkrCmdCode);     // Command code
    UART_TKR_WriteTxData(0x00);           // No data bytes

    // Wait around for the data to transmit
    uint32 tStart = time();
    while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {                                           
        if (time() - tStart > 200) {
            addError(ERR_TX_FAILED, code, 0xFF);
            tkrLED(false);
            return;
        }
    }                                
    // Now look for the echo coming back from the Tracker.
    if (tkrCmdCode != 0x67 && tkrCmdCode != 0x6C) {
        int rc = getTrackerData(TKR_ECHO_DATA);
        if (rc != 0) {
            addError(ERR_GET_TKR_DATA, rc, tkrCmdCode);
        }
    }
    nDataReady = 0;  // Suppress the echo from being sent out to the world
    tkrLED(false);
}

// Find the minimum time difference between two 8-bit time stamps
uint8 minTdif(uint8 t1, uint8 t2) {
    uint8 tMin, tMax;
    if (t1>t2) {
        tMin = t2;
        tMax = t1;
    } else {
        tMin = t1;
        tMax = t2;
    }
    int period = 200;
    uint8 tD1 = tMax - tMin;
    uint8 tD2 = tMin + period - tMax;
    if (tD1 < tD2) return tD1;
    else return tD2;
}

// Configure all of the ASICs according to the settings stored in RAM
void configureASICs() {
    uint8 dataBytes[9];
    
    // First load all of the configuration registers with one wild-card call
    tkrCmdCode = 0x12;
    dataBytes[0] = 0x1F;
    dataBytes[1] = tkrConfigReg[0];
    dataBytes[2] = tkrConfigReg[1];
    dataBytes[3] = tkrConfigReg[2];
    sendTrackerCmd(0x00, tkrCmdCode, 4, dataBytes, TKR_ECHO_DATA);
    
    // Next load all of the threshold DACs individually
    tkrCmdCode = 0x11;
    for (uint8 tkrFPGAaddress=0; tkrFPGAaddress<numTkrBrds; ++tkrFPGAaddress) {
        for (uint8 chipAddress=0; chipAddress<MAX_TKR_ASIC; ++chipAddress) {
            dataBytes[0] = chipAddress;
            dataBytes[1] = tkrConfig[tkrFPGAaddress][chipAddress].threshDAC;
            sendTrackerCmd(tkrFPGAaddress, tkrCmdCode, 2, dataBytes, TKR_ECHO_DATA);
        }
    }
    
    // Next load all of the data masks. Use wild card for the chip if all channels are to be enabled, as is the usual case
    tkrCmdCode = 0x13;
    for (uint8 tkrFPGAaddress=0; tkrFPGAaddress<numTkrBrds; ++tkrFPGAaddress) {
        dataBytes[0] = 0x1F;
        for (int i=0; i<8; ++i) dataBytes[1+i] = 0xFF;
        sendTrackerCmd(tkrFPGAaddress, tkrCmdCode, 9, dataBytes, TKR_ECHO_DATA);
        for (uint8 chipAddress=0; chipAddress<MAX_TKR_ASIC; ++chipAddress) {
            dataBytes[0] = chipAddress;
            bool allOn = true;
            for (int i=0; i<8; ++i) {
                dataBytes[1+i] = tkrConfig[tkrFPGAaddress][chipAddress].datMask[i];
                if (dataBytes[1+i] != 0xFF) allOn = false;
            }
            if (!allOn) sendTrackerCmd(tkrFPGAaddress, tkrCmdCode, 9, dataBytes, TKR_ECHO_DATA);
        }
    }    
    
    // Next load all of the trigger masks. Use a wild card to cover the typical case of all enabled.
    tkrCmdCode = 0x14;
    for (uint8 tkrFPGAaddress=0; tkrFPGAaddress<numTkrBrds; ++tkrFPGAaddress) {
        dataBytes[0] = 0x1F;
        for (int i=0; i<8; ++i) dataBytes[1+i] = 0xFF;
        sendTrackerCmd(tkrFPGAaddress, tkrCmdCode, 9, dataBytes, TKR_ECHO_DATA);
        for (uint8 chipAddress=0; chipAddress<MAX_TKR_ASIC; ++chipAddress) {
            dataBytes[0] = chipAddress;
            bool allOn = true;
            for (int i=0; i<8; ++i) {
                dataBytes[1+i] = tkrConfig[tkrFPGAaddress][chipAddress].trgMask[i];
                if (dataBytes[1+i] != 0xFF) allOn = false;
            }
            if (!allOn) sendTrackerCmd(tkrFPGAaddress, tkrCmdCode, 9, dataBytes, TKR_ECHO_DATA);
        }
    }    
    
    nDataReady = 0;   // To prevent the last echo from being sent out from the PSOC
}

// Reset all of the Tracker board FPGAs (soft reset) and the ASICs (soft reset)
// Unfortunately, the ASIC reset destroys the configuration, so the ASICs have to be reconfigured.
void resetAllTrackerLogic() {
    for (uint8 brd=0; brd<numTkrBrds; ++brd) {
        tkrCmdCode = 0x04;
        sendSimpleTrackerCmd(brd, tkrCmdCode);         
    }
    tkrCmdCode = 0x0C;   // ASIC soft reset
    cmdData[0] = 0x1F;   // All chips selected
    sendTrackerCmd(0x00, tkrCmdCode, 1, cmdData, TKR_ECHO_DATA);
    configureASICs();
}

// Read the ASIC configuration register 
void readASICconfig(uint8 FPGA, uint8 chip) {
    tkrCmdCode = 0x22; // Config read command code 
    sendTrackerCmd(FPGA, tkrCmdCode, 1, &chip, 0);
}

// Should be called after startup to make the specifiedTracker board FPGA go through a calibration sequence to center
// the timing for communication, to minimize bit errors.
void calibrateInputTiming(uint8 FPGA) {
    sendSimpleTrackerCmd(FPGA, 0x81);      // Tell the input circuits to self calibrate
    CyDelay(1);
    for (uint8 chip=0; chip<12; ++chip) {  // Each ASIC communication path to the FPGA gets calibrated in turn
        for (uint8 i=0; i<5; ++i) {        // Read the configuration register several times to provide data transitions for calibration
            readASICconfig(FPGA, chip);
            nDataReady = 0;          // Throw away the resulting data so it doesn't get out to the world
        }
    }
    CyDelay(2);
    sendSimpleTrackerCmd(FPGA, 0x82);      // Tell the input circuit to set its delay to the calibrated value
}

// Calibrate the communication timing for all of the Tracker boards
void calibrateAllInputTiming() {
    for (int brd=0; brd<numTkrBrds; ++brd) {
        calibrateInputTiming(brd);
    }
}

// Copy TOF information from the buffer into which the DMA writes
void copyTOF_DMA(char which, bool cleanUp) {
    if (which != 'B') {
        for (int i=0; i<nTOF_DMA_samples; ++i) {
            if (tofA_sampleArray[i] == 0) continue;
            tofA.shiftReg[tofA.ptr] = tofA_sampleArray[i];
            tofA_sampleArray[i] = 0;
            tofA.clkCnt[tofA.ptr] = tofA_clkArray[i] ;
            tofA.filled[tofA.ptr] = true;
            tofA.ptr++;
            if (tofA.ptr >= TOFMAX_EVT) tofA.ptr = 0;
        }
        if (cleanUp) {  // Grab any events that may be stuck in the shift register FIFO. This normally doesn't find anything
                        // as long as the DMA was started up with the FIFO empty, but we check anyway, just in case.
            while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) {
                uint32 AT = ShiftReg_A_ReadData();
                tofA.shiftReg[tofA.ptr] = AT;
                tofA.clkCnt[tofA.ptr] = Cntr8_Timer_ReadCount();  // Note: this timer value may be off by now
                tofA.filled[tofA.ptr] = true;
                tofA.ptr++;
                if (tofA.ptr >= TOFMAX_EVT) tofA.ptr = 0;
            }
        }
    }
    if (which != 'A') {
        for (int i=0; i<nTOF_DMA_samples; ++i) {
            if (tofB_sampleArray[i] == 0) continue;
            tofB.shiftReg[tofB.ptr] = tofB_sampleArray[i];
            tofB_sampleArray[i] = 0;
            tofB.clkCnt[tofB.ptr] = tofB_clkArray[i];
            tofB.filled[tofB.ptr] = true;
            tofB.ptr++;
            if (tofB.ptr >= TOFMAX_EVT) tofB.ptr = 0;
        }
        if (cleanUp) {  // Grab any events that may be stuck in the shift register FIFO
            while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) {
                uint32 BT = ShiftReg_B_ReadData();
                tofB.shiftReg[tofB.ptr] = BT;
                tofB.clkCnt[tofB.ptr] = Cntr8_Timer_ReadCount();  // Note: this timer value may be off by now
                tofB.filled[tofB.ptr] = true;
                tofB.ptr++;
                if (tofB.ptr >= TOFMAX_EVT) tofB.ptr = 0;
            }
        }
    }
}

// Functions for loading and reading the configuration of the TOF chip via SPI, either 4-bit or 8-bit.
// Before calling, first the write or read command must be sent to the chip.
// Each call writes or reads a single byte.
void writeTOFdata(uint8 dataByte) {
    if (SPIM_DATA_WIDTH == 4) {
        uint8 nibH = (dataByte>>4) & 0x0F;
        uint8 nibL = dataByte & 0x0F;
        SPIM_WriteTxData(nibH);
        SPIM_WriteTxData(nibL);
    } else {
        SPIM_WriteTxData(dataByte);
    }
}

// Function to read TOF chip register settings via SPI in case the RX interrupt is not used.
// Note that TOF timing data flows out over LVDS lines directly into the PSOC
uint8 readTOFdata() {
    uint8 dataByte;
    if (SPIM_DATA_WIDTH == 4) {
        while (SPIM_GetRxBufferSize() == 0) writeTOFdata(0x00);
        uint8 nibH = SPIM_ReadRxData();
        while (SPIM_GetRxBufferSize() == 0) writeTOFdata(0x00);
        uint8 nibL = SPIM_ReadRxData();
        dataByte = ((nibH<<4) & 0xF0) | (nibL & 0x0F); 
    } else {
        writeTOFdata(0x00);  // Needed in order to produce 8 SCLK cycles
        while (SPIM_GetRxBufferSize() == 0);
        dataByte = SPIM_ReadRxData();
    }
    return dataByte;
}

// Get the current count from one of the channel singles-rate counters
uint32 getChCount(int cntr) {
    uint32 count = 0;
    switch (cntr) {
        case 0:
          count = ch1Count*255 + Cntr8_V1_1_ReadCount();
          break;
        case 1:
          count = ch2Count*255 + Cntr8_V1_2_ReadCount();
          break;       
        case 2:
          count = ch3Count*255 + Cntr8_V1_3_ReadCount();
          break;
        case 3:
          count = ch4Count*255 + Cntr8_V1_4_ReadCount();
          break;
        case 4:
          count = ch5Count*255 + Cntr8_V1_5_ReadCount();
          break;
    }
    return count;
}

// Move TOF data out of the DMA buffers each time the maximum number of DMA TDs is used up
CY_ISR(isrTOFnrqA) {
    copyTOF_DMA('A', false);
}
CY_ISR(isrTOFnrqB) {
    copyTOF_DMA('B', false);
}

// Read out the shift register when a TOF stop event arrives for channel A
// This is only used when DMA of the TOF data is not employed.
CY_ISR(Store_A) {
    if (ShiftReg_A_GetIntStatus() == ShiftReg_A_STORE) {
        while (ShiftReg_A_GetFIFOStatus(ShiftReg_A_OUT_FIFO) != ShiftReg_A_RET_FIFO_EMPTY) {
            uint32 AT = ShiftReg_A_ReadData();
            tofA.shiftReg[tofA.ptr] = AT;
            tofA.clkCnt[tofA.ptr] = Cntr8_Timer_ReadCount();
            tofA.filled[tofA.ptr] = true;
            tofA.ptr++;
            if (tofA.ptr >= TOFMAX_EVT) tofA.ptr = 0;
            if (outputTOF) {    // Send data directly to the PC for this special debugging mode
                uint8 oReg[7];
                oReg[0] = 0xAA;
                oReg[1] = (AT & 0x0000FF00)>>8;
                oReg[2] =  AT & 0x000000FF;
                oReg[3] = (AT & 0xFF000000)>>24;
                oReg[4] = (AT & 0x00FF0000)>>16;            
                uint16 clk16 = (uint16)time();
                oReg[5] = (uint8)((clk16 & 0xFF00)>>8);
                oReg[6] = (uint8)(clk16 & 0x00FF);   
                if (USBUART_GetConfiguration() != 0u) {
                    while(!USBUART_CDCIsReady());
                    USBUART_PutData(oReg,7);
                }
            }
        }
    }
}

// Read out the shift register when a TOF stop event arrives for channel B
// This is only used when DMA of the TOF data is not employed.
CY_ISR(Store_B) {
    if (ShiftReg_B_GetIntStatus() == ShiftReg_B_STORE) { 
        while (ShiftReg_B_GetFIFOStatus(ShiftReg_B_OUT_FIFO) != ShiftReg_B_RET_FIFO_EMPTY) {
            uint32 BT = ShiftReg_B_ReadData();
            tofB.shiftReg[tofB.ptr] = BT;
            tofB.clkCnt[tofB.ptr] = Cntr8_Timer_ReadCount();
            tofB.filled[tofB.ptr] = true;
            tofB.ptr++;
            if (tofB.ptr >= TOFMAX_EVT) tofB.ptr = 0;
            if (outputTOF) {  // Send data directly to the PC for this special debugging mode
                uint8 oReg[7];
                oReg[0] = 0xBB;
                oReg[1] = (BT & 0x0000FF00)>>8;
                oReg[2] =  BT & 0x000000FF;
                oReg[3] = (BT & 0xFF000000)>>24;
                oReg[4] = (BT & 0x00FF0000)>>16;
                uint16 clk16 = (uint16)time();
                oReg[5] = (uint8)((clk16 & 0xFF00)>>8);
                oReg[6] = (uint8)(clk16 & 0x00FF);  
                if (USBUART_GetConfiguration() != 0u) {
                    while(!USBUART_CDCIsReady());
                    USBUART_PutData(oReg,7);
                }
            }
        }
    }
}

// Turn off an LED once the interval timer has timed out (this is just to make the LED stay on long enough to be visible)
CY_ISR(intTimer) {
    Timer_1_ReadStatusRegister();
    Pin_LED_TKR_Write(0);
    Pin_LED_DAT_Write(0);
    Timer_1_Stop();
}

// Increment the internal clock count every second, and also make the LED blink
CY_ISR(clk200) {       // Interrupt every second (200 ticks of the 5ms period clock)
    isr_GO_Disable();  // Don't allow a GO to interrupt while incrementing this counter
    clkCnt += 200;     // Increment the clock counter used for time stamps
    isr_GO_Enable();
    uint8 status = Pin_LED1_Read();
    status = ~status;
    Pin_LED1_Write(status);
}

// Interrupts to keep count of PMT singles rates. These interrupt every time the 8-bit hardware counter turns over.
CY_ISR(isrCh1)
{
    ch1Count++;
}

CY_ISR(isrCh2)
{
    ch2Count++;
}

CY_ISR(isrCh3)
{
    ch3Count++;
}

CY_ISR(isrCh4)
{
    ch4Count++;
}

CY_ISR(isrCh5)
{
    ch5Count++;
}

// Receive and store commands from the Main PSOC via the UART, using a circular FIFO buffer
CY_ISR(isrUART) {
    while (UART_CMD_ReadRxStatus() & UART_CMD_RX_STS_FIFO_NOTEMPTY) {
        uint16 theByte = UART_CMD_GetByte();
        if ((theByte & 0xDF00) != 0) {
            uint8 code = (uint8)((theByte & 0xDF00)>>8);
            addError(ERR_UART_CMD, code, (uint8)theByte);
        }
        cmdFIFO[fifoWritePtr] = (uint8)theByte;
        fifoWritePtr = WRAPINC(fifoWritePtr, MX_FIFO);
        if (fifoWritePtr == fifoReadPtr) {
            fifoReadPtr = WRAPINC(fifoReadPtr, MX_FIFO);  // Throw out the oldest byte
            addErrorOnce(ERR_FIFO_OVERFLOW, byte32(clkCnt, 0), byte32(clkCnt, 1));
        }
    }
}

// Receive data from the Tracker over the UART using a circular FIFO buffer
CY_ISR(isrTkrUART) {
    if (tkrWritePtr != tkrReadPtr) { // Ignore the interrupt if the software buffer is full
        while (UART_TKR_ReadRxStatus() & UART_TKR_RX_STS_FIFO_NOTEMPTY) {
            uint16 theByte = UART_TKR_GetByte();
            if ((theByte & 0xDF00) != 0) {
                uint8 code = (uint8)((theByte & 0xDF00)>>8);
                addError(ERR_UART_TKR, code, (uint8)theByte);
            }
            tkrBuf[tkrWritePtr] = (uint8)theByte;
            if (tkrReadPtr < 0) tkrReadPtr = tkrWritePtr;
            if (tkrWritePtr == MAX_TKR - 1) tkrWritePtr = 0;
            else tkrWritePtr++;
            if (tkrWritePtr == tkrReadPtr) {   // FIFO overflow condition, very bad!
                addError(ERR_TKR_BUFFER_OVERFLOW, byte32(cntGO, 0), byte32(cntGO, 1));
            }
        }
    } else {  // Throwing away the data here will probably cause the DAQ to fail. . .
        while (UART_TKR_ReadRxStatus() & UART_TKR_RX_STS_FIFO_NOTEMPTY) UART_TKR_GetByte();
    }
}

// GO signal (system trigger). Start the full event readout.
// This ISR has highest priority
CY_ISR(isrGO) {
    // The trigger should always be enabled for this to be called, but we could check just in case. . .
    //if (!isTriggerEnabled()) addError(ERR_TRG_NOT_ENABLED,byte32(cntGO,2),byte32(cntGO,1));

    trgStatus = Status_Reg_Trg_Read();     // Save this status for the eventual event readout
    triggered = true;                      // Signal that an event is ready to read out
    timeStamp = time();                    // Save for the event readout 
    timeStamp8 = Cntr8_Timer_ReadCount();  // Save for the TOF event analysis and readout
    cntGO1save = cntGO1;                   // Save for the event readout
    // Disable the trigger until the event readout has been completed
    Control_Reg_Trg_Write(0);  //triggerEnable(false);  avoid function call

    // At this point execution returns to its normal flow, allowing other interrupts. The remainder of the
    // event readout process is done in main(), in the infinite for loop.
}

// Interrupt to count triggers that occur when the trigger is disabled
CY_ISR(isrGO1) {
    cntGO1++;     
}

// External signal to force a software reset of this PSOC
CY_ISR(isrRST) {
    CySoftwareReset();
}

// Turn on the LED light on the double RJ45 connector that indicates that data are being sent out. The light
// will turn off automatically after sufficient time has passed to make it visible.
void dataLED(bool on) {
    if (on) {
        Pin_LED_DAT_Write(1);
    } else {
        Timer_1_Start();
    }
}

// Set the time delay following the signal edge detector, to prevent retriggering on noise
void setSettlingWindow(uint8 dt) {
    //TrigWindow_V1_1_Count7_1_WritePeriod(dt);
    TrigWindow_V1_2_Count7_1_WritePeriod(dt);
    TrigWindow_V1_3_Count7_1_WritePeriod(dt);
    TrigWindow_V1_4_Count7_1_WritePeriod(dt);
    TrigWindow_V1_5_Count7_1_WritePeriod(dt);
}

void makeEvent() {
    cntGO++;  // The event number counter

    // Stop acquiring TOF hits until the trigger is re-enabled.
    TOFenable(false);
    if (TOF_DMA) {  
        // Copy the TOF information from where the DMA dumped it 
        copyTOF_DMA('t', true);
    } 
    
    timeDate = RTC_1_ReadTime();
    triggered = false;

    // Read the digitized PMT data after waiting for the digitizers to finish
    uint t0 = time();
    uint8 evtStatus = Status_Reg_M_Read();
    if (!(evtStatus & 0x08)) {
        int InterruptState = CyEnterCriticalSection(); 
        while (!(Status_Reg_M_Read() & 0x08)) {   // Wait here for the done signal
            if (time() - t0 > 10 || t0 > time()) {
                addError(ERR_PMT_DAQ_TIMEOUT, (uint8)cntGO, (uint8)(cntGO >> 8));
                break;
            }
        }
        CyExitCriticalSection(InterruptState);
    }
    // Read out the 5 SAR ADCs one at a time
    uint16 adcArray[5];
    adcArray[0] = 0;
    adcArray[1] = 0;
    adcArray[2] = 0;
    adcArray[3] = 0;
    adcArray[4] = 0;
    int dummy = 0;   // Variable to do some useless calculation on, just to delay the CPU a little bit
    for (int ch=0; ch<5; ++ch) {
        set_ADC_SSN(SSN_SAR[ch]);
        int InterruptState = CyEnterCriticalSection();
        Control_Reg_1_Write(0x01);
        if (ADCsoftReset) {            // This reset operation will occur only on the first ADC read operation after power-on
            if (ch == 7) dummy = dummy + 3; // Delay to stop the read operation between 2nd and 8th SCLK cycles, for soft reset
            else dummy = dummy + 4;         // This delay results in the CS going down at the start of the 5th SCLK cycle.
            set_ADC_SSN(0);                   
            adcArray[ch] = 4095;            // /rubbish DAC reading for the first event
        } else {
            while ((Status_Reg_M_Read() & 0x20) == 0);
            adcArray[ch] = ShiftReg_ADC_ReadRegValue();
        }
        CyExitCriticalSection(InterruptState);
    }
    set_ADC_SSN(SSN_None);
    if (dummy > 0) ADCsoftReset = false;
    
    // Check that a tracker trigger was received and whether data are ready
    // This check generally works the first try and can maybe be removed in the long run.
    uint8 tkrDataReady = 0;
    uint8 nTry =0;
    if (readTracker) {
        while (tkrDataReady != TKR_DATA_READY) {
            tkrCmdCode = 0x57;   // Command to check whether Tkr data are ready
            sendTrackerCmd(0x00, tkrCmdCode, 0x00, cmdData, TKR_HOUSE_DATA);
            if (nTkrHouseKeeping > 0) {
                nTkrHouseKeeping = 0;    // Keep the housekeeping data from being sent out to the world
                if (tkrHouseKeeping[0] == TKR_DATA_READY) {
                    tkrDataReady = TKR_DATA_READY;    // Yes, an event is ready
                    break;
                } else if (tkrHouseKeeping[0] == TKR_DATA_NOT_READY) {
                    tkrDataReady = TKR_DATA_NOT_READY;    // No, an event is not ready
                } else {
                    addError(ERR_TKR_BAD_STATUS, tkrHouseKeeping[0], nTry);
                }
            } else {
                addError(ERR_TKR_BAD_STATUS, tkrDataReady, nTry);
            }
            nTry++;
            if (nTry > 9) {
                addError(ERR_TKR_BAD_STATUS, tkrDataReady, nTry+1);
                break;
            }
        }        
        uint8 rc = 0xDF;
        if (tkrDataReady == TKR_DATA_READY) {
            // Start the read of the Tracker data by sending a read-event command
            tkrCmdCode = 0x01;
            cmdData[0] = 0x00;
            while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {
                addErrorOnce(ERR_TKR_FIFO_NOT_EMPTY, tkrCmdCode, tkrDataReady);
            }
            rc = sendTrackerCmd(0x00, tkrCmdCode, 0x01, cmdData, TKR_EVT_DATA);    
        } 
        if (rc != 0) {
            addErrorOnce(ERR_GET_TKR_EVENT, rc, byte32(cntGO, 0));
            UART_TKR_ClearTxBuffer();
            UART_TKR_ClearRxBuffer(); 
            resetAllTrackerLogic();
            makeDummyTkrEvent(0, 0, 0, 6);
            numTkrResets++;
        }
    } else {
        makeDummyTkrEvent(0, 0, 0, 6);
    }

    // Search for nearly coincident TOF data. Note that each TOF chip channel operates asynchronously w.r.t. the
    // instrument trigger, so we have to correlate the two channels with each other and with the event
    // by looking at the course timing information.
    uint8 timeStamp8m1;
    if (timeStamp8 == 0) timeStamp8m1 = 199;
    else timeStamp8m1 = timeStamp8 - 1; 
    
    int nI=0;
    uint8 idx[TOFMAX_EVT];
    for (int i=0; i<TOFMAX_EVT; ++i) {           // Make a list of TOF hits in channel A
        int iptr = tofA.ptr - i - 1;             // Work backwards in time, starting with the most recent measurement
        if (iptr < 0) iptr = iptr + TOFMAX_EVT;  // Wrap around the circular buffer
        if (!tofA.filled[iptr]) continue;        // Use only entries filled since the previous readout
        if (timeStamp8 == tofA.clkCnt[iptr] || timeStamp8 == tofA.clkCnt[iptr]+1) {
            idx[nI] = iptr;                      // Only look at entries within two 5ms clock periods of the event time stamp
            ++nI;
        }
    }
    uint16 aCLK = 65535;
    uint16 bCLK = 65535;
    uint16 aTOF = 65535;
    uint16 bTOF = 65535;
    int16 dtmin = 32767;
    int nJ=0;
    for (int j=0; j<TOFMAX_EVT; ++j) {           // Loop over the TOF hits in channel B
        int jptr = tofB.ptr - j - 1;             // Work backwards in time, starting with the most recent measurement
        if (jptr < 0) jptr = jptr + TOFMAX_EVT;  // Wrap around the circular buffer
        if (!tofB.filled[jptr]) continue;        // Use only entries filled since the previous readout
        // Look only at entries filled within two 5 ms clock periods of the event time stamp
        if (!(tofB.clkCnt[jptr] == timeStamp8 || tofB.clkCnt[jptr] == timeStamp8m1)) continue;
        uint32 BT = tofB.shiftReg[jptr];
        uint16 stopB = (uint16)(BT & 0x0000FFFF);          // Stop time for channel B
        uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);     // Reference clock for channel B
        int32 timej = refB*8333 + stopB;                     // Full time for channel B in 10 picosecond units
        ++nJ;
        for (int i=0; i<nI; ++i) {                          // Loop over the channel A hits
            int iptr = idx[i];
            if (minTdif(tofA.clkCnt[iptr], tofB.clkCnt[jptr]) > 1) continue; // Two channels must be within +- 1 clock period
            uint32 AT = tofA.shiftReg[iptr];
            uint16 stopA = (uint16)(AT & 0x0000FFFF);       // Stop time for channel A
            uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);  // Reference clock for channel A
            int32 timei = refA*8333 + stopA;                  // Full time for channel A in 10 picosecond units
            // Here we try to handle cases in which a reference clock rolled over
            int16 dt;
            if (refA >= 60001 && refB == 0) {
                dt = (int16)((timej + 500000000) - timei);
            } else if (refB >= 60001 && refA == 0) {
                dt = (int16)(timej - (timei + 500000000)); 
            } else {
                dt = (int16)(timej - timei);
            }
            if (abs(dt) < abs(dtmin)) { // Keep the smallest time difference of all combinations
                dtmin = dt;
                aCLK = tofA.clkCnt[iptr];  // Save the clock and reference counts for debugging
                bCLK = tofB.clkCnt[jptr];
                aTOF = refA;
                bTOF = refB;
            }
        }
    }
    
    // Build the event by filling the output buffer according to the output format.
    // Pack the time and date information into a 4-byte unsigned integer
    uint32 timeWord = ((uint32)timeDate->Year - 2000) << 26;
    timeWord = timeWord | ((uint32)timeDate->Month << 22);
    timeWord = timeWord | ((uint32)timeDate->DayOfMonth << 17);
    timeWord = timeWord | ((uint32)timeDate->Hour << 12);
    timeWord = timeWord | ((uint32)timeDate->Min << 6);
    timeWord = timeWord | ((uint32)timeDate->Sec); 
    //
    // Start the event with a 4-byte header (spells ZERO) in ASCII
    dataOut[0] = 0x5A;
    dataOut[1] = 0x45;
    dataOut[2] = 0x52;
    dataOut[3] = 0x4F;
    dataOut[4] = byte16(runNumber, 0);
    dataOut[5] = byte16(runNumber, 1);
    dataOut[6] = byte32(cntGO, 0);     // Event number
    dataOut[7] = byte32(cntGO, 1);
    dataOut[8] = byte32(cntGO, 2);
    dataOut[9] = byte32(cntGO, 3);
    dataOut[10] = byte32(timeStamp, 0); // Time stamp
    dataOut[11] = byte32(timeStamp, 1);
    dataOut[12] = byte32(timeStamp, 2);
    dataOut[13] = byte32(timeStamp, 3);
    dataOut[14] = byte32(cntGO1save, 0);   // Trigger count
    dataOut[15] = byte32(cntGO1save, 1);
    dataOut[16] = byte32(cntGO1save, 2);
    dataOut[17] = byte32(cntGO1save, 3);
    dataOut[18] = byte32(timeWord, 0); // Time and date
    dataOut[19] = byte32(timeWord, 1);
    dataOut[20] = byte32(timeWord, 2);
    dataOut[21] = byte32(timeWord, 3);
    dataOut[22] = trgStatus;
    uint16 T1mV = adcArray[2]; 
    uint16 T2mV = adcArray[4]; 
    uint16 T3mV = adcArray[1]; 
    uint16 T4mV = adcArray[3]; 
    uint16 GmV =  adcArray[0]; 
    dataOut[23] = byte16(T1mV, 0);   // T1
    dataOut[24] = byte16(T1mV, 1);
    dataOut[25] = byte16(T2mV, 0);   // T2
    dataOut[26] = byte16(T2mV, 1);
    dataOut[27] = byte16(T3mV, 0);   // T3
    dataOut[28] = byte16(T3mV, 1);
    dataOut[29] = byte16(T4mV, 0);   // T4
    dataOut[30] = byte16(T4mV, 1);
    dataOut[31] = byte16(GmV, 0);    // G
    dataOut[32] = byte16(GmV, 1);
    dataOut[33] = byte16(dtmin, 0);  // TOF
    dataOut[34] = byte16(dtmin, 1);
    dataOut[35] = byte16(tkrData.triggerCount, 0);
    dataOut[36] = byte16(tkrData.triggerCount, 1);
    dataOut[37] = tkrData.cmdCount;
    dataOut[38] = (tkrData.trgPattern & 0xC0) | (evtStatus & 0x37);
    if (debugTOF) {  // Extra TOF information for debugging
        dataOut[39] = nI;   // Number of TOF readouts since the last trigger
        dataOut[40] = nJ; 
        dataOut[41] = byte16(aTOF,0);    // TOF chip reference clock 
        dataOut[42] = byte16(aTOF,1);
        dataOut[43] = byte16(bTOF,0);
        dataOut[44] = byte16(bTOF,1);
        dataOut[45] = byte16(aCLK,0);    // Internal clock at time of TOF event
        dataOut[46] = byte16(aCLK,1);
        dataOut[47] = byte16(bCLK,0);
        dataOut[48] = byte16(bCLK,1);
        dataOut[49] = tkrData.nTkrBoards;
        nDataReady = 50;
    } else {
        dataOut[39] = tkrData.nTkrBoards;
        nDataReady = 40;
    }
    if (doCRCcheck) {  // Check whether the hitslist CRCs match what the TKR calculated.
        for (int brd=0; brd<tkrData.nTkrBoards; ++brd) {
            if (!checkCRC(tkrData.boardHits[brd].nBytes, tkrData.boardHits[brd].hitList)) {
                    addError(ERR_BAD_CRC, brd, (uint8)cntGO);
                }
        }
    }
    for (int brd=0; brd<tkrData.nTkrBoards; ++brd) {
        // Min. # bytes needed: board address, hitlist length, hitlist, 4-byte trailer
        if (nDataReady >= MAX_DATA_OUT - (5 + tkrData.boardHits[brd].nBytes)) {
            // There is no room for this tracker board's data, but see if we can fit in an empty dummy readout
            if (nDataReady < MAX_DATA_OUT - 10) {
                //dataOut[nDataReady++] = brd;
                dataOut[nDataReady++] = 5;       // Number of bytes in the (empty) board hit list
                dataOut[nDataReady++] = 0xE7;    // Identifier byte for the hit list
                dataOut[nDataReady++] = brd;     // Board FPGA address (normally also the layer number)
                dataOut[nDataReady++] = 0;       // Event tag plus error flag, all set to zero
                dataOut[nDataReady++] = 0x09;    // Number of chips reporting (0) plus first 4 bits of CRC   
                dataOut[nDataReady++] = 0x30;    // 2 more CRC bits, set to 0, followed by 11, followed by 0 to byte boundary
                continue;
            }
            if (debugTOF) {        // For a truncated event, enter the number of boards that did read out.
                dataOut[49] = brd;
            } else {
                dataOut[39] = brd;
            }
            addError(ERR_EVT_TOO_BIG, dataOut[6], dataOut[10]);
            break;  // We're really out of space. The event will be truncated.
        }
        if (tkrData.boardHits[brd].hitList == NULL) {  // In case the heap memory ran out, enter a dummy layer packet. . .
            dataOut[nDataReady++] = 5;
            dataOut[nDataReady++] = 0xE7;
            dataOut[nDataReady++] = brd;
            dataOut[nDataReady++] = 1;
            dataOut[nDataReady++] = 0x07;   // A bad CRC will flag this error
            dataOut[nDataReady++] = 0x30;
        } else {
            dataOut[nDataReady++] = tkrData.boardHits[brd].nBytes;
            for (int b=0; b<tkrData.boardHits[brd].nBytes; ++b) {
                dataOut[nDataReady++] = tkrData.boardHits[brd].hitList[b];
            }
            free(tkrData.boardHits[brd].hitList);
            tkrData.boardHits[brd].nBytes = 0;
        }
        tkrData.boardHits[brd].nBytes = 0;  // Zero this out to facilitate debugging
    }
    tkrData.nTkrBoards = 0;  // Zero this out to facilitate debugging
    
    // Four byte trailer, spells FINI in ASCII
    dataOut[nDataReady++] = 0x46;
    dataOut[nDataReady++] = 0x49;
    dataOut[nDataReady++] = 0x4E;
    dataOut[nDataReady++] = 0x49;
    eventDataReady = true;
    for (int j=0; j<TOFMAX_EVT; ++j) {
        tofA.filled[j] = false;
        tofB.filled[j] = false;
    }
    tofA.ptr = 0;
    tofB.ptr = 0;           
    tkrData.nTkrBoards = 0;
    ch1CtrSave = Cntr8_V1_1_ReadCount();
    ch2CtrSave = Cntr8_V1_2_ReadCount();
    ch3CtrSave = Cntr8_V1_3_ReadCount();
    ch4CtrSave = Cntr8_V1_4_ReadCount();
    ch5CtrSave = Cntr8_V1_5_ReadCount();
    ch1CountSave = ch1Count;
    ch2CountSave = ch2Count;
    ch3CountSave = ch3Count;
    ch4CountSave = ch4Count;
    ch5CountSave = ch5Count;
} // end of subroutine makeEvent

void tkrRateMonitor(bool *waitingTkrRateCnt) {
    uint32 diff;
    if (*waitingTkrRateCnt) {
        if (time() > tkrClkCntStart) {
            diff = time() - tkrClkCntStart;
        } else {
            diff = 0xffffffff - (tkrClkCntStart - time() - 1);                       
        }     
        if (diff >= 220) {  // Allow 1 second for the Tracker to finish its count
            *waitingTkrRateCnt = false;
            bool trgStat = isTriggerEnabled();
            if (trgStat) triggerEnable(false);
            bool tkrErr = false;
            uint16 theRate[MAX_TKR_BOARDS];
            for (int brd=0; brd<numTkrBrds; ++brd) {
                theRate[brd] = 0;
                uint8 cmdData[1];
                sendTrackerCmd(brd, 0x6D, 0, cmdData, TKR_HOUSE_DATA);  // Get counts from tracker
                if (nTkrHouseKeeping == 0) {
                    addError(ERR_MISSING_HOUSEKEEPING,brd,0x5D);
                    tkrErr = true;
                } else {
                    theRate[brd] = (tkrHouseKeeping[0]<<8 & 0xff00) | tkrHouseKeeping[1];
                }
                nTkrHouseKeeping = 0;
            }
            if (!tkrErr) {
                if (nTkrMonSamples == tkrMonitorNumAvg) {
                    for (int brd=0; brd<numTkrBrds; ++ brd) {
                        tkrMonitorRates[brd] = tkrMonitorSums[brd]/nTkrMonSamples;
                        tkrMonitorSums[brd] = theRate[brd];
                    }
                    nTkrMonSamples = 1;
                } else {
                    for (int brd=0; brd<numTkrBrds; ++ brd) {
                        tkrMonitorSums[brd] += theRate[brd];
                    }
                    nTkrMonSamples++;
                }
                tkrClkAtStart = time();
            }
            if (trgStat) triggerEnable(true);
        }
    } else {
        if (time() > tkrClkAtStart) {
            diff = time() - tkrClkAtStart;
        } else {
            diff = 0xffffffff - (tkrClkAtStart - time() - 1);                       
        }
        if (diff >= tkrMonitorInterval*200) {
            // Send a command to the tracker to accumulate trigger counts for 1 second
            bool trgStat = isTriggerEnabled();
            if (trgStat) triggerEnable(false);
            sendSimpleTrackerCmd(0, 0x6C);
            tkrClkCntStart = time();
            *waitingTkrRateCnt = true;
            if (trgStat) triggerEnable(true);
        }
    }
} // end of subroutine trkRateMonitor

// Monitoring of PMT singles rates
void pmtRateMonitor(bool *waitingPmtRateCnt) {
    uint32 diff;
    if (*waitingPmtRateCnt) {
        uint32 now = time();
        if (now > pmtClkCntStart) {
            diff = now - pmtClkCntStart;
        } else {
            diff = 0xffffffff - (pmtClkCntStart - now - 1);
        }     
        if (diff >= 200*pmtDeltaT) {  // Allow at least 10 seconds, to get good precision
            *waitingPmtRateCnt = false;
            int InterruptState = CyEnterCriticalSection();
            now = time();
            if (now > pmtClkCntStart) {
                pmtMonitorTime = (uint16)(now - pmtClkCntStart);
            } else {
                pmtMonitorTime = (uint16)(0xffffffff - (pmtClkCntStart - now - 1));                       
            }  
            for (int cntr=0; cntr<MAX_PMT_CHANNELS; ++cntr) {
                pmtMonitorSums[cntr] = (uint16)(getChCount(cntr) - pmtCntInit[cntr]);
            }
            pmtClkAtStart = now;
            CyExitCriticalSection(InterruptState);
        }
    } else {
        uint32 now = time();
        if (now > pmtClkAtStart) {
            diff = now - pmtClkAtStart;
        } else {
            diff = 0xffffffff - (pmtClkAtStart - now - 1);                       
        }
        if (diff >= 200*pmtMonitorInterval) {
            int InterruptState = CyEnterCriticalSection();
            pmtClkCntStart = time();
            *waitingPmtRateCnt = true;
            for (int cntr=0; cntr<MAX_PMT_CHANNELS; ++cntr) {
                pmtCntInit[cntr] = getChCount(cntr);
            }
            CyExitCriticalSection(InterruptState);
        }
    }
} // end of subroutine pmtRateMonitor

// Data goes out by USBUART, for bench testing, or by SPI to the main PSOC
// Format: 3 byte aligned packeckets with a 3 byte header (0xDC00FF) and 3 byte EOR (0xFF00FF)       
uint8 sendEvtData(uint8 nDataBytes, uint8 dataPacket[], uint8 command, uint8 cmdData[]) {
    uint8 Padding[2];
    Padding[0] = '\x01';
    Padding[1] = '\x02';
    if (outputMode != USBUART_OUTPUT) {
        if (Pin_Busy_Read()) return 0;   // Don't send anything if the Main PSOC isn't ready to receive
    }
    if (nDataReady > 0) {    // Send out a command echo only if there are also data to send
        dataLED(true);
        if (!cmdDone) { // Output is event data, not a command response
            if (debugTOF) dataPacket[4] = 0xDB;
            else dataPacket[4] = 0xDD;
            nDataBytes = 0;   // Set to zero just in case a value is still hanging around in this variable
        } else {              // Output directly responding to a command
            dataPacket[4] = command;               
        }
        dataPacket[3] = nDataReady + nDataBytes;
        uint16 nPadding = 3 - (nDataBytes + nDataReady)%3;
        if (nPadding == 3) nPadding = 0;
        dataPacket[5] = nDataBytes;
        // Header data packet:
        // 0xDC
        // 0x00
        // 0xFF
        // data record length
        // command echo
        // number command data bytes
        if (outputMode != USBUART_OUTPUT) set_SPI_SSN(SSN_Main, false);
        if (outputMode == USBUART_OUTPUT) {  // Output the header
            if (USBUART_GetConfiguration() != 0u) {
                while(USBUART_CDCIsReady() == 0u);
                USBUART_PutData(dataPacket, 6);  
            }
        } else {
            for (int i=0; i<6; ++i) {
                SPIM_WriteTxData(dataPacket[i]);
            }
        }
        if (nDataBytes > 0) {
            if (outputMode == USBUART_OUTPUT) {  // Output the command data echo
                if (USBUART_GetConfiguration() != 0u) {
                    while(!USBUART_CDCIsReady());
                    USBUART_PutData(cmdData, nDataBytes);
                }
            } else {
                for (int i=0; i<nDataBytes; ++i) {
                    SPIM_WriteTxData(cmdData[i]);
                }
            }
        }
        if (outputMode == USBUART_OUTPUT) {    // output the data 
            if (USBUART_GetConfiguration() != 0u) {
                uint16 bytesRemaining = nDataReady;
                const uint16 mxSend = 64;
                int offset = 0;
                while (bytesRemaining > 0) {
                    if (USBUART_CDCIsReady()) {
                        if (bytesRemaining > mxSend) {
                            USBUART_PutData(&dataOut[offset], mxSend);
                            offset += mxSend;
                            bytesRemaining -= mxSend;
                        } else {
                            USBUART_PutData(&dataOut[offset], bytesRemaining); 
                            bytesRemaining = 0;
                        }
                    }
                }
                if (nPadding > 0) {
                    while(!USBUART_CDCIsReady());
                    USBUART_PutData(Padding, nPadding);
                }
                while(!USBUART_CDCIsReady());
                USBUART_PutData(&dataPacket[6], 3);  
            }
        } else {         
            for (int i=0; i<nDataReady; ++i) {
                SPIM_WriteTxData(dataOut[i]);
            }
            for (int i=0; i<nPadding; ++i) {
                SPIM_WriteTxData(Padding[i]);
            }
            for (int i=6; i<9; ++i) {
                SPIM_WriteTxData(dataPacket[i]);
            }
        }

        nDataReady = 0;
        if (eventDataReady) {   // re-enable the trigger after event data has been output
            int readoutTime = time() - timeStamp;
            readTimeAvg += readoutTime;
            nReadAvg++;
            if (!endingRun) {
                triggerEnable(true);
                TOFenable(true);
            }
            eventDataReady = false;
        }
        if (cmdDone) {  // The command is completely finished once the echo and data have gone out
            nDataBytes = 0;
            awaitingCommand = true;
            cmdDone = false;
        }
        dataLED(false);
    } else {            // Don't send an echo if the command doesn't result in data. Command is finished.
        awaitingCommand = true;
        cmdDone = false;
    }
    return nDataBytes;
} // end of the sendEvtData subroutine

// Interpret and act on commands received from USB-UART or the Main PSOC
void interpretCommand(uint8 tofConfig[]) {
    uint16 DACsetting12;
    uint32 tStart;
    uint8 DACaddress = 0;
    uint16 thrSetting;
    uint8 nCalClusters;
    uint8 fpgaAddress;
    uint8 chipAddress;   
    uint8 dataBytes[9];
    int rc;
    // If the trigger is enabled, ignore all commands besides disable trigger, 
    // so that nothing can interrupt the readout.
    if (command == '\x44' || !isTriggerEnabled()) {
        switch (command) { // Interpret all of the commands via this switch
            case '\x01':         // Load a threshold DAC setting
                switch (cmdData[0]) {
                    case 0x05: 
                        thrSetting = (uint16)cmdData[1];
                        thrSetting = (thrSetting<<8) | (uint16)cmdData[2];
                        rc = loadDAC(I2C_Address_DAC_Ch5, thrSetting);
                        if (rc != 0) {
                            addError(ERR_DAC_LOAD, rc, I2C_Address_DAC_Ch5);
                        }
                        break;
                    case 0x01:
                        VDAC8_Ch1_SetValue(cmdData[1]);
                        thrDACsettings[0] = cmdData[1];
                        break;
                    case 0x02:
                        VDAC8_Ch2_SetValue(cmdData[1]);
                        thrDACsettings[1] = cmdData[1];
                        break;
                    case 0x03:
                        VDAC8_Ch3_SetValue(cmdData[1]);
                        thrDACsettings[2] = cmdData[1];
                        break;
                    case 0x04:
                        VDAC8_Ch4_SetValue(cmdData[1]);
                        thrDACsettings[3] = cmdData[1];
                        break;
                }
                break;
            case '\x02':         // Get a threshold DAC setting
                if (cmdData[0] == 0x05) {
                    nDataReady = 2;
                    rc = readDAC(I2C_Address_DAC_Ch5, &DACsetting12);
                    if (rc != 0) {
                        DACsetting12 = 0;
                        addError(ERR_DAC_READ, rc, DACaddress);
                    }
                    dataOut[0] = (uint8)((DACsetting12 & 0xFF00)>>8);
                    dataOut[1] = (uint8)(DACsetting12 & 0x00FF);
                } else if (cmdData[0] < 5) {
                    nDataReady = 1;
                    dataOut[0] = thrDACsettings[cmdData[0]-1];
                } else {
                    nDataReady = 1;
                    dataOut[0] = 0;
                }
                break;
            case '\x03':         // Read back all of the accumulated error codes
                if (nErrors == 0) {
                    nDataReady = 3;
                    dataOut[0] = 0x00;
                    dataOut[1] = 0xEE;
                    dataOut[2] = 0xFF;
                    break;
                }
                nDataReady = nErrors*3;
                for (int i=0; i<nErrors; ++i) {
                    dataOut[i*3] = errors[i].errorCode;
                    dataOut[i*3 + 1] = errors[i].value0;
                    dataOut[i*3 + 2] = errors[i].value1;                                    
                }
                nErrors = 0;
                break;
            case '\x04':        // Load the TOF DACs
                if (cmdData[0] == 1) {
                    DACaddress = I2C_Address_TOF_DAC1;                                    
                } else if (cmdData[0] == 2) {                                    
                    DACaddress = I2C_Address_TOF_DAC2;
                } else break;
                uint16 thrSetting = (uint16)cmdData[1];                                
                thrSetting = (thrSetting<<8) | (uint16)cmdData[2];
                rc = loadDAC(DACaddress, thrSetting);
                if (rc != 0) {
                    addError(ERR_TOF_DAC_LOAD, rc, DACaddress);
                }
                break;
            case '\x05':        // Read the TOF DAC settings                               
                if (cmdData[0] == 1) {
                    DACaddress = I2C_Address_TOF_DAC1;
                } else if (cmdData[0] == 2) {
                    DACaddress = I2C_Address_TOF_DAC2;
                } else break;
                rc = readDAC(DACaddress, &DACsetting12);
                if (rc != 0) {
                    DACsetting12 = 0;
                    addError(ERR_TOF_DAC_READ, rc, DACaddress);                                         
                }
                
                nDataReady = 2;
                dataOut[0] = (uint8)((DACsetting12 & 0xFF00)>>8);
                dataOut[1] = (uint8)(DACsetting12 & 0x00FF);
                break;
            case '\x06':        // Turn LED on or off, for communication test
                if (cmdData[0] == 1) {
                    LED2_OnOff(true);
                } else {
                    LED2_OnOff(false);
                }
                break;
            case '\x07':        // Return the version number
                nDataReady = 2;
                dataOut[0] = MAJOR_VERSION;
                dataOut[1] = MINOR_VERSION;
                break;
            case '\x10':        // Send an arbitrary command to the tracker
                tkrCmdCode = cmdData[1];
                // Ignore commands that are supposed to be internal to the tracker,
                // to avoid confusing the tracker logic.
                if (tkrCmdCode == 0x52 || tkrCmdCode == 0x53) break;
                sendTrackerCmd(cmdData[0], cmdData[1], cmdData[2], &cmdData[3], 0);
                break;
            case '\x54':        // Load the ASIC configuration registers
                tkrCmdCode = 0x12;
                chipAddress = 0x1F;
                fpgaAddress = 0x00;
                tkrConfigReg[0] = cmdData[0];
                tkrConfigReg[1] = cmdData[1];
                tkrConfigReg[2] = cmdData[2];
                dataBytes[0] = chipAddress;
                dataBytes[1] = tkrConfigReg[0];
                dataBytes[2] = tkrConfigReg[1];
                dataBytes[3] = tkrConfigReg[2];
                sendTrackerCmd(fpgaAddress, tkrCmdCode, 4, dataBytes, TKR_ECHO_DATA);
                break;
            case '\x55':        // Load an ASIC threshold DAC
                tkrCmdCode = 0x11;
                fpgaAddress = cmdData[0] & 0x07;
                chipAddress = cmdData[1] & 0x1F;
                if (chipAddress != 0x1F && chipAddress >= MAX_TKR_ASIC) break;
                thrSetting = cmdData[2];
                if (chipAddress == 0x1F) {  // Update the default settings in the PSOC RAM
                    for (int chip=0; chip<MAX_TKR_ASIC; ++chip) {
                        tkrConfig[fpgaAddress][chip].threshDAC = thrSetting;
                    }
                } else {
                    tkrConfig[fpgaAddress][chipAddress].threshDAC = thrSetting;
                }
                dataBytes[0] = chipAddress;
                dataBytes[1] = thrSetting;
                sendTrackerCmd(fpgaAddress, tkrCmdCode, 2, dataBytes, TKR_ECHO_DATA);
                break;
            case '\x41':        // Load a tracker ASIC mask register
                fpgaAddress = cmdData[0] & 0x07;
                chipAddress = cmdData[1] & 0x1F;
                if (chipAddress != 0x1F && chipAddress >= MAX_TKR_ASIC) break;
                uint8 regType = cmdData[2] & 0x03;
                uint8 fill = cmdData[3] & 0x01;
                nCalClusters = cmdData[4];
                if (nCalClusters > (nDataBytes - 5)/2) {
                    nCalClusters = (nDataBytes - 5)/2;
                }   
                int ptr = 5;
                uint64 mask = 0;
                for (int i=0; i<nCalClusters; ++i) {
                    uint64 mask0 = 0;
                    int nch = cmdData[ptr];
                    int ch0 = 64-nch-cmdData[ptr+1];
                    mask0 = mask0 + 1;
                    for (int j=1; j<nch; ++j) {
                        mask0 = mask0<<1;
                        mask0 = mask0 + 1;
                    }
                    mask0 = mask0<<ch0;
                    mask = mask | mask0;
                    ptr = ptr + 2;
                }
                if (fill) mask = ~mask;
                uint8 maskBytes[8];
                for (int j=0; j<8; ++j) {  // The TKR FPGA expects the most significant byte first
                    maskBytes[7-j] = (uint8)(mask & 0x00000000000000FF);
                    mask = mask>>8;
                }
                // Select the correct tracker command code for the register load and update the defaults in the PSOC RAM
                if (regType == CALMASK) { 
                    tkrCmdCode = '\x15';
                } else if (regType == DATAMASK) {
                    tkrCmdCode = '\x13';
                    if (chipAddress == 0x1F) {
                        for (int chip=0; chip<MAX_TKR_ASIC; ++chip) {
                            for (int i=0; i<8; ++i) {
                                tkrConfig[fpgaAddress][chip].datMask[i] = maskBytes[i];
                            }
                        }
                    } else {
                        for (int i=0; i<8; ++i) {
                            tkrConfig[fpgaAddress][chipAddress].datMask[i] = maskBytes[i];
                        }
                    }
                } else {
                    tkrCmdCode = '\x14';
                    if (chipAddress == 0x1F) {
                        for (int chip=0; chip<MAX_TKR_ASIC; ++chip) {
                            for (int i=0; i<8; ++i) {
                                tkrConfig[fpgaAddress][chip].trgMask[i] = maskBytes[i];
                            }
                        }
                    } else {
                        for (int i=0; i<8; ++i) {
                            tkrConfig[fpgaAddress][chipAddress].trgMask[i] = maskBytes[i];
                        }
                    }
                }

                dataBytes[0] = chipAddress;
                for (int j=0; j<8; ++j) {
                    dataBytes[1+j] = maskBytes[j];
                }
                sendTrackerCmd(fpgaAddress, tkrCmdCode, 9, dataBytes, TKR_ECHO_DATA);
                break;
            case '\x42':        // Start a tracker calibration sequence
                // First send a calibration strobe command
                tkrLED(true);
                tkrCmdCode = '\x02';
                while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {
                    addErrorOnce(ERR_TKR_FIFO_NOT_EMPTY, tkrCmdCode, 0x3F);
                }
                UART_TKR_WriteTxData('\x00');
                UART_TKR_WriteTxData(tkrCmdCode);
                UART_TKR_WriteTxData('\x03');
                UART_TKR_WriteTxData('\x1F');
                uint8 FPGA = cmdData[0];
                uint8 trgDelay = cmdData[1];
                uint8 trgTag = cmdData[2] & 0x03;
                uint8 byte2 = (trgDelay & 0x3f)<<2;
                byte2 = byte2 | trgTag;
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(byte2);
                while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                UART_TKR_WriteTxData(FPGA);
                // Wait around for up to a second for all the data to transmit
                tStart = time();
                while (!(UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_EMPTY)) {                                           
                    if (time() - tStart > 200) {
                        addError(ERR_TX_FAILED, tkrCmdCode, command);
                        tkrLED(false);
                        break;
                    }
                }    
                // Catch the trigger output and send back to the computer                              
                getTrackerBoardTriggerData(FPGA);
                tkrLED(false);
                break;
            case '\x43':   // Send a tracker read-event command for calibration events
                tkrCmdCode = 0x01;
                trgTag = (cmdData[0] & 0x03) | 0x04;
                sendTrackerCmd(0x00, tkrCmdCode, 1, &trgTag, TKR_EVT_DATA);
                
                // Then send the data out as a tracker-only event
                dataOut[0] = 0x5A;
                dataOut[1] = 0x45;
                dataOut[2] = 0x52;
                dataOut[3] = 0x4F;
                dataOut[4] = tkrData.nTkrBoards;
                nDataReady = 5;
                for (int brd=0; brd<tkrData.nTkrBoards; ++brd) {
                    if (nDataReady > MAX_DATA_OUT - (5 + tkrData.boardHits[brd].nBytes)) {
                        addError(ERR_EVT_TOO_BIG, dataOut[6], dataOut[10]);
                        break;
                    }
                    //dataOut[nDataReady++] = brd;
                    dataOut[nDataReady++] = tkrData.boardHits[brd].nBytes;
                    for (int b=0; b<tkrData.boardHits[brd].nBytes; ++b) {
                        dataOut[nDataReady++] = tkrData.boardHits[brd].hitList[b];
                    }
                    free(tkrData.boardHits[brd].hitList);
                    tkrData.boardHits[brd].nBytes = 0;
                }
                dataOut[nDataReady++] = 0x46;
                dataOut[nDataReady++] = 0x49;
                dataOut[nDataReady++] = 0x4E;
                dataOut[nDataReady++] = 0x49;
                tkrLED(false);
                break;
            case '\x0C':        // Reset the TOF chip
                set_SPI_SSN(SSN_TOF, true);
                writeTOFdata(powerOnRESET);
                set_SPI_SSN(SSN_None, false);
                break;
            case '\x0D':        // Modify TOF configuration (disable trigger first)
                if (cmdData[0] < TOFSIZE-1) {
                    tofConfig[cmdData[0]] = cmdData[1];  // Change only a single byte
                    set_SPI_SSN(SSN_TOF, true);
                    writeTOFdata(writeConfig);
                    for (int i=0; i<TOFSIZE; ++i) {
                        writeTOFdata(tofConfig[i]);
                    }
                    CyDelay(1);
                    set_SPI_SSN(SSN_None, false);
                }
                break;
            case '\x0E':        // Read the TOF IC configuration 
                set_SPI_SSN(SSN_TOF, true);
                SPIM_ClearRxBuffer();
                writeTOFdata(readConfig);
                readTOFdata();
                for (int bt=0; bt<TOFSIZE; ++bt) {
                    dataOut[bt] = readTOFdata();
                }
                nDataReady = TOFSIZE;
                set_SPI_SSN(SSN_None, false);
                break;
            case '\x20':        // Read bus voltages (positive only)
                readI2Creg(2, cmdData[0], INA226_BusV_Reg, dataOut);
                nDataReady = 2;
                break;
            case '\x21':        // Read currents (Note: bit 15 is a sign bit, 2's complement)
                readI2Creg(2, cmdData[0], INA226_ShuntV_Reg, dataOut);
                nDataReady = 2;
                break;
            case '\x22':        // Read the board temperature
                readI2Creg(2, I2C_Address_TMP100, TMP100_Temp_Reg, dataOut);
                nDataReady = 2;
                break;
            case '\x23':        // Read an RTC register
                readI2Creg(1, I2C_Address_RTC, cmdData[0], dataOut);
                nDataReady = 1;
                break;
            case '\x24':        // Write an RTC register
                loadI2Creg(I2C_Address_RTC , cmdData[0], cmdData[1]);
                break;
            case '\x26':       // Read a barometer register
                readI2Creg(1, I2C_Address_Barometer, cmdData[0], dataOut);
                nDataReady = 1;
                break;
            case '\x27':       // Load a barometer register
                loadI2Creg(I2C_Address_Barometer, cmdData[0], cmdData[1]);
                break;
            case '\x30':       // Set the output mode
                if (cmdData[0] == USBUART_OUTPUT || cmdData[0] == SPI_OUTPUT) {
                    outputMode = cmdData[0];
                }
                if (outputMode == SPI_OUTPUT) set_SPI_SSN(SSN_Main, true);
                break;
            case '\x31':       // Initialize the SPI interface
                SPIM_Init();
                SPIM_Enable();
                break;
            case '\x32':       // Send TOF info to USB-UART (temporary testing)
                outputTOF = true;                               
                break;
            case '\x3F':
                outputTOF = false;
                break;
            case '\x34':       // Get the number of TOF events stored
                nDataReady = 2;
                dataOut[0] = tofA.ptr;
                dataOut[1] = tofB.ptr;
                break;
            case '\x35':       // Read most recent TOF event from channel A or B (for testing)
                nDataReady = 9;
                int InterruptState = CyEnterCriticalSection();
                if (cmdData[0] == 0) {                                    
                    uint8 idx = tofA.ptr - 1;
                    if (idx < 0) idx = idx + TOFMAX_EVT;
                    if (tofA.filled[idx]) {
                        uint32 AT = tofA.shiftReg[idx];
                        uint16 stopA = (uint16)(AT & 0x0000FFFF);
                        uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);
                        dataOut[0] = (uint8)((refA & 0xFF00)>>8);
                        dataOut[1] = (uint8)(refA & 0x00FF);
                        dataOut[2] = 0;
                        dataOut[3] = (uint8)((stopA & 0xFF00)>>8);
                        dataOut[4] = (uint8)(stopA & 0x00FF);
                        dataOut[5] = 0;
                        dataOut[6] = (uint8)((tofA.clkCnt[idx] & 0xFF00)>>8);
                        dataOut[7] = (uint8)(tofA.clkCnt[idx] & 0x00FF);
                        dataOut[8] = tofA.ptr;
                        for (int j=0; j<TOFMAX_EVT; ++j) {
                            tofA.filled[j] = false;
                        }
                        tofA.ptr = 0;
                    } else {
                        for (int i=0; i<8; ++i) dataOut[i] = 0;
                        dataOut[8] = idx;
                    }
                } else {
                    uint8 idx = tofB.ptr - 1;
                    if (idx < 0) idx = idx + TOFMAX_EVT;
                    if (tofB.filled[idx]) {
                        uint32 BT = tofB.shiftReg[idx];
                        uint16 stopB = (uint16)(BT & 0x0000FFFF);
                        uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);
                        dataOut[0] = (uint8)((refB & 0xFF00)>>8);
                        dataOut[1] = (uint8)(refB & 0x00FF);
                        dataOut[2] = 0;
                        dataOut[3] = (uint8)((stopB & 0xFF00)>>8);
                        dataOut[4] = (uint8)(stopB & 0x00FF);
                        dataOut[5] = 0;
                        dataOut[6] = (uint8)((tofB.clkCnt[idx] & 0xFF00)>>8);
                        dataOut[7] = (uint8)(tofB.clkCnt[idx] & 0x00FF);
                        dataOut[8] = tofB.ptr;
                        for (int j=0; j<TOFMAX_EVT; ++j) {
                            tofB.filled[j] = false;
                        }
                        tofB.ptr = 0;
                    } else {
                        for (int i=0; i<8; ++i) dataOut[i] = 0;
                        dataOut[8] = idx;
                        for (int j=0; j<TOFMAX_EVT; ++j) {
                            tofB.filled[j] = false;
                            tofA.filled[j] = false;
                        }
                        tofB.ptr = 0;
                        tofA.ptr = 0;
                    }
                }
                CyExitCriticalSection(InterruptState);
                break; 
            case '\x36':     // Set a trigger mask
                if (cmdData[0] == 1) {
                    setTriggerMask('e', cmdData[1]);
                } else if (cmdData[0] == 2) {
                    setTriggerMask('p', cmdData[1]);
                }
                break;
            case '\x37':    // Read a channel counter
                nDataReady = 5;
                switch (cmdData[0]) {
                    case 0x01:
                        dataOut[4] = Cntr8_V1_1_ReadCount();
                        dataOut[3] = byte32(ch1Count, 3);
                        dataOut[2] = byte32(ch1Count, 2);
                        dataOut[1] = byte32(ch1Count, 1);
                        dataOut[0] = byte32(ch1Count, 0);
                        break;
                    case 0x02:
                        dataOut[4] = Cntr8_V1_2_ReadCount();
                        dataOut[3] = byte32(ch2Count, 3);
                        dataOut[2] = byte32(ch2Count, 2);
                        dataOut[1] = byte32(ch2Count, 1);
                        dataOut[0] = byte32(ch2Count, 0);
                        break;
                    case 0x03:
                        dataOut[4] = Cntr8_V1_3_ReadCount();
                        dataOut[3] = byte32(ch3Count, 3);
                        dataOut[2] = byte32(ch3Count, 2);
                        dataOut[1] = byte32(ch3Count, 1);
                        dataOut[0] = byte32(ch3Count, 0);
                        break;
                    case 0x04:
                        dataOut[4] = Cntr8_V1_4_ReadCount();
                        dataOut[3] = byte32(ch4Count, 3);
                        dataOut[2] = byte32(ch4Count, 2);
                        dataOut[1] = byte32(ch4Count, 1);
                        dataOut[0] = byte32(ch4Count, 0);
                        break;
                    case 0x05:
                        dataOut[4] = Cntr8_V1_5_ReadCount();
                        dataOut[3] = byte32(ch5Count, 3);
                        dataOut[2] = byte32(ch5Count, 2);
                        dataOut[1] = byte32(ch5Count, 1);
                        dataOut[0] = byte32(ch5Count, 0);
                        break;
                }
                break;
            case '\x38':   // Reset the logic and counters, after reading back 24 bits of the clock count
                nDataReady = 3;
                uint32 now = time();
                dataOut[0] = (uint8)((now & 0x00FF0000)>>16);
                dataOut[1] = (uint8)((now & 0x0000FF00)>>8);
                dataOut[2] = (uint8)(now & 0x000000FF);
                logicReset();
                break;
            case '\x39':   // Set trigger prescales
                if (cmdData[0] == 1) {
                    Cntr8_V1_TKR_WritePeriod(cmdData[1]);
                } else if (cmdData[0] == 2) {
                    Cntr8_V1_PMT_WritePeriod(cmdData[1]);
                }
                break;
            case '\x3A':   // Set the settling time allowed for PMT signals
                setSettlingWindow(cmdData[0]);
                break;
            case '\x3B':   // Enable or disable the trigger
                if (cmdData[0] == 1) {
                    triggerEnable(true);
                } else if (cmdData[0] == 0) {
                    triggerEnable(false);
                }
                break;
            case '\x44':  // End a run and send out the run summary
                triggerEnable(false);
                endingRun = true;
                endData[0] = byte32(cntGO1, 0);
                endData[1] = byte32(cntGO1, 1);
                endData[2] = byte32(cntGO1, 2);
                endData[3] = byte32(cntGO1, 3);
                endData[4] = byte32(cntGO, 0);
                endData[5] = byte32(cntGO, 1);
                endData[6] = byte32(cntGO, 2);
                endData[7] = byte32(cntGO, 3);
                break;
            case '\x50':  // Send counter information
                nDataReady = 6;
                dataOut[0] = byte16(cmdCountGLB, 0); 
                dataOut[1] = byte16(cmdCountGLB, 1); 
                dataOut[2] = byte16(cmdCount, 0);
                dataOut[3] = byte16(cmdCount, 1);
                dataOut[4] = nCmdTimeOut;
                dataOut[5] = numTkrResets;
                break;
            case '\x3C':  // Start a run
                InterruptState = CyEnterCriticalSection();
                for (int j=0; j<TOFMAX_EVT; ++j) {
                    tofA.filled[j] = false;
                    tofB.filled[j] = false;
                }
                readTimeAvg = 0;
                nReadAvg = 0;
                clkCnt = 0;
                tofA.ptr = 0;
                tofB.ptr = 0;
                ch1Count = 0;
                ch2Count = 0;
                ch3Count = 0;
                ch4Count = 0;
                ch5Count = 0;
                cntGO1 = 0;
                CyExitCriticalSection(InterruptState);
                runNumber = cmdData[0];
                runNumber = (runNumber<<8) | cmdData[1];
                readTracker = (cmdData[2] == 1);
                debugTOF = (cmdData[3] == 1);
                cntGO = 0;
                numTkrResets = 0;
                triggerEnable(true);
                Control_Reg_Pls_Write(PULSE_CNTR_RST);
                // Enable the tracker trigger
                if (readTracker) {
                    tkrCmdCode = 0x65;
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(0x00);    // Address byte
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(tkrCmdCode);    // Trigger enable
                    while (UART_TKR_ReadTxStatus() & UART_TKR_TX_STS_FIFO_FULL);
                    UART_TKR_WriteTxData(0x00);    // Number of data bytes
                    uint8 rc = getTrackerData(TKR_ECHO_DATA);   // Get the echo. Note that any delay put before this results in
                                                                // the first few bytes of the echo getting missed. Don't know why.
                    if (rc != 0) {
                        addError(ERR_TKR_TRG_ENABLE, dataOut[2], rc);;
                    }
                }
                TOFenable(true);
                nDataReady = 0;  // Don't send the tracker echo back to the UART
                nDataBytes = 0;  // Also, avoid sending an echo from this command
                cmdDone = false;
                awaitingCommand = true;
                endingRun = false;
                break;
            case '\x3D':  // Return trigger enable status
                nDataReady =1;
                if (isTriggerEnabled()) dataOut[0] = 1;
                else dataOut[0] = 0;
                break;
            case '\x3E':  // Return trigger mask register
                nDataReady = 1;
                uint8 reg = 0;
                if (cmdData[0] == 1) {
                    reg = getTriggerMask('e');
                } else if (cmdData[0] == 2) {
                    reg = getTriggerMask('p');
                }
                dataOut[0] = reg;
                break;
            case '\x33':    // Read a saved channel counter, from end of run
                nDataReady = 5;
                switch (cmdData[0]) {
                    case 0x01:
                        dataOut[4] = Cntr8_V1_1_ReadCount();
                        dataOut[3] = byte32(ch1CountSave, 3);
                        dataOut[2] = byte32(ch1CountSave, 2);
                        dataOut[1] = byte32(ch1CountSave, 1);
                        dataOut[0] = byte32(ch1CountSave, 0);
                        break;
                    case 0x02:
                        dataOut[4] = Cntr8_V1_2_ReadCount();
                        dataOut[3] = byte32(ch2CountSave, 3);
                        dataOut[2] = byte32(ch2CountSave, 2);
                        dataOut[1] = byte32(ch2CountSave, 1);
                        dataOut[0] = byte32(ch2CountSave, 0);
                        break;
                    case 0x03:
                        dataOut[4] = Cntr8_V1_3_ReadCount();
                        dataOut[3] = byte32(ch3CountSave, 3);
                        dataOut[2] = byte32(ch3CountSave, 2);
                        dataOut[1] = byte32(ch3CountSave, 1);
                        dataOut[0] = byte32(ch3CountSave, 0);
                        break;
                    case 0x04:
                        dataOut[4] = Cntr8_V1_4_ReadCount();
                        dataOut[3] = byte32(ch4CountSave, 3);
                        dataOut[2] = byte32(ch4CountSave, 2);
                        dataOut[1] = byte32(ch4CountSave, 1);
                        dataOut[0] = byte32(ch4CountSave, 0);
                        break;
                    case 0x05:
                        dataOut[4] = Cntr8_V1_5_ReadCount();
                        dataOut[3] = byte32(ch5CountSave, 3);
                        dataOut[2] = byte32(ch5CountSave, 2);
                        dataOut[1] = byte32(ch5CountSave, 1);
                        dataOut[0] = byte32(ch5CountSave, 0);
                        break;
                }
                break;
            case '\x40':  // Read all TOF data (for testing)
                nDataReady = 3;
                uint8 nA = 0;
                uint8 nB = 0;
                InterruptState = CyEnterCriticalSection();
                if (TOF_DMA) copyTOF_DMA('t', true);
                for (int i=0; i<TOFMAX_EVT; ++i) {
                    if (tofA.filled[i]) ++nA;
                    if (tofB.filled[i]) ++nB;
                }
                dataOut[2] = nTOF_DMA_samples;
                int maxTOFhit = MAX_DATA_OUT/12;
                if (nA > maxTOFhit || nB > maxTOFhit) {
                    dataOut[2] = 2;
                    if (nA > maxTOFhit) nA = maxTOFhit;
                    if (nB > maxTOFhit) nB = maxTOFhit;
                }
                dataOut[0] = nA;
                dataOut[1] = nB;
                int iptr = tofA.ptr;
                int jptr = tofB.ptr;
                uint8 cnt = 0;
                for (int i=0; i<TOFMAX_EVT; ++i) {
                    --iptr;
                    if (iptr < 0) iptr += TOFMAX_EVT;
                    if (!tofA.filled[iptr]) continue;                                    
                    uint32 AT = tofA.shiftReg[iptr];
                    uint16 stopA = (uint16)(AT & 0x0000FFFF);
                    uint16 refA = (uint16)((AT & 0xFFFF0000)>>16);
                    dataOut[nDataReady++] = byte16(refA,0);
                    dataOut[nDataReady++] = byte16(refA,1);
                    dataOut[nDataReady++] = byte16(stopA,0);
                    dataOut[nDataReady++] = byte16(stopA,1);
                    dataOut[nDataReady++] = byte16(tofA.clkCnt[iptr],0);
                    dataOut[nDataReady++] = byte16(tofA.clkCnt[iptr],1);
                    ++cnt;
                    if (cnt >= nA) break;
                }
                cnt = 0;
                for (int i=0; i<TOFMAX_EVT; ++i) {
                    --jptr;
                    if (jptr < 0) jptr += TOFMAX_EVT;
                    if (!tofB.filled[jptr]) continue;
                    uint32 BT = tofB.shiftReg[jptr];
                    uint16 stopB = (uint16)(BT & 0x0000FFFF);
                    uint16 refB = (uint16)((BT & 0xFFFF0000)>>16);
                    dataOut[nDataReady++] = byte16(refB,0);
                    dataOut[nDataReady++] = byte16(refB,1);
                    dataOut[nDataReady++] = byte16(stopB,0);
                    dataOut[nDataReady++] = byte16(stopB,1);
                    dataOut[nDataReady++] = byte16(tofB.clkCnt[jptr],0);
                    dataOut[nDataReady++] = byte16(tofB.clkCnt[jptr],1);
                    ++cnt;
                    if (cnt >= nB) break;
                }
                for (int j=0; j<TOFMAX_EVT; ++j) {
                    tofB.filled[j] = false;
                    tofA.filled[j] = false;
                }
                tofB.ptr = 0;
                tofA.ptr = 0;
                CyExitCriticalSection(InterruptState);
                break;
            case '\x45': // Set the time and date of the real-time-clock
                RTC_1_DisableInt();         
                timeDate = RTC_1_ReadTime();   // Set the pointer to some memory location, then overwrite
                timeDate->Sec = cmdData[0];
                timeDate->Min = cmdData[1];
                timeDate->Hour = cmdData[2];
                timeDate->DayOfWeek = cmdData[3];
                timeDate->DayOfMonth = cmdData[4];
                timeDate->DayOfYear = cmdData[6] + cmdData[5]*256;
                timeDate->Month = cmdData[7];
                timeDate->Year = cmdData[9] + cmdData[8]*256;
                RTC_1_WriteTime(timeDate);                   
                RTC_1_EnableInt();
                break;
            case '\x46': // get the time and date of the real-time-clock
                nDataReady = 10;
                timeDate = RTC_1_ReadTime();
                dataOut[0] = timeDate->Sec;
                dataOut[1] = timeDate->Min;
                dataOut[2] = timeDate->Hour;
                dataOut[3] = timeDate->DayOfWeek;
                dataOut[4] = timeDate->DayOfMonth;
                dataOut[5] = timeDate->DayOfYear/256;
                dataOut[6] = timeDate->DayOfYear%256;
                dataOut[7] = timeDate->Month;
                uint16 year = timeDate->Year;
                dataOut[8] = year/256;
                dataOut[9] = year%256;
                break;
            case '\x47': // Reset the tracker state machines and Tracker ASICs
                resetAllTrackerLogic();
                break;
            case '\x48': // Calibrate the input timing on one or every Tracker FPGA board
                if (cmdData[0] > 7) {
                    calibrateAllInputTiming();
                } else {
                    calibrateInputTiming(cmdData[0]);
                }
                break;
            case '\x49': // Read accumulated tracker layer rates
                dataOut[0] = 0x6D;
                dataOut[1] = numTkrBrds;                                
                for (int brd=0; brd<numTkrBrds; ++brd) {
                    dataOut[2 + brd*2] = byte16(tkrMonitorRates[brd],0);
                    dataOut[2 + brd*2 + 1] = byte16(tkrMonitorRates[brd],1);
                }
                nDataReady = 2*(1+numTkrBrds);
                break;
            case '\x4A': // Set up and initiate tracker rate monitoring
                if (cmdData[0] == 0 || cmdData[1] == 0) {
                    monitorTkrRates = false;
                    waitingTkrRateCnt = false;
                    break;
                }
                tkrMonitorInterval = cmdData[0];  // Number of seconds between monitoring events
                if (tkrMonitorInterval < 2) tkrMonitorInterval = 2;
                tkrMonitorNumAvg = cmdData[1];    // Number of successive measurements to average over
                nTkrMonSamples = 0;
                for (int brd=0; brd<numTkrBrds; ++brd) {
                    tkrMonitorSums[brd] = 0;
                }
                tkrClkAtStart = time();        
                monitorTkrRates = true;
                waitingTkrRateCnt = false;
                break;
            case '\x52': // Set up and initiate PMT singles rate monitoring
                if (cmdData[0] == 0 || cmdData[1] == 0) {
                    monitorPmtRates = false;
                    waitingPmtRateCnt = false;
                    break;
                }
                pmtDeltaT = cmdData[0];             // Number of seconds over which to accumulate counts
                pmtMonitorInterval = cmdData[1];    // Interval in seconds between successive measurements
                if (pmtMonitorInterval < 2*pmtDeltaT) pmtMonitorInterval = 2*pmtDeltaT;
                InterruptState = CyEnterCriticalSection();
                pmtClkCntStart = time();
                for (int cntr=0; cntr<MAX_PMT_CHANNELS; ++cntr) {
                    pmtCntInit[cntr] = getChCount(cntr);
                }
                CyExitCriticalSection(InterruptState);
                monitorPmtRates = true;
                waitingPmtRateCnt = true;
                break;
            case '\x53':  // Get the PMT counter rates
                dataOut[0] = byte16(pmtMonitorTime,0);
                dataOut[1] = byte16(pmtMonitorTime,1);
                for (int cntr=0; cntr<MAX_PMT_CHANNELS; ++cntr) {
                    dataOut[2+2*cntr] = byte16(pmtMonitorSums[cntr], 0);
                    dataOut[2+2*cntr+1] = byte16(pmtMonitorSums[cntr], 1);
                }
                nDataReady = 12;
                break;
            case '\x4B': // Set the delay between trigger and peak detector readout
                if (cmdData[0] < 127) setPeakDetResetWait(cmdData[0]);
                else addError(ERR_BAD_CMD_INPUT,command,cmdData[0]);
                break;
            case '\x4C': // Enable or disable the TOF data accumulation
                if (cmdData[0] == 1) {
                    for (int j=0; j<TOFMAX_EVT; ++j) {
                        tofA.filled[j] = false;
                        tofB.filled[j] = false;
                    }
                    tofB.ptr = 0;
                    tofA.ptr = 0;
                    TOFenable(true);
                } else {
                    TOFenable(false);
                }
                break;
            case '\x4D': // Select TOF acquisition mode
                if (cmdData[0] == 1) {
                    TOF_DMA = true;
                    isr_TOFnrqA_Enable();
                    isr_TOFnrqB_Enable();
                    ShiftReg_A_DisableInt();
                    ShiftReg_B_DisableInt();
                } else {
                    TOF_DMA = false;
                    isr_TOFnrqA_Disable();
                    isr_TOFnrqB_Disable();
                    ShiftReg_A_EnableInt();
                    ShiftReg_B_EnableInt();
                }
                break;
            case '\x4E': // Select whether to check the Tracker CRC
                if (cmdData[0] == 1) {
                    doCRCcheck = true;
                } else {
                    doCRCcheck = false;
                }
                break;
            case '\x4F': // Set the tracker trigger delay for PMT triggers
                Count7_TrgDly_WritePeriod(cmdData[0]);
                break;
            case '\x51': // Get the average event readout time
                nDataReady = 8;
                dataOut[0] = byte32(readTimeAvg, 0);
                dataOut[1] = byte32(readTimeAvg, 1);
                dataOut[2] = byte32(readTimeAvg, 2);
                dataOut[3] = byte32(readTimeAvg, 3);
                dataOut[4] = byte32(nReadAvg, 0);
                dataOut[5] = byte32(nReadAvg, 1);
                dataOut[6] = byte32(nReadAvg, 2);
                dataOut[7] = byte32(nReadAvg, 3);
                break;
            case '\x56': // Set up the Tracker ASIC configuration. Set up the # of layers and power on the ASICs first!
                if (cmdData[0] > 8 || cmdData[0] == 0) {
                    addError(ERR_TKR_NUM_BOARDS, cmdData[0], 0x77);
                    break;
                }
                tkrCmdCode = 0x0F;
                sendTrackerCmd(0x00, tkrCmdCode, 1, cmdData, TKR_ECHO_DATA);     // Set the number of layers to read out
                tkrCmdCode = 0x08;
                sendTrackerCmd(0x00, tkrCmdCode, 0, cmdData, TKR_ECHO_DATA);     // Turn on the ASIC power
                configureASICs();
                break;
        } // End of command switch
    } else { // Log an error if the user is sending spurious commands while the trigger is enabled
        addError(ERR_CMD_IGNORE, command, 0);
    }
} // end of interpretCommand subroutine

// Set up the Time-of-Flight DMA (to reduce the number of CPU interrupts if the TOF channels are noisy)
void tofDMAsetup() {
    TOF_DMA = true;
  
    // TOF DMA setup. Not used if TOF_DMA = false, but still in place.
    nTOF_DMA_samples = CyDmaTdFreeCount()/4 - 2;   // Maximize the number of TDs that we can use.
    if (nTOF_DMA_samples > TOF_DMA_MAX_NO_OF_SAMPLES) nTOF_DMA_samples = TOF_DMA_MAX_NO_OF_SAMPLES;
    
    // DMA Configuration for TOF shift register A 
    DMA_TOFA_Chan = DMA_TOFA_DmaInitialize(TOF_DMA_BYTES_PER_BURST, TOF_DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    CyDmaChDisable(DMA_TOFA_Chan);
    // Allocate a new transition descriptor for each sample
    for (int i=0; i<nTOF_DMA_samples; ++i) {
        tofA_sampleArray[i] = 0;
        DMA_TOFA_TD[2*i] = CyDmaTdAllocate();
        DMA_TOFA_TD[2*i+1] = CyDmaTdAllocate();
    }
    for (int i=0; i<nTOF_DMA_samples; ++i) {
        CyDmaTdSetConfiguration(DMA_TOFA_TD[2*i], TOF_DMA_BYTES_PER_BURST, DMA_TOFA_TD[2*i+1], 
                                                    CY_DMA_TD_AUTO_EXEC_NEXT | CY_DMA_TD_INC_SRC_ADR);
        if (i < nTOF_DMA_samples-1) {                
            CyDmaTdSetConfiguration(DMA_TOFA_TD[2*i+1], 1, DMA_TOFA_TD[2*i+2], 0);
        } else {
            CyDmaTdSetConfiguration(DMA_TOFA_TD[2*i+1], 1, DMA_TOFA_TD[0], DMA_TOFA__TD_TERMOUT_EN);
        }
        CyDmaTdSetAddress(DMA_TOFA_TD[2*i], LO16((uint32)ShiftReg_A_OUT_FIFO_VAL_LSB_PTR), LO16((uint32)&tofA_sampleArray[i]));
        CyDmaTdSetAddress(DMA_TOFA_TD[2*i+1], LO16((uint32)&Cntr8_Timer_Result_Reg), LO16((uint32)&tofA_clkArray[i]));
    }
    CyDmaChSetInitialTd(DMA_TOFA_Chan, DMA_TOFA_TD[0]);
    CyDmaChPriority(DMA_TOFA_Chan,2);
    CyDmaChRoundRobin(DMA_TOFA_Chan,1);
    
    // DMA Configuration for TOF shift register B 
    DMA_TOFB_Chan = DMA_TOFB_DmaInitialize(TOF_DMA_BYTES_PER_BURST, TOF_DMA_REQUEST_PER_BURST, 
                 HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    CyDmaChDisable(DMA_TOFB_Chan);
    for (int i=0; i<nTOF_DMA_samples; ++i) {
        tofB_sampleArray[i] = 0;
        DMA_TOFB_TD[2*i] = CyDmaTdAllocate();
        DMA_TOFB_TD[2*i+1] = CyDmaTdAllocate();
    }
    for (int i=0; i<nTOF_DMA_samples; ++i) {
        CyDmaTdSetConfiguration(DMA_TOFB_TD[2*i], TOF_DMA_BYTES_PER_BURST, DMA_TOFB_TD[2*i+1], 
                                                    CY_DMA_TD_AUTO_EXEC_NEXT | CY_DMA_TD_INC_SRC_ADR);
        if (i < nTOF_DMA_samples-1) {                
            CyDmaTdSetConfiguration(DMA_TOFB_TD[2*i+1], 1, DMA_TOFB_TD[2*i+2], 0);
        } else {
            CyDmaTdSetConfiguration(DMA_TOFB_TD[2*i+1], 1, DMA_TOFB_TD[0], DMA_TOFB__TD_TERMOUT_EN);
        }
        CyDmaTdSetAddress(DMA_TOFB_TD[2*i], LO16((uint32)ShiftReg_B_OUT_FIFO_VAL_LSB_PTR), LO16((uint32)&tofB_sampleArray[i]));
        CyDmaTdSetAddress(DMA_TOFB_TD[2*i+1], LO16((uint32)&Cntr8_Timer_Result_Reg), LO16((uint32)&tofB_clkArray[i]));
    }
    CyDmaChSetInitialTd(DMA_TOFB_Chan, DMA_TOFB_TD[0]); 
    CyDmaChPriority(DMA_TOFB_Chan,2);
    CyDmaChRoundRobin(DMA_TOFB_Chan,1);
} // end of tofDMAsetup subroutine

//   M      M      A      IIIII  N    N
//   M M  M M     A A       I    NN   N
//   M   M  M    A   A      I    N N  N
//   M      M   AAAAAAA     I    N  N N
//   M      M  A       A    I    N   NN
//   M      M A         A IIIII  N    N
//
// Event PSOC main program
int main(void)
{     
    // Load the default Tracker configuration from EEPROM
    EEPROM_1_Start();
    boardMAP[0] = 6; //2;   // C
    boardMAP[1] = 7;   // H
    boardMAP[2] = 1;   // B
    boardMAP[3] = 0;   // A
    boardMAP[4] = 6;   // G
    boardMAP[5] = 5;   // F
    boardMAP[6] = 8;   // I
    boardMAP[7] = 4;   // E
    uint16 base = MAX_TKR_PCB*MAX_TKR_ASIC*SIZEOF_EEPROM_ROW;
    int base2 = 8*12*16;
    if (base != base2) {
        base2 = base2 + 999;
    }
    for (int lyr=0; lyr<MAX_TKR_BOARDS; ++lyr) {
        int brd = boardMAP[lyr];
        for (int chip=0; chip<MAX_TKR_ASIC; ++chip) {
            for (int i=0; i<8; ++i) {
                tkrConfig[lyr][chip].datMask[i] = EEPROM_1_ReadByte((brd*MAX_TKR_ASIC + chip)*SIZEOF_EEPROM_ROW + i);
                tkrConfig[lyr][chip].trgMask[i] = EEPROM_1_ReadByte((brd*MAX_TKR_ASIC + chip)*SIZEOF_EEPROM_ROW + 8 + i);              
            }
            tkrConfig[lyr][chip].threshDAC = EEPROM_1_ReadByte(base + brd*SIZEOF_EEPROM_ROW + chip);
        }
    }
    for (int i=0; i<3; ++i) {
        tkrConfigReg[i] = EEPROM_1_ReadByte(base + MAX_TKR_PCB*SIZEOF_EEPROM_ROW + i);
    }
    
    outputMode = SPI_OUTPUT;  // Default mode for sending out data  
    doCRCcheck = false;
    triggered = false;
    tkrData.nTkrBoards = 0;
    tofA.ptr = 0;
    tofB.ptr = 0;
    readTimeAvg = 0;
    nReadAvg = 0;
    outputTOF = false;
    for (int i=0; i<TOFMAX_EVT; ++i) {
        tofA.filled[i] = false;
        tofB.filled[i] = false;
    }
    
    nDataReady = 0;
    clkCnt = 0;
    nTkrHouseKeeping = 0;
    readTracker = false;
    debugTOF = false;
    
    fifoWritePtr = 0;
    fifoReadPtr = 0;
    
    runNumber = 0;
    timeStamp = time();
    
    uint8 *buffer;        // Buffer for incoming commands
    uint8 USBUART_buf[BUFFER_LEN];
    
    uint8 code[256];  // ASCII code translation to hex nibbles
    for (int i=0; i<256; ++i) code[i] = 0;
    code[49] = 1;
    code[50] = 2;
    code[51] = 3;
    code[52] = 4;
    code[53] = 5;
    code[54] = 6;
    code[55] = 7;
    code[56] = 8;
    code[57] = 9;
    code[65] = 10;
    code[97] = 10;   // not case sensitive for ABCDEF
    code[66] = 11;
    code[98] = 11;
    code[67] = 12;
    code[99] = 12;
    code[68] = 13;
    code[100] = 13;
    code[69] = 14;
    code[101] = 14;
    code[70] = 15;
    code[102] = 15;
    
    // Buffer for output of a 3-byte data packet. Set the invariant parts of the header and trailer bytes.
    uint8 dataPacket[9];
    dataPacket[0] = VAR_HEAD;
    dataPacket[1] = '\x00';
    dataPacket[2] = '\xFF';
    dataPacket[6] = '\xFF';
    dataPacket[7] = '\x00';
    dataPacket[8] = '\xFF';
    
    set_SPI_SSN(SSN_None,true);
    
    // Turn all the LEDs off
    Pin_LED1_Write(0);
    Pin_LED2_Write(0);
    Pin_LED_TKR_Write(0);
    Pin_LED_DAT_Write(0);
    
    // General hardware logic reset (not including the tracker), and reset of counters
    logicReset();
    
    CyGlobalIntEnable; /* Enable global interrupts. */

    /* Initialize interrupts */
    isr_timer_StartEx(intTimer);
    isr_timer_Disable();
    isr_clk200_StartEx(clk200);
    isr_clk200_Disable();
    isr_Store_A_StartEx(Store_A);
    isr_Store_A_Disable();
    isr_Store_B_StartEx(Store_B);
    isr_Store_B_Disable();
    isr_TOFnrqA_StartEx(isrTOFnrqA);
    isr_TOFnrqA_Disable();
    isr_TOFnrqB_StartEx(isrTOFnrqB);
    isr_TOFnrqB_Disable();
    isr_Ch1_StartEx(isrCh1);
    isr_Ch1_Disable();
    isr_Ch2_StartEx(isrCh2);
    isr_Ch2_Disable();
    isr_Ch3_StartEx(isrCh3);
    isr_Ch3_Disable();
    isr_Ch4_StartEx(isrCh4);
    isr_Ch4_Disable();
    isr_Ch5_StartEx(isrCh5);
    isr_Ch5_Disable();
    isr_GO1_StartEx(isrGO1);
    isr_GO1_Disable();
    isr_GO_StartEx(isrGO);
    isr_GO_Disable();
    isr_UART_StartEx(isrUART);
    isr_UART_Disable();
    isr_rst_StartEx(isrRST);
    isr_rst_Disable();
    isr_TKR_StartEx(isrTkrUART);
    isr_TKR_Disable();
    
    tkrWritePtr = 0;    // Initialize the write pointer for the tracker UART RX FIFO
    tkrReadPtr = -1;    // Negative indicates that no data are available to read
    
    // Initialize pointers for the UART command buffer
    cmdReadPtr = 0;   
    cmdWritePtr = 0;
    for (uint i=0; i<MX_CMDS; ++i) cmd_buffer[i].nBytes = 0;
    
    /* Start up the various hardware components */
    I2C_2_Start();
    Count7_TrgDly_Start();
    Count7_TrgDly_WritePeriod(16);    // Default delay of the PMT trigger, in units of 83.3ns
    
    // Set up the counter used for timing. It counts a 200 Hz clock derived from the watch crystal, and every 200 counts
    // i.e. once each second, it interrupts the CPU, which then increments a 1 Hz count. The time() function adds the two
    // counts together to get a time tag that increments every 5 ms. Note that the main purpose of the 200 Hz clock is to
    // send a hardware reset to the time-of-flight chip every 5 ms, so that we know exactly when its counting starts.
    Cntr8_Timer_WritePeriod(200);
    
    // Counter for the delay time to wait before resetting the peak detectors.
    Count7_3_Start();
    // The peak detector output takes about 2us to settle down after its upward swing, so at 24MHz this should be at least 48 ticks to set
    // the time to start digitizing. This also affects the time to wait for the digitizers to finish (about 1 us needed) and the length
    // of time to hold the peak detectors in reset.
    setPeakDetResetWait(32);

    // TOF shift registers
    ShiftReg_A_Start();
    ShiftReg_B_Start();
    
    // Shift register for capturing the data from the external SAR ADCs
    ShiftReg_ADC_Start();
    
    // A single master SPI communicates with the TOF chip and the Main PSOC
    SPIM_Start();
    
    // SPI SSN codes for the external SAR ADCs
    SSN_SAR[0] = SSN_CH1;
    SSN_SAR[1] = SSN_CH2;
    SSN_SAR[2] = SSN_CH3;
    SSN_SAR[3] = SSN_CH4;
    SSN_SAR[4] = SSN_CH5;
    
    USBUART_Start(USBFS_DEVICE, USBUART_3V_OPERATION);
    
    // We use all 4 internal comparators for the PHA
    Comp_Ch1_Start();
    Comp_Ch2_Start();
    Comp_Ch3_Start();
    Comp_Ch4_Start();
    
    // Internal and external voltage DACs
    VDAC8_Ch1_Start();
    VDAC8_Ch1_SetValue(THRDEF);   // This is in DAC counts, 4 mV/bit
    VDAC8_Ch2_Start();
    VDAC8_Ch2_SetValue(THRDEF);
    VDAC8_Ch3_Start();
    VDAC8_Ch3_SetValue(THRDEF);
    VDAC8_Ch4_Start();
    VDAC8_Ch4_SetValue(THRDEF);

    loadDAC(I2C_Address_DAC_Ch5, 0x000F);
    loadDAC(I2C_Address_TOF_DAC1, 0x00FF);
    loadDAC(I2C_Address_TOF_DAC2, 0x00FF);
 
    UART_TKR_Start();   // UART for communication with the tracker
    UART_CMD_Start();   // Snail-paced UART for receiving commands from the Main PSOC

    // Start counters buried inside of the edge detectors for the trigger inputs
    //TrigWindow_V1_1_Count7_1_Start();
    TrigWindow_V1_2_Count7_1_Start();
    TrigWindow_V1_3_Count7_1_Start();
    TrigWindow_V1_4_Count7_1_Start();
    TrigWindow_V1_5_Count7_1_Start();
    setSettlingWindow(24);
    
    // Start the internal real-time-clock component
    RTC_1_Start();
    
    // Configure the i2c Real-Time-Clock if that bus extends to the event PSOC (normally not)
    // loadI2Creg(I2C_Address_RTC, 0x00, 0x59);
    // loadI2Creg(I2C_Address_RTC, 0x07, 0x80);         

    // Set up the DMA for the time-of-flight
    tofDMAsetup();
    
    // Counters for loading TOF shift registers. The periods are set in the schematic and should never change!
    Count7_1_Start();
    Count7_2_Start();
    
    // Default configuration of the TOF chip. The second byte should be 0x05 for stop events to be accepted.
    // That may not turned on here by default, but rather it may be turned on when the master trigger is enabled.
    // The reference clock is 12 MHz, which has a period of 83333 picoseconds. With 16 bits it will count up to
    // about 5.46 milliseconds. We reset it every 5 milliseconds, so it should only get up to a count of 60,000
    // which is EA60 in hex. We set the LSB by dividing the reference clock period by 8333, which is ~10ps.
    // Only 14 bit are needed, then, for the stop clocks, but 16 bits are read out. The maximum stop-clock count
    // should be 8333, or hex 208D.
    // Addr0: B5  Make active STOPA, STOPB, REFCLK, LVDS LCLK & LCLKOUT, Ref clk reset
    // Addr1: 05  Activate A and B top inputs; no channel combine; standard resolution
    // Addr2: 0C  Ref and Stop both set to 16 bits; single data rate, no common read, std FIFO
    // Addr3: 8D  Ref Clk divisions = 00208D = 8333 which sets the LSB to be 10 picoseconds with a 12 MHz reference clock
    // Addr4: 20  Ref Clk divisions
    // Addr5: 00  Ref Clk divisions
    // Addr6: 00  Normal LVDS operation; no test pattern
    // Addr7: 08  0ps LVDS adjustment
    // Addr8 through 15 are defaults
    // Addr16 00  Differential LVDS input
    uint8 tofConfig[TOFSIZE] = {0xB5, 0x05, 0x0C, 0x8D, 0x20, 0x00, 0x00, 0x08, 0xA1, 0x13, 0x00,
                                0x0A, 0xCC, 0xCC, 0xF1, 0x7D, 0x00};
    
    // Set up the configuration of the TOF chip AS6501:
    SPIM_ClearTxBuffer();
    SPIM_ClearRxBuffer();
    
    // Reset the TOF chip first
    set_SPI_SSN(SSN_TOF, true);
    writeTOFdata(powerOnRESET);
    CyDelay(1);
    
    // Set up the default AS6501 TOF configuration
    set_SPI_SSN(SSN_TOF, true);
    writeTOFdata(writeConfig);
    for (int i=0; i<TOFSIZE; ++i) {
        writeTOFdata(tofConfig[i]);
    }
    CyDelay(1);
    
    // Enable the TOF chip
    set_SPI_SSN(SSN_TOF, true);
    writeTOFdata(TOF_enable); 
    
    cmdCountGLB = 0;               // Count of all command packets received
    cmdCount = 0;                  // Count of all event PSOC commands received
    int dCnt = 0;                  // To count the number of data bytes received
    nCmdTimeOut = 0;
    numTkrResets = 0;
    const uint8 eventPSOCaddress = '\x08';
    
    // Set up the default trigger configuration
    Cntr8_V1_TKR_WritePeriod(255);    // Tracker trigger prescale
    Cntr8_V1_PMT_WritePeriod(255);    // PMT hadron trigger prescale
    setTriggerMask('e',0x01);
    setTriggerMask('p',0x05);

    // Set interrupt priorities, enable interrupts and configure TOF shift register interrupt signals
    // The communications ISRs can only be interrupted by the trigger GO, which is fast.
    isr_UART_SetPriority(5);  // This needs high priority to prevent the hardware FIFO from overfilling
    isr_UART_Enable();     
    isr_timer_SetPriority(7);   
    isr_timer_Enable();
    isr_clk200_SetPriority(7);
    isr_clk200_Enable(); 

    // Usually DMA would be used for the TOF, in which case these two ISRs get disabled
    isr_Store_A_SetPriority(5);
    ShiftReg_A_SetIntMode(ShiftReg_A_STORE_INT_EN);  // This can hang up indefinitely if the TOF chip is not set up properly
    ShiftReg_A_EnableInt();  
    isr_Store_B_SetPriority(5);
    ShiftReg_B_SetIntMode(ShiftReg_B_STORE_INT_EN);  // This can hang up indefinitely if the TOF chip is not set up properly 
    ShiftReg_B_EnableInt();

    // ISRs for the DMA, to move the acquired TOF events into the large buffer
    isr_TOFnrqA_SetPriority(5);
    isr_TOFnrqB_SetPriority(5);

    isr_Ch1_SetPriority(7);
    isr_Ch1_Enable();
    isr_Ch2_SetPriority(7);
    isr_Ch2_Enable();
    isr_Ch3_SetPriority(7);
    isr_Ch3_Enable();
    isr_Ch4_SetPriority(7);
    isr_Ch4_Enable();
    isr_Ch5_SetPriority(7);
    isr_Ch5_Enable();
    isr_GO1_SetPriority(7);
    isr_GO1_Enable();
    isr_GO_SetPriority(4);    // Processing a trigger request gets the highest priority
    isr_GO_Enable();
    isr_rst_SetPriority(3);   // Give this system reset highest priority, so it can interrupt any ISR
    isr_rst_Enable();
    isr_TKR_SetPriority(5);   // This needs high priority to prevent the hardware FIFO from overfilling
    isr_TKR_Enable();

    numTkrBrds = MAX_TKR_BOARDS;  // By default all Tracker boards are present.
    eventDataReady = false;
    awaitingCommand = true;   
    
    uint32 cmdStartTime = time();
    cmdDone = false;        // If true, A command has been fully received but data have not yet been sent back
    set_SPI_SSN(0, true);   // Deselect all SPI slaves
    triggerEnable(false);
    endingRun = false;

    for (int brd=0; brd<MAX_TKR_BOARDS; ++brd) {
        tkrMonitorRates[brd] = 0;
    }   
    monitorTkrRates = false;
    waitingTkrRateCnt = false;
    
    monitorPmtRates = false;
    waitingPmtRateCnt = false;
    
    ADCsoftReset=true;
    
    // Infinite loop, breaks only if there is a hardware or software reset, or power cycle
    for(;;)
    {
        if (USBUART_IsConfigurationChanged() != 0u) {
            /* Wait for USB-UART Device to enumerate */
            while (USBUART_GetConfiguration() == 0u);
            /* Enumeration is done, enable OUT endpoint to receive data from Host */
            USBUART_CDC_Init();
        }

        // Tracker rate monitoring
        if (monitorTkrRates) {
            tkrRateMonitor(&waitingTkrRateCnt);
        }
        
        // PMT singles rate monitoring
        if (monitorPmtRates) {
            pmtRateMonitor(&waitingPmtRateCnt);
        }
        
        if (triggered && !cmdDone && awaitingCommand) {    // Delay the data output if a command is in progress 
            makeEvent();
        }
        if (!triggered && endingRun) {
            endingRun = false;
            nDataReady = 8;
            for (int i=0; i<8; ++i) {
                dataOut[i] = endData[i];
            }
        }
        
        if (nDataReady > 0 || cmdDone) {   
            nDataBytes = sendEvtData(nDataBytes, dataPacket, command, cmdData);
        }
        
        // Time-out protection in case the expected data for a command are never sent.
        // The command buffers are completely flushed, hoping for a fresh start
        if (!awaitingCommand) {
            if (time() - cmdStartTime > TIMEOUT) {
                int InterruptState = CyEnterCriticalSection();
                awaitingCommand = true;
                cmdDone = false;
                nDataBytes = 0;     
                cmdWritePtr = 0;
                cmdReadPtr = 0;
                fifoWritePtr = 0;
                fifoReadPtr = 0;
                CyExitCriticalSection(InterruptState);
                nCmdTimeOut++;
                addError(ERR_CMD_TIMEOUT,command,dCnt);
            }
        }        
        
        // Parse the FIFO of commands from the Main PSOC, via UART. Identify commands
        // from the <CR><LF> characters, and move complete commands into the command buffer.
        int numBytes = ACTIVELEN(fifoReadPtr, fifoWritePtr, MX_FIFO);
        if (numBytes >= CMD_LENGTH) {   
            // Search backkwards for the <CR><LF> termination of the latest command
            int tmpRdPtr = WRAPDEC(fifoWritePtr, MX_FIFO);
            for (int i=0; i<numBytes+1-CMD_LENGTH; ++i) {
                if (cmdFIFO[tmpRdPtr] == LF && cmdFIFO[WRAPDEC(tmpRdPtr, MX_FIFO)] == CR) {
                    isr_UART_Disable();
                    fifoReadPtr = (tmpRdPtr + 1 - CMD_LENGTH + MX_FIFO) % MX_FIFO;
                    for (int j=0; j<CMD_LENGTH; ++j) {
                        cmd_buffer[cmdWritePtr].buf[j] = cmdFIFO[fifoReadPtr];
                        fifoReadPtr = WRAPINC(fifoReadPtr, MX_FIFO);
                    }
                    isr_UART_Enable();
                    cmd_buffer[cmdWritePtr].nBytes = CMD_LENGTH;
                    if (WRAPINC(cmdWritePtr, MX_CMDS) == cmdReadPtr) {
                        addError(ERR_CMD_BUF_OVERFLOW, byte32(clkCnt,0), byte32(clkCnt,1)); // This will almost certainly make a mess if it happens!
                    } else {
                        cmdWritePtr = WRAPINC(cmdWritePtr, MX_CMDS);  
                        cmd_buffer[cmdWritePtr].nBytes = 0;
                    }
                    break;
                }
                if (tmpRdPtr == fifoReadPtr) break;  // This check shouldn't be necessary
                tmpRdPtr = WRAPDEC(tmpRdPtr, MX_FIFO);
            }
        }

        // Get a 9-byte command input from the UART or USB-UART
        // The two should not be used at the same time (no reason for that, anyway)
        int count = 0;
        if (nDataReady == 0) { // Don't interpret a new command if data are still going out
            if (USBUART_GetConfiguration() != 0u) {    // Looking for a command from USB-UART
                buffer = USBUART_buf;
                if (USBUART_DataIsReady()) {
                    count = USBUART_GetAll(buffer);
                }
            }
            if (count == 0 && cmdReadPtr != cmdWritePtr) { // Looking for a command from UART 
                count = cmd_buffer[cmdReadPtr].nBytes;
                buffer = cmd_buffer[cmdReadPtr].buf;
                cmd_buffer[cmdReadPtr].nBytes = 0;
                cmdReadPtr = WRAPINC(cmdReadPtr, MX_CMDS);
            }
        }
        if (count == CMD_LENGTH) {  // We got a complete command string in triplicate. Accept it if 2 out of 3 agree.
            bool badCMD = false;
            for (int i=0; i<9; ++i) {   // Check that all 3 command copies are identical
                if (buffer[i] != buffer[i+9] || buffer[i] != buffer[i+18]) {
                    badCMD = true;
                    break;
                }
            }  
            if (badCMD) {
                badCMD = false;
                for (int i=0; i<9; ++i) {   // Check that the first two command copies are identical
                    if (buffer[i] != buffer[i+9]) {
                        badCMD = true;
                        break;
                    }
                }
                if (badCMD) {
                    badCMD = false;
                    for (int i=0; i<9; ++i) {   // Check that the first and third command copies are identical
                        if (buffer[i] != buffer[i+18]) {
                            badCMD = true;
                            break;
                        }
                    }
                    if (badCMD) {
                        badCMD = false;
                        for (int i=0; i<9; ++i) {   // Check that the last two command copies are identical
                            if (buffer[i+9] != buffer[i+18]) {
                                badCMD = true;
                                addError(ERR_BAD_CMD, code[buffer[i+9]], i);
                                break;
                            }
                        }
                        if (!badCMD) for (int i=0; i<9; ++i) buffer[i] = buffer[i+9];
                    }
                }
            }
            if (!badCMD) {
                if (buffer[0] != 'S' || buffer[8] != 'W') {
                    addError(ERR_BAD_CMD_FORMAT,buffer[0],buffer[8]);
                } else {
                    if (awaitingCommand) cmdCountGLB++;
                    uint8 nib3 = code[buffer[3]];
                    uint8 nib4 = code[buffer[4]];
                    uint8 addressByte = (nib3<<4) | nib4;
                    uint8 PSOCaddress = (addressByte & '\x3C')>>2;
                    if (PSOCaddress == eventPSOCaddress) {                    
                        uint8 nib1 = code[buffer[1]];  // No check on code. Illegal characters get translated to 0.
                        uint8 nib2 = code[buffer[2]];
                        uint8 dataByte = (nib1<<4) | nib2;
                        if (awaitingCommand) {         // This is the start of a new command
                            awaitingCommand = false;
                            cmdStartTime = time();
                            cmdCount++;
                            dCnt = 0;
                            nDataBytes = ((addressByte & '\xC0') >> 4) | (addressByte & '\x03');
                            command = dataByte;
                            if (nDataBytes == 0) cmdDone = true;
                        } else {                       // Receiving data from a command in progress
                            uint8 byteCnt = ((addressByte & '\xC0') >> 4) | (addressByte & '\x03');
                            if (byteCnt != 0) {
                                cmdData[byteCnt-1] = dataByte;
                            } else {
                                addError(ERR_BAD_BYTE, command, dCnt);
                                badCMD = true;
                            }
                            dCnt++;
                            if (dCnt == nDataBytes) cmdDone = true; 
                        }
                    }
                    if (cmdDone && badCMD) {
                        cmdDone = false;
                        awaitingCommand = true;  // Abort a command with a bad data byte
                        nDataBytes = 0;
                    }
                    if (cmdDone && !badCMD) {
                        interpretCommand(tofConfig);
                    }
                }
            } // End of command polling            
        }
        
        // Send out Tracker housekeeping data immediately after receiving it from the Tracker
        if (!isTriggerEnabled() && nTkrHouseKeeping>0) {
            nDataReady = nTkrHouseKeeping + 7;
            dataOut[0] = nDataReady;
            dataOut[1] = 0xC7;
            dataOut[2] = nTkrHouseKeeping;
            dataOut[3] = byte16(tkrCmdCount,0);
            dataOut[4] = byte16(tkrCmdCount,1);
            dataOut[5] = tkrHouseKeepingFPGA;
            dataOut[6] = tkrCmdCode;
            for (int i=0; i<nTkrHouseKeeping; ++i) {
                dataOut[6+i] = tkrHouseKeeping[i];
            }
            nTkrHouseKeeping = 0;
        } 
    }
}

/* [] END OF FILE */
