/*
 *  Arduino test code for Usb Max3421e using a X10 CM15A.
 *  Version 1a  -  10 Jan 2022   Copyright 2022 S. Cooper
 *
 *  You may redistribute and modify this documentation under the terms of the CERN OHL v.1.2.
 *  This documentation is distributed WITHOUT ANY EXPRESS OR IMPLIED WARRANTY, 
 *  INCLUDING OF MERCHANTABILITY, SATISFACTORY QUALITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *  Please see the CERN OHL v.1.2 for applicable conditions. (http://ohwr.org/cernohl).
 * 
 *  This pogram for study with 3.3V zeroMax3421e shield using arduino ZERO
 *  
 *  The program should work with an 5V Arduino UNO using a blue board usb shield 2.0 also.
 *  
 *  The funstion of the program is to show one way to read a usb CM15A device,
 *    using a MAX3421e usb chip in HOST mode. The CM15A needs to be plugged in
 *    before the program is started.
 *    
 *  The program is designed to not use any usb libraries.
 *  
 *  This code was tested with an Arduino UNO and Arduino ZERO
 *      usb sheilds used,  UNO shield uhs20s , ZERO shield zeroMax3421e.
 *         zeroMax3421e relased under CERN OHL ver1.2 files on github.
 *      Both shields use a MAX3421E usb chip, SS is on pin D10
 *  
 *  Source information for program:
 *    usb11.pdf usb version 1.1 spec
 *    USB in a NutShell
 *    AN3785.pdf from maxim for MAX3421E usb chip
 *    AN4000.pdf from maxim for MAX3420E odd usb chip
 *    www linuxha com/athome/common/cm15d/cm15d
 *    www linuxha com/USB/cm15a
 *    www linuxha com/athome/common/x10.xtc797.txt
 *    chome nerpa tech/mcu/lightweight-usb-host-part-3/
 *    gerritniezen com/usb-host-shield-on-espruino-part-2
 *    Check Arduino software using "USB Host Shield Library 2.0"
 *      Source code has a lot of good information.
 */

// Do not include any usb files, they are not needed.
#include <SPI.h>

#if defined(ARDUINO_SAMD_ZERO) && defined(SERIAL_PORT_USBVIRTUAL)
  #define Serial SERIAL_PORT_USBVIRTUAL
#endif

int MAX3421_SS_PIN = 10;

// registers
#define rRCVFIFO        0x08
#define rSNDFIFO        0x10
#define rSUDFIFO        0x20
#define rRCVBC          0x30
#define rSNDBC          0x38
#define rUSBIRQ         0x68
#define rUSBCTL         0x78
#define rCPUCTL         0x80
#define rPINCTL         0x88
#define rREVISION       0x90
#define rHIRQ           0xc8
#define rMODE           0xd8
#define rHIEN           0xd0
#define rPERADDR        0xe0
#define rHCTL           0xe8
#define rHXFR           0xf0
#define rHRSL           0xf8

// PINCTL Bitmaps
#define bmFDUPSPI       0x10
#define bmINTLEVEL      0x08
#define bmPOSINT        0x04
#define bmGPXB          0x02
#define bmGPXA          0x01
// GPX pin selections
#define GPX_OPERATE     0x00
#define GPX_VBDET       0x01
#define GPX_BUSACT      0x02
#define GPX_SOF         0x03

// rUSBCTL Bitmaps
#define bmCHIPRES       0x20
#define bmPWRDOWN       0x10

// CPUCTL Bitmaps
#define bmPUSLEWID1     0x80
#define bmPULSEWID0     0x40
#define bmIE            0x01

// rHIRQ Bitmaps
#define bmBUSEVENTIRQ   0x01
#define bmRWUIRQ        0x02
#define bmRCVDAVIRQ     0x04
#define bmSNDBAVIRQ     0x08
#define bmSUSDNIRQ      0x10
#define bmCONDETIRQ     0x20
#define bmFRAMEIRQ      0x40
#define bmHXFRDNIRQ     0x80

// rHRSL Bitmaps
#define bmRCVTOGRD      0x10
#define bmSNDTOGRD      0x20
#define bmKSTATUS       0x40
#define bmJSTATUS       0x80
#define bmSE0           0x00    // disconnect
#define bmSE1           0xc0    // illegal
// Host errors
#define hrSUCCESS       0x00
#define hrBUSY          0x01
#define hrBADREQ        0x02
#define hrUNDEF         0x03
#define hrNAK           0x04
#define hrSTALL         0x05
#define hrTOGERR        0x06
#define hrWRONGPID      0x07
#define hrBADBC         0x08
#define hrPIDERR        0x09
#define hrPKTERR        0x0A
#define hrCRCERR        0x0B
#define hrKERR          0x0C
#define hrJERR          0x0D
#define hrTIMEOUT       0x0E
#define hrBABBLE        0x0F

// rHCTL Bitmaps
#define bmBUSRST        0x01
#define bmFRMRST        0x02
#define bmSAMPLEBUS     0x04
#define bmSIGRSM        0x08
#define bmRCVTOG0       0x10
#define bmRCVTOG1       0x20
#define bmSNDTOG0       0x40
#define bmSNDTOG1       0x80

// rHIEN Bitmaps
#define bmBUSEVENTIE    0x01
#define bmRWUIE         0x02
#define bmRCVDAVIE      0x04
#define bmSNDBAVIE      0x08
#define bmSUSDNIE       0x10
#define bmCONDETIE      0x20
#define bmFRAMEIE       0x40
#define bmHXFRDNIE      0x80

// rMODE Bitmaps
#define bmHOST          0x01
#define bmLOWSPEED      0x02
#define bmHUBPRE        0x04
#define bmSOFKAENAB     0x08
#define bmSEPIRQ        0x10
#define bmDELAYISO      0x20
#define bmDMPULLDN      0x40
#define bmDPPULLDN      0x80

#define MODE_FS_HOST    (bmDPPULLDN|bmDMPULLDN|bmHOST|bmSOFKAENAB)
#define MODE_LS_HOST    (bmDPPULLDN|bmDMPULLDN|bmHOST|bmLOWSPEED|bmSOFKAENAB)

//  Library/Arduino15/packages/arduino/hardware/samd/1.8.12/cores/arduino/Arduino.h/USB/USB_host.h
//          tokSETUP changed to tokenSETUP so samd (ZERO) would work without compile errors
// rHXFR
#define tokenSETUP  0x10
#define tokenIN     0x00
#define tokenOUT    0x20
#define tokenINHS   0x80
#define tokenOUTHS  0xA0
#define tokenISOIN  0x40
#define tokenISOOUT 0x60

/* Standard Device Requests */
#define USB_REQUEST_GET_STATUS                  0
#define USB_REQUEST_CLEAR_FEATURE               1
#define USB_REQUEST_SET_FEATURE                 3
#define USB_REQUEST_SET_ADDRESS                 5
#define USB_REQUEST_GET_DESCRIPTOR              6
#define USB_REQUEST_SET_DESCRIPTOR              7
#define USB_REQUEST_GET_CONFIGURATION           8
#define USB_REQUEST_SET_CONFIGURATION           9
#define USB_REQUEST_GET_INTERFACE               10
#define USB_REQUEST_SET_INTERFACE               11
#define USB_REQUEST_SYNCH_FRAME                 12

/* Device Request Constants */
#define USB_SETUP_HOST_TO_DEVICE                0x00
#define USB_SETUP_DEVICE_TO_HOST                0x80
#define USB_SETUP_TYPE_STANDARD                 0x00
#define USB_SETUP_TYPE_CLASS                    0x20
#define USB_SETUP_TYPE_VENDOR                   0x40
#define USB_SETUP_RECIPIENT_DEVICE              0x00
#define USB_SETUP_RECIPIENT_INTERFACE           0x01
#define USB_SETUP_RECIPIENT_ENDPOINT            0x02
#define USB_SETUP_RECIPIENT_OTHER               0x03

/* USB bDescriptorType  */
#define USB_DESCRIPTOR_DEVICE                   0x0100
#define USB_DESCRIPTOR_CONFIGURATION            0x0200
#define USB_DESCRIPTOR_STRING                   0x0300

#define USB_DESCRIPTOR_INTERFACE                0x04
#define USB_DESCRIPTOR_ENDPOINT                 0x05
#define USB_DESCRIPTOR_DEVICE_QUALIFIER         0x06
#define USB_DESCRIPTOR_OTHER_SPEED              0x07
#define USB_DESCRIPTOR_INTERFACE_POWER          0x08
#define USB_DESCRIPTOR_OTG                      0x09

struct getDescriptor_t {
    uint8_t  requestType;
    uint8_t  recipient;
    uint16_t request;
    uint16_t value;
    uint16_t index;
};
struct getDescriptor_t getDeviceDescriptor;
struct getDescriptor_t getConfigDescriptor;
struct getDescriptor_t getStringDescriptor;
struct getDescriptor_t setDeviceDescriptor;

// USB Setup Packet, only for union byte SPI output.
struct usb_setupPacket_t {
  uint8_t  bmRequestType;
  uint8_t  bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
};
const int union_size = sizeof(usb_setupPacket_t);
union btPacket_t{
 usb_setupPacket_t usb_setupPacket;
 uint8_t byteArray[union_size];
};
btPacket_t setupUSB;

// USB String Descriptor Structure 
struct USB_STRING_DESCRIPTOR{
  uint8_t bLength;                    // size of this descriptor
  uint8_t bDescriptorType;            // type, USB_DSC_STRING
  uint8_t bString[64];                // buffer for string         
};

uint8_t bmRcvToggle_epIN = bmRCVTOG0;
uint8_t bmSndToggle_epIN = bmSNDTOG0;
uint8_t epINnum = 0;                   // set by printconfigDescriptor

uint8_t bmRcvToggle_epOUT = bmRCVTOG0;
uint8_t bmSndToggle_epOUT = bmSNDTOG0;
uint8_t epOUTnum = 0;                   // set by printconfigDescriptor

uint8_t currentEp = 0;
uint8_t spiInBuf[100];             // SPI buffer for transferIn
uint16_t cm15aMaxPktSize = 4;      // set by printconfigDescriptor

//   x10HouseStr not used by program   Human Display Use   (Char) - 'A' = index
char x10HouseStr[][4]  = { "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P"};
int x10HouseCodes[]    = {0x60,0xE0,0x20,0xA0,0x10,0x90,0x50,0xD0,0x70,0xF0,0x30,0xB0,0x00,0x80,0x40,0xC0};

//   x10DeviceStr not used by program   Human Display Use  (Char) - '1' = index
char x10DeviceStr[][4] = {"1", "2", "3", "4", "5", "6", "7", "8", "9", "10","11","12","13","14","15","16"};
int x10DeviceCodes[]   = {0x06,0x0E,0x02,0x0A,0x01,0x09,0x05,0x0D,0x07,0x0F,0x03,0x0B,0x00,0x08,0x04,0x0C};

uint8_t x10CmdAddr = 0x04;    // X10 Command Address
uint8_t x10CmdType = 0x06;    // X10 Command Type
uint8_t x10AnswAck = 0x55;    // X10 ACK character
uint8_t x10SetTime = 0x9B;    // X10 CM15a set date/time/etc
uint8_t x10CmdStat = 0x8B;    // X10 CM15a memory dump

int incomingByte = 0;    // read data from serial monitor.

boolean errorSoWhileForever = false;

// ========================================================================
// ========================================================================
// ========================================================================

void setup() {
  uint8_t result;
  
  Serial.begin(115200);
  while (! Serial) {
    Serial.println(F("Waiting for Serial debug to open."));
    delay(5);
  }
  Serial.println("");
  Serial.println("USB Max3421e for CM15A Program");
  Serial.println("");

  pinMode(MAX3421_SS_PIN, OUTPUT);
  digitalWrite(MAX3421_SS_PIN, HIGH);
  
  SPI.begin();

  result = Max3421Init();             // setup MAX3421E chip.  result true=no_error
  if (result) {                       // returned true for all OK
    result = ParseUsbCM15aOptions();  // Enumeration of usb CM15a
    if (result)  {
       result = ConfigUsbCM15a();     // setup usb CM15a options
    } else {
      Serial.println(F("ParseUsbCM15aOptions Failed.. System stopped"));
    }
  } else{
    Serial.println(F("Max3421Init Failed.. System stopped"));
  }
  if (result)  {
    // config done, set for data mode
    result = readReg(rHCTL) & 0x0F; 
    bmRcvToggle_epIN = bmRCVTOG0;
    bmSndToggle_epOUT = bmSNDTOG0;
    result = result | bmRcvToggle_epIN;
    result = result | bmSndToggle_epOUT;
    writeReg(rHCTL, result);

    Serial.println(F("================= USB setup done ================="));
  } else {
    Serial.println("");
    Serial.println(F("USB Setup ERROR !!"));
  }

  if (result == 0) errorSoWhileForever = true;
  
  Serial.println("");
  Serial.println(F("Arduino setup() done."));
  Serial.println("");
  showCmds();
}


// used to display byte data as HEX
const char *hexnib_rep[32] = {
  [ 0] = "0", [ 1] = "1", [ 2] = "2", [ 3] = "3",
  [ 4] = "4", [ 5] = "5", [ 6] = "6", [ 7] = "7",
  [ 8] = "8", [ 9] = "9", [10] = "A", [11] = "B",
  [12] = "C", [13] = "D", [14] = "E", [15] = "F",
};
void byteToHEXPrint(uint8_t byte, char* hexstr)
{
  sprintf(hexstr,"%s%s", hexnib_rep[byte >> 4], hexnib_rep[byte & 0x0F]);
}


// used to display byte data as binary
const char *bitnib_rep[16] = {
  [ 0] = "0000", [ 1] = "0001", [ 2] = "0010", [ 3] = "0011",
  [ 4] = "0100", [ 5] = "0101", [ 6] = "0110", [ 7] = "0111",
  [ 8] = "1000", [ 9] = "1001", [10] = "1010", [11] = "1011",
  [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};
void byteToBinPrint(uint8_t byte, char* hexstr)
{
  sprintf(hexstr,"%s_%s", bitnib_rep[byte >> 4], bitnib_rep[byte & 0x0F]);
}

// ====================================
// ====== SPI transfer functions ======

uint8_t readReg(uint8_t reg) {
  uint8_t result8;

  //SPI.beginTransaction(settingsA);
  digitalWrite (MAX3421_SS_PIN, LOW);
  delayMicroseconds (20);
  result8 = SPI.transfer(reg);
  delayMicroseconds (20);
  result8 = SPI.transfer(0x00);
  delayMicroseconds (20);
  digitalWrite (MAX3421_SS_PIN, HIGH);
  delayMicroseconds (20);
  SPI.endTransaction();
  return result8;
}


// multiple-byte SPI read
void doBytes_read( byte mxregister, byte nbytes, uint8_t* recstr)
{
  uint8_t item,cnt;

  cnt = 0;
  digitalWrite (MAX3421_SS_PIN, LOW);
  delayMicroseconds (20);
  SPI.transfer(mxregister);
  while( cnt < nbytes ) {
    item = SPI.transfer(0x00);    // send empty byte, read SPI contents
    delayMicroseconds (20);
    *recstr = item;
    recstr++;
    cnt++;
    }
  delayMicroseconds (20);
  digitalWrite (MAX3421_SS_PIN, HIGH);
  delayMicroseconds (20);
  SPI.endTransaction();
}


uint8_t writeReg(uint8_t reg, uint8_t what) {
  uint8_t result8;

  digitalWrite (MAX3421_SS_PIN, LOW);
  delayMicroseconds (20);
  result8 = SPI.transfer(reg + 2);     // enable write bit
  delayMicroseconds (20);
  result8 = SPI.transfer(what);
  delayMicroseconds (20);
  digitalWrite (MAX3421_SS_PIN, HIGH);
  delayMicroseconds (20);
  SPI.endTransaction();
  return result8;
}

// multiple-byte SPI write
void doBytes_write(uint8_t mxregister, uint8_t nbytes) {
  uint8_t sendbyte,cnt;

  cnt = 0;
  digitalWrite (MAX3421_SS_PIN, LOW);
  delayMicroseconds (20);
  SPI.transfer(mxregister + 2);     // enable write bit
  while(cnt <  nbytes) {
    sendbyte = setupUSB.byteArray[cnt];      // gobal var, yes it is bad form..
    SPI.transfer(sendbyte);
    cnt++;
    }
  delayMicroseconds (20);
  digitalWrite (MAX3421_SS_PIN, HIGH);
  delayMicroseconds (20);
  SPI.endTransaction();
}

// ====================================
// ====================================

// setup MAX3421e chip
boolean Max3421Init(void) {
  boolean showDebug = false;
  uint8_t result,bus_sample;
  int16_t i;
  char convStr[10];
  
  Serial.println(F("[ Max3421Init: Start chip config ]"));
  Serial.println("");

  Serial.println(F("rPINCTL Setup SPI for Arduino"));
  writeReg(rPINCTL, bmFDUPSPI | bmINTLEVEL);
  delay(100);
  result = readReg(rPINCTL);
  if (showDebug) Serial.print(F("rPINCTL show all bits set: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if ((result == 0) || (result > 253)) {
    Serial.println(F("rPINCTL Setup SPI Second try"));
    Serial.println(F("  Arduino ZERO cold boot needs second try?"));
    writeReg(rPINCTL, bmFDUPSPI | bmINTLEVEL);
    delay(100);
    result = readReg(rPINCTL);
    Serial.print(F("rPINCTL show all bits set2: "));
    byteToBinPrint(result, &convStr[0]);
    Serial.println(convStr);
    if ((result == 0) || (result > 253)) {
      Serial.println(F(" SPI setup failed!! "));
      Serial.println("");
      return(false);
    }
  }
  if (showDebug) Serial.println(F("rPINCTL SPI setup done"));
  
  if (showDebug) Serial.println("");
  result = readReg(rREVISION);
  Serial.print(F("Chip revision DEC: "));
  Serial.print(result, DEC);
  Serial.print(F(" , HEX: "));
  Serial.println(result, HEX);
  if (showDebug) Serial.println(F("Chip revision read shows SPI working..."));
  
// RESET CHIP
  if (showDebug) Serial.println(F("Reset MAX3421 chip"));
  if (showDebug) Serial.println("");
  writeReg(rUSBCTL, bmCHIPRES);
  delay(2);
  writeReg(rUSBCTL, 0x00);
  delay(2);

// RESET USB BUSS
  i = 0;
  if (showDebug) Serial.println(F("Buss reset writeReg_rHCTL,bmBUSRST_ up to 50ms"));
  result = readReg(rHCTL) | bmBUSRST;   // add reset flag
  writeReg(rHCTL, result);
  if (showDebug) Serial.print(F("Buss reset status "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  while(result == 1) {
    i++;
    result = readReg(rHCTL) & bmBUSRST;
    Serial.print(F("Waiting Buss reset status "));
    byteToBinPrint(result, &convStr[0]);
    Serial.println(convStr);;
    if (i > 10) {
      Serial.println(F("Buss reset Failed2!!"));
      return(false);
      }
    delay(50);
    }
  if (showDebug) Serial.println(F("Buss reset done..."));
  if (showDebug) Serial.println("");

  i = 0;
  result = 0;
  if (showDebug) Serial.println(F("Attempt to settle Oscillator"));
  while(result == 0) {
    i++;
    result = readReg(rUSBIRQ) & 0x01;
    if (i > 10) {
      Serial.println(F("Oscillator won't settle!!"));
      return(false);
      }
    if (showDebug) result = readReg(rUSBIRQ);
    if (showDebug) Serial.print("rUSBIRQ BIN: ");
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
    delay(200);
    }
  if (showDebug) Serial.print(F("Oscillator attempts to settle: "));
  if (showDebug) Serial.println(i);
  if (showDebug) Serial.println("");

// Set to host mode and pull down resistors on
  if (showDebug) Serial.println(F("Switch to HOST mode."));
  if (showDebug) Serial.println(F("Do not change host config bits before this change."));
  if (showDebug) Serial.println(F("rMODE Set bmDPPULLDN|bmDMPULLDN|bmHOST"));   // bmSEPIRQ
  writeReg(rMODE, bmDPPULLDN|bmDMPULLDN|bmHOST);
  delay(100);

  if (showDebug) result = readReg(rMODE);
  if (showDebug) Serial.print(F("rMODE show all bits set: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  Serial.println(F("rHIEN Set bmFRAMEIE bmCONDETIE"));
  writeReg(rHIEN, bmFRAMEIE|bmCONDETIE);                   // bmBUSEVENTIE

  if (showDebug) result = readReg(rHRSL);
  if (showDebug) Serial.print(F("rHRSL show  all  bits xfer done readReg JS KS SN RC: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  if (showDebug) Serial.println(F("Set USB Address 0"));
  result = readReg(rPERADDR);
  if (showDebug) Serial.print(F("Show all bits Pre  setup rPERADDR: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  writeReg(rPERADDR, 0);
  result = readReg(rPERADDR);
  if (showDebug) Serial.print(F("Show all bits Post setup rPERADDR: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("rHCTL SET bmSAMPLEBUS for connected probe"));
  result = readReg(rHCTL);
  result |= bmSAMPLEBUS;
  writeReg(rHCTL,result);
  delay(200);

  if (showDebug) Serial.println("");
  result = readReg(rHIRQ);   // connected=1001000  disconnected=1101000
  if (showDebug) Serial.print(F("rHIRQ show all bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  result &= bmCONDETIRQ;      // 0=connected  32=disconnected
  if (result == 0) {
    if (showDebug) Serial.print(F("rHIRQ read bmCONDETIRQ status 0=CONNECTED "));
    if (showDebug) Serial.println(result);
  } else {
    Serial.print(F("rHIRQ read bmCONDETIRQ status 32=NOT CONNECTED "));
    Serial.println(result);
  }

  if (showDebug) result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("rHIRQ show all bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  
  if (showDebug) Serial.println(F("Check bmKSTATUS and bmJSTATUS"));
  bus_sample = 0;
  i = 0;
  while(bus_sample == 0) {
    i++;
    result = readReg(rHRSL);
    bus_sample = result & (bmKSTATUS | bmJSTATUS);
    if (showDebug) Serial.print(F("  rHRSL raw result: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
    if (showDebug) Serial.print(F("  rHRSL bus_sample: "));
    if (showDebug) byteToBinPrint(bus_sample, &convStr[0]);
    if (showDebug) Serial.println(convStr);
    if (showDebug) result = readReg(rHIRQ);      // 0=connected  32=disconnected
    if (showDebug) Serial.print(F("  rHIRQ current: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);

    delay(100);
    if (i > 5) {
      Serial.println("");
      Serial.println(F("bus_sample failed, did NOT find device !!"));
      Serial.println(F("CM15A not found. "));
      Serial.println(F("CM15A must be plugged in before starting program."));
      Serial.println("");
      return(false);
    }
    delay(200);
  }

  if (showDebug) Serial.println(F("Set usb speed of device based on bmKSTATUS or bmJSTATUS"));
  switch(bus_sample) {
    case (bmJSTATUS):
      if ( (readReg(rMODE) & bmLOWSPEED) == 0) {
         writeReg(rMODE, MODE_FS_HOST);  //start full-speed host
         Serial.println(F("Speed bmJSTATUS full: MODE_FS_HOST"));
      } else {
         writeReg(rMODE, MODE_LS_HOST); //start low-speed host
         Serial.println(F("Speed bmJSTATUS low: MODE_LS_HOST"));
      }
    break;
    case (bmKSTATUS):
      if ( (readReg(rMODE) & bmLOWSPEED) == 0) {
         writeReg(rMODE, MODE_LS_HOST);  //start full-speed host
         Serial.println(F("Speed bmKSTATUS low: MODE_LS_HOST"));
      } else {
         writeReg(rMODE, MODE_FS_HOST); //start low-speed host
         Serial.println(F("Speed bmKSTATUS full: MODE_FS_HOST"));
      }
      break;
    case (bmSE1):              //illegal state
      Serial.println(F("Speed illegal state"));
      return(false);
      break;
    case (bmSE0):              //disconnected state
      Serial.print(F("BusSpeed: Disconnected state, no device: "));
      Serial.println(bus_sample);
      return(false);
    break;
    default:
      Serial.println(F("Speed ERROR UNKNOWN"));
      return(false);
    break;
  }

  if (showDebug) result = readReg(rMODE);
  if (showDebug) Serial.print(F("rMODE show all bits set: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) Serial.println("");
  writeReg(rHIRQ, bmCONDETIRQ);       // clear interrupt
  result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("rHIRQ clear bmCONDETIRQ int done, bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  writeReg(rCPUCTL, bmIE);            // enable interrupt pin
  result = readReg(rCPUCTL);
  if (showDebug) Serial.print(F("rCPUCTL enable bmIE int, show all bits set: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) Serial.println("");

// RESET USB BUSS  again
  if (showDebug) result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("rHIRQ before buss reset flag bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN before buss reset flag bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rMODE);
  if (showDebug) Serial.print(F("rMODE before buss reset flag bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHCTL);
  if (showDebug) Serial.print(F("rHCTL before buss reset flag bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHCTL);
  if (showDebug) Serial.print(F("rHCTL SND1 SND0 RCV1 RCV0 _ SIGR SAMP FRMR BUST: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHRSL);
  if (showDebug) Serial.print(F("rHRSL JSTA KSTA SNDT RCVT _  H3   H2   H1    H0: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  i = 0;
  if (showDebug) Serial.println(F("Buss reset writeReg_rHCTL,bmBUSRST_ 50ms"));
  
  result = readReg(rHCTL) | bmBUSRST;   // add reset flag
  writeReg(rHCTL, result);
  
  result = readReg(rHCTL) & bmBUSRST;
  if (showDebug) Serial.print(F("Buss reset status "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  while(result == 1) {
    i++;
    result = readReg(rHCTL) & bmBUSRST;
    Serial.print(F("Waiting Buss reset status "));
    byteToBinPrint(result, &convStr[0]);
    Serial.println(convStr);
    if (i > 10) {
      Serial.println(F("Buss reset Failed2!!"));
      return(false);
      }
    delay(30);
    }
  if (showDebug) result = readReg(rHCTL);
  if (showDebug) Serial.print(F("rHCTL SND1 SND0 RCV1 RCV0 _ SIGR SAMP FRMR BUST: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHRSL);
  if (showDebug) Serial.print(F("rHRSL JSTA KSTA SNDT RCVT _  H3   H2   H1    H0: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  Serial.println(F("rHCTL SET bmSAMPLEBUS for connected probe"));
  result = readReg(rHCTL);
  result |= bmSAMPLEBUS;
  writeReg(rHCTL,result);
  delay(200);

  Serial.println(F("rHIEN Set bmFRAMEIE bmCONDETIE"));
  writeReg(rHIEN, bmFRAMEIE|bmCONDETIE);

  Serial.println(F("rHCTL Set bmRcvToggle_epIN"));
  result = readReg(rHCTL);
  result |= bmRcvToggle_epIN;
  result |= bmSndToggle_epIN;
  writeReg(rHCTL, result);
  
  if (showDebug) result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("rHIRQ HXFR FRAM COND SUSD _ SNDB RCVD RWUI BUSE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN HXFR FRAM COND SUSD _ SNDB RCVD RWUI BUSE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rMODE);
  if (showDebug) Serial.print(F("rMODE DPDN DMDN DELA SEPI _ SOFK HUBP LSPD HOST: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHCTL);
  if (showDebug) Serial.print(F("rHCTL SND1 SND0 RCV1 RCV0 _ SIGR SAMP FRMR BUST: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) result = readReg(rHRSL);
  if (showDebug) Serial.print(F("rHRSL JSTA KSTA SNDT RCVT _  H3   H2   H1    H0: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

// check for SOF received
  if (readReg(rHIRQ) & bmFRAMEIRQ) {
    if (showDebug) Serial.println(F("rHIRQ & bmFRAMEIRQ mask, Have received first SOF"));
    Serial.println(F("[ Max3421Init Done ]"));
    Serial.println("");
    if (showDebug) Serial.println("");
    if (showDebug) showHCTL(1);
    if (showDebug) showHRSL(1);
    if (showDebug) Serial.println("");
    if (showDebug) Serial.println("");
    return(true);
  }
  if (showDebug) Serial.println(F("rHIRQ & bmFRAMEIRQ mask, no SOF received, ERROR"));
  if (showDebug) Serial.println(F("For this TEST program, CM15a must be plugged in before program starts."));
  Serial.println("");
  Serial.println(F("[ Max3421Init ERROR ] "));
  Serial.println("");

  return(false);      // #################### MAX3421e INIT CODE END ####################
}


// expand reg data for easy view, show current toggle settings
void showHCTL(uint8_t doVSpc) {   // if doVSpc add 6 spaces to output
  uint8_t result,testr;
  char convStr[10];
  
  result = readReg(rHCTL);
  if (doVSpc) Serial.print(F("      "));
  Serial.print(F("rHCTL BIN "));
  byteToBinPrint(result, &convStr[0]);
  Serial.println(convStr);
  if (doVSpc) Serial.print(F("      "));

  testr = result & 0x20;
  if (testr) Serial.print(F("    RCVTOG1=1"));
  else  Serial.print(F("    RCVTOG1=0"));

  testr = result & 0x10;
  if (testr) Serial.print(F("  RCVTOG0=1"));
  else  Serial.print(F("  RCVTOG0=0"));

  Serial.print(F("  bmRcvToggle_epIN=0x"));
  Serial.print(bmRcvToggle_epIN,HEX);

  testr = result & 0x30;
  if (testr == 0) Serial.print(F("   ERROR, BOTH RCVTOGs 0s"));
  if (testr == 0x30) Serial.print(F("   ERROR, BOTH RCVTOGs 1s"));
  Serial.println(F(""));
  if (doVSpc) Serial.print(F("      "));

  testr = result & 0x80;
  if (testr) Serial.print(F("    SNDTOG1=1"));
  else  Serial.print(F("    SNDTOG1=0"));

  testr = result & 0x40;
  if (testr) Serial.print(F("  SNDTOG0=1"));
  else  Serial.print(F("  SNDTOG0=0"));

  Serial.print(F("  bmTrxToggle_epIN=0x"));
  Serial.print(bmSndToggle_epIN,HEX);

  testr = result & 0xC0;
  if (testr == 0) Serial.print(F("   ERROR, BOTH SNDTOGs 0s"));
  if (testr == 0xC0) Serial.print(F("   ERROR, BOTH SNDTOGs 1s"));

  Serial.println(F(""));
}


// expand reg data for easy view
void showHRSL(uint8_t doVSpc) {
  uint8_t result,testr;
  char convStr[10];
  
  result = readReg(rHRSL);
  if (doVSpc) Serial.print(F("      "));
  Serial.print(F("rHRSL BIN "));
  byteToBinPrint(result, &convStr[0]);
  Serial.print(convStr);

  testr = result & 0x20;
  if (testr) Serial.print(F(" SNDTOGRD=1"));
  else  Serial.print(F(" SNDTOGRD=0"));

  testr = result & 0x010;
  if (testr) Serial.print(F(" RCVTOGRD=1"));
  else  Serial.print(F(" RCVTOGRD=0"));
  
  Serial.println(F(""));
}


// send data in FIFO register to device
uint8_t dispatchPacket(uint8_t token, uint8_t ep, boolean filter) {
  boolean showDebug = false;
  uint8_t result, loopcnt;
  byte errCount = 0;
  uint8_t transferResult;
  char convStr[10];

  if (filter == false)
    Serial.println(F("      [ Begin dispatchPacket ] "));

  if (showDebug) Serial.print(F("      Token: "));
  if (showDebug) Serial.print(token);
  if (showDebug) Serial.print(F(" ep: "));
  if (showDebug) Serial.println(ep);
  
  if (showDebug) showHCTL(1);
  if (showDebug) showHRSL(1);

  if (showDebug) result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("      rHIRQ should need to be cleared bmHXFRDNIRQ if needed "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) {
    result = readReg(rHIRQ);
    if (result & bmHXFRDNIRQ) {
      writeReg(rHIRQ, bmHXFRDNIRQ);    //clear the interrupt if set
      Serial.println(F("      rHIRQ bmHXFRDNIRQ old interrupt cleared. "));
    } else {
      Serial.println(F("      rHIRQ bmHXFRDNIRQ no need to clear. "));
    }
  }
  
  if (showDebug) showHRSL(1);

  if (showDebug) result = readReg(rHIRQ);
  if (showDebug) Serial.print(F("      rHIRQ show all bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("      rHIEN show all bits: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  loopcnt = 1;
  while(loopcnt < 6) {
    writeReg(rHIRQ, bmHXFRDNIRQ);      // clear interrupt
    result = readReg(rHIRQ);
    if (showDebug) Serial.print(F("      dispatchPacket clear bmHXFRDNIRQ STEP1: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
    
    if (showDebug) Serial.println(F("      writeReg_rHXFR, token | ep_ Done "));
    writeReg(rHXFR, token | ep);     // launch transfer

    if (showDebug) showHRSL(1); 

    result = readReg(rHIRQ);
    if (showDebug) Serial.print(F("      dispatchPacket wait for bmHXFRDNIRQ STEP2  pre: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
    
    while ((readReg(rHIRQ) & bmHXFRDNIRQ) == 0) {
      if (filter == false)
        Serial.println(F(" *** dispatchPacket waiting for bmHXFRDNIRQ to be done ***"));
      }
    result = readReg(rHIRQ);
    if (showDebug) Serial.print(F("      dispatchPacket done waiting for bmHXFRDNIRQ STEP3 post: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);

    writeReg(rHIRQ, bmHXFRDNIRQ);      // clear interrupt
    result = readReg(rHIRQ);
    if (showDebug) Serial.print(F("      dispatchPacket clear interrupt bmHXFRDNIRQ STEP4 post: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
// -------------------------------------------------------
    transferResult = readReg(rHRSL) & 0x0f;
    switch(transferResult) {
      case hrSUCCESS:
          // Serial.println(F("      dispatchPacket hrSUCCESS"));
        break;
        
      case hrNAK:
        if (showDebug) Serial.println(F("      dispatchPacket no data hrNAK, retrying.."));
        break;

      case hrBUSY:
        if (showDebug) Serial.println(F("      dispatchPacket response error hrBUSY, retrying.."));
        errCount++;
        if (errCount > 20) return(hrBUSY); 
        break;
 
      case hrTIMEOUT:
        Serial.println(F("      dispatchPacket response error hrTIMEOUT........"));
        errCount++;
        if (errCount > 3) return(hrTIMEOUT);
        delay(50);
        break;
        
      case hrSTALL:
        Serial.println(F("      dispatchPacket response error hrSTALL, were dead, locked up, start over. "));
        return(transferResult);
        break;
        
      case hrJERR:
        Serial.println(F("      dispatchPacket response error hrJERR......."));
        errCount++;
        if (errCount >= 2) return(hrJERR); 
        delay(200);
        break;
 
      case hrBADREQ:
        Serial.println(F("      dispatchPacket response error hrBADREQ......."));
        errCount++;
        if (errCount > 1) return(hrBADREQ); 
        delay(200);
      break;
      
      case hrBADBC:
        Serial.println(F("      dispatchPacket response error hrBADBC......."));
        errCount++;
        if (errCount > 1) return(hrBADBC); 
        delay(200);
      break;
      
      case hrTOGERR:
        Serial.println(F("      dispatchPacket response error hrTOGERR......."));
        return(hrTOGERR); 
      break;
      
      default:
        if (showDebug) Serial.print(F("      dispatchPacket default: "));
        if (showDebug) Serial.println(transferResult);
      break;
      }
    if (transferResult == 0) {
      if (showDebug) Serial.println(F("      dispatchPacket transfer worked OK"));
      break;   // 0 = no error
    }
  loopcnt++;
  }
  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("      rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug)  Serial.println(convStr);
  
  if (showDebug) showHCTL(1);
  if (showDebug) showHRSL(1);

  //if (filter == false)
   // Serial.println(F("      [ Done dispatchPacket ] "));
  return transferResult;
}


// setup packet uses DATA0
boolean controlTransferIn(getDescriptor_t setup, uint8_t mxlength ) {
  boolean showDebug = false;
  uint8_t result;
  char convStr[10];

  if (showDebug) Serial.println(F("[ Begin controlTransferIn ]"));

  if (showDebug) Serial.println(F("rPERADDR set addr to 0"));
  writeReg(rPERADDR, 0);
  
  result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) Serial.print(F("rHIEN enable bmHXFRDNIE: "));
  result |= bmHXFRDNIE;
  writeReg(rHIEN, result);
  if (showDebug) result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.usb_setupPacket.bmRequestType = setup.requestType;  // 0x80 REQUEST_VENDOR
  setupUSB.usb_setupPacket.bRequest = setup.request;           // 
  setupUSB.usb_setupPacket.wValue   = setup.value;             // 0x0100;  
  setupUSB.usb_setupPacket.wIndex   = setup.index;             // 0;
  setupUSB.usb_setupPacket.wLength  = mxlength;                // 18

  if (showDebug) Serial.println(F("controlTransferIn send setup config to rSUDFIFO"));
  
  doBytes_write(rSUDFIFO, 8);     // move packet into rSUDFIFO

  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  if (showDebug) Serial.println(F("Set bmRcvToggle_epIN to bmRCVTOG0"));
  if (showDebug) Serial.println(F("Set bmTrxToggle_epIN to bmSNDTOG0"));
  result = readReg(rHCTL) & 0x0F;
  bmRcvToggle_epIN = bmRCVTOG0;
  result = result | bmRcvToggle_epIN;
  result = result | bmSndToggle_epIN;
  writeReg(rHCTL, result);
  if (showDebug) showHCTL(0);
  
  result = dispatchPacket(tokenSETUP, 0, true);
  if (result) {
    Serial.print(F("controlTransferIn dispatchPacket error: "));
    Serial.println(result,HEX);
    return(false);
  } else {
    if (showDebug) Serial.println(F("controlTransferIn calling dispatchPacket done."));
  }
  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  
  if (showDebug) Serial.println(F("[ Done controlTransferIn ] "));
  if (showDebug) Serial.println("");
  if (showDebug) Serial.println("");
  return(true);
}


// return false/0 if had error
boolean transferIn(uint8_t ep, uint8_t mxlength) {
  boolean showDebug = false;
  uint8_t result, x, packetSize, loopcnt;
  uint16_t transferLength = 0;
  uint8_t maxTransferLoopCnt = 10;
  char convStr[10];

  // load false data for trouble shooting
  for (x=0; x< 80; x++) spiInBuf[x] = 0xDD;
  
  if (showDebug) Serial.println(F("[ Begin transferIn ]"));

  result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);

  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);

  loopcnt = 1;
  while (loopcnt < maxTransferLoopCnt) {
    if (showDebug) Serial.print(F("TransferIn loop pass: "));
    if (showDebug) Serial.println(loopcnt);

    if (showDebug) Serial.println(F("Do  writeReg_rHCTL, bmRcvToggle_epIN_ bmRCVTOG0 bmRCVTOG1"));
    result = readReg(rHCTL) & 0x0F; 
    bmRcvToggle_epIN = bmRcvToggle_epIN == bmRCVTOG0 ? bmRCVTOG1 : bmRCVTOG0;  // Ternary operator
    result = result | bmRcvToggle_epIN;
    result = result | bmSndToggle_epIN;
    writeReg(rHCTL, result);

    if (showDebug) showHCTL(0);
    if (showDebug) showHRSL(0);

    result = dispatchPacket(tokenIN, ep, true);
    if (result) {
      Serial.println("");
      Serial.print(F("transferIn dispatchPacket had error  0x0f mask: "));
      byteToBinPrint(result, &convStr[0]);
      Serial.println(convStr);
      return(false);
    } else {
      if ( (readReg(rHIRQ) & bmRCVDAVIRQ) == 0) {
        Serial.println(F("transferIn bmRCVDAVIRQ error..."));
        return(false);
      }
      packetSize = readReg(rRCVBC);
      if (showDebug) Serial.print(F("transferIn - READ in "));
      if (showDebug) Serial.print(packetSize);
      if (showDebug) Serial.print(F(" bytes of data from SPI: "));
      
      doBytes_read(rRCVFIFO, packetSize, &spiInBuf[transferLength]);

      writeReg(rHIRQ, bmRCVDAVIRQ);   // clear interrupt

      transferLength += packetSize;   // update location for next start point.

      delay(50);
    }
    if (packetSize < 1) {
      showHCTL(0);
      showHRSL(0);
      Serial.println(F("transferIn error"));
      return (false);
    }
    if (transferLength >= mxlength) {
      if (showDebug) showHCTL(0);
      if (showDebug) showHRSL(0);
      if (showDebug) Serial.println(F("[ TransferIn Done ]"));
      if (showDebug) Serial.println("");
      if (showDebug) Serial.println("");
      return (true);
    }
    loopcnt++;
  }
  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  return(true);
}


void printDeviceDescriptor(void) {
  uint8_t dataPos = 0;
  char convStr[10];

  Serial.println(F("[ printDeviceDescriptor Device descriptor: ]"));
  
  Serial.print(F("Descriptor Length:     "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  Serial.print(F(" HEX, "));
  Serial.print(spiInBuf[dataPos]);
  Serial.println(F(" DEC")); 
  dataPos++;

  Serial.print(F("Descriptor type:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("USB version:           "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  dataPos++;

  Serial.print(F("Device class:          "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Device Subclass:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Device Protocol:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Max.packet size:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Vendor  ID:            "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  dataPos++;

  Serial.print(F("Product ID:            "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  dataPos++;

  Serial.print(F("Revision ID:           "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  dataPos++;

  Serial.print(F("Mfg.string index:      "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Prod.string index:     "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Serial number index:   "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Number of conf.:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);

  Serial.println("");
}


void printconfigDescriptor(void) {
  uint8_t dataPos = 0;
  uint8_t tmpbyte,hidlength;
  char convStr[10];

  Serial.println(F("[ printconfigDescriptor Configuration descriptor: ]"));

  Serial.print(F("Length:                "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Descriptor Type:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  Serial.println(F(" USB_DESCRIPTOR_CONFIGURATION"));
  dataPos++;
  
  Serial.print(F("Total length of data:  "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  Serial.print(F(" HEX, "));
  Serial.print(spiInBuf[dataPos+1]);
  Serial.print(spiInBuf[dataPos]);
  Serial.println(F(" DEC"));
  dataPos++;
  dataPos++;

  Serial.print(F("Num.intf:              "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Conf.value:            "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Conf.string:           "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Attr.:                 "));
  byteToBinPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Max.pwr:               "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  
// =======================================================

  Serial.println("");
  Serial.println(F("Interface descriptor: "));

  Serial.print(F("Length:                "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Descriptor Type        "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);  
  Serial.print(convStr);
  Serial.println(" USB_DESCRIPTOR_INTERFACE=4");
  dataPos++;
  
  Serial.print(F("Intf.number:           "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Alt.:                  "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Endpoints:             "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Intf. Class:           "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  if (spiInBuf[dataPos] == 3) 
    Serial.print(F("              Human Interface Device=0x03 "));
  Serial.println("");
  dataPos++;

  Serial.print(F("Intf. Subclass:        "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Intf. Protocol:        "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Intf.string:           "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

// =======================================================

  Serial.println("");
  
  if ((spiInBuf[dataPos] == 7) and (spiInBuf[dataPos+1] == 5)) {
    Serial.println(F("Endpoint descriptor:     "));

    Serial.print(F("Length:                    "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.println(convStr);
    dataPos++;

    Serial.print(F("Description Type:          "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.println(F("          USB_DESCRIPTOR_ENDPOINT=5"));
    dataPos++;

    Serial.print(F("Endpoint Address:          "));
    byteToBinPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    if (spiInBuf[dataPos] & 0x80) {
      Serial.print(F("   IN  EP "));
      tmpbyte = spiInBuf[dataPos] & 0x0F;
      epINnum = tmpbyte;
    } else  {
      Serial.print(F("   OUT  EP "));
      tmpbyte = spiInBuf[dataPos] & 0x0F;
      epOUTnum = tmpbyte;
    }
    Serial.print(tmpbyte,HEX);
    Serial.println("");
    dataPos++;
  
    Serial.print(F("Endpoint Attr:             "));
    byteToBinPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    tmpbyte = spiInBuf[dataPos] & 0x03;
    if (tmpbyte == 3)
      Serial.print("   End point uses Interrupt transfers=3");
    Serial.println("");
    dataPos++;

    Serial.print(F("Max.pkt size:              "));
    byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
    Serial.print(convStr);
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.println(convStr);
    cm15aMaxPktSize = spiInBuf[dataPos];  // CM15A Max.pkt size   XXXX
    dataPos++;
    dataPos++;

    Serial.print(F("Polling interval:          "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.print(F("          Check at least every "));
    Serial.print(spiInBuf[dataPos],DEC);
    Serial.print(F(" ms"));
    Serial.println("");
    Serial.println("");
    dataPos++;

   } else {
    Serial.println(F("HID descriptor:"));

    Serial.print(F("Length:                    "));  // 9 mouse, 7 X10
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.println(convStr);
    hidlength = spiInBuf[dataPos];
    Serial.print(F("          hidlength: "));
    Serial.println(hidlength);
    dataPos++;

    Serial.print(F("Descriptor Type   :        "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.println(F("          HID Descriptor For Mouse=0x21 "));
    dataPos++;

    Serial.print(F("HID Class Specification:   "));
    byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
    Serial.print(convStr);
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.println(convStr);
    dataPos++;
    dataPos++;

    Serial.print(F("Country Code:              "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.println("");
    dataPos++;

    Serial.print(F("Number class descriptors:  "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.println("");
    dataPos++;

    Serial.print(F("Report descriptor type:    "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.println("");
    dataPos++;

    Serial.print(F("Report descriptor length:  "));
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.print(convStr);
    Serial.print(F(","));
    dataPos++;
    byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
    Serial.println(convStr);
    dataPos++;
  }

// =======================================================

  Serial.println("");
  Serial.println(F("Endpoint descriptor:     "));

  Serial.print(F("Length:                    "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Description Type:          "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  Serial.println(F("          USB_DESCRIPTOR_ENDPOINT=5"));
  dataPos++;

  Serial.print(F("Endpoint Address:          "));
  byteToBinPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  if (spiInBuf[dataPos] & 0x80) {
    Serial.print(F("   IN  EP "));
    tmpbyte = spiInBuf[dataPos] & 0x0F;
    epINnum = tmpbyte;
  } else  {
    Serial.print(F("   OUT  EP "));
    tmpbyte = spiInBuf[dataPos] & 0x0F;
    epOUTnum = tmpbyte;
  }
  // tmpbyte = spiInBuf[dataPos] & 0x0F;
  Serial.print(tmpbyte,HEX);
  Serial.println("");
  dataPos++;
  
  Serial.print(F("Endpoint Attr:             "));
  byteToBinPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  tmpbyte = spiInBuf[dataPos] & 0x03;
  if (tmpbyte == 3)
    Serial.print(F("   End point uses Interrupt transfers=3"));
  Serial.println("");
  dataPos++;

  Serial.print(F("Max.pkt size:              "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  
  cm15aMaxPktSize = spiInBuf[dataPos];
  Serial.print(F("set cm15aMaxPktSize:       "));
  Serial.println(cm15aMaxPktSize);
  dataPos++;
  dataPos++;

  Serial.print(F("Polling interval:          "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.print(convStr);
  Serial.print(F("          Check at least every "));
  Serial.print(spiInBuf[dataPos],DEC);
  Serial.print(F(" ms"));
  Serial.println("");
  dataPos++;
}


void printString4byteInfo(void) {
  uint8_t dataPos = 0;
  char convStr[10];

  Serial.println("");
  Serial.println(F("[ Get String descriptor 4 byte header: ]"));

  Serial.print(F("Length:                "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("Descriptor Type:       "));
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;

  Serial.print(F("LANGID_0: for wIndex   "));
  byteToHEXPrint(spiInBuf[dataPos+1], &convStr[0]);
  Serial.print(convStr);
  byteToHEXPrint(spiInBuf[dataPos], &convStr[0]);
  Serial.println(convStr);
  dataPos++;
  dataPos++;
  Serial.println("");
}

/*
---- requestType   Device to Host  0x80 = 10000000 ----
D7 Data Phase Transfer Direction
  0 = Host to Device
  1 = Device to Host
D6..5 Type
  0 = Standard
  1 = Class
  2 = Vendor
  3 = Reserved
D4..0 Recipient
  0 = Device
  1 = Interface
  2 = Endpoint
  3 = Other
  4.. = Reserved
 */
boolean ParseUsbCM15aOptions(void) {       // return true=ok  false=had errors
  boolean showDebug = false;
  uint8_t result,x,y;

  getDeviceDescriptor.requestType = 0x80;  // Bit Map Device to Host
  getDeviceDescriptor.recipient = 0;       // not used
  getDeviceDescriptor.request = 0x0006;    // USB_REQUEST_GET_DESCRIPTOR
  getDeviceDescriptor.value   = 0x0100;    // USB_DESCRIPTOR_DEVICE = 0x0100
  getDeviceDescriptor.index   = 0x0000;

  Serial.println(F("[ ParseUsbCM15aOptions ControlTransferIn get device descriptor ]"));
  result = controlTransferIn(getDeviceDescriptor,18);
  delay(50);

  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transfer In device descriptor ----"));
  result = transferIn(0,18);
  if (result == 0) {          // had error
    Serial.println(F("---- ParseUsbCM15aOptions ERROR from transferIn ----"));
    return(false);
  }
  Serial.println("");
  for (x=0; x<18; x++) {
    result = spiInBuf[x];
    Serial.print(result,HEX);
    Serial.print("");
  }
  Serial.println("");
  printDeviceDescriptor();

  getConfigDescriptor.requestType = 0x80;  // REQUEST_VENDOR;  Device to Host 0x80 = 10000000
  getConfigDescriptor.recipient   = 0;     // not used
  getConfigDescriptor.request = 0x0006;    // USB_REQUEST_GET_DESCRIPTOR
  getConfigDescriptor.value   = 0x0200;    // USB_DESCRIPTOR_CONFIGURATION = 0x0200
  getConfigDescriptor.index   = 0x0000;

  if (showDebug) Serial.print("");
  if (showDebug) Serial.print("");
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions Control get config descriptor SIZE ----"));
  // Get config descriptor length
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions ControlTransferIn get config descriptor SIZE ----"));
  result = controlTransferIn(getConfigDescriptor, 4);
  if (result == false) {
    Serial.println(F("---- ParseUsbCM15aOptions controlTransferIn getConfigDescriptor, 4 ----"));
    return(false);
  }
  
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transferIn get config desc SIZE for controlTransferIn ----"));
  result = transferIn(0, 4);
  if (result == false) {
    Serial.println(F("---- ParseUsbCM15aOptions transferIn 0, 4 ----"));
    return(false);
  }
  if (showDebug) Serial.print(F("---- ParseUsbCM15aOptions Size of ConfigDescriptor bytes from above spiInBuf[2]: "));
  if (showDebug) Serial.print(spiInBuf[2]);
  if (showDebug) Serial.println(F(" DEC ---- "));
  for (x=0; x<4; x++) {
    if (showDebug) y = spiInBuf[x];
    if (showDebug) Serial.print(y);
    if (showDebug) Serial.print("");
  }
  if (showDebug) Serial.println("");
    
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions controlTransferIn _ getConfigDescriptor ---- "));
  uint8_t configDescriptorLength = spiInBuf[2];  // byte location of size for next request
  result = controlTransferIn(getConfigDescriptor, configDescriptorLength);

  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transferIn _ configDescriptor ---- "));
  result = transferIn(0, configDescriptorLength);
  if (result == false) {
    Serial.println(F("---- ParseUsbCM15aOptions transferIn 0, configDescriptorLength ----"));
    return(false);
  }
  Serial.println("");
  for (x=0; x<configDescriptorLength; x++) {
    y = spiInBuf[x];
    Serial.print(y,HEX);
    Serial.print("");
  }
  Serial.println("");

  printconfigDescriptor();
  if (showDebug) Serial.println("");

  getStringDescriptor.requestType = 0x80;  // REQUEST_VENDOR;  0x80 = 10000000
  getStringDescriptor.recipient = 0;       // not used
  getStringDescriptor.request = 0x0006;    // USB_REQUEST_GET_DESCRIPTOR
  getStringDescriptor.value   = 0x0300;    // USB_DESCRIPTOR_STRING = 0x0300
  getStringDescriptor.index   = 0x0000;

  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions ControlTransferIn get string descriptor ----"));
  result = controlTransferIn(getStringDescriptor,4);
  delay(50);
    
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transfer In string descriptor ----"));
  result = transferIn(0,4);
  if (result == 0) {          // error
    Serial.println(F("---- ParseUsbCM15aOptions ERROR string from transferIn ----"));
    return(false);
  }
  if (showDebug) Serial.print(F("---- ParseUsbCM15aOptions Size of String ConfigDesc bytes from above spiInBuf[2]: "));
  if (showDebug) Serial.println(F(" DEC ---- "));
  for (x=0; x<4; x++) {
    if (showDebug) result = spiInBuf[x];
    if (showDebug) Serial.print(result,HEX);
    if (showDebug) Serial.print("");
  }
  if (showDebug) Serial.println("");
  printString4byteInfo();

  getStringDescriptor.requestType = 0x80;  // REQUEST_VENDOR;  Device to Host 0x80 = 10000000
  getStringDescriptor.recipient   = 0;     // not used
  getStringDescriptor.request = 0x0006;    // USB_REQUEST_GET_DESCRIPTOR
  getStringDescriptor.value   = 0x0301;    // USB_DESCRIPTOR_STRING = 0x03XX  XX is index in to strings
  getStringDescriptor.index   = 0x0409;    // language code for English US

  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions ControlTransferIn get string descriptor ----"));
  result = controlTransferIn(getStringDescriptor,2);     // read first 2 byte to get length of string
  delay(50);
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transfer In string descriptor ----"));
  result = transferIn(0,2);
  if (result == 0) {          // error
    Serial.println(F("---- ParseUsbCM15aOptions ERROR string from transferIn ----"));
    return(false);
  }
  uint8_t despLen = spiInBuf[0];
  if (showDebug) Serial.print(spiInBuf[0]);
  if (showDebug) Serial.print(" ");
  if (showDebug) Serial.print(spiInBuf[1]);
  if (showDebug) Serial.print(" ");

  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions ControlTransferIn get string descriptor ----"));
  result = controlTransferIn(getStringDescriptor,despLen);
  delay(50);
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions transfer In string descriptor ----"));
  result = transferIn(0,despLen);
  if (result == 0) {          // error
    Serial.println(F("---- ParseUsbCM15aOptions ERROR string from transferIn ----"));
    return(false);
  }
  if (showDebug) Serial.println(F("---- ParseUsbCM15aOptions Size of String ConfigDesc bytes from above spiInBuf[0]: "));

  Serial.print("Product: ");
  for (x=2; x<despLen; x++) {
    result = spiInBuf[x];
    if (result > 31) {
      Serial.write(result);
      Serial.print("");
    }
  }
  Serial.println("");
  if (showDebug) Serial.println(F("[ ParseUsbCM15aOptions Done ]"));
  return(true);
}

// =====================================================

// setup packet uses DATA0
boolean setupTransferOut(getDescriptor_t setup, uint8_t mxlength,uint8_t addr) {
  boolean showDebug = false;
  uint8_t result;
  char convStr[10];

  if (showDebug) Serial.println(F("[ Begin setupTransferOut ]"));

  if (showDebug) Serial.println(F("rPERADDR set MX address"));
  writeReg(rPERADDR, addr);
  
  result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  if (showDebug) Serial.print(F("rHIEN enable bmHXFRDNIE: "));
  result |= bmHXFRDNIE;
  writeReg(rHIEN, result);
  result = readReg(rHIEN);
  if (showDebug) Serial.print(F("rHIEN show all bits bmHXFRDNIE: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.usb_setupPacket.bmRequestType = setup.requestType;  // 0x80 REQUEST_VENDOR Bit-map of request type
  setupUSB.usb_setupPacket.bRequest = setup.request;           // 
  setupUSB.usb_setupPacket.wValue   = setup.value;             // 0x0100;  
  setupUSB.usb_setupPacket.wIndex   = setup.index;             // 0;
  setupUSB.usb_setupPacket.wLength  = mxlength;                // 0

  doBytes_write(rSUDFIFO, 8);
  
  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  if (showDebug) Serial.println(F("Set bmRcvToggle_epIN to bmRCVTOG0"));
  if (showDebug) Serial.println(F("Set bmTrxToggle_epIN to bmSNDTOG0"));

  result = readReg(rHCTL) & 0x0F; 
  bmSndToggle_epIN = bmSndToggle_epIN == bmSNDTOG0 ? bmSNDTOG1 : bmSNDTOG0;
  result = result | bmSndToggle_epIN;
  result = result | bmRcvToggle_epIN;
  writeReg(rHCTL, result);
    
  if (showDebug) showHCTL(0);
  
  result = dispatchPacket(tokenSETUP, 0, true);
  if (result) {
    Serial.print(F("setupTransferOut dispatchPacket error: "));
    Serial.println(result,HEX);
    return(false);
  } else {
    if (showDebug) Serial.println(F("setupTransferOut calling dispatchPacket done."));
  }
  if (showDebug) showHCTL(0);
  if (showDebug) showHRSL(0);
  
  if (showDebug) Serial.println(F("[ Done setupTransferOut ] "));
  return(true);
}


// Write back to use the default device config
boolean ConfigUsbCM15a(void) {    // return true=ok  false=had errors
  boolean showDebug = false;
  uint8_t result;

  // set usb device address
  setDeviceDescriptor.requestType = 0x00;  // REQUEST_VENDOR;
  setDeviceDescriptor.recipient = 0;       // not used
  setDeviceDescriptor.request = 0x0005,    // USB_REQUEST_SET_ADDRESS   5
  setDeviceDescriptor.value   = 0x0001;    // new address for usb device
  setDeviceDescriptor.index   = 0x0000;    // not used

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("[ ConfigUsbCM15a setupTransferOut set address ]"));
  setupTransferOut(setDeviceDescriptor,0,0);    // setDeviceDescriptor, mxlength, address
  delay(300);

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("---- ACK address setup ----"));
  result = dispatchPacket(tokenINHS, 0, true);
  if (result) Serial.println(F("      ConfigUsbCM15a dispatchPacket tokenINHS ERROR"));
  if (showDebug) Serial.println("");

  // set configuration
  setDeviceDescriptor.requestType = 0x00;  // REQUEST_VENDOR;
  setDeviceDescriptor.recipient = 0;       // not used
  setDeviceDescriptor.request = 0x0009,    // USB_REQUEST_SET_CONFIGURATION  9
  setDeviceDescriptor.value   = 0x0001;    // 
  setDeviceDescriptor.index   = 0x0000;    // not used

  if (showDebug) Serial.println(F("---- ConfigUsbCM15a setupTransferOut set config option ----"));
  setupTransferOut(setDeviceDescriptor,0,1);    // setDeviceDescriptor, mxlength, address
  delay(50);

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("---- ACK config setup ----"));
  result = dispatchPacket(tokenINHS, 0, true);
  if (result) {      // 0=noErr else error returned
    Serial.println(F("      ConfigUsbCM15a dispatchPacket tokenINHS ERROR"));
    Serial.println("");
    Serial.println(F("--------------------------------------------------"));
    return(false);
  }

  if (showDebug) Serial.println(F("      ConfigUsbCM15a dispatchPacket tokenINHS NO Errors"));
  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("--------------------------------------------------"));
  return(true);
}


uint8_t checkCM15a_epIN(void) {
  boolean showDebug = true;
  uint8_t x, result, packetSize;
  uint16_t transferLength = 0;
  
  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("checkCM15a_epIN do dispatchPacket - check for X10 IN data"));

  Serial.println(F("checkCM15a_epIN clear bmRCVDAVIRQ"));
  writeReg(rHIRQ, bmRCVDAVIRQ);
  if (showDebug) Serial.println("");
  
    packetSize = readReg(rRCVBC);
    if (showDebug) Serial.print(F("checkCM15a_epIN Pre - READ in "));
    if (showDebug) Serial.print(packetSize);
    if (showDebug) Serial.println(F(" bytes of data from SPI: "));
    if (packetSize >0) {
      for (x=0; x< 80; x++) spiInBuf[x] = 0xDD;
      doBytes_read(rRCVFIFO, packetSize, &spiInBuf[0]);
      Serial.print(F("checkCM15a_epIN Pre - READ in "));
      for (x=0; x<packetSize; x++) {
      Serial.print(spiInBuf[x],HEX);
      Serial.print(" ");
      }
    Serial.println("");
    }
  Serial.println(F("checkCM15a_epIN clear bmRCVDAVIRQ"));
  writeReg(rHIRQ, bmRCVDAVIRQ);         // clear interrupt
  if (showDebug) Serial.println("");

  Serial.println(F("checkCM15a_epIN do dispatchPacket check"));
  result = dispatchPacket(tokenIN, epINnum, true);
  if (result) {
    if (result == hrNAK) {    // (result == hrBUSY)
      if (showDebug) Serial.println(F("checkCM15a_epIN dispatchPacket result error 4=hrNAK, no data: "));
      if (showDebug) Serial.println(F("checkCM15a_epIN hrNAK no data?: "));
      return(result);
    } else {
      Serial.print(F("checkCM15a_epIN result error hrBUSY=1 hrTOGERR=6: "));
      Serial.println(result);
      return(result);
    }
  } else {
    if (showDebug) Serial.println(F("checkCM15a_epIN dispatchPacket result OK, Data waiting to read."));
  
    // load false data for trouble shooting
    for (x=0; x< 10; x++) spiInBuf[x] = 0xDD;

    if ( (readReg(rHIRQ) & bmRCVDAVIRQ) == 0) {
      Serial.println(F("checkCM15a_epIN transferIn bmRCVDAVIRQ error..."));
      return(99);
    }
    packetSize = readReg(rRCVBC);
    if (showDebug) Serial.print(F("checkCM15a_epIN transferIn - READ in "));
    if (showDebug) Serial.print(packetSize);
    if (showDebug) Serial.println(F(" bytes of data from SPI: "));

    doBytes_read(rRCVFIFO, packetSize, &spiInBuf[transferLength]);

    Serial.print(F("checkCM15a_epIN doBytes_read HEX: "));
    for (x=0; x<packetSize; x++) {
      Serial.print(spiInBuf[x],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    writeReg(rHIRQ, bmRCVDAVIRQ);          // clear interrupt

    if (showDebug) Serial.println(F("checkCM15a_epIN Do bmRcvToggle_epIN"));
    result = readReg(rHCTL) & 0x0F; 
    bmRcvToggle_epIN = bmRcvToggle_epIN == bmRCVTOG0 ? bmRCVTOG1 : bmRCVTOG0;
    result = result | bmRcvToggle_epIN;
    result = result | bmSndToggle_epIN;
    writeReg(rHCTL, result);

    if (showDebug) Serial.println("");

    packetSize = readReg(rRCVBC);
    if (showDebug) Serial.print(F("checkCM15a_epIN Post - READ in "));
    if (showDebug) Serial.print(packetSize);
    if (showDebug) Serial.println(F(" bytes of data from SPI: "));

    //transferLength += packetSize;        // multi packet update - location for next start point.
    }
  if (showDebug) Serial.println("");
  return(0);
}


uint8_t drianEpIn(void) {
  boolean showDebug = true;
  uint8_t x,result,packetSize;

  result = readReg(rHCTL) & 0x0F; 
  result = result | bmRcvToggle_epIN;
  result = result | bmSndToggle_epIN;
  writeReg(rHCTL, result);
    
  result = dispatchPacket(tokenIN, epINnum, true);   // token , ep, debugDisplayFirstLine
  if (result) {
    if (result == hrNAK) {    // (result == hrBUSY)
      // if (showDebug) Serial.println(F("checkCM15a_epIN dispatchPacket result 4=hrNAK, no data: "));
      return(result);
    } else {
      Serial.print(F("drianEpIn result error hrBUSY=1 hrTOGERR=6: "));
      Serial.println(result);
      return(result);
    }
  } else {
    if (showDebug) Serial.println(F("drianEpIn dispatchPacket result OK, Data waiting to read."));
  
    for (x=0; x< 10; x++) spiInBuf[x] = 0xDD;  // load false data for trouble shooting

    if ( (readReg(rHIRQ) & bmRCVDAVIRQ) == 0) {
      Serial.println(F("drianEpIn transferIn bmRCVDAVIRQ error..."));
      return(99);
    }
    packetSize = readReg(rRCVBC);
    if (showDebug) Serial.print(F("drianEpIn transferIn - READ in "));
    if (showDebug) Serial.print(packetSize);
    if (showDebug) Serial.println(F(" bytes of data from SPI: "));
    doBytes_read(rRCVFIFO, packetSize, &spiInBuf[0]);

    Serial.print(F("drianEpIn doBytes_read HEX: "));
    for (x=0; x<packetSize; x++) {
      Serial.print(spiInBuf[x],HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    writeReg(rHIRQ, bmRCVDAVIRQ);          // clear interrupt

    result = readReg(rHCTL) & 0x0F; 
    bmRcvToggle_epIN = bmRcvToggle_epIN == bmRCVTOG0 ? bmRCVTOG1 : bmRCVTOG0;
    result = result | bmRcvToggle_epIN;
    result = result | bmSndToggle_epIN;
    writeReg(rHCTL, result);
    Serial.println("");
  }
  return(spiInBuf[0]);
}


uint8_t get_CM15a_StatusDump(void) {  // get CM15A status
  boolean showDebug = true;
  uint8_t result;
  char convStr[10];

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("[ get_CM15a_StatusDump Begin ] "));

  result = readReg(rHCTL) & 0x0F; 
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);
  
  result = readReg(rHIEN);
  result |= bmHXFRDNIE;      // clear bit 0x80, SIE bit set when it has completed a host transfer
  writeReg(rHIEN, result);

  result = readReg(rHIRQ) & bmSNDBAVIRQ;  // bmSNDBAVIRQ, SIE set after send data in SNDFIFO to perpheral
  if (showDebug) Serial.print(F("get_CM15a_StatusDump rHIRQ bmSNDBAVIRQ 0x08: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.byteArray[0] = x10CmdStat;
  doBytes_write(rSNDFIFO, 1);
  writeReg(rSNDBC,1);
  if (showDebug) Serial.print(F("get_CM15a_StatusDump rSNDBC writeReg: "));
  if (showDebug) Serial.println(x10CmdStat,HEX);
  
  result = dispatchPacket(tokenOUT, epOUTnum, true);
  if (result > 0) {
    if (showDebug) Serial.print(F("get_CM15a_StatusDump dispatchPacket result: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
  } else if (showDebug) Serial.println(F("get_CM15a_StatusDump dispatchPacket done"));

  if (showDebug) Serial.println(F("[ get_CM15a_StatusDump Done ] "));
  if (showDebug) Serial.println("");

  result = readReg(rHCTL) & 0x0F; 
  bmSndToggle_epOUT = bmSndToggle_epOUT == bmSNDTOG0 ? bmSNDTOG1 : bmSNDTOG0;
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);
    
  return(true);
}


// Basically we send the time update packet and receive nothing back, as follows: 
// Sample Send: 9b 30 45 0a 3e 81 60 00
uint8_t CM15a_SetTime(void) {    // set date and time, to stop A5's from CM15A
  boolean showDebug = true;
  uint8_t result;
  char convStr[10];

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("[ CM15a_SetTime Begin ] "));

  result = readReg(rHCTL) & 0x0F; 
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);
  
  result = readReg(rHIEN);
  result |= bmHXFRDNIE;      // clear bit 0x80, SIE bit set when it has completed a host transfer
  writeReg(rHIEN, result);

  result = readReg(rHIRQ) & bmSNDBAVIRQ;  // bmSNDBAVIRQ, SIE set after send data in SNDFIFO to perpheral
  if (showDebug) Serial.print(F("CM15a_SetTime rHIRQ bmSNDBAVIRQ 0x08: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.byteArray[0] = x10SetTime;  // x10SetTime = 0x9B command to set data time
  setupUSB.byteArray[1] = 0x01;        // sec
  setupUSB.byteArray[2] = 0x02;        // minutes   0 to 119 odd
  setupUSB.byteArray[3] = 0x03;        // hours/2   0 to 11
  setupUSB.byteArray[4] = 0x3e;
  setupUSB.byteArray[5] = 0x81;
  setupUSB.byteArray[6] = 0x60;
  setupUSB.byteArray[7] = 0x00;

  doBytes_write(rSNDFIFO, 8);
  writeReg(rSNDBC,8);
  Serial.print(F("CM15a_SetTime rSNDBC writeReg: "));
  Serial.println(x10CmdStat,HEX);
  
  result = dispatchPacket(tokenOUT, epOUTnum, true);
  if (result > 0) {
    if (showDebug) Serial.print(F("CM15a_SetTime dispatchPacket result: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
  } else if (showDebug) Serial.println(F("CM15a_SetTime dispatchPacket done"));

  if (showDebug) Serial.println(F("[ CM15a_SetTime Done ] "));
  if (showDebug) Serial.println("");

  result = readReg(rHCTL) & 0x0F; 
  bmSndToggle_epOUT = bmSndToggle_epOUT == bmSNDTOG0 ? bmSNDTOG1 : bmSNDTOG0;
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);
  
  return(true);
}


//                                ?c_index used for array access
uint8_t X10_Output_Cmds(uint8_t hc_index, uint8_t dc_index, uint8_t action) {
  boolean showDebug = false;
  uint8_t result;
  uint16_t delayTime;
  uint8_t houseDeviceBin,houseActionBin;
  char convStr[10];

  if (showDebug) Serial.println("");
  if (showDebug) Serial.println(F("[ X10_Output_Cmds Begin ] "));

  houseActionBin = x10HouseCodes[hc_index] | 0x03;  // failsafe to off if bad value
  
  houseDeviceBin = x10HouseCodes[hc_index] | x10DeviceCodes[dc_index];
  if (action == 2)       // On
    houseActionBin = x10HouseCodes[hc_index] | 0x02;
  if (action == 3)       // Off
    houseActionBin = x10HouseCodes[hc_index] | 0x03;
    
  // debug info
  if (showDebug) Serial.print(F("houseDeviceBin: "));
  if (showDebug) Serial.println(houseDeviceBin,HEX);
  
  if (showDebug) Serial.print(F("houseActionBin: "));
  if (showDebug) Serial.println(houseActionBin,HEX);

  result = readReg(rHCTL) & 0x0F; 
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);

  result = readReg(rHIEN);
  result |= bmHXFRDNIE;      // clear bit 0x80, SIE bit set when it has completed a host transfer
  writeReg(rHIEN, result);
  
  result = readReg(rHIRQ) & bmSNDBAVIRQ;  // bmSNDBAVIRQ, SIE set after send data in SNDFIFO to perpheral
  if (showDebug) Serial.print(F("X10_Output_Cmds rHIRQ bmSNDBAVIRQ 0x08: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.byteArray[0] = x10CmdAddr;
  setupUSB.byteArray[1] = houseDeviceBin;
  doBytes_write(rSNDFIFO, 2);
  writeReg(rSNDBC,2);
  if (showDebug) Serial.print(F("X10_Output_Cmds rSNDBC 2 bytes writeReg: "));
  if (showDebug) Serial.print(setupUSB.byteArray[0],HEX);
  if (showDebug) Serial.print(" ");
  if (showDebug) Serial.println(setupUSB.byteArray[1],HEX);
  
  result = dispatchPacket(tokenOUT, epOUTnum, true);
  if (result > 0) {
    if (showDebug) Serial.print(F("X10_Output_Cmds dispatchPacket ERROR: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
  }

  result = readReg(rHCTL) & 0x0F; 
  bmSndToggle_epOUT = bmSndToggle_epOUT == bmSNDTOG0 ? bmSNDTOG1 : bmSNDTOG0;
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);

  result = x10LongWait(1,&delayTime);
  Serial.print(F("Wait Response1: "));
  if (result == x10AnswAck) {           // x10AnswAck = 0x55
    Serial.print(delayTime);
    Serial.println(F(" ms"));
  }
  else Serial.println(result,HEX);
  
  Serial.println(F("===="));

  result = readReg(rHCTL) & 0x0F; 
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);

  result = readReg(rHIEN);
  result |= bmHXFRDNIE;      // clear bit, SIE bit set when it has completed a host transfer
  writeReg(rHIEN, result);

  result = readReg(rHIRQ) & bmSNDBAVIRQ;
  if (showDebug) Serial.print(F("X10_Output_Cmds rHIRQ bmSNDBAVIRQ 0x08: "));
  if (showDebug) byteToBinPrint(result, &convStr[0]);
  if (showDebug) Serial.println(convStr);
  
  setupUSB.byteArray[0] = x10CmdType;
  setupUSB.byteArray[1] = houseActionBin;
  doBytes_write(rSNDFIFO, 2);
  writeReg(rSNDBC,2);
  if (showDebug) Serial.print(F("X10_Output_Cmds rSNDBC 2 bytes writeReg2: "));
  if (showDebug) Serial.print(setupUSB.byteArray[0],HEX);
  if (showDebug) Serial.print(" ");
  if (showDebug) Serial.println(setupUSB.byteArray[1],HEX);
  
  result = dispatchPacket(tokenOUT, epOUTnum, true);
  if (result > 0) {
    if (showDebug) Serial.print(F("X10_Output_Cmds dispatchPacket ERROR2: "));
    if (showDebug) byteToBinPrint(result, &convStr[0]);
    if (showDebug) Serial.println(convStr);
  }

  result = x10LongWait(2,&delayTime);
  Serial.print(F("Wait Response2: "));
  if (result == x10AnswAck) {
    Serial.print(delayTime);
    Serial.println(F(" ms"));
  }
  else Serial.println(result,HEX);
  
  result = readReg(rHCTL) & 0x0F; 
  bmSndToggle_epOUT = bmSndToggle_epOUT == bmSNDTOG0 ? bmSNDTOG1 : bmSNDTOG0;
  result = result | bmSndToggle_epOUT;
  result = result | bmRcvToggle_epOUT;
  writeReg(rHCTL, result);

  if (showDebug) Serial.println(F("[ X10_Output_Cmds Done ] "));
  Serial.println("");
   
  return(true);
}


// sending on/off cmds via CM15A to devices can take 150 to 500 ms for the ACK to happen
uint16_t x10LongWait(uint8_t phase, uint16_t *waitUsed) {         // phase = step1 or step2
  unsigned long startMsTime;
  unsigned long currentMsTime = 0L;
  uint8_t result, x, packetSize;
  uint16_t totalDelay;
  
  result = readReg(rHCTL) & 0x0F; 
  result = result | bmRcvToggle_epIN;
  result = result | bmSndToggle_epIN;
  writeReg(rHCTL, result);

  startMsTime = millis();
  while (result > 0) {
    delay(10);
    result = dispatchPacket(tokenIN, epINnum, true);
    if (result) {
      if (result == hrNAK) {    // hrNAK = no data so far
        //return(result);
      } else {
        Serial.print(F("x10LongWait error hrBUSY=1 hrTOGERR=6: "));
        Serial.println(result);
        return(result);
      }
    }
    currentMsTime = millis();
    if (currentMsTime > (startMsTime + 1000L) ) return(hrJERR);
  }

// Data ready to read in

  for (x=0; x<10; x++) spiInBuf[x] = 0xDD;   // load false data for trouble shooting

  if ( (readReg(rHIRQ) & bmRCVDAVIRQ) == 0) {
    Serial.print(F("x10LongWait rHIRQ bmRCVDAVIRQ error, in phase: "));
    Serial.println(phase);
    return(99);
  }
  packetSize = readReg(rRCVBC);
  doBytes_read(rRCVFIFO, packetSize, &spiInBuf[0]);

  writeReg(rHIRQ, bmRCVDAVIRQ);              // clear interrupt

  result = readReg(rHCTL) & 0x0F; 
  bmRcvToggle_epIN = bmRcvToggle_epIN == bmRCVTOG0 ? bmRCVTOG1 : bmRCVTOG0;
  result = result | bmRcvToggle_epIN;
  result = result | bmSndToggle_epIN;
  writeReg(rHCTL, result);

  totalDelay = uint16_t(currentMsTime - startMsTime);
  *waitUsed = totalDelay;
  return(spiInBuf[0]);
}


void showCmds(void) {
  Serial.println(F("Use Arduino Serial Monitor for testing"));
  Serial.println(F("Sample send command: A,1,2"));
  Serial.println(F("Turns On HouseCode=A, DeviceCode=1, ON=2"));
  Serial.println("");
  Serial.println(F("H,D,2 = Turn device on"));
  Serial.println(F("H,D,3 = Turn device off"));
  Serial.println(F("5 = CM15A status"));
  Serial.println(F("6 = Set CM15A time / date / etc"));
  Serial.println("");
}


void loop() {
  uint8_t House,Device,Action;
  String cmdString,wStr;
  char workstr[80];

  if (errorSoWhileForever) {
    Serial.println("");
    Serial.println(F("Had setup error, so program stopped."));
    while(1);
  }

  if (Serial.available() > 0) {
    cmdString = Serial.readString();
    if (cmdString.length() > 5) {
      cmdString.toUpperCase();

      // calc index values
      House = cmdString[0] -'A';     // array x10HouseCodes[] indexed from 0 , 0 = 'A'
      
      if (cmdString[3] == ',') {
        wStr = cmdString[2];
        Device = wStr.toInt() - 1;

        wStr = cmdString[4];
        Action = wStr.toInt();
      } else {
        wStr = cmdString[2];
        wStr.concat(cmdString[3]);
        Device = wStr.toInt() - 1;

        wStr = cmdString[5];
        Action = wStr.toInt();
      }
      
      if (Action == 2) {                         // 2 = On
        X10_Output_Cmds(House,Device,Action);
        showCmds();
        }
      if (Action == 3) {                         // 3 = Off
        X10_Output_Cmds(House,Device,Action);
        showCmds();
        }
    }
  if (cmdString.length() < 3) {
    
      Action = cmdString[0];
      sprintf(workstr," Action %c",Action);
      Serial.println(workstr);
      
      if (Action == '5') {                // dump memory status
        get_CM15a_StatusDump();
        delay(50);
        drianEpIn();
        delay(50);
        drianEpIn();
        delay(50);
        drianEpIn();
        showCmds();
        }
      if (Action == '6') {                   // set date time
        CM15a_SetTime();    // cold boot needs settime, no ACK response from settime, else A5's
        showCmds();
        }
      }
    }
  drianEpIn();  // clear stray A5's from CM15A that has not had it's date/time set
  delay(10);
}
