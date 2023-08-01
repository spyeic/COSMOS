//*******************************************************/
/************************************************************************/
/*  Author:  Shirley Chiang                                             */
/*  Copyright 2022, University of California, Davis                     */
/************************************************************************/
/*  New version for Teensy 4.1, July 2022                                                                     */
/*  Approach procedure with Tunneling Feedback under Interrupt Control  */
/*  Data file sent to PC over second serial port                        */
/*  Read tunneling current ADS8691 18 bit ADC, straight binary for -12.488V to +12.488V   */
/*  Quad DAC8734, 16 bit 2's complement, +32767 = +14.356V; 32768=-14.372V */
/*                                                                      */
/* Incorporate code for small stepper motor 28byj-48 5V  */
/* Using Arduino Stepper library                         */
/* Steps per output revolution 32*64 =2048               */
/***********************************************************************/

/************************************************************************/
/*  Board Support:  Teensy 4.1 processor  					                            */
/************************************************************************/
#include <PID_vInt.h>             // PID library for feedback control of tunneling current
#include <Wire.h>                 //Include I2C library
#include <LiquidCrystal_I2C.h>    //Include Liquid Crystal library
#include <Stepper.h>
#include <SD.h>
#include <SPI.h>                  //include SPI library
#include <Adafruit_GFX.h>    // Core graphics library
#include <ili9488_t3_font_Arial.h>
#include <ili9488_t3_font_ArialBold.h>
#include <ILI9488_t3.h>
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSerif12pt7b.h>


const int chipSelect = BUILTIN_SDCARD;  //to read microSD card on Teensy
// Filename for SD card, format "8 chars.3 chars"
char FileNameF[12]="DF1.hex";   //forward data file
char FileNameB[12]="DB1.hex";   //backward data file
char FileNamePars[12]="DP.txt"; //text file with parameters
char FileNameNumberStr[12];
short int FileNameNumber=1;
File dataFileF, dataFileB, parsFile;
int LengthLine;

//ILI9488 parameters, using SPI1, pins 35,36,37
#define TRY_EXTMEM
#ifdef TRY_EXTMEM
#if !defined(ARDUINO_TEENSY41)
#undef TRY_EXTMEM
#if defined(ENABLE_EXT_DMA_UPDATES)
#error "This Version only works with Teensy 4.1 with External memory"
#endif
#endif
#endif

#define ROTATION 3

#define USE_SPI1
#if defined(USE_SPI1)
#if defined(__IMXRT1062__)  // Teensy 4.x 
#define TFT_RST 36
#define TFT_DC 37
#define TFT_CS 35
#define TFT_SCK 27
#define TFT_MISO 1
#define TFT_MOSI 26
#endif
#endif

ILI9488_t3 tft = ILI9488_t3(&SPI1, TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);
uint16_t our_pallet[] = {
  ILI9488_BLACK,  ILI9488_RED, ILI9488_GREEN,  ILI9488_BLUE,   ILI9488_WHITE,
  ILI9488_YELLOW, ILI9488_ORANGE, ILI9488_CYAN, ILI9488_PINK
};

Adafruit_GFX_Button button;

// Let's allocate the frame buffer ourself.
// BUGBUG: IF RAFB is 4 this won't fit
#if ! defined(ENABLE_EXT_DMA_UPDATES)
DMAMEM RAFB tft_frame_buffer[ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT];
#endif

#ifdef TRY_EXTMEM
EXTMEM RAFB extmem_frame_buffer[ILI9488_TFTWIDTH * ILI9488_TFTHEIGHT];
#endif

uint8_t use_extmem = 0;
uint8_t use_dma = 0;
uint8_t use_clip_rect = 0;
uint8_t use_set_origin = 0;
uint8_t use_fb = 0;

#define ORIGIN_TEST_X 50
#define ORIGIN_TEST_Y 50


#define DEFNUMPTS 512     // Default number of points and linesvalues when starting program
#define DEFNUMLINES 512
#define MAXNUMPTS 512    // Maximum values for array dimensions
#define MAXNUMLINES 512

// Create an IntervalTimer object
IntervalTimer myTimer1;   // use for PID interrupt
IntervalTimer myTimer2;   // use for timing interval between data points

//pins to monitor interrupt timing
const int Timer1Pin = 40;
const int Timer2Pin = 41;
const int ScanLineTrigger = 39;
bool Timer1Flag = false;
bool Timer2Flag = false;

#define FORWARD 1			// Directions for stepping motor
#define BACKWARD -1
//---( Number of steps per revolution of INTERNAL motor in 4-step mode )---
#define STEPS_PER_MOTOR_REVOLUTION 32
//---( Steps per OUTPUT SHAFT of gear reduction )---
#define STEPS_PER_OUTPUT_REVOLUTION 32 * 64  //2048  

/*-----( Declare objects )-----*/
// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins to which it is attached
//
//The pin connections need to be 4 pins connected
// to Motor Driver In1, In2, In3, In4  and then the pins entered
// here in the sequence 1-3-2-4 for proper sequencing
// Connect In1, In2, In3, In4 to Arduino pins 28,29,30,31
Stepper small_stepper(STEPS_PER_MOTOR_REVOLUTION, 28, 30, 29, 31);

unsigned int steps = 1; // set default to walk one step
int savedirection = FORWARD; // initial direction is FORWARD, sample moves towards tip

// Bias voltage will be set to +1.0V; corresponding 16 bit 2's complement = 2282
double Voltage = 1.0;
short int bias = 2282; //   =1.0*32767/14.356, send to DAC 3

//Initialize x and y to 0.0V
double x = 0;
double y = 0;
short int xv, yv;
short int xvinit = 0;  // send to DAC 1
short int yvinit = 0;  // send to DAC 2
boolean fscan = true;		// forward scan direction initially
boolean xscan = false;	// xscan off initially
boolean yscan = false;	// yscan off initially

//Pointers to 2D data arrays
EXTMEM int16_t DataArray1F[MAXNUMLINES][MAXNUMPTS];  // storage for forward data array 1
EXTMEM int16_t DataArray1B[MAXNUMLINES][MAXNUMPTS];  // storage for backward data array 1

//Buffer parameters
int i, j;
float LineBufferF[MAXNUMPTS];
float LineBufferB[MAXNUMPTS];
unsigned short int buffer1[2 * MAXNUMPTS];  // to store forward and backward data arrays
unsigned short int buffer2[2 * MAXNUMPTS];
short int *buf;
int numbyteswritten;
int totalNumbyteswritten;
unsigned long time1, time2, timediff, totaltime;
boolean writebuffer1 = false; // flag for writing buffer1 data to serial port
boolean writebuffer2 = false; // flag for writing buffer2 data to serial port
boolean usebuffer1 = true; // flag for storing data in buffer 1
// if usebuffer1=false, then data is stored in buffer 2

// Image parameters
int numpoints = DEFNUMPTS;  // default number of points in scan line
int numlines = DEFNUMLINES; // default number of lines in image
int stepsize = 1;  // stepsize in DAC counts
int TimeInterval_mSec = 3;  // use interrupt timer 2 to time acquisition of data points
int TimeInterval_uSec = 3000; // initial value 3mS=3000uS
int pointcounter = 0;  // current point number
boolean imagedone = true;    //flag for image finished
int linenumber = 0;    // current scan line number
int counter = 0;
//unsigned short int z[MAXNUMLINES][MAXNUMPTS]; // array to save z values during scan

//Setup PID parameters
double Setpoint, Input, Output;
//Specify initial tuning parameters per mS
// double Kp=0.05, Ki=0.001, Kd=0.;    // Initial values for PID tuning
// double Kp = 0.1, Ki = 0.0005, Kd = 0.; // Initial values for PID tuning, 7/23/2017
double Kp=1.0, Ki=0.00001, Kd=0.0;      // Initial values for PID tuning, 8/4/2022

bool FlagFeedbackOn = false;  // Flag to turn on Feedback, initially off
// Tunneling current in nA
double Current = 1.0;
//Tunnel current setpoint for Shirley's preamp
// 1nA through 500MOhm gives PreAmpOut = -0.5V

// Voltage divider 0.321 on input of Analog Shield gives A/D input -0.1605
// Analog shield A/D Count = [(32767*ADIn)/5.]+32767 = 31715

// 1 Nanoamp corresponds to -0.5V out put from preamp, with 500MOhm feedback resistor
// Digitize this value with 18 bit ADC i=in ADS8691 chip
// 0 is -12.288V, 0x3FFFF=2^18=262144 is +12.288V; 2^17=131072 corresponds to 0V
// [(-0.5V/12.288V)*131072]+131072 = 125739
#define NANOAMP 125739  // 2's complement integer value corresponding to 1.0nA

unsigned int IntegerCurrent;

//Initialize z to 0 (z piezo full retracted), when voltage applied to brass piece
//Initialize z to 65535 (z piezo full retracted), when z voltage applied to quadrants on back
short int zout = 32767; // z piezo full retracted, corresponds to +14.356V
// full forward would be 32768=-14.372V

// DIRECT direction for PID control for Shirley's xyz amplifiers on breadboard
// REVERSE for Shirley's amps, now that another stage was added for z
// DIRECT direction when z voltage applied to back of piezo instead of brass piece!
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

boolean apprflag = false;  //Turn approach off initially
boolean rampflag = true; //Ramp on for approach initially
boolean walkapprflag = true; // Walk 1 step during approach, initially on

//LCD parameters: Addr 0x27 for COSMOS displays, 20 chars & 4 lines
//LCD parameters: Addr 0x3F for Physics 122A displays, 20 chars & 4 lines
LiquidCrystal_I2C lcd(0x27, 20, 4);
float lcdInput, lcdOutput;   // variables to display on lcd

//Parameters for DAC8734
#define DAC_8734_CS_PIN  4  // Set chip select to pin 4
#define DAC_8734_LDAC_PIN 5 // Set Ldac bar to pin 5
#define DAC_8734_RESET_PIN 6  // Set Reset bar to pin 6
#define VREF 7.500
//#define VREF 5.000

// Default values below assume 0-10V Unipolar or -5 to +5 bipolar output
#define DAC_VFSR VREF*2 // may be 0 - 10 or -5 to +5
#define DAC_8734_Command 0 // command register
#define DAC_8734_DataBase 0x04 // register offset to zero cal base register
#define DAC_8734_CalZeroBase 0x08 // register offset to zero cal base register
#define DAC_8734_CalGainBase 0x0C // register offset to gain cal base register
#define DACMAX 0xFFFF
#define DACMIN 0x0000

// Parameters for ADS8691 18 bit ADC chip
#define ADS_8691_CS_PIN  7  // Set chip select bar to pin 7
#define ADS_8691_RVS_PIN 8  // Set pin 8 to monitor RVS
#define ADS_8691_RST_PIN 9  // Set pin 9 to Reset bar
unsigned int Data18Bits;
bool ReadWriteCmd = false;

// 32-bit spi buffer for ADC
// declared as a uint32_t for alignment reasons
// also allows it to be written (e.g. to a nop with spi_buffer_adc_32=0) with a single STRD instruction
uint32_t spi_buffer_adc_32;
uint8_t *spi_buffer_adc=(uint8_t*)&spi_buffer_adc_32;

//Creates 32 bit union of 3 different types of data for ADC input
// 4 bytes, one 32-bit integer, or two 16-bit integers
typedef volatile union ADC_Data
{ unsigned char ByteBuffer[4];
  unsigned int IntData;
  uint16_t Int16Data[2];
} DataIn32bits;
volatile DataIn32bits DataIn;   // Define union variable name DataIn

// ADC routines for ADS8691 follow

// define the buffer size...
#define serialbufferSize 50
#define commandDelimeters "|,.- "

// Now the real variables
char inputBuffer[serialbufferSize]   ;
int serialIndex = 0; // keep track of where we are in the buffer
byte SPI_Buffer[4];

// End of real variables

//********************************************************
// ADS8691 code starts here
//********************************************************

// set up the speed, data order and data mode
// See https://www.arduino.cc/en/Reference/SPI for more details

SPISettings settingsADS8691(25000000, MSBFIRST, SPI_MODE0);     // 25MHz, setting mode 0

// The ADS8691 uses Mode 0 SPI, has an 8 bit address byte followed by 3 bytes of data
// high byte first, high bit first
// This is the register descriptions
// Reg addr,  description
const unsigned int DEVICE_ID_REG = 0;
const unsigned int RST_PWRCTL_REG = 0x04;
// 08H  SDI_CTL_REG
// 0CH  SDO_CTL_REG
// 10H  DATAOUT_CTL_REG
// 14H  RANGE_SEL_REG
// 20H  ALARM_REG
// 24H  ALARM_H_TH_REG
// 28H  ALARM_L_TH_REG

unsigned int BusData = 0;
const unsigned int ReadCmd = 0xc8000000; //Command to read half word of 16 bits
const unsigned int ReadHalfWordPWRCTLREG = 0xc8040000; // Read RST_PWRCTL_REG
const unsigned int Write2BytesPWRCTLREG = 0xd0046900; // Write 2 bytes to PWRCTL_REG
const unsigned int ReadHalfWordRANGESELREG = 0xc8140000; // Read RANGE_SEL_REG
const unsigned int WriteByteRANGESELREG = 0xd4140000; // Write byte to RANGE_SEL_REG

const unsigned int ReadByteCmd = 0x48000000; //Command to read byte of 8 bits
const unsigned int WriteCmd = 0xd0000000; //Command to Write half word of 16 bits
unsigned int BusCmd;
unsigned int ReadAdr = 0;
int param = 0;

// Reset ADC
void ResetADC(void)
{ digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  digitalWrite(ADS_8691_RST_PIN, LOW); // Select the Chip
  digitalWrite(ADS_8691_RST_PIN, HIGH); // Release the Chip
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Start conversion
}

// do an adc transfer, with appropriate timing. Does *not* call SPI.beginTransaction or endTransaction,
// since multiple transfers may need to be done without interrupts in between, so code calling this function
// should do that manually

// also doesn't check RVS: again, this should be done manually if/when it is appropriate
inline void adc_transfer(byte *spi_buffer,size_t buffer_length) {
  digitalWrite(ADS_8691_CS_PIN,LOW);
  delayNanoseconds(8); // tSU_CSCK in datasheet
  SPI.transfer(spi_buffer,buffer_length);
  delayNanoseconds(8); // tHT_CKCS in datasheet
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
}

/*
// Read value of 32-bit register reg.
uint32_t Read_ADC_register(byte reg)
{ reg &= 0xFC;
  uint8_t spi_buffer[4];
  uint32_t retval;
  spi_buffer[0]=0xC8; //read_hword
  spi_buffer[1]=reg;
  spi_buffer[2]=0;
  spi_buffer[3]=0;
  SPI.beginTransaction(settingsADS8691); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  delayNanoseconds(8); // tSU_CSCK in datasheet
  SPI.transfer(spi_buffer,4);
  delayNanoseconds(8); // tHT_CKCS in datasheet
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  //Don't end transaction yet! This would allow interrupts to corrupt the read.

  for(int i=0; i<65535; ++i) if (digitalRead(ADS_8691_RVS_PIN)) break;
  if(i==65535) Serial.println("Error: RVS pin not high after a long time. May cause problems.");

  spi_buffer[0]=0xC8; //read_hword
  spi_buffer[1]=reg+2; //read second half of register
  spi_buffer[2]=0;
  spi_buffer[3]=0;
  
  digitalWrite(ADS_8691_CS_PIN, LOW
}*/

void Read2Bytes(int cmdreg)
{ SPI.beginTransaction(settingsADS8691); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  ADC_Command(cmdreg);
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  SPI_Buffer[0] = SPI.transfer(0); // Read high order byte
  SPI_Buffer[1] = SPI.transfer(0); // Read low order byte
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  SPI.endTransaction();
  ReadWriteCmd = true;
}

void WriteByte(int cmdreg, int param)
{ unsigned int cmd;
  cmd = (param & 0xFFFF);
  Serial.print("cmdreg = ");
  Serial.println(cmdreg, HEX);
  cmd = cmdreg | cmd;
  Serial.print("cmd = ");
  Serial.println(cmd, HEX);
  SPI.beginTransaction(settingsADS8691); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  ADC_Command(cmd);   // Send out 4 byte command over SPI bus
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  //  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  //  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  SPI.endTransaction();
  ReadWriteCmd = true;
}

void ADC_Command(int reg)   //sends 4 byte command to ADC
{ byte Transferbyte = 0;
  SPI.beginTransaction(settingsADS8691); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(ADS_8691_CS_PIN, LOW); // Select the Chip
  //  Serial.println("Printing 4 transfer bytes for command");
  Transferbyte = reg >> 24;
  // Serial.println(Transferbyte, HEX);
  SPI.transfer(Transferbyte); // Highest byte first
  Transferbyte = reg >> 16;
  // Serial.println(Transferbyte, HEX);
  SPI.transfer(Transferbyte); // 2nd highest byte
  Transferbyte = reg >> 8;
  // Serial.println(Transferbyte, HEX);
  SPI.transfer(Transferbyte); // 3rd highest byte
  Transferbyte = reg;
  // Serial.println(Transferbyte, HEX);
  SPI.transfer(Transferbyte); // Lowest data byte
  digitalWrite(ADS_8691_CS_PIN, HIGH); // Release the Chip
  SPI.endTransaction();
}

uint32_t ADC_Convert()
{ 
  SPI.beginTransaction(settingsADS8691); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  for (int i = 0; i < 1000; ++i)
    if (digitalReadFast(ADS_8691_RVS_PIN) == HIGH)  break;
    
  spi_buffer_adc_32=0; //send a NOP to the ADC to request a conversion

  adc_transfer(spi_buffer_adc,4);
  for (int i = 0; i < 1000; ++i)
    if (digitalReadFast(ADS_8691_RVS_PIN) == HIGH)  break;

  spi_buffer_adc_32=0;    //send another NOP to the ADC, conversion result will be returned in spi_buffer_adc
  adc_transfer(spi_buffer_adc,4);
  
  SPI.endTransaction();

  // conversion result returned in D[31:14]
  // spi_buffer_adc[0]=D[31:24]
  // spi_buffer_adc[1]=D[23:16]
  // spi_buffer_adc[2]=D[15:8], so spi_buffer_adc[2] >> 6 is D[15:14]
  // then shifting these appropriately an bitwise OR gives the result with proper endianness
  // result: {D[31:24],D[23:16],D[15:14]}=D[31:14]
  return (spi_buffer_adc[0] << 10) | (spi_buffer_adc[1] << 2) | (spi_buffer_adc[2] >> 6);
}

//DAC8734 routines follow
// define the buffer size...
#define serialbufferSize 50
#define commandDelimeters "|,.- "

// Now the real variables
//char inputBuffer[serialbufferSize]   ;
//int serialIndex = 0; // keep track of where we are in the buffer
// End of real variables

//********************************************************
// DAC8734 code starts here
//********************************************************

// PICK THE RIGHT chip select for the right board
#define DAC_8734_CS_PIN  4  // Set chip select to pin 4
#define DAC_8734_LDAC_PIN 5 // Set Ldac bar to pin 5
#define DAC_8734_RESET_PIN 6  // Set Reset bar to pin 6

#define VREF 7.500
//#define VREF 5.000
// Default values below assume 0-10V Unipolar or -5 to +5 bipolar output
#define DAC_VFSR VREF*2 // may be 0 - 10 or -5 to +5
#define DAC_8734_Command 0 // command register
#define DAC_8734_DataBase 0x04 // register offset to zero cal base register
#define DAC_8734_CalZeroBase 0x08 // register offset to zero cal base register
#define DAC_8734_CalGainBase 0x0C // register offset to gain cal base register
#define DACMAX 0xFFFF
#define DACMIN 0x0000

// default setup of DAC8734, these can be adjusted if needed, some functions also change them to change mode etc.
int DAC_Gain0 = 1; // 0 = *2, 1 = *4
int DAC_Gain1 = 1; // 0 = *2, 1 = *4
int DAC_Gain2 = 1; // 0 = *2, 1 = *4
int DAC_Gain3 = 1; // 0 = *2, 1 = *4
int DAC_GPIO0 = 1; // Not connected. 1 = Group A in Unipolar 0=Bipolar (External connection to control pin)
int DAC_GPIO1 = 1; // Not connected. 1 = Group B in Unipolar 0=Bipolar (External connection to control pin)
int DAC_PD_A = 0;  // 1 = group A power down
int DAC_PD_B = 0;  // 1 = group B power down
int DAC_DSDO = 0;  // 1 = Disable SDO bit.

// set up the speed, data order and data mode
// See https://www.arduino.cc/en/Reference/SPI for more details

SPISettings settingsDAC8734(20000000, MSBFIRST, SPI_MODE1);     //20MHz, setting mode 1

unsigned char ByteRead[3];

// Sine table, used to create a reasonable sinewave on a dac output, 256 16bit values, one complete cycle
byte val = 0;
unsigned int Sin_tab[256] = {  32768, 33572, 34376, 35178, 35980, 36779, 37576, 38370, 39161, 39947, 40730, 41507, 42280,
                               43046, 43807, 44561, 45307, 46047, 46778, 47500, 48214, 48919, 49614, 50298, 50972, 51636,
                               52287, 52927, 53555, 54171, 54773, 55362, 55938, 56499, 57047, 57579, 58097, 58600, 59087,
                               59558, 60013, 60451, 60873, 61278, 61666, 62036, 62389, 62724, 63041, 63339, 63620, 63881,
                               64124, 64348, 64553, 64739, 64905, 65053, 65180, 65289, 65377, 65446, 65496, 65525, 65535,
                               65525, 65496, 65446, 65377, 65289, 65180, 65053, 64905, 64739, 64553, 64348, 64124, 63881,
                               63620, 63339, 63041, 62724, 62389, 62036, 61666, 61278, 60873, 60451, 60013, 59558, 59087,
                               58600, 58097, 57579, 57047, 56499, 55938, 55362, 54773, 54171, 53555, 52927, 52287, 51636,
                               50972, 50298, 49614, 48919, 48214, 47500, 46778, 46047, 45307, 44561, 43807, 43046, 42280,
                               41507, 40730, 39947, 39161, 38370, 37576, 36779, 35980, 35178, 34376, 33572, 32768, 31964,
                               31160, 30358, 29556, 28757, 27960, 27166, 26375, 25589, 24806, 24029, 23256, 22490, 21729,
                               20975, 20229, 19489, 18758, 18036, 17322, 16617, 15922, 15238, 14564, 13900, 13249, 12609,
                               11981, 11365, 10763, 10174, 9598, 9037, 8489, 7957, 7439, 6936, 6449, 5978, 5523, 5085, 4663,
                               4258, 3870, 3500, 3147, 2812, 2495, 2197, 1916, 1655, 1412, 1188, 983, 797, 631, 483, 356, 247,
                               159, 90, 40, 11, 1, 11, 40, 90, 159, 247, 356, 483, 631, 797, 983, 1188, 1412, 1655, 1916, 2197,
                               2495, 2812, 3147, 3500, 3870, 4258, 4663, 5085, 5523, 5978, 6449, 6936, 7439, 7957, 8489, 9037,
                               9598, 10174, 10763, 11365, 11981, 12609, 13249, 13900, 14564, 15238, 15922, 16617, 17322,
                               18036, 18758, 19489, 20229, 20975, 21729, 22490, 23256, 24029, 24806, 25589, 26375, 27166,
                               27960, 28757, 29556, 30358, 31160, 31964
                            };

//Calibration table, you will need to adjust these values to suit your build
//int DAC_CAL_tab[8] = { 0x08, 0xfF, 0x0F, 0x4F,   0x3f, 0x80, 0x80, 0x80}; // zero's then gain's
int DAC_CAL_tab[8] = { 0x00, 0x00, 0x00, 0x00,   0x00, 0x00, 0x00, 0x00}; // zero's then gain's

// The DAC8734 uses Mode 1 SPI, has an 8 bit address byte followed by 16bit data
// high byte first, high bit first
// These are the register descriptions
// Reg   description
// 0    Control Register
// 1    Monitor
// 2    Not Used
// 3    Not Used
// 4    DAC 0 Data Register
// 5    DAC 1 Data Register
// 6    DAC 2 Data Register
// 7    DAC 3 Data Register
// 8    DAC 0 Zero Cal Register
// 9    DAC 1 Zero Cal Register
// a    DAC 2 Zero Cal Register
// b    DAC 3 Zero Cal Register
// c    DAC 0 Zero Gain Register
// d    DAC 1 Zero Gain Register
// e    DAC 2 Zero Gain Register
// f    DAC 3 Zero Gain Register


// the most basic function, write to register "reg", with a value "val"
void WriteDACRegister(byte reg, unsigned int val)
{ SPI.beginTransaction(settingsDAC8734); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code

  delayNanoseconds(5);

  digitalWrite(DAC_8734_CS_PIN, LOW); // Select the Chip
  
  delayNanoseconds(11); // t4 in DAC8734 datasheet: required time from CS falling edge before SCLK falling edge
  
  byte transfer_buffer[3]={reg,byte((val >> 8) & 0xFF),byte(val & 0xFF)}; // SPI transfer order: register number (1 byte), high byte of value, low byte of value
  
  /*SPI.transfer(reg & 0xFF); // Select the target register
  SPI.transfer(val >> 8); // Send the High Data Byte
  SPI.transfer(val & 0xFF); // Send the Low Data Byte*/

  SPI.transfer(transfer_buffer,3);

  delayNanoseconds(15); // t7 in DAC8734: SCLK falling edge to CS rising edge
  
  digitalWrite(DAC_8734_CS_PIN, HIGH); // Release the Chip
  
  SPI.endTransaction();
  //    Serial.print("Writing hex register number ");
  //    Serial.print(reg, HEX);
  //    Serial.print(" with hex value ");
  //    Serial.println(val, HEX);
}

void SendReadCmdRegister(byte reg, unsigned int val)
{ SPI.beginTransaction(settingsDAC8734); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(DAC_8734_CS_PIN, LOW); // Select the Chip
  SPI.transfer(reg); // Select the target register
  SPI.transfer(val >> 8); // Send the High Data Byte
  SPI.transfer(val & 0xFF); // Send the Low Data Byte
  digitalWrite(DAC_8734_CS_PIN, HIGH); // Release the Chip
  SPI.endTransaction();
  Serial.print("Writing hex register number ");
  Serial.print(reg, HEX);
  Serial.print(" with hex value ");
  Serial.println(val, HEX);
}


void Read3Bytes (void)
{ SPI.beginTransaction(settingsDAC8734); // ensures the SPI bus is in the right mode for this device irrespective of other devices used in the code
  digitalWrite(DAC_8734_CS_PIN, LOW); // Select the Chip
  ByteRead[2] = SPI.transfer(0);
  ByteRead[1] = SPI.transfer(0);
  ByteRead[0] = SPI.transfer(0);
  digitalWrite(DAC_8734_CS_PIN, HIGH); // Release the Chip
  SPI.endTransaction();
}

// Setup a default DAC using the variables set above
// note the variables can be chaned then recall InitDac to change mode
// of operation of the DAC
// Not this --Currently set to Unipolar, 2* Gain and Powered up

// Bipolar pins wired to ground, 4*Gain, powered up
void InitDAC()
{ // for now, gain of 2, gpio hiZ,
  int DAC_INIT = 0x0000 | DAC_PD_A << 12 | DAC_PD_B << 11 | DAC_GPIO1 << 9 | DAC_GPIO0 << 8 | DAC_DSDO << 7 | DAC_Gain3 << 5 | DAC_Gain2 << 4 | DAC_Gain1 << 3 | DAC_Gain0 << 2 ;
  Serial.print("Command Reg = 0x"); Serial.print(DAC_INIT, HEX); Serial.print(" "); Serial.print(DAC_INIT, BIN); Serial.println("b");
  WriteDACRegister(DAC_8734_Command, DAC_INIT);
}

// Helper Function to output to a specified DAC, a Desired value between 0 and 65535
// this could represent 0V-10V, -5 to +5V, -15V to +15V, depending on mode
void SetDAC(byte channel, unsigned int value)
{
  if (channel > 3)
    Serial.println("DAC  must be 0 - 3");
  else
  { WriteDACRegister(DAC_8734_DataBase + channel, value);
    //  Pulse LDAC low to update DAC latch. This isn't necessary- just tie it low. If you do want this, though, need to add some delays
    //    delayNanoseconds(30); // t9 in DAC8734: rising edge of CS to falling edge of LDAC
    //    digitalWrite(DAC_8734_LDAC_PIN, LOW); // Set LDAC Bar low
    //    delayNanoseconds(25); // t10 in DAC8734: LDAC low pulse length
    //    digitalWrite(DAC_8734_LDAC_PIN, HIGH); // Set LDAC Bar high
  }
}

//Output the Calibration table to the DAC for all channels
void CalibrateDAC()
{ Serial.println("Inside CalibrateDAC");
  for (int x = 0 ; x < 4 ; x++)
  {
    WriteDACRegister(DAC_8734_CalZeroBase + x, DAC_CAL_tab[x]) ;// Zero cal
    WriteDACRegister(DAC_8734_CalGainBase + x, DAC_CAL_tab[x + 4]); // Gain cal
  }
}
// Simple iteration through the sine table above to simulate a sinewave
// you can change the table contents above to send whatever wave shape you want
void sine( byte channel, unsigned int loopcount)
{
  if (loopcount > 65535) loopcount = 65535;
  if (channel > 3) channel = 0;
  Serial.print("Sine on DAC "); Serial.print(channel); Serial.print(" Cycles= "); Serial.println(loopcount);
  for (unsigned int y = 0 ; y < loopcount ; y++)
  {
    for (int x = 0 ; x < 256 ; x++) {
      SetDAC(channel, (Sin_tab[x] - 32768));   // sine table for bipolar output
      delayNanoseconds(60);
    }
  }
  Serial.println("Completed sine wave output");
}

void setup()
{ lcd.init();     // Initialize LCD display
  lcd.backlight();
  DisplayLCD();   // Show initial parameters

  Serial.begin(115200);    //setup Serial interface; baud rate ignored for Teensy
  PrintCmdList();
  Serial1.begin(115200);    // Initialize 2nd serial port to 115200 baud to send data to PC

  pinMode(DAC_8734_CS_PIN, OUTPUT);
  pinMode(DAC_8734_LDAC_PIN, OUTPUT);
  pinMode(DAC_8734_RESET_PIN, OUTPUT);
  digitalWrite(DAC_8734_CS_PIN, HIGH);    // Chip select to high
  digitalWrite(DAC_8734_RESET_PIN, HIGH); // Reset bar to high
  digitalWrite(DAC_8734_LDAC_PIN, LOW);   // Tie LDAC Bar low- DAC latches after every individual SPI transaction

  SPI.begin();
  SPI.usingInterrupt(IRQ_PIT); // causes beginTransaction to disable this interrupt
  InitDAC();
  CalibrateDAC();  // output the DAC_CAL_tab to the DAC

  // set initial DAC outputs
  SetDAC(0, zout);   // Initial z value completely withdrawn
  // Initialize x and y piezo voltages to 0
  xv = xvinit;
  yv = yvinit;
  SetDAC(1, xv); //write xv to DAC1
  SetDAC(2, xv); //write yv to DAC2
  SetDAC(3, bias);   // Set the DAC 3 to bias voltage
  Serial.println("Retract z, initialize x and y to 0, set bias voltage to 1.0 V");
  // Initialize ADS8691 ADC chip
  pinMode(ADS_8691_CS_PIN, OUTPUT);
  pinMode(ADS_8691_RVS_PIN, INPUT);
  pinMode(ADS_8691_RST_PIN, OUTPUT);
  digitalWrite(ADS_8691_CS_PIN, HIGH);    // Chip select to high
  digitalWrite(ADS_8691_RST_PIN, HIGH);   // Reset bar to high
  //  ResetADC();
  ReadWriteCmd = false;

  fscan = true;  //forward scan

  InitializeDataArrays();
  //  Initialize counters for writing data
  counter = 0;
  totalNumbyteswritten = 0;
  totaltime = 0;
  Setpoint = NANOAMP;      //Sets tunnel current setpoint for 1.0 nA
  myPID.SetOutputLimits(-32767., 32767.);   //DAC8734 D/A range

  //Initialize SD card
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) 
    Serial.println("Card failed, or not present");
  else Serial.println("card initialized.");

  pinMode(Timer1Pin, OUTPUT);   // use to show PID interrupt
  pinMode(Timer2Pin, OUTPUT);   // use to show data point timing
  pinMode(ScanLineTrigger, OUTPUT);   // use to trigger scope at end of scan line
  digitalWrite(Timer1Pin, LOW);
  digitalWrite(Timer2Pin, LOW);
  digitalWrite(ScanLineTrigger, LOW);
}

void loop()
{ //if (Timer1Flag) doPID();
  if (apprflag) approach();
  DisplayLCD(); //Show parameters on LCD display
  Command();   //Check for serial character; if available, execute command
  /*
    // Write data to serial port if it is ready
    if (imagedone==false) WriteSerialDataBuffer();
    if ( (imagedone==true)&& ((writebuffer1==true)||(writebuffer2==true)) ) WriteSerialDataBuffer();  // Write last line of data
  */
}

// Version for Small Stepper motor 28byj-48 5V
void Walk(int direction, unsigned int numsteps)
{
  small_stepper.setSpeed(300);
  if (savedirection != direction) //change direction
  { savedirection = direction;
    delay(500);   //Wait half second before moving when changing direction
  }
  numsteps = abs(numsteps); // positive number to go backward
  if (direction == FORWARD)   numsteps = -(numsteps); // negative number to go forward
  small_stepper.step(numsteps);
}

void Command()
{
  char c;
  double NewVoltage, NewCurrent;
  unsigned int NewSetpoint;
  short int Newbias;
  int dacnum, numcycles;  // for output sine wave
  if (Serial.available())
  { c = Serial.read();
    if (c == 'f')  // forward
    { delay(1);   // Wait 1mS before sending step pulses
      Serial.print("Walk forward ");
      Serial.print(steps);
      Serial.println(" steps");
      Walk(FORWARD, steps);
    }
    else if (c == 'b')  // backward
    { delay(1);   // Wait 1mS before sending step pulses
      Serial.print("Walk backward ");
      Serial.print(steps);
      Serial.println(" steps");
      Walk(BACKWARD, steps);
    }
    else if (c == 'n')  // to enter number of steps
    { steps = Serial.parseInt();
      Serial.print("steps = ");
      Serial.println(steps);
    }
    else if (c == 't')  // toggle Feedback on
    { FlagFeedbackOn = (FlagFeedbackOn ? false : true);
      if (FlagFeedbackOn) FeedbackOn();
      else FeedbackOff();
      Serial.print("Toggled Feedback. Feedback now ");
      if (FlagFeedbackOn) Serial.println("ON");
      else Serial.println("OFF");
    }
    else if (c == 'a')   // restart approach process
    { apprflag = true;    // should restart approach
      imagedone = true; // turn off scans
      delay(1);   // Wait 1mS before sending step pulses
      Serial.println("Starting approach");
    }
    else if (c == 'r')   // Toggle whether ramping during approach process
    { rampflag = (rampflag ? false : true);
      Serial.print("Toggled rampflag for approach. rampflag is now ");
      if (rampflag) Serial.println("ON");
      else Serial.println("OFF");
    }
    else if (c == 'u')   // Toggle whether walking during approach process
    { walkapprflag = (walkapprflag ? false : true);
      Serial.print("Toggled walkapprflag for approach. walkapprflag is now ");
      if (walkapprflag) Serial.println("ON");
      else Serial.println("OFF");
    }
    else if (c == 'q')   // quit approach process
    { apprflag = false;     // stops approach
      imagedone = true; // turn off scans
      Serial.println("Ending approach");
    }
    else if (c == 'p') // change proportional constant for PID
    { Kp = Serial.parseFloat();
      PrintPIDParameters();
      myPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
    }
    else if (c == 'i') // change integral constant for PID
    { Ki = Serial.parseFloat();
      PrintPIDParameters();
      myPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
    }
    else if (c == 'd') // change differential constant for PID
    { Kd = Serial.parseFloat();
      PrintPIDParameters();
      myPID.SetTunings(Kp, Ki, Kd); //Set the PID gain constants and start running
    }
    else if (c == 'e') // Print PID parameters without changing any
    { PrintPIDParameters();
    }
    else if (c == 'v') // change tunneling voltage
    { NewVoltage = Serial.parseFloat();
      Newbias = (NewVoltage * 32767. / 14.376); // range is +32767=+14.376V; 0=0V; -32767=-14.376V
      if ( (Newbias < -32767) || (Newbias > 32767) || (Newbias == 0) ) // New V out of range or zero
      { Serial.println("New Voltage out of range from -14.376V to +14.376V or zero");
        return;
      }
      Voltage = NewVoltage;
      bias = Newbias;
      SetDAC(3, bias);
      Serial.print("New tunneling voltage= ");
      Serial.print(Voltage, 4);
      Serial.print("V corresponds to ");
      Serial.print(bias);
      Serial.println(" D/A counts");
    }
    else if (c == 'c') // change tunneling current setpoint
    { Serial.println("Enter current as floating point mumber in nA, assumes 500MOhm feedback resistor");
      NewCurrent = Serial.parseFloat();
      NewSetpoint  = NewCurrent * 131072 * (-0.5) / 12.288 + 131072; // preamp inverts current sign
      // 1nA through 500MOhm gives PreAmpOut = -0.5V
      if ( (NewSetpoint < 0) || (NewSetpoint > 262144) ||      // New I out of range
           ((NewSetpoint > 132100) && (NewSetpoint < 130100) ) ) // or too close to zero
      { Serial.println("New Current out of A/D range from -12.288V to +12.288V or zero");
        return;
      }
      Current = NewCurrent;
      Setpoint = NewSetpoint;
      Serial.print("New tunneling current= ");
      Serial.print(Current, 4);
      Serial.print("nA corresponds to ");
      Serial.print(Setpoint, 4);
      Serial.println(" A/D counts");
    }
    else if (c == 'g') // Take image
    { imagedone = false;      // flag to start image
      pointcounter = 0;
      linenumber = 0;
      LengthLine=(sizeof(int))*numpoints;
      totalNumbyteswritten = 0;
      fscan = true; // forward x scan first
      xv = xvinit;
      yv = yvinit;
      InitializeDataArrays();
      dataFileF=SD.open(FileNameF, FILE_WRITE);   //open data files
      dataFileB=SD.open(FileNameB, FILE_WRITE);
      parsFile=SD.open(FileNamePars, FILE_WRITE);
      Serial.println(FileNameF);
      Serial.println(dataFileF);
      Serial.println("Start image. ");
      myTimer2.begin(Timer2Service, TimeInterval_uSec);
      myTimer2.priority(75);
      if (xscan) Serial.print("Starting repeated x scans. ");
      if (yscan) Serial.print("Also scanning y for image.");
    }
    else if (c == 'h') // Halt scan
    { imagedone = true;
      xv = xvinit;
      yv = yvinit;
      SetDAC(1, xv);  // Put xv and yv back to initial values
      SetDAC(2, yv);
      myTimer2.end();
      //Write acquired data to disk;
      WriteParametersToFile();
      CloseFiles();
      Serial.println("Aborting image");
    }
    else if (c == 'x')   // toggle x scan on or off
    { xscan = (xscan ? false : true);
      Serial.print("Toggled xscan. When image is taken, xscan is now ");
      if (xscan) Serial.println("ON");
      else Serial.println("OFF");
    }
    else if (c == 'y')  // toggle y scan on or off
    { yscan = (yscan ? false : true);
      Serial.print("Toggled yscan. When image is taken, yscan is now ");
      if (yscan) Serial.println("ON");
      else Serial.println("OFF");
    }
    else if (c == 'w')  // Withdraw tip by turning feed back off, Z=0
    { FlagFeedbackOn = false;
      FeedbackOff();
      //zout=0;      // z voltage applied to brass side
      zout = 32767;  // z voltage applied to quadrants on back
      SetDAC(0, zout);
      lcdOutput = zout;
      Serial.println("Feedback off. Withdraw tip by setting Z=32767");
    }
    else if (c == 'm') // Move to xy point, coordinates in DAC counts
    { xv = xvinit = Serial.parseInt();
      yv = yvinit = Serial.parseInt();
      SetDAC(1, xv);
      SetDAC(2, yv);
    }
    else if (c == 'z') // Change value of z piezo
    { zout = Serial.parseInt();
      SetDAC(0, zout);
      lcdOutput = zout;
      Serial.print("Changed z output voltage to ");
      Serial.println(zout);
    }
    else if (c == 's')  // Change scan parameters: s numpts, stepsize in D/A counts, timeinterval mS"
    { numlines = numpoints = Serial.parseInt();
      Serial.print("New number of data points = number of data lines = ");
      Serial.println(numpoints);
      stepsize = Serial.parseInt();
      Serial.print("New stepsize in D/A = ");
      Serial.println(stepsize);
      TimeInterval_mSec = Serial.parseInt();
      TimeInterval_uSec = 1000 * TimeInterval_mSec;
      Serial.print("Time interval between data points in mS = ");
      Serial.println(TimeInterval_mSec);
    }
    else if (c == 'j')  // placeholder for new command
    {
    }
    else if (c == 'k')  // placeholder for new command
    {
    }
    else if (c == 'o')  // placeholder for new command
    {
    }
    else if (c == 'l')  // place holder for new command
    {
    }
    else if (c == 'C')  // Start A/D conversion and print value
    { Data18Bits=ADC_Convert();
      //    Serial.print("Tell ADC to convert ");
      //    for (int i=0; i<=3; i++)
      //      {Serial.print("Data byte");
      //      Serial.print(i);
      //      Serial.print(" ");
      //      Serial.println(DataIn.ByteBuffer[i], HEX);
      //      }
      Serial.print("4 bytes no shift ");
      for(int i=0;i<4;++i) { Serial.print(spi_buffer_adc[i], HEX); Serial.print(" "); }
      //Data18Bits = (DataIn.IntData >> 14) & 0x3FFFF;
      Serial.print(", 18 bit Hex ");
      Serial.print(Data18Bits, HEX);
      Serial.print(", 18 bit decimal ");
      Serial.print(Data18Bits);
      float f = ADCToVolts(Data18Bits);
      Serial.print(", Volts ");
      Serial.println(f, 6);
    }
    else if (c == 'D')  // D dacNum, analog Val
    { int dacNum = Serial.parseInt();
      int analogVal = Serial.parseInt();
      if (dacNum < 0 || dacNum > 3) dacNum = 0;
      if (analogVal < DACMIN) analogVal = DACMIN;
      if (analogVal > DACMAX) analogVal = DACMAX;
      SetDAC(dacNum, analogVal);   // Set the DAC dacNum to analogVal
      Serial.print("Setting Dac ");
      Serial.print(dacNum);
      Serial.print(" to ");
      Serial.println(analogVal);
    }
    else if (c == 'I') // Initialize and calibrate DAC
    { InitDAC();
      CalibrateDAC();
      Serial.println("Initialize and calibrate DAC");
    }
    else if (c == 'S')  // S dacnum, numcycles; put out sinewave on DAC
    { dacnum = Serial.parseInt();
      Serial.print("dacnum for sine wave = ");
      Serial.println(dacnum);
      numcycles = Serial.parseInt();
      Serial.print("numcycles for sine wave = ");
      Serial.println(numcycles);
      sine(dacnum, numcycles);
    }
     else if (c=='F') // F FileNamePrefixStr
      {
//       FileNamePrefix=Serial.readString();
//       Serial.println(FileNamePrefix);
//       s1=FileNamePrefix+FileNameNumber;
//       //Serial.println(itoa(FileNameNumber));
//       //strcpy(FileNamePrefix, FileName);
//       Serial.println(s1);
//       
//       FileName=FileNamePrefix+String(FileNumber);
//       Serial.println(FileName);
      }
    else if(c == '/') {
      DumpADCRegisters();
    }
  }   // end if serial available
}

void PrintPIDParameters()
{ Serial.print("Kp= ");
  Serial.print(Kp, 8);
  Serial.print(" Ki= ");
  Serial.print(Ki, 8);
  Serial.print(" Kd= ");
  Serial.println(Kd, 8);
}

//read adc registers and dump them to console
//they seem to match the reset values given in the datasheet
//except for bit 12 in the halfword at address 0
void DumpADCRegisters() {
  SPI.beginTransaction(settingsADS8691);
  int last_address=0;
  uint32_t spi_buffer_a;
  uint8_t *spi_buffer=(uint8_t*)&spi_buffer_a;
  for(int i=0;i<=0x2c;i+=2) {
    if(i==0x18) i=0x20;
    if(i!=0x2c) {
      spi_buffer[0]=0xc8;
      spi_buffer[1]=i;
    } else {
      spi_buffer[0]=0;
      spi_buffer[1]=0;
    }
    spi_buffer[2]=0;
    spi_buffer[3]=0;
    adc_transfer(spi_buffer,4);
    delayNanoseconds(1000);
    if(i!=0) {
      Serial.print("Register halfword at address ");
      Serial.print(last_address,HEX);
      Serial.print(": "); 
      Serial.println(*(uint16_t*)spi_buffer,HEX);
    }
    last_address=i;
  }
  SPI.endTransaction();
}

void DisplayLCD()
{ lcdInput = Input;
  if (FlagFeedbackOn) lcdOutput = Output;
  lcd.setCursor(0, 0);
  lcd.print("Vt=");
  lcd.print(Voltage, 3);
  lcd.print("  It=");
  lcd.print(Current, 3);
  lcd.setCursor(0, 1);
  lcd.print("I");
  lcd.print(lcdInput, 0);
  lcd.print(" O");
  lcd.print(lcdOutput, 0);
  lcd.print("    ");
  lcd.setCursor(16, 1);
  lcd.print("mS");
  lcd.print(TimeInterval_mSec);
  lcd.setCursor(0, 2);
  lcd.print("x     ");
  lcd.setCursor(1, 2);
  lcd.print(xv);
  lcd.setCursor(7, 2);
  lcd.print("y     ");
  lcd.setCursor(8, 2);
  lcd.print(yv);
  lcd.setCursor(14, 2);
  lcd.print("S     ");
  lcd.setCursor(15, 2);
  lcd.print(stepsize);
  lcd.print(" ");
  lcd.setCursor(18, 2);
  if (xscan) lcd.print("X");  else lcd.print(" ");
  if (yscan) lcd.print("Y");  else lcd.print(" ");
  lcd.setCursor(0, 3);
  lcd.print("P ");
  lcd.print(pointcounter);
  lcd.print("  ");
  lcd.setCursor(7, 3);
  lcd.print("L ");
  lcd.print(linenumber);
  lcd.print("   ");
  lcd.setCursor(15, 3);
  lcd.print("T ");
  lcd.print(numpoints);
}

// New version of approach 5/29/2017, edited 7/24/2022
void approach()
{ float f;
  FlagFeedbackOn = false;
  FeedbackOff();
  Serial.println("Beginning approach subroutine");
  // Applying voltage to brass:  z voltage =0 corresponds to withdrawing tip
  //                             z voltage =65535 corresponds to tip full forward
  // Applying z voltage to quadrants on back: z voltage=32767 corresponds to withdrawing tip
  //                                          z voltaage = -32767 corresponds to tip full forward

  //DAC values for DAC8734: z=+32767 corresponds to withdrawing tip
  //                        z=-32767 corresponds to tip full forward
  for (long int iramp = 32767; iramp > -32767; iramp -= 200) // Slowly extend z piezo to maximum value
  { if (rampflag) SetDAC(0, iramp); // Change to DAC8734 output
    if (rampflag) delayMicroseconds(200);    // Make delay longer, now 0.2mS
    IntegerCurrent=ADC_Convert();  //read tunnel current from preamp on ADS8691
    //IntegerCurrent = (DataIn.IntData >> 14) & 0x3FFFF; //incoming data to 18 bit number stored in 32 bit int
    Serial.print("iramp= ");
    Serial.print(iramp);
    Serial.print(", IntegerCurrent =");
    Serial.print(IntegerCurrent);
    f = ADCToVolts(IntegerCurrent); // convert to volts
    Serial.print(" f= ");
    Serial.println(f, 6);
    if ((IntegerCurrent > 132139) || (IntegerCurrent < 130005)) // more than 0.2nA or less than -0.2nA
    { IntegerCurrent = ADC_Convert();     // check current 2nd time
      if ((IntegerCurrent > 132139) || (IntegerCurrent < 130005)) // more than 0.2nA or less than -0.2nA
      { FeedbackOn();
        FlagFeedbackOn = true;
        Serial.println("Tunneling current >0.2nA or < -0.2nA; turn approach off");
        Serial.print(" IntegerCurrent =");
        Serial.print(IntegerCurrent);
        f = ADCToVolts(IntegerCurrent); // convert to volts
        Serial.print(" f= ");
        Serial.println(f, 6);
        apprflag = false; // We have tunneling current so turn approach off
        return;
      }
    }
  }
  // no current, so retract z piezo, print AD counts, walk 1 motor step
  SetDAC(0, zout);   //retract z piezo
  Serial.println("After extension and retraction of z piezo");
  f = ADCToVolts(IntegerCurrent); // convert to volts
  Serial.print(" f= ");
  Serial.println(f, 6);

  if (walkapprflag)
  { Serial.println("Walk 1 motor steps");
    Walk(FORWARD, 1);
    delay(200);
  }
}

//  Interrupt service routine sets flag to do PID digital feedback, every 20uS
void Timer1Service()
{ digitalWrite(Timer1Pin, HIGH);    // Start pulse on Timer1Pin to check interrupt service routine timing
  //Timer1Flag=true;  // Sets flag so that PID computation is done next
  Input = ADC_Convert();  //read voltage from preamp, request ADS8691 18 bit conversion
  myPID.Compute();
  int Value=Output;
  SetDAC(0, Value);
  digitalWrite(Timer1Pin, LOW);    // End pulse to check interrupt timing
// float f=ADCToVolts((unsigned int)Input);
// Serial.println(f);
  asm volatile("dsb"); // Ensures that interrupt does not fire twice, see page 44 in IMRXT1060 manual
}

// Timer interrupt service routine to take data points
void Timer2Service()
{ if (imagedone == false)
  { digitalWrite(Timer2Pin, HIGH);    // Start pulse on pin Timer2Pin to start trigger for datapoint)
    { if (xscan) SetDAC(1, xv);
      if (xscan)
      { if (fscan) {
          xv += stepsize;
          if (xv > 32767) xv = 32767;
          DataArray1F[linenumber][pointcounter] = (int16_t)Output;  //18 bit z value stored in 4 bytes
//          Serial.print("array value ");
//          Serial.println(DataArray1F[linenumber][pointcounter]);
          dataFileF.write(&(DataArray1F[linenumber][pointcounter]),2);
//          Serial.print("number writing to file ");
//          Serial.println(DataArray1F[linenumber][pointcounter]);
        }
        else
        { xv -= stepsize;
          if (xv < -32767) xv = -32767;
          DataArray1B[linenumber][pointcounter] = (int16_t)Output;  //18 bit z value stored in 4 bytes
          dataFileB.write(&(DataArray1B[linenumber][pointcounter]),2);
        }
      }
      pointcounter++;
      if (pointcounter >= numpoints) fscan = false;
      if (pointcounter >= (2 * numpoints))
      { fscan = true;
        pointcounter = 0;
        if (yscan) {
          yv += stepsize;
          SetDAC(2, yv);
        }
        if (xscan) xv = xvinit;    //reinitialize xv to beginning of line
        
        digitalWrite(ScanLineTrigger,HIGH);
        delayMicroseconds(1);
        digitalWrite(ScanLineTrigger,LOW);
//        dataFileF.write(&(DataArray1F[linenumber][0]),LengthLine);
//        for (int i=0; i<LengthLine; i++)
//             Serial.print(DataArray1F[linenumber][i]);
//        dataFileB.write(&(DataArray1B[linenumber][0]),LengthLine);
        linenumber++;
        //Write line to local display
      }
      if (linenumber >= numlines)
      { imagedone = true;
        yv = yvinit;
        SetDAC(2, yv);
        WriteParametersToFile();
        CloseFiles();
        Serial.println("Image finished");
        myTimer2.end();
      }
    }
    digitalWrite(Timer2Pin, LOW);    // Finish pulse on pin Timer2Pin to end trigger for datapoint
  }
  asm volatile("dsb"); // Ensures that interrupt does not fire twice, see page 44 in IMRXT1060 manual
}

void CloseFiles()
{       dataFileF.close();
        dataFileB.close();
        parsFile.close();
 
  
}
void WriteParametersToFile()
{       parsFile.print(numpoints);
        parsFile.println(" number of points");
        parsFile.print(numlines);
        parsFile.println(" number of lines");
        parsFile.print("If Image finished early. ");
        parsFile.print(pointcounter); 
        parsFile.println(" number of points");
        parsFile.print(linenumber);
        parsFile.println(" number of lines");
        parsFile.print(bias);
        parsFile.println(" V, sample voltage");
        parsFile.print(Current);
        parsFile.println(" nA, tunneling current"); 
}

void FeedbackOn()
{ myPID.SetMode(AUTOMATIC);    //turns PID on
  myTimer1.begin(Timer1Service, 100);  //Timer1Pin changes every 100uS
  myTimer1.priority(200);              //lower priority than scanning, range 0-255
}

void FeedbackOff()
{ myPID.SetMode(MANUAL);    //turns PID off
  myTimer1.end();
}


void PrintCmdList()      //Print list of commands
{ Serial.println("Type: f forward; b backward; n #steps");
  Serial.println("  v voltage; c current; p Kp; i Ki; d Kd; e print PID parameters");
  Serial.println("  a approach; q quit approach; r rampflag during approach; u walkapprflag");
  Serial.println("  t toggles feedback on and off; w withdraw tip & feedback off");
  Serial.println("  x toggles xcan on and off; y toggles yscan on and off");
  Serial.println("  g start image; h abort image; m x y move to xy point");
  Serial.println("  s numpts, stepsize in D/A counts, timeinterval mS (scan parameters)");
  Serial.println("  z zvalue in D/A counts");
  Serial.println("  C ADC convert and print value; I initialize and calibrate DAC");
  Serial.println("  D dacNum, analogVal; S dacNum, numcycles for sine wave");
  Serial.println("  I reset and recalibrate DAC");
}

// Converts 18 bit ADC value to volts, 0 to 0x3FFFF = -12.288V to +12.288V
float ADCToVolts(unsigned int ADCVal)
{ float ZeroVolts = 131072;
  float f = ((float)ADCVal - ZeroVolts) * 12.288 /ZeroVolts;
  return f;
}

void InitializeDataArrays()
{ int i, j;
  for (i = 0; i < MAXNUMPTS; i++)
    for (j = 0; j < MAXNUMLINES; j++)
    { DataArray1F[j][i] = (int16_t)0;
      DataArray1B[j][i] = (int16_t)0;
    }
}

void LineSlopeSubtract(int linenumber, int ** zArray, float *zLineBuf)
{float slope;
int i, arrayMax, arrayMin, zz;
//if ((linenumber==0) || (linenumber==(numlines-1)))
   slope = (zArray[linenumber][numpoints-1] - zArray[linenumber][0])/(float)(numpoints-1); 
//      print(" slope = ");
//      print(slope);

//slope=0.0;
   for( i = 0; i < numpoints; i++)
        {zz = ((zArray[linenumber][i])-slope*i-zArray[linenumber][0]);    // new z value with slope subtracted
      if( i == 0) 
        {arrayMax = arrayMin = zz;  
        }
       else
        {if(zz > arrayMax) arrayMax = zz;
        if(zz < arrayMin) arrayMin = zz;
        }
      Serial.print(" arrayMin = ");
      Serial.print(arrayMin);
      Serial.print(" arrayMax = ");
      Serial.println(arrayMax);

        //zz=z[linenumber][i];
        Serial.print(" zz= ");
        Serial.print(zz);
        float b;
        b=map( zz, arrayMin, arrayMax, 0, 255);
        Serial.print(" b= ");
        Serial.print(b);
       
        char a = (char) b;
        Serial.print(" a= ");
        Serial.print(a,HEX);
//        imageName.pixels[linenumber*numpoints + i] = color( a, a, a);   //write to framebuffer, pixels 0 to 479, every other line
      }
// imageName.updatePixels();  //update framebuffer
}
