/****************************************************************
 * Sensor_Pipeline.ino
 * Part of the Hybrid Atelier 2024 REU
 * Authors: Lejun Liao & Rohita Konjeti
 * Adapted From: Owen Lyke @ SparkFun Electronics
 * Created: 5/30/2024
 *
 * https://github.com/liaolc/2024REUNeon 
 *
 ***************************************************************/

/** IMU **/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif
//ICM_20948_I2C myICM;

/** BUTTON **/
#include <SparkFun_Qwiic_Button.h>
QwiicButton button;

/** OpenLog **/
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
OpenLog myLog; //Create instance


void setup()
{

  SERIAL_PORT.begin(115200);
  
  while (!SERIAL_PORT)
  {
  };
  delay(2000);

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  /** BUTTON INITIALIZATION **/
  if (button.begin() == false) {
    SERIAL_PORT.println("Device did not acknowledge button");
    while (1);
  }
  SERIAL_PORT.println("Button acknowledged");
  while(!button.isPressedQueueEmpty()){
    SERIAL_PORT.println("Button pressed queue not empty");
    button.popPressedQueue();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
  /** OpenLog Initialization **/
  String fileName = "NewFile.txt";
  myLog.begin();
  Serial.println("OpenLog initialized");
  myLog.println("OpenLog initialized for Sensor_Pipeline.ino");
  if(myLog.size(fileName) == -1) { //Create new file since it doesn't exist
    SERIAL_PORT.print(fileName);
    SERIAL_PORT.println(" not found, Creating new file");
    myLog.append(fileName);
    myLog.println("File beginning");
    myLog.syncFile();
  }
  else {
    SERIAL_PORT.print(fileName);
    SERIAL_PORT.println(" file found, appending");
  }

}

unsigned long init_time = millis();
bool startFlag = false;
void loop()
{
  unsigned long lastClickTime = button.timeSinceLastClick();
  if (button.isPressed() == true) {
    SERIAL_PORT.println("Button is pressed");
    if(lastClickTime > 150) { //if single press: start collect if not yet started
      if(!startFlag) {
        startFlag = true;
        SERIAL_PORT.println("First Button click, Data Collection Started");
      }
      else if (startFlag) { // eventually single press = pause data writing
        SERIAL_PORT.println("Single button click acknowledged, but data collection has started");
      }
    }
    else { //double click registered -- eventually double click = write to disk
      SERIAL_PORT.println("Time since last click:");
      SERIAL_PORT.println(lastClickTime);
      if(startFlag) {
        startFlag = false; 
        SERIAL_PORT.println("Data collection paused, click to continue");
      }
      else if(!startFlag) {
        SERIAL_PORT.println("Data collection already paused");
      }
    }
    
    while (button.isPressed() == true)
      delay(10);
    SERIAL_PORT.println("Button is not pressed");
  } 
  if(startFlag) {
    digitalWrite(LED_BUILTIN, HIGH);
    
    if (myICM.dataReady()) {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
                              //    printRawAGMT( myICM.agmt );     // Uncomment this to see the raw values, taken directly from the agmt structure
      if (millis() - init_time > 1000) {
        printScaledAGMT(&myICM); // This function takes into account the scale settings from when the measurement was made to calculate the values with units
        writeScaledAGMT(&myICM);
        init_time = millis(); 
      }
    }
    else {
      if (millis() - init_time > 500) {
        SERIAL_PORT.println("Waiting for data");
        init_time = millis();
      }
    }
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

}

// Below here are some helper functions to print the data nicely!

void printPaddedInt16b(int16_t val)
{
  if (val > 0)
  {
    SERIAL_PORT.print(" ");
    if (val < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (val < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  else
  {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 1000)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 100)
    {
      SERIAL_PORT.print("0");
    }
    if (abs(val) < 10)
    {
      SERIAL_PORT.print("0");
    }
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt)
{
  SERIAL_PORT.print("RAW. Acc [ ");
  printPaddedInt16b(agmt.acc.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.acc.axes.z);
  SERIAL_PORT.print(" ], Gyr [ ");
  printPaddedInt16b(agmt.gyr.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.gyr.axes.z);
  SERIAL_PORT.print(" ], Mag [ ");
  printPaddedInt16b(agmt.mag.axes.x);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.y);
  SERIAL_PORT.print(", ");
  printPaddedInt16b(agmt.mag.axes.z);
  SERIAL_PORT.print(" ], Tmp [ ");
  printPaddedInt16b(agmt.tmp.val);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    SERIAL_PORT.print("-");
  }
  else
  {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      SERIAL_PORT.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    SERIAL_PORT.print(-val, decimals);
  }
  else
  {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void printScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  SERIAL_PORT.print("Scaled. Acc (mg) [ ");
  printFormattedFloat(sensor->accX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->accZ(), 5, 2);
  SERIAL_PORT.print(" ], Gyr (DPS) [ ");
  printFormattedFloat(sensor->gyrX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->gyrZ(), 5, 2);
  SERIAL_PORT.print(" ], Mag (uT) [ ");
  printFormattedFloat(sensor->magX(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magY(), 5, 2);
  SERIAL_PORT.print(", ");
  printFormattedFloat(sensor->magZ(), 5, 2);
  SERIAL_PORT.print(" ], Tmp (C) [ ");
  printFormattedFloat(sensor->temp(), 5, 2);
  SERIAL_PORT.print(" ]");
  SERIAL_PORT.println();
}

void writeFormattedFloat(float val, uint8_t leading, uint8_t decimals)
{
  float aval = abs(val);
  if (val < 0)
  {
    myLog.print("-");
  }
  else
  {
    myLog.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++)
  {
    uint32_t tenpow = 0;
    if (indi < (leading - 1))
    {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++)
    {
      tenpow *= 10;
    }
    if (aval < tenpow)
    {
      myLog.print("0");
    }
    else
    {
      break;
    }
  }
  if (val < 0)
  {
    myLog.print(-val, decimals);
  }
  else
  {
    myLog.print(val, decimals);
  }
  myLog.syncFile();
}

#ifdef USE_SPI
void writeScaledAGMT(ICM_20948_SPI *sensor)
{
#else
void writeScaledAGMT(ICM_20948_I2C *sensor)
{
#endif
  myLog.print("Scaled. Acc (mg) [ ");
  writeFormattedFloat(sensor->accX(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->accY(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->accZ(), 5, 2);
  myLog.print(" ], Gyr (DPS) [ ");
  writeFormattedFloat(sensor->gyrX(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->gyrY(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->gyrZ(), 5, 2);
  myLog.print(" ], Mag (uT) [ ");
  writeFormattedFloat(sensor->magX(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->magY(), 5, 2);
  myLog.print(", ");
  writeFormattedFloat(sensor->magZ(), 5, 2);
  myLog.print(" ], Tmp (C) [ ");
  writeFormattedFloat(sensor->temp(), 5, 2);
  myLog.print(" ]");
  myLog.println();
  myLog.syncFile();
}
