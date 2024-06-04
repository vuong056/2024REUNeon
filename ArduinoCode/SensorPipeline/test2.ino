#include "ICM_20948.h"
#include <SparkFun_Qwiic_Button.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <SparkFun_MicroPressure.h>

#define SERIAL_PORT Serial
#define SPI_PORT SPI
#define CS_PIN 2
#define WIRE_PORT Wire
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM;
#else
ICM_20948_I2C myICM;
#endif

QwiicButton button;
OpenLog myLog;
SparkFun_MicroPressure mpr;

#define LOG_INTERVAL 250 // milliseconds between logging data
String nextFileName = "";

void setup() {
  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) {};
  delay(2000);

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  bool initialized = false;
  while (!initialized) {
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok) {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } else {
      initialized = true;
    }
  }

  if (button.begin() == false) {
    SERIAL_PORT.println("Device did not acknowledge button");
    while (1);
  }
  SERIAL_PORT.println("Button acknowledged");
  while (!button.isPressedQueueEmpty()) {
    button.popPressedQueue();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  
  myLog.begin();
  Serial.println("OpenLog initialized");
  myLog.println("OpenLog initialized for Sensor_Pipeline.ino");

  myLog.println("Time to make a directory");
  myLog.changeDirectory("..");
  myLog.searchDirectory("*/");

  String dirName = getLastDirectoryName();
  if (dirName == "") {
    dirName = "LOG1";
  } else {
    int logNum = dirName.substring(3).toInt() + 1;
    dirName = "LOG" + String(logNum);
  }
  myLog.makeDirectory(dirName);
  myLog.changeDirectory(dirName);

  nextFileName = "Test1";
  SERIAL_PORT.print(dirName);
  SERIAL_PORT.println(F(" Directory Created"));

  if (!mpr.begin()) {
    SERIAL_PORT.println("Can't connect to MicroPressure Sensor");
    while (1);
  } else {
    SERIAL_PORT.println("MicroPressure Sensor Connected");
  }
}

String getLastDirectoryName() {
  String lastDirName = "";
  String dirName = myLog.getNextDirectoryItem();
  while (dirName != "") {
    if (dirName.indexOf("LOG") > -1) {
      lastDirName = dirName;
    }
    SERIAL_PORT.println(dirName);
    dirName = myLog.getNextDirectoryItem();
  }
  SERIAL_PORT.println("DONE GETTING DIRECTORY NAMES");
  return lastDirName;
}

void createNewLogFile() {
  String fileName = nextFileName + ".txt";
  myLog.append(fileName);
  myLog.print(fileName);
  myLog.println(" File Created");
  int fileNum = nextFileName.substring(4).toInt() + 1;
  nextFileName = "Test" + String(fileNum);
  SERIAL_PORT.println(nextFileName);
  myLog.println("IMU Output: Scaled");
  myLog.println("Acc (mg) X, Y, Z, Gyr (DPS) X, Y, Z, Mag (uT) X, Y, Z, Temp, Pa, PSI, atm");
}

unsigned long init_time = millis();
bool recording = false;

void loop() {
  if (button.isPressed()) {
    SERIAL_PORT.println("Button is pressed");
    if (!recording) {
      // Start recording
      createNewLogFile();
      SERIAL_PORT.println("Button click, Data Collection Started");
      recording = true;
    } else {
      // Stop recording
      SERIAL_PORT.println("Button click, Data Collection Ended.");
      myLog.print(millis());
      myLog.println(" Data Collection Ended.");
      myLog.syncFile();
      recording = false;
    }
    while (button.isPressed()) delay(10);  // Debouncing
  }

  if (recording) {
    digitalWrite(LED_BUILTIN, HIGH);
    if (myICM.dataReady()) {
      if (millis() - init_time > LOG_INTERVAL) {
        myICM.getAGMT();
        printScaledAGMT(&myICM);
        myLog.print(millis());
        myLog.print(" ");
        writeScaledAGMT(&myICM);
        myLog.syncFile();
        init_time = millis();
      }
    } else {
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

void printPaddedInt16b(int16_t val) {
  if (val > 0) {
    SERIAL_PORT.print(" ");
    if (val < 10000) SERIAL_PORT.print("0");
    if (val < 1000) SERIAL_PORT.print("0");
    if (val < 100) SERIAL_PORT.print("0");
    if (val < 10) SERIAL_PORT.print("0");
  } else {
    SERIAL_PORT.print("-");
    if (abs(val) < 10000) SERIAL_PORT.print("0");
    if (abs(val) < 1000) SERIAL_PORT.print("0");
    if (abs(val) < 100) SERIAL_PORT.print("0");
    if (abs(val) < 10) SERIAL_PORT.print("0");
  }
  SERIAL_PORT.print(abs(val));
}

void printRawAGMT(ICM_20948_AGMT_t agmt) {
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

void printFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    SERIAL_PORT.print("-");
  } else {
    SERIAL_PORT.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      SERIAL_PORT.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    SERIAL_PORT.print(-val, decimals);
  } else {
    SERIAL_PORT.print(val, decimals);
  }
}

#ifdef USE_SPI
void printScaledAGMT(ICM_20948_SPI *sensor) {
#else
void printScaledAGMT(ICM_20948_I2C *sensor) {
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

void writeFormattedFloat(float val, uint8_t leading, uint8_t decimals) {
  float aval = abs(val);
  if (val < 0) {
    myLog.print("-");
  } else {
    myLog.print(" ");
  }
  for (uint8_t indi = 0; indi < leading; indi++) {
    uint32_t tenpow = 0;
    if (indi < (leading - 1)) {
      tenpow = 1;
    }
    for (uint8_t c = 0; c < (leading - 1 - indi); c++) {
      tenpow *= 10;
    }
    if (aval < tenpow) {
      myLog.print("0");
    } else {
      break;
    }
  }
  if (val < 0) {
    myLog.print(-val, decimals);
  } else {
    myLog.print(val, decimals);
  }
}

void writePressure() {
  myLog.print(mpr.readPressure(PA));
  myLog.print(",");
  myLog.print(mpr.readPressure());
  myLog.print(",");
  myLog.print(mpr.readPressure(ATM));
}

#ifdef USE_SPI
void writeScaledAGMT(ICM_20948_SPI *sensor) {
#else
void writeScaledAGMT(ICM_20948_I2C *sensor) {
#endif
  writeFormattedFloat(sensor->accX(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->accY(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->accZ(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->gyrX(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->gyrY(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->gyrZ(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->magX(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->magY(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->magZ(), 5, 2);
  myLog.print(",");
  writeFormattedFloat(sensor->temp(), 5, 2);
  myLog.print(",");
  writePressure();
  myLog.println();
}
