
// get raw data, using Sparkfun library
// Derived by SJR from
/****************************************************************
   Example1_Basics.ino
   ICM 20948 Arduino Library Demo
   Use the default configuration to stream 9-axis IMU data
   Owen Lyke @ SparkFun Electronics
   Original Creation Date: April 17 2019

   Please see License.md for the license information.

   Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
//#define SPI_FREQ 50000 // You can override the default SPI frequency
#define CS_PIN 10     // Which pin you connect CS to. Used only when "USE_SPI" is defined

ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object


// gyro offset values for calibration
float gyro[3] = {0};
int offset_count = 500; //average this many values for gyro
int acc_mag_count = 300; //collect this many values for acc/mag calibration

void setup()
{

  SERIAL_PORT.begin(9600);
  while (!SERIAL_PORT)
  {
  };
  delay(10000);

  SPI_PORT.begin();


  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    myICM.begin(CS_PIN, SPI_PORT); //, SPI_FREQ); // Here we are using the user-defined SPI_FREQ as the clock speed of the SPI bus

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
    // Choose whether or not to start the magnetometer
  // myICM.startupMagnetometer();
  if (myICM.status != ICM_20948_Stat_Ok)
  {
    SERIAL_PORT.print(F("startupMagnetometer returned: "));
    SERIAL_PORT.println(myICM.statusString());
  }
  // find gyro offsets
  SERIAL_PORT.println(F("Hold sensor still for gyro offset calibration ..."));
  delay(5000);

  float goff;
  int i;

  for (i = 0; i < offset_count; i++) {
    if (myICM.dataReady())
    {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      gyro[0] += myICM.gyrX();
      gyro[1] += myICM.gyrY();
      gyro[2] += myICM.gyrZ();
    }
    delay(50);
  } //done with gyro
  SERIAL_PORT.print("Gyro offsets x, y, z: ");
  for (i = 0; i < 3; i++) {
    goff = (float)gyro[i] / offset_count;
    SERIAL_PORT.print(goff, 5);
    SERIAL_PORT.print(", ");
  }
  SERIAL_PORT.println();

  SERIAL_PORT.println(F("Turn sensor SLOWLY and STEADILY in all directions until done"));
  delay(5000);
  SERIAL_PORT.println(F("Starting..."));

  // get values for calibration of acc/mag
  for (i = 0; i < acc_mag_count; i++) {
    if (myICM.dataReady())
    {
      myICM.getAGMT();         // The values are only updated when you call 'getAGMT'
      printScaledAGMT(&myICM);     // raw values, taken directly from the agmt structure
      delay(200);
    }
    else
    {
      delay(100); //wait for data ready
    }
  }
  SERIAL_PORT.print(F("Done collecting"));
}

void loop() {}



void printScaledAGMT(ICM_20948_SPI *sensor)
{
  SERIAL_PORT.print(sensor->accX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->accZ());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->magX());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->magY());
  SERIAL_PORT.print(", ");
  SERIAL_PORT.print(sensor->magZ());
  SERIAL_PORT.println();
}

