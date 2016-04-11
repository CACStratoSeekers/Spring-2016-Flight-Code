/* This code was compiled from Adafruit libraries and example sketches for:
 *  Arduino Mega
 *  Ultimate GPS
 *  SD shield
 *  LSM9DS0, "9 Degrees of Freedom" board
 *  TSL2561, digital Ambient Light sensor
 *  BMP180, Pressure, temperature, altitude sensor
 *  INA219, In-Line current sensor
 * 
 * All parts were ordered from Adafruit. They have full, detailed tutorials on all their sensors,
 * as well as very helpful libraries and example sketches, and respond quickly to questions posted on their forum. 
 * I highly recommend purchasing your sensors and parts from them!
 * 
 * Other helpful resources and tutorials can be found at TopTechBoys.com,
 * and JeremyBlum.com. Both sites have Arduino tutorials used in the creation of this code.
 * 
 * This code was written for a High Altitude Balloon project by the StratoSeekers team 
 * at Central Arizona College, Spring 2016. 
 */


#include "Wire.h"//for i2c communication
#include "SD.h"//this is the Adafruit SD library... you need to "hide" your default Arduino SD library in order to use this
#include "SPI.h"
#include "RTClib.h"// allows us to call for time from SD shield
#include "SoftwareSerial.h"//allows use of non-default serial pins 
#include "Adafruit_Sensor.h"//unified sensor library... needed for calling sensor functions
#include "Adafruit_LSM9DS0.h"//defined below as "lsm"
#include "Adafruit_TSL2561_U.h"//defined below as "tsl"
#include "Adafruit_BMP085_U.h"//defined below as "bmp"
//#include "Adafruit_GPS.h"
#include "Adafruit_INA219.h"//defined below as "ina219"

const int chipSelect = 10;

float refresh_rate = 1000;//sets the "delay" time in milliseconds for datalogging. only have to change in this place for whole sketch
int redLedpin = 5;
int blueLedpin = 6;
int greenLedpin = 7;
int onLED = (1000);
int offLED = (2000);
int greenOn = (5000);



Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

Adafruit_LSM9DS0 lsm =  Adafruit_LSM9DS0(1000);

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

Adafruit_INA219 ina219 = Adafruit_INA219(00000);

//#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   //#define Serial SerialUSB
//#endif


/* 
   Regular Arduino UNO Connections (For default I2C )
   ===========
   Connect SCL to analog 5 (Won't work for Mega) (Yellow wire for SCL)
   Connect SDA to analog 4 (Won't work for Mega) (Orange wire for SDA)
   Connect VCC to 5V DC                          (Red wire for VCC,Power)
   Connect GROUND to common ground               (Black, or Green wire for Ground)

   ****CHANGE IF USING MEGA****
   *connect SDA to pin 20(SDA on mega) (Orange wire for SDA)
   *connect SCL to pin 21(SCL on mega) (Yellow wire for SCL)
 */

//#include "Adafruit_GPS.h"
//#if ARDUINO >= 100
// #include <SoftwareSerial.h>
//#else
  // Older Arduino IDE requires NewSoftSerial, download from:
  // http://arduiniana.org/libraries/newsoftserial/
// #include <NewSoftSerial.h>
 // DO NOT install NewSoftSerial if using Arduino 1.0 or later!
//#endif
 
 //GPS connections
 // Connect the GPS Power pin to 5V         (Red wire for Power)
// Connect the GPS Ground pin to ground     (Black or Green wire for Ground)
// If using software serial (sketch example default):
//   Connect the GPS TX (transmit) pin to Digital 3
//   Connect the GPS RX (receive) pin to Digital 2
// If using hardware serial (e.g. Arduino Mega):
//   Connect the GPS TX (transmit) pin to Arduino RX1, RX2 or RX3   (White wire for GPS TX to Arduino RX)
//   Connect the GPS RX (receive) pin to matching TX1, TX2 or TX3   (Blue wire for GPS RX to Arduino TX)
//also: connect long leg of LED to FIX pin, and short leg to ground. 
//connected this way, LED blinks automatically depending on Fix status (once per second while searching for fix, once per 15 seconds when fix is found)

//GPS communications: If using software serial, keep these lines enabled
// (you can change the pin numbers to match your wiring):
//#if ARDUINO >= 100
//  SoftwareSerial mySerial(3, 2);
//#else
//  NewSoftSerial mySerial(3, 2);
//#endif
//Adafruit_GPS GPS(&mySerial);
// If using hardware serial (e.g. Arduino Mega), comment
// out the above six lines and enable this line instead:
//****************Adafruit_GPS GPS(&Serial1); /// Use this one!!!!!!!

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
//*********************************#define GPSECHO  true

// this keeps track of whether we're using the interrupt
// off by default!
//*********************boolean usingInterrupt = false;
//*************************void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy



                ////////////////////////////////////////////////////////


void displaySensorDetails(void)
{
  sensor_t sensor;
  bmp.getSensor(&sensor);
/*  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" hPa");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" hPa");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" hPa");  
  Serial.println("------------------------------------");
  Serial.println("");*/
  delay(500);

    //sensor_t sensor;
  tsl.getSensor(&sensor);
  /*Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");*/
  delay(500);
  

  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  /*Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));*/
  
  delay(500);
}


              //////////////////////////////////////////////////////////

void configureSensor(void)
{
  /* You can also manually set the gain or enable auto-gain support */
  // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
  // tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
  
  /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */

  /* Update these values depending on what you've set above! */  
  /*Serial.println("------------------------------------");
  Serial.print  ("Gain:         "); Serial.println("Auto");
  Serial.print  ("Timing:       "); Serial.println("13 ms");
  Serial.println("------------------------------------");
*/

   // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
  //Serial.println("9DOF configured @: accel_2G, mag_2Gauss, gyro_245DPS.");
  
}


 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  // put your setup code here, to run once:

   //while (!Serial);  // the Leonardo will 'wait' until the USB plug is connected

  // connect at 115200 so we can read the GPS fast enuf and
  // also spit it out
  Serial.begin(9600);//changed in our case to match rate for rest of code
 /* Serial.println("Adafruit GPS logging start test!");

  // 9600 NMEA is the default baud rate for MTK - some use 4800
  GPS.begin(9600);
  
  // You can adjust which sentences to have the module emit, below
  // Default is RMC + GGA
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);//this is the setting needed to create a Google Earth track of the flight later
  // Default is 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a whole lot easier!
  useInterrupt(true);
  delay(500);
  //Serial.print("\nSTARTING LOGGING....");
  //if (GPS.LOCUS_StartLogger())
    //Serial.println(" STARTED!");
  //else
    //Serial.println(" no response :(");
  delay(1000);
  */

RTC_DS1307 RTC;// must have a coincell battery in the RTC slot on SD Shield, (adafruit shield)
pinMode(10, OUTPUT);

Wire.begin();
RTC.begin();
//if(!RTC.isrunning()){
  //Serial.println("RTC is NOT running! check battery!");
  //if so, unplug arduino, uncomment next line, upload and run code, then unplug, recomment line and upload again.
  //RTC.adjust(DateTime(_DATE_, _TIME_)); //This sets the RTC to your computer's time at instant of compile. upload immediately
//}
  //#ifndef ESP8266
    //while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
 // #endif
 // uint32_t currentFrequency;//may want to comment out these lines for flight code?!!!?

pinMode(redLedpin, OUTPUT);

  Serial.begin(9600);
 // while(!Serial){
   // ;}//wait for serial port to connect. Needed for Leonardo only
  
  //Serial.print("Initializing SD card...");
  if(!SD.begin(10,11,12,13)){
    //Serial.println("Card failed, or not present");
    while(1);
  }
  //Serial.println("card initialized.");
  digitalWrite(redLedpin, HIGH);
  delay(greenOn);
  digitalWrite(redLedpin, LOW);
  delay(offLED);
  digitalWrite(redLedpin, HIGH);
  delay(onLED);
  digitalWrite(redLedpin, LOW);
  delay(offLED);
  digitalWrite(redLedpin, HIGH);
  delay(onLED);
  digitalWrite(redLedpin, LOW);// the LED should blink 2 times, 1sec on, 2sec off.
delay(1000);
  



//Create/open file on SD card to hold data. if file exists, this command opens it
  File dataFile;
  dataFile = SD.open("Log.csv", FILE_WRITE);
  //create a header line to go at the top of the excel spreadsheet
  String headerString = "Time, hPa, tempC, Alt, Lux, busV, shuntmV, loadV, currentmA, accelX, aY, aZ, magX, mY, mZ, gyroX, gY, gZ, TempC"; 
  if(!dataFile){
    //Serial.println("failed to open flightData.csv");
    while(1);
  }
  if(dataFile)
  { 
    //Serial.println("file created");
    dataFile.println(" , , , , , , , , , , , , , , , , , , ");//prints a blank line above header, in case there was already data in the file
    dataFile.println(headerString);
    //Serial.println("header printed");
    }
  dataFile.close();
  delay(250);
    digitalWrite(blueLedpin, HIGH);
  delay(onLED);
  digitalWrite(blueLedpin, LOW);
  delay(offLED);
    digitalWrite(blueLedpin, HIGH);
  delay(onLED);
  digitalWrite(blueLedpin, LOW);
  delay(offLED);
    digitalWrite(blueLedpin, HIGH);
  delay(onLED);
  digitalWrite(blueLedpin, LOW);//This will make the LED blink 3 times, 1sec on, 2sec off each time
delay(1000);
  
  //Serial.println("Pressure Sensor Test"); Serial.println("");

  
  
  /* Initialise the bmp180 sensor */
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    //Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  //Serial.println("Light Sensor Test"); Serial.println("");
  
  /* Initialise the tsl2561 light sensor */
  if(!tsl.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    //Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

// Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();//This line is absolutely necessary so the sensor can pick up the low V&A on our solar array

  //Serial.println("Measuring voltage and current with INA219 ...");
  

  //Serial.println("LSM9DS0 9DOF Sensor Test"); 
  //Serial.println("");
  
  // Initialise the 9dof sensor 
  lsm.begin();
  if(!lsm.begin()){
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    //Serial.print("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  //Serial.println("Found LSM9DS0 9DOF");
  
  /* Display some basic information on the sensors */
  displaySensorDetails();

  
  /* Setup the sensor/s gain and integration time, etc. */
  configureSensor();
  
  /* We're ready to go! */
  //Serial.println("");



  digitalWrite(greenLedpin, HIGH);
  delay(greenOn);
  digitalWrite(greenLedpin, LOW);//LED on for 3 seconds... void loop will now start
  delay(offLED);
  
}





void loop() {
  // put your main code here, to run repeatedly:

  //Create a "String" variable called dataString, 
  //used for collecting data to write to SD.
  String dataString = ""; //We will add to this using dataString += String(...); for each datapoint

String currentTime = "";
  RTC_DS1307 RTC;
  DateTime now = RTC.now();
  
  currentTime += String(now.hour(), DEC);
  currentTime += ":";
  currentTime += String(now.minute(), DEC);
  currentTime += ":";
  currentTime += String(now.second(), DEC);
  
  dataString += String(currentTime);
  dataString += ",";
  

  /* Get a new sensor event */ 
  sensors_event_t event;
  bmp.getEvent(&event);
 
  /* Display the results (barometric pressure is measure in hPa) */
  if (event.pressure)
  {
    /* Display atmospheric pressue in hPa */
    //Serial.print("Pressure:    ");
    //Serial.print(event.pressure);
    //Serial.println(" hPa");

    dataString += String(event.pressure);//adds bmp pressure to data file
    dataString += ","; //adding a comma after every data point is necessary
    
    /* Calculating altitude with reasonable accuracy requires pressure    *
     * sea level pressure for your position at the moment the data is     *
     * converted, as well as the ambient temperature in degress           *
     * celcius.  If you don't have these values, a 'generic' value of     *
     * 1013.25 hPa can be used (defined as SENSORS_PRESSURE_SEALEVELHPA   *
     * in sensors.h), but this isn't ideal and will give variable         *
     * results from one day to the next.                                  *
     *                                                                    *
     * You can usually find the current SLP value by looking at weather   *
     * websites or from environmental information centers near any major  *
     * airport.                                                           *
     *                                                                    *
     * For example, for Paris, France you can check the current mean      *
     * pressure and sea level at: http://bit.ly/16Au8ol                   */
     
    /* First we get the current temperature from the BMP085 */
    float temperature;
    bmp.getTemperature(&temperature);
    //Serial.print("Temperature: ");
    //Serial.print(temperature);
    //Serial.println(" C");

    dataString += String(temperature);
    dataString += ",";

    /* Then convert the atmospheric pressure, and SLP to altitude         */
    /* Update this next line with the current SLP for better results      */
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    //Serial.print("Altitude:    "); 
    //Serial.print(bmp.pressureToAltitude(seaLevelPressure,
//                                        event.pressure)); 
    //Serial.println(" m");
    //Serial.println("");
    //print bmp calculated altitude to datafile
    dataString += String(bmp.pressureToAltitude(seaLevelPressure,event.pressure));
    dataString += ",";
  }
  else//if the above is untrue, then carry out commands below
  {
    //Serial.println("Sensor error");
    dataString += String("error");//this would be printed instead of altitude 
    dataString += ",";
  }


    /* Get a new sensor event */ 
  //sensors_event_t event;
  tsl.getEvent(&event);
 
  /* Display the results (light is measured in lux) */
  if (event.light)
  {
    //Serial.print(event.light); Serial.println(" lux");
    dataString += String(event.light);
    dataString += ",";
  }
  else
  {
    /* If event.light = 0 lux the sensor is probably saturated
       and no reliable data could be generated! */
    //Serial.println("Sensor overload");
    dataString += String("error");
    dataString += ",";
  }


  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");

  dataString += String(busvoltage);
  dataString += ",";
  dataString += String(shuntvoltage);
  dataString += ",";
  dataString += String(loadvoltage);
  dataString += ",";
  dataString += String(current_mA);
  


    /* Get a new sensor event */ 
  sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  //Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");
  dataString += String(accel.acceleration.x);
  dataString += ",";
  dataString += String(accel.acceleration.y);
  dataString += ",";
  dataString += String(accel.acceleration.z);
  dataString += ",";

  // print out magnetometer data
  //Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
  dataString += String(mag.magnetic.x);
  dataString += ",";
  dataString += String(mag.magnetic.y);
  dataString += ",";
  dataString += String(mag.magnetic.z);
  dataString += ",";
  
  // print out gyroscopic data
  //Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  //Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  //Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");
  dataString += String(gyro.gyro.x);
  dataString += ",";
  dataString += String(gyro.gyro.y);
  dataString += ",";
  dataString += String(gyro.gyro.z);
  dataString += ",";
  
  // print out temperature data
  //Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");
  dataString += String(temp.temperature);
  dataString += ",";
  
  //Serial.println("**********************\n");


File dataFile = SD.open("Log.csv", FILE_WRITE);
if(dataFile){
  dataFile.println(dataString);
  dataFile.close();
  //Serial.println("printing data to file");
}
  digitalWrite(greenLedpin, HIGH);
  delay(250);
  digitalWrite(greenLedpin, LOW);
  delay(refresh_rate);

}


/******************************************************************/
/*
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO && c) {
#ifdef UDR0
    UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
  }
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }*/
//}
