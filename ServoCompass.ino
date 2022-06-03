// So far, all these libs are available from the arduino library manager

#include <TinyGPSPlus.h> //https://github.com/Tinyu-Zhao/TinyGPSPlus-ESP32 - which uses SoftwareSerial
#include <SoftwareSerial.h>
#include <PCA9685.h>     // https://github.com/FaBoPlatform/FaBoPWM-PCA9685-Library
#include <mpu9250.h>     // https://github.com/bolderflight/mpu9250
// the bolder flight libs also req their units (https://github.com/bolderflight/units), 
// (possiby) navigation (https://github.com/bolderflight/navigation), and eigan (https://github.com/bolderflight/eigen) libs.
#include <navigation.h>

// create the PCA9685 pwm driver object
PCA9685 pca9685;

// create the 9dof object
bfs::Mpu9250 imu;
PCA9685::DurationMicroseconds servo_pulse_duration;

// setup the soft serial port and gps, create the object.
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
bool printGPSdata;
// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

// Some values for playing with a servo
// This could also be a rudder feedback sensor at some point.
const int analogInPin = 34;          // which pin is the wiper of the pot attached to?
int potValue = 0;                    // we write the value of the pot into this int
int mappedValue = 0;                 // we map the potValue into a range the servo can use
int degrees = 0;                     // we will map the orientation of the servo in approx degrees
int servoChannel = 0;                // which channel on the PCA9685 are we on?
int servo_pulse_duration_min = 650;  // what is the lowest value to send to the servo?
int servo_pulse_duration_max = 2350; // what is the highest value to sent to the servo

// some accel values from the 9dof
float ax,ay,az;
// some gyro values from the 9dof
float gx,gy,gz;
// some mag values from the 9dof
float mx,my,mz;
// the temp of the chip
float temp;

void setup(){
  // start the serial port for debugging
  Serial.begin(115200);
  Serial.println("RESET");
  Serial.println();
  // start the soft serial port for the GPS
  ss.begin(GPSBaud);

  Serial.println("configuring device.");
  /* Start the I2C bus */
  Serial.println("init i2c");
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("config the mpu9250");
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  
  Serial.println("init the mpu9250");
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
    Serial.println("config the mpu9250 rate divider");
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }

  Serial.println("init pca9685");
  pca9685.setupSingleDevice(Wire,0x40);
  Serial.println("config pca9685 output enable pin");
  pca9685.setupOutputEnablePin(2);
  Serial.println("enable pca9685 output enable pin");
  pca9685.enableOutputs(2);
  Serial.println("set pca9685 servo freq");
  pca9685.setToServoFrequency();
  Serial.println("set pca9685 servo value to the lowest setting");
  servo_pulse_duration = servo_pulse_duration_min;
}

void loop()
{
      // check that the soft serial port is availalbe
    while (ss.available() > 0)
      if (gps.encode(ss.read()))
        printGPSdata = true; 

    // every 5 seconds, check for updated gps info
    if (millis() > 5000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while(true);
    }
  // get the potValue and map it to what the servo wants and compute approx degrees
  potValue = analogRead(analogInPin);
  mappedValue = map(potValue, 0, 4096, servo_pulse_duration_min, servo_pulse_duration_max);
  degrees = map(potValue, 0, 4096, 0, 180);

  // set the pulse duration in miliseconds to the mappedValue
  servo_pulse_duration = mappedValue;

  // read the imu, if data is updated, do something with it.
  if ((imu.Read()) && (imu.new_mag_data())) {
  // go get the info off the 9dof
    ax = imu.accel_x_mps2();
    ay = imu.accel_y_mps2();
    az = imu.accel_z_mps2();
    gx = imu.gyro_x_radps();
    gy = imu.gyro_y_radps();
    gz = imu.gyro_z_radps();
    mx = imu.mag_x_ut();
    my = imu.mag_y_ut();
    mz = imu.mag_z_ut();
    temp = imu.die_temp_c();
  }
 
  // if we are within Xms of a half or whole second, print the stuff
  if ((millis() % 500) <= 11  ){
    // print the info
    /*
    Serial.print("potValue: ");
    Serial.println(potValue);
    Serial.print("mappedValue: ");
    Serial.println(mappedValue);
    Serial.print("degrees: ");
    Serial.println(degrees);
    */
    Serial.print("ax: ");
    Serial.print(ax);
    Serial.print("\tay:");
    Serial.print(ay);
    Serial.print("\taz: ");
    Serial.print(az);
    
    Serial.print("\tgx: ");
    Serial.print(gx);
    Serial.print("\tgy: ");
    Serial.print(gy);
    Serial.print("\tgz: ");
    Serial.print(gz);
    
    Serial.print("\tmx: ");
    Serial.print(mx);
    Serial.print("\tmy: ");
    Serial.print(my);
    Serial.print("\tmz: ");
    Serial.print(mz);
    
    Serial.print("\ttemp: ");
    Serial.println(temp);
    
    if (printGPSdata){
      displayInfo();
      printGPSdata = false;
    }
    
    }
    
  // Now that we've done all the computation and display, lets set the servo value
  pca9685.setChannelServoPulseDuration(servoChannel,servo_pulse_duration);

  //delay(250); // dont use delay, it slows the whole loop. use millis() and a modulo to do what you want when you want.
}

void displayInfo()
{
  if (gps.location.isValid())
  {
    Serial.print("lat:");
    Serial.print(gps.location.lat(), 6);
    Serial.print("\tlong:");
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print("\t\tDate: ");
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  if (gps.time.isValid())
  {
    Serial.print("\tTime: ");
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(":");
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(":");
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(".");
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}
