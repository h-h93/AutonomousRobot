
#include "DHT.h"
#include "SoftwareSerial.h"
#include <avr/pgmspace.h>
#include <Wire.h>
#include <Servo.h>
#include "SimpleTimer.h"
#include <Adafruit_GPS.h> //create vriables to text temp gps and gas data back
#include <GPRS_Shield_Arduino.h>

//using pin 7 9 10 and 6 for car
/************************Hardware Related Macros************************************/
#define         MQ_PIN                       (0)     //define which analog input channel you are going to use
#define         RL_VALUE                     (5)     //define the load resistance on the board, in kilo ohms
#define         RO_CLEAN_AIR_FACTOR          (9.83)  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
//which is derived from the chart in datasheet

/***********************Software Related Macros************************************/
#define         CALIBARAION_SAMPLE_TIMES     (50)    //define how many samples you are going to take in the calibration phase
#define         CALIBRATION_SAMPLE_INTERVAL  (500)   //define the time interal(in milisecond) between each samples in the
//cablibration phase
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interal(in milisecond) between each samples in 
//normal operation

/**********************Application Related Macros**********************************/
#define   GAS_LPG  (0) //these are stored in progmem by using the
#define   GAS_CO   (1) // keyword define it stores them in prog
#define   GAS_SMOKE (2)

/*********************Macro for DHT22**********************************************/
#define DHTPIN 2     // what digital pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321

/*****************************Globals***********************************************/
// change these variables to be stored in eeprom
float LPGCurve[3] = {2.3, 0.21, -0.47}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
float COCurve[3] = {2.3, 0.72, -0.34};  //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15)
float SmokeCurve[3] = {2.3, 0.53, -0.44}; //two points are taken from the curve.
//with these two points, a line is formed which is "approximately equivalent"
//to the original curve.
//data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22)
float Ro = 10;                 //Ro is initialized to 10 kilo ohms

float LPGLevel;
float COLevel;
float SmokeLevel;

// declare gps and gprs //////////////
SoftwareSerial mySerial(11, 12);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  true
float latitude;
float longitude;
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
SoftwareSerial sim900(7,8);
uint32_t timer = millis(); // for gps
SimpleTimer timerSendText;
int messageIndex = 0;
char message[200];
char phoneNum[11];
char Time[12];

// temp sesnor var /////////////////
DHT dht(DHTPIN, DHTTYPE);
float heatIndexCelsius;
float humidity;
float temperature;

// vehicle var //////////////////
//Robot speed is 8.875 in/s
//rotation spedd is 120°/s
//Declare Servos
Servo leftservo;  //Left wheel servo
Servo rightservo; //Right wheel servo
Servo scanservo;  //Ping Sensor Servo
const int pingPin = 13;  //Pin that the Ping sensor is attached to.
const int leftservopin = 9; //Pin number for left servo
const int rightservopin = 10; // Pin number for right servo
const int scanservopin = 6;   // Pin number for scan servo
const int distancelimit = 20;   //If something gets this many inched from
                              // the robot it stops and looks for where fo go.
int scantime = 0;
int lastscantime = 0;
char sensorpos = 'L';
long oldtime = 0;
long timesinceturnedleft = 0;
long timesinceturnedright = 0;



void setup()
{
  Serial.begin(115200);
  GPS.begin(9600);
  sim900.begin(9600);
  leftservo.attach(leftservopin); //Attach left servo to its pin.
  rightservo.attach(rightservopin); // Attch the right servo
  scanservo.attach(scanservopin); // Attach the scan servo
  Serial.begin(9600);//UART setup, baudrate = 9600bps
  Serial.print("Calibrating...\n");
  Ro = MQCalibration(MQ_PIN);//Calibrating the sensor. Please make sure the sensor is in clean air                                                     //when you perform the calibration
  Serial.print("Ro=");
  Serial.print(Ro);
  Serial.print("kohm");
  Serial.print("\n");
  //2 7 9 10 6 in use
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  dht.begin();
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
    // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  //useInterrupt(true);
  delay(20000);
  timerSendText.setInterval(15000, textData);

}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
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
  }
}

void loop()
{
  timerSendText.run();
  // begin obstacle avoidance code //////
  int leftdistance = 90;
  int rightdistance = 90;
  go();  // if nothing is wrong the go forward using go() function below.
  if(millis()>oldtime+300){
  if(sensorpos == 'L'){
      leftdistance = ping();
      sensorpos = 'R';
  }
  else{
      rightdistance = ping();
      sensorpos = 'L';
  }
  oldtime = millis();
  }
  switch (sensorpos){
  case 'L':
      scanservo.write(70);
      break;
  case 'R':
      scanservo.write(110);
      break;
  }
  if(leftdistance<distancelimit){
  if (millis()<timesinceturnedleft + 500){
      goback(500);
      turnright(120);
  }
  else{
      turnright(25);
  }
    timesinceturnedright = millis();
  }
  if(rightdistance<distancelimit){
    if(millis()<timesinceturnedright + 500){
      goback(500);
      turnleft(120);
  }
  else{
      turnleft(25);
  }
    timesinceturnedleft = millis();
  } // end obstacle avoidance code //////

  // begin mq2 code /////
  Serial.print("LPG:");
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG) );
  Serial.print( "ppm" );
  Serial.print("    ");
  Serial.print("CO:");
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO) );
  Serial.print( "ppm" );
  Serial.print("    ");
  Serial.print("SMOKE:");
  Serial.print(MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE) );
  Serial.print( "ppm" );
  Serial.print("\n");

  LPGLevel = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_LPG);
  COLevel = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_CO);
  SmokeLevel = MQGetGasPercentage(MQRead(MQ_PIN) / Ro, GAS_SMOKE);
  
  delay(200); // remove delay we don't need it just text every 2 minutes
  // end mq2 code ////
  
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  humidity = dht.readHumidity();
  // Read temperature as Celsius (the default)
  temperature = dht.readTemperature();
  // Read temperature as Fahrenheit (isFahrenheit = true)

  // Check if any reads failed and exit early (to try again).
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  } 

  // Compute heat index in Celsius (isFahreheit = false)
  heatIndexCelsius = dht.computeHeatIndex(temperature, humidity, false);

  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.print(" %\t");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print("*C ");
  Serial.print("Heat index: ");
  Serial.print(heatIndexCelsius);
  Serial.print("*C ");
  Serial.println(" ");
  //delay(2000); // can remove these delays and print statements all we need to do is just make sure we send a text every 2 minutes or so with current temp gps location gas readings etc..
  // end temp code //// 
  
   // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  /*
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    */
    if (GPS.fix) {
      latitude = (GPS.latitudeDegrees,4);
      longitude = (GPS.longitudeDegrees,4);
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      // use these print statements data to text back
      Serial.print("Location (in degrees, works with Google Maps): ");
      Serial.print(GPS.latitudeDegrees, 4);
      Serial.print(", "); 
      Serial.println(GPS.longitudeDegrees, 4);
      // ends here
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
  
   
  

}

/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
         across the load resistor and its resistance, the resistance of the sensor
         could be derived.
************************************************************************************/
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE * (1023 - raw_adc) / raw_adc));
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  Ro of the sensor
  Remarks: This function assumes that the sensor is in clean air. It use
         MQResistanceCalculation to calculates the sensor resistance in clean air
         and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about
         10, which differs slightly between different sensors.
************************************************************************************/
float MQCalibration(int mq_pin)
{
  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {      //take multiple samples
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;                 //calculate the average value

  val = val / RO_CLEAN_AIR_FACTOR;                      //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  //according to the chart in the datasheet

  return val;
}
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
         The Rs changes as the sensor is in the different consentration of the target
         gas. The sample times and the time interval between samples could be configured
         by changing the definition of the macros.
************************************************************************************/
float MQRead(int mq_pin)
{
  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         gas_id      - target gas type
  Output:  ppm of the target gas
  Remarks: This function passes different curves to the MQGetPercentage function which
         calculates the ppm (parts per million) of the target gas.
************************************************************************************/
int MQGetGasPercentage(float rs_ro_ratio, int gas_id)
{

  if ( gas_id == GAS_LPG ) {
    return MQGetPercentage(rs_ro_ratio, LPGCurve);
  } else if ( gas_id == GAS_CO ) {
    return MQGetPercentage(rs_ro_ratio, COCurve);
  } else if ( gas_id == GAS_SMOKE ) {
    return MQGetPercentage(rs_ro_ratio, SmokeCurve);
  }

  return 0;
}

/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
         pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm)
         of the line could be derived if y(rs_ro_ratio) is provided. As it is a
         logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic
         value.
************************************************************************************/
int  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10, ( ((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}

// vehicle obstacle avoidance methods //// 
int ping(){
  long duration, inches;
  //Send Pulse
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);
  //Read Echo
  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
 
  // convert the time into a distance
  inches = microsecondsToInches(duration);
  Serial.print("Ping:  ");
  Serial.println(inches);
  return round(inches);
}
 
void go(){
  leftservo.write(30);
  rightservo.write(150);
}
 
void turnleft(float t){
  leftservo.write(150);
  rightservo.write(150);
  t=(t/120)*1000;
 
  Serial.print("Turning left for this many seconds: ");
  Serial.println(t);
  delay(t);
}
  
void turnright(float t){
  leftservo.write(30);
  rightservo.write(30);
  t=(t/120)*1000;
 
  Serial.print("Turning right for this many seconds: ");
  Serial.println(t);
  delay(t);
}   
 
void goforward(int t){
  leftservo.write(30);
  rightservo.write(150);
  delay(t);
}
 
void goback(int t){
  leftservo.write(150);
  rightservo.write(30);
  delay(t);
}
 
void stopmotors(){
  leftservo.write(90);
  rightservo.write(90);
}   
 
char scan(){
  int lval, lcval, cval, rcval, rval;
  int maximum = 0;
  int choice;
 
  scanservo.write(25); //Look left
  delay(300);
  lval = ping();
  if(lval>maximum){
  maximum = lval;
  choice = 1;
}
 
  scanservo.write(45); //Look left center
  delay(300);
  lcval = ping();
  if(lcval>maximum){
  maximum = lcval;
  choice = 2;
}
 
  scanservo.write(90); //Look center
  delay(300);
  cval = ping();
  if(cval>maximum){
  maximum = cval;
  choice = 3;
}
 
  scanservo.write(135); //Look right center
  delay(300);
  rcval = ping();
  if(rcval>maximum){
  maximum = rcval;
  choice = 4;
}
 
  scanservo.write(155);  //Look right
  delay(300);
  rval = ping();
  if(rval>maximum){
  maximum = rval;
  choice = 5;
}
 
if(maximum<=distancelimit){choice = 6;}
 
  scanservo.write(88);  //center scan servo
  delay(300);
 
  Serial.print("Choice:  ");
  Serial.println(choice);
  return choice;
}
 
long microsecondsToInches(long microseconds){
  return microseconds / 74 / 2;
}
 
long microsecondsToCentimeters(long microseconds){
  return microseconds / 29 / 2;
} 
// end vehicle obstacle avoidance methods ///

// text back data function //////////////////
void textData(){
  sim900.print("AT+CMGF=1\r");
  delay(200);
  sim900.println("AT+CMGS = \"+447479927812\"");
  delay(200);
  sim900.println((String) temperature + " " + (String) heatIndexCelsius + " " + (String) humidity + " "  + 
    (String) latitude + " " + (String) longitude + " " + (String) LPGLevel + " " + (String) COLevel + " " + (String) SmokeLevel);
  delay(200);
  sim900.print((char)26);
  delay(200);
  sim900.println();
 }
