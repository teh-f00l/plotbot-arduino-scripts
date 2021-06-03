//#include "WiFiEsp.h"
#include <OneWire.h>
//#include <Arduino.h>
#include <SoftwareSerial.h>
#include <ArduinoJson.h>


//SoftwareSerial wifiSerial(0, 1); // RX, TX

//int status = WL_IDLE_STATUS;     // the Wifi radio's status

#define StartConvert 0
#define ReadTemperature 1
#include <dht11.h>
dht11 DHT;
#define DHT11_PIN 4
//hello
//// CO2 sample code
//// define pin for CO2
#define MG_PIN (A0)

#define         BOOL_PIN                     (3)
#define         DC_GAIN                      (8.5)   //define the DC gain of amplifier
#define         READ_SAMPLE_INTERVAL         (50)    //define how many samples you are going to take in normal operation
#define         READ_SAMPLE_TIMES            (5)     //define the time interval(in milisecond) between each samples in

#define         ZERO_POINT_VOLTAGE           (0.220) //define the output of the sensor in volts when the concentration of CO2 is 400PPM
#define         REACTION_VOLTGAE             (0.030) //define the voltage drop of the sensor when move the sensor from air into 1000ppm CO2

// set data types for variables

// use gps to caluclate time



float           CO2Curve[3]  =  {2.602,ZERO_POINT_VOLTAGE,(REACTION_VOLTGAE/(2.602-3))};

int  analogForPh = A2;
const byte numReadings = 20;     //the number of sample times
byte ECsensorPin = A1;  //EC Meter analog output,pin on analog 1
byte DS18B20_Pin = 2; //DS18B20 signal, pin on digital 2
unsigned int AnalogSampleInterval=25,printInterval=700,tempSampleInterval=850;  //analog sample interval;Serial print interval;temperature sample interval
unsigned int readings[numReadings];      // the readings from the analog input
byte index = 0;                  // the index of the current reading
unsigned long AnalogValueTotal = 0;                  // the running total
unsigned int AnalogAverage = 0,averageVoltage=0;                // the average
unsigned long AnalogSampleTime,printTime,tempSampleTime;
float temperature,ECcurrent, val, ph; 
char result [50];


 
//Temperature chip i/o
OneWire ds(DS18B20_Pin);  // on digital pin 2



void setup() {
 // initialize Serial communication with computer:
//  Serial.begin(9600);
  Serial.begin(115200);
  pinMode(BOOL_PIN, INPUT);     //set pin to input

 

  // this line can be improved, tuned for multiple sensors rather than just the one
  
  digitalWrite(BOOL_PIN, HIGH);                    //turn on pullup resistors, set it for 5V sensors


  Serial1.begin(115200);
//Serial1.begin(9600);
  
  Serial.print("MG-811 Demostration\n");


  
  // initialize all the readings to 0:
  for (byte thisReading = 0; thisReading < numReadings; thisReading++)
    readings[thisReading] = 0;
  TempProcess(StartConvert);   //let the DS18B20 start the convert
  
  // get sample times for sensors in milliseconds
  AnalogSampleTime=millis();
  printTime=millis();
  tempSampleTime=millis();
}

void loop() {
  
  delay(10000);
  Serial.write("\n");
  //   Every once in a while,sample the analog value and calculate the average.
  //  */
    if(millis()-AnalogSampleTime>=AnalogSampleInterval)   
    {
     AnalogSampleTime=millis();
      // subtract the last reading:
     AnalogValueTotal = AnalogValueTotal - readings[index];
     // read from the sensor:
      readings[index] = analogRead(ECsensorPin);
      // add the reading to the total:
      AnalogValueTotal = AnalogValueTotal + readings[index];
      // advance to the next position in the array:
      index = index + 1;
      // if we're at the end of the array...
      if (index >= numReadings)
      // ...wrap around to the beginning:
      index = 0;
  //    // calculate the average:
      AnalogAverage = AnalogValueTotal / numReadings;
    }
    /*
     Every once in a while,MCU read the temperature from the DS18B20 and then let the DS18B20 start the convert.
     Attention:The interval between start the convert and read the temperature should be greater than 750 millisecond,or the temperature is not accurate!
    */
     if(millis()-tempSampleTime>=tempSampleInterval) 
    {
      tempSampleTime=millis();
      temperature = TempProcess(ReadTemperature);  // read the current temperature from the  DS18B20
      TempProcess(StartConvert);                   //after the reading,start the convert for next reading
    }




//// CO2 Loop

    int percentage;
    float volts;

    volts = MGRead(MG_PIN);
    Serial.print( "SEN0159:" );
    Serial.print(volts);
    Serial.print( "V           " );

    percentage = MGGetPercentage(volts,CO2Curve);
    Serial.print("CO2:");
    if (percentage == -1) {
        Serial.print( "<400" );
    } else {
      
        Serial.print(percentage);
    }
String sendC02 = String(percentage);
//Serial1.write(percentage);
    Serial.print( "ppm" );
    Serial.print("\n");

    if (digitalRead(BOOL_PIN) ){
        Serial.print( "=====BOOL is HIGH======" );
    } else {
        Serial.print( "=====BOOL is LOW======" );
    }

    Serial.print("\n");

    delay(500);


    volts = MGRead(MG_PIN);
    Serial.print( "SEN0159:" );
    Serial.print(volts);
    Serial.print( "V           " );


    
     /*
     Every once in a while,print the information on the Serial monitor.
    */
    String sendEC;
    
    if(millis()-printTime>=printInterval)
    {
      printTime=millis();
      averageVoltage=AnalogAverage*(float)5000/1024;

      
      float TempCoefficient=1.0+0.0185*(temperature-25.0);
      float CoefficientVolatge=(float)averageVoltage/TempCoefficient;   
      ECcurrent=CoefficientVolatge; 
      dtostrf(ECcurrent,2,2,result);
      Serial.write("EC:");
     
      //variable to send data
      sendEC = result;
//      String a = String(ECcurrent);
      Serial.print(sendEC);
      Serial.write(result);
      Serial.write("ms/cm");
      Serial.write("\n");
      
    }
    
  // print results with callibration equations
  Serial.write("Ph Value is:");
  val = analogRead(analogForPh);
  ph = val*5.0/1024;
  ph = ph*4.3-1.9;
  dtostrf(ph,3,2,result);

//  Serial1.print(ph);
  //variable to send data
  String sendPH = result;
  
  Serial.write(result);
  Serial.write("\n");
  int chk;
  chk = DHT.read(DHT11_PIN); 
  
  Serial.write("Humidity is: ");
  dtostrf(DHT.humidity,3,2,result);
  Serial.write(result);
  
  //variable to send data
//  Serial1.print(DHT.humidity);
  String sendHumidity = result;
  
  Serial.write("\n");


  Serial.write("Temperature is: ");
  dtostrf(DHT.temperature,3,2,result);
  Serial.write(result);
  Serial.write("\n");
  
    //variable to send data
//    Serial1.print(DHT.temperature);
  String sendTemperature = result;

  // send data to nodemcu

  
//  String data = "";
  
//  data.concat(sendTemperature);
//  data.concat(" ");
//  data.concat(sendHumidity);
//  data.concat(" ");
//  data.concat(sendEC);
//  data.concat(" ");
//  data.concat(sendC02);
//  data.concat(" ");
//  data.concat(sendPH);
//    wifiSerial.print(sendTemperature,sendHumidity, sendEC, sendC02, sendPH)
sendC02 = "400";
//    time_t t = now();
//    Serial.println(timeStatus()); 
//    Serial.println(t);
  DynamicJsonDocument doc(1024);
  doc["type"] = "response";
  doc["ph"] = sendPH;
  doc["temp"] = sendTemperature;
  doc["co2"] = sendC02;
  doc["ec"] = sendEC;
  serializeJson(doc,Serial);
   String str = "<pH="+sendPH+"&EC="+ sendEC +"&Temp="+sendTemperature+"&Humidity="+sendHumidity+ "&Co2=" + sendC02+">";
//  wifiSerial.write(str);
//    Serial.println(str);
//Serial1.write(str);
  
}




/*
ch=0,let the DS18B20 start the convert;ch=1,MCU read the current temperature from the DS18B20.
*/
float TempProcess(bool ch)
{
  //returns the temperature from one DS18B20 in DEG Celsius
  static byte data[12];
  static byte addr[8];
  static float TemperatureSum;
  if(!ch){
          if ( !ds.search(addr)) {
              Serial.write("no more sensors on chain, reset search!");
              ds.reset_search();
              return 0;
          }      
          if ( OneWire::crc8( addr, 7) != addr[7]) {
              Serial.print("CRC is not valid!");
              return 0;
          }        
          if ( addr[0] != 0x10 && addr[0] != 0x28) {
              Serial.print("Device is not recognized!");
              return 0;
          }      
          ds.reset();
          ds.select(addr);
          ds.write(0x44,1); // start conversion, with parasite power on at the end
  }
  else{  
          byte present = ds.reset();
          ds.select(addr);    
          ds.write(0xBE); // Read Scratchpad            
          for (int i = 0; i < 9; i++) { // we need 9 bytes
            data[i] = ds.read();
          }         
          ds.reset_search();           
          byte MSB = data[1];
          byte LSB = data[0];        
          float tempRead = ((MSB << 8) | LSB); //using two's compliment
          TemperatureSum = tempRead / 16;
    }
          return TemperatureSum;  
}

//
//// CO2 Stuff
float MGRead(int mg_pin)
{
    int i;
    float v=0;

    for (i=0;i<READ_SAMPLE_TIMES;i++) {
        v += analogRead(mg_pin);
        delay(READ_SAMPLE_INTERVAL);
    }
    v = (v/READ_SAMPLE_TIMES) *5/1024 ;
    return v;
}

// C02 stuff
int  MGGetPercentage(float volts, float *pcurve)
{
   if ((volts/DC_GAIN )>=ZERO_POINT_VOLTAGE) {
      return -1;
   } else {
      return pow(10, ((volts/DC_GAIN)-pcurve[1])/pcurve[2]+pcurve[0]);
   }
}
