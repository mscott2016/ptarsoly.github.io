#include <Adafruit_Sensor.h>


/***************************************************************************
  This is a library example for the HMC5883 magnentometer/compass

  Designed specifically to work with the Adafruit HMC5883 Breakout
  http://www.adafruit.com/products/1746
 
  *** You will also need to install the Adafruit_Sensor library! ***

  These displays use I2C to communicate, 2 pins are required to interface.
-
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries with some heading example from
  Love Electronics (loveelectronics.co.uk)
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the version 3 GNU General Public License as
 published by the Free Software Foundation.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.

 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


unsigned int buttonPin = 11; 
int ledState = LOW;
//int count=0;
float  zeroMeanx = 0 ;
unsigned long zeroCoux, zeroValx ;
float  zeroMeany  = 0;
unsigned long  zeroCouy, zeroValy ;
float  zeroMeanz = 0 ;
unsigned long  zeroCouz, zeroValz ;
int lastKnownButtonState = LOW;
unsigned long lastTimeButtonChanged = 0;
unsigned int debounceDelay = 50;
int buttonState = LOW;
int calibrationPin = 13;
int lad = HIGH;
int starttime = 0;
 unsigned long startTimeAver = 0;
   unsigned long waittime = 1000; 
   
    int waitTimeOver = 250; 
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void) 
{
  pinMode( buttonPin, INPUT);
  Serial.begin(9600);
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  pinMode(calibrationPin,OUTPUT);
  sensors_event_t event; 
  mag.getEvent(&event);
  
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  
  digitalWrite(calibrationPin, LOW);  
  
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

float meanx = 0 ;
float meany = 0 ;
float meanz = 0 ;
float avX;
float avY;
float avZ;
int count = 0;
bool zero = false;
float  xcal = 6;
float  ycal = 8;
float  zcal = 12;
void averageValue( bool zeros)
{
  
  sensors_event_t event; 
  mag.getEvent(&event);

if(millis() - starttime >= waittime)
  {
    starttime = millis();
 if(millis() - startTimeAver >= waitTimeOver)
  {
   startTimeAver = millis();
  if (count < 4 ){
  meanx += xcal+event.magnetic.x;
  meany += ycal+event.magnetic.y;
  meanz += zcal+event.magnetic.z;
  zeroMeanx  += meanx ;
  zeroMeany += meany ;
  zeroMeanz += (meanz) ;
  zeroCoux ++;
  count++; 
  
  if (count == 4)
  {
    avX =  meanx / 4;
     avY =  meany / 4;
      avZ =  meanz / 4;
      zeroMeanx  = avX ;
   zeroMeany  = avY ; zeroMeanz  = avZ ;
  Serial.print("X: "); Serial.print(/*map(abs(*/avX/*),val1Min,val1Max,0,55))*/); Serial.print("  ");
 Serial.print("Y: "); Serial.print(/*map(abs(*/avY/*),val2Min,val2Max,0,55))*/); Serial.print("  ");
  Serial.print("Z: "); Serial.print(/*map(abs(*/avZ/*),val3Min,val3Max,0,55))*/); Serial.print("  ");Serial.println("uT");
    count = 0;
    avX =  0;
     avY = 0;
      avZ =  0;
     meanx = 0;
  meany = 0 ;
  meanz = 0;
    }
  }
   }
 
    
 }


  if (zeros)
  {
    xcal  = (-1) *( zeroMeanx );
    ycal  = (-1)* (zeroMeany );
    zcal  = (-1)*( zeroMeanz );
     zero = false;
    Serial.println("dd");
    Serial.println(ycal);
     Serial.println("dd");
    Serial.println(xcal);
     Serial.println("dd");
   // Serial.println(ycal);
    }
 }

void loop(void) 
{
  int buttonValue = digitalRead(buttonPin);
  //Serial.println(buttonValue);
if (buttonValue != lastKnownButtonState) {
  // Record the time that the button changed from its last known state.
  lastTimeButtonChanged = millis();
}
if ((millis() - lastTimeButtonChanged) > debounceDelay) {
    if (buttonValue != buttonState) {
       buttonState = buttonValue;
    // only toggle the LED if the new button state is HIGH
    if (buttonState == HIGH) {
  //   count++;
       //Serial.println("yyyy");
        zero = true;
         } 
    
    }
  } 
lastKnownButtonState = buttonValue;
  /* Get a new sensor event */ 
  averageValue( zero);
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
 //Serial.print("X: "); Serial.print(/*map(abs(*/6+event.magnetic.x/*),val1Min,val1Max,0,55))*/); Serial.print("  ");
 // Serial.print("Y: "); Serial.print(/*map(abs(*/8+event.magnetic.y/*),val2Min,val2Max,0,55))*/); Serial.print("  ");
 //Serial.print("Z: "); Serial.print(/*map(abs(*/12+event.magnetic.z/*),val3Min,val3Max,0,55))*/); Serial.print("  ");Serial.println("uT");
  
//delay(500);
}
