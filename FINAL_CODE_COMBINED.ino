#include "DHT.h"

#define DHTPIN A1 

#define DHTTYPE DHT11  
DHT dht(DHTPIN, DHTTYPE);

char Incoming_value = 0;

int redLed = 10;
int greenLed = 11;
int buzzer = 12;
int smokeA2 = A2;
int sensorThres = 80;  

#define ReedSensor 2
#define reedOpenLED 9
#define reedClosedLED 8

int motionLED = 7; 
int motionDetector = 3;
int pirState = LOW;

const int VOLT_PIN = A3;

void setup() 
{
  Serial.begin(9600); 

  /////////LIGHTS//////////
  pinMode(13, OUTPUT); 

  /////////SMOKE DETECTOR//////
  pinMode(redLed, OUTPUT);
  pinMode(greenLed, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(smokeA2, INPUT); 

  ////////TEMP+HUMD////////
//  Serial.println(F("DHTxx test!"));
  dht.begin();  

  ////////REED SWITCH//////
  pinMode(ReedSensor, INPUT);
  pinMode(reedOpenLED, OUTPUT);
  pinMode(reedClosedLED, OUTPUT);

  ///////MOTION SENSOR///////
  pinMode(motionLED, OUTPUT);
  pinMode(motionDetector, INPUT);
}

void loop()
{

  /////////LIGHTS//////////
  if(Serial.available() > 0)  
  {
    Incoming_value = Serial.read();           
    if(Incoming_value == '1')             
      digitalWrite(13, HIGH);  
    else if(Incoming_value == '0')       
      digitalWrite(13, LOW);
  }

  ////////TEMP+HUMD///////////

  delay(1000);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float f = dht.readTemperature(true);

    if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }
  Serial.print(t,1);
  Serial.print("C");
  Serial.print("|");
  Serial.print(h,1);
  Serial.print("%");
    
  //////////POWER USAGE MONITOR////////
  int powerValue = analogRead(VOLT_PIN);
  float volt = powerValue * 5.0 / 1023.0;
    Serial.print("|");
    Serial.print( powerValue, 1 );
    Serial.print("P");
    Serial.print("|");
   Serial.print( volt, 1);
   Serial.print("V");
    
/////////SMOKE DETECTOR////////
    int analogSensor = analogRead(smokeA2);
    
  Serial.print("|");
  Serial.print(analogSensor);
  if (analogSensor > sensorThres)
  {
    digitalWrite(redLed, HIGH);
    digitalWrite(greenLed, LOW);
    tone(buzzer, 1000);
  }
  else
  {
    digitalWrite(redLed, LOW);
    digitalWrite(greenLed, HIGH);
    noTone(buzzer);
  }

  ///////////REED SWITCH//////
  bool Reedvalue = digitalRead(ReedSensor);
  Serial.print("|");
 Serial.print(Reedvalue);

   if (Reedvalue == 0) {
    digitalWrite(reedClosedLED, LOW);
    digitalWrite(reedOpenLED, HIGH);
  } else {
    digitalWrite(reedOpenLED, LOW);
    digitalWrite(reedClosedLED, HIGH);
  }

  /////////MOTION DETECTOR/////////
  int Motionvalue = 0; 
  Motionvalue = digitalRead(motionDetector);
  Serial.print("|");
   Serial.println(Motionvalue);
  if (Motionvalue == HIGH) {
    digitalWrite(motionLED, HIGH);
    if (pirState == LOW) { 
      tone(buzzer, 1000);
      pirState = HIGH;
    } 
    }else {
    digitalWrite(motionLED, LOW);
    if (pirState == HIGH) {
      pirState = LOW;
    }
  }
delay(1000);
  }
