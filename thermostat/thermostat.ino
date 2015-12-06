#include <DHT.h>

#include <math.h>
#include <Wire.h>
#include <PID_v1.h>


#define DHTPIN 2 
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Constants

//int B = 3975;                  // B value of the thermistor

int relay= 4;

unsigned long WATTS_CLEVER_DEVICE_ID = 0x62E650;
unsigned char ON_CODES[3] = {0xF,0xD,0xA};
unsigned char OFF_CODES[3] = {0x7, 0x5, 0x2};

// PID tuning
double KP=45;    // 2.2 degrees out = 100% heating
double KI=0.05;  // 3% per degree per minute
double KD=0;     // Not yet used
unsigned long windowSize = 1200000; // 20 minutes (ish)

// State
int i;
double setPoint = 22.0;
double temperature, pidOutput, currentWindowPidOutput = 0;
unsigned long windowStartTime;
boolean heaterOn = false;

// Objects

PID myPID(&temperature, &pidOutput, &setPoint, KP, KI, KD, DIRECT);

void setup() {
 
  Serial.begin(9600);  
   
  windowStartTime = millis();
  
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 100);
  
   pinMode(relay, OUTPUT);
 
}

void loop() {
  if(i % 25000 == 0) temperature = readTemperature();
  //delay(20);
  
   
  if(i % 2500 == 0) myPID.Compute();
  if(i % 5000 == 0) updateDisplay();
  
  if(i % 25000 == 0) updateOutput();
  
  i++;
}

float readTemperature() {
 

  float temperature = dht.readTemperature();
  if (isnan(temperature) ) {
    Serial.println("Failed to read from DHT sensor!");
    temperature = setPoint;
    
  }
  
  
  return temperature;
  delay(4000);
}

void updateDisplay() {
  if(i % 200 == 0) {
    Serial.print("");
    Serial.print(temperature);
    Serial.print(", ");
    Serial.print(pidOutput, 0);
    Serial.print(", ");
    Serial.print(heaterOn);
    Serial.print(", ");
    Serial.print(myPID.GetKp());
    Serial.print(", ");
    Serial.println(myPID.GetKi());
  }
  
 
}

void updateOutput() {
  unsigned long now = millis();
  if(now - windowStartTime > windowSize) { 
    //time to shift the window
    windowStartTime += windowSize;
    currentWindowPidOutput = pidOutput;
  }
  
  Serial.print("Window Size: ");
  Serial.print(windowSize);
//
  Serial.print(" Window elapsed: ");
  Serial.print((now - windowStartTime) * 100);
//  
  Serial.print(" Div: ");
  Serial.println((now - windowStartTime) * 100 / windowSize);
  
  if(currentWindowPidOutput * windowSize > ((now - windowStartTime) * 100)) {
    if(!heaterOn){
      heaterOn = true;
      Serial.println("ON");
      setHeaterState(true);
       //digitalWrite(relay, HIGH);
    }
  }
  else if(heaterOn) {
    heaterOn = false;
    Serial.println("OFF");
    setHeaterState(false);
     //digitalWrite(relay, LOW);
  }
  
  // Every 400 cycles (about 8 seconds) refresh the heater state
  if(i % 400 == 0) {
    setHeaterState(heaterOn); 
  }
}

void setHeaterState(boolean on) {
  digitalWrite(relay, on ? HIGH:LOW);
  //long code = WATTS_CLEVER_DEVICE_ID + (on ? ON_CODES[1] : OFF_CODES[1]);
  //sendCode(code);
}

