/*
--------- Sensors List ----------
Ultrasonic Sensor * 6
Water Flow Sensor * 1
RTC * 1
TDS Sensor * 1
pH Sensor * 1
BH1750 Light Sensor * 1
DHT11 * 1

temp,tds,ph,humidity,water flow, light intensity, nutsol level, water level, nutA, nutB, ph up level, ph down level
------- Actuators List ----------
74HC595 SIPO Shift Register * 2
L298N * 4
Peristaltic Pump * 4
12V Diaphragm Water Pump * 1
High Pressure Diaphragm Water Pump * 1
DC Motor Fan * 1
775 DC Motor * 1
DC T8 LED Light * 2
-----------------------------------
*/

/*
Include required libraries to make the code work otherwise you will get errors
1. Wire.h (pre - installed)
2. dht - Sharing along with code
3. BH1750 by Christopher Laws - Install from library Manager
4. Rtc by Makuna - Install from library Manager

disease codes
H - No disease
T - Tipburn (Calcium) is detected
B - Brown Spots (Boron Toxicity)  is detected
Y - Yellowing & Wilting (Nitrogen) is detected
N - Gray White Spots is detected

Expected Input from Orange Pi in the form below else Arduino will mess up parsing the serial input data
<mode, health, ppm lower threshold (int), ppm upper threshold (int), pH lower threshold (float), pH upper threshold (float)>
<O, H, 400, 450, 5.5, 6.5>
*/

#include <Wire.h>
#include <dht.h>
#include <BH1750.h>
#include <ThreeWire.h>
#include <RtcDS1302.h>

char datestring[20];
unsigned long p_time_now, pump_time_now;

int previousDay;
int currentDay;
int start_day;
int percent;
//========================================
//For Serial Send
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char modeFromOrangePi[numChars] = { 0 };
char healthFromOrangePi[numChars] = { 0 };
int ppmlowerthresholdFromOrangePi = 0;
int ppmupperthresholdFromOrangePi = 0;
float pHlowerthresholdFromOrangePi = 0.0;
float pHupperthresholdFromOrangePi = 0.0;

bool newData = false;
//=================================

//For Saver Mode
long previousMillis = 0;
//Define pins for Ultrasonic Sensors (System Checking)
//U6 - pH Up
const int trigPin6 = 23;
const int echoPin6 = 25;
//U9 - pH Down
const int trigPin9 = 27;
const int echoPin9 = 29;
//U8 - Nutrient A
const int trigPin8 = 31;
const int echoPin8 = 33;
//U11 - Nutrient B
const int trigPin11 = 39;
const int echoPin11 = 41;
//U12  - Water Reservoir
const int trigPin12 = 43;
const int echoPin12 = 45;
//U10 Nut Sol Reservoir
const int trigPin10 = 47;
const int echoPin10 = 49;

//**********************************

//Define Water Level Sensor (System Checking)
#define WaterLevelSensorPin A0
//**********************************

//Define Water Flow Sensor (System Checking)
//Pin to be defined
#define WaterFlow A0
float TIME = 0;
float FREQUENCY = 0;
float WATER = 0;
float TOTAL = 0;
float LS = 0;
//**********************************

//Define TDS Sensor (Growth factor Sensor)
#define TdsSensorPin A2
#define VREF 5.0   //analog reference voltage(Volt) of the ADC
#define SCOUNT 30  // analog reference voltage(Volt) of the ADC

int analogBuffer[SCOUNT];  // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 25;  // current temperature for compensation
float humidity = 0;
float ppm = 0;
//**********************************

//Define pH Sensor (Growth factor Sensor)
#define pHSensorPin A1
int samples = 10;
float adc_resolution = 1024.0;
float pH = 0.0;
//**********************************

//Define DHT Sensor (temperature and humidity) (Growth factor Sensor)
#define dhtPin 16
dht DHT;
//**********************************

//Define BH1750 Sensor (Light intensity) (Growth factor Sensor)
BH1750 GY302;
uint16_t lux;
float light = 0.0;
//**********************************

//Define DS1302 RTC module (Growth factor Sensor)
ThreeWire myWire(18, 17, 19);  //IO,SCLK,CE
RtcDS1302<ThreeWire> Rtc(myWire);
//**********************************

//Define Actuators

//Solenoid Valves
//U24 Close for misting, Open for Refilling
#define solenoid24 36
//U35  Close for Refilling, Open for Misting
#define solenoid35 34
//U36
#define solenoid36 30

//LED Light
#define lights 10

//Motor Driver
//U28
#define in1_28 34  //P1
#define in2_28 36  //P1
#define in3_28 38  //P2
#define in4_28 40  //P2
#define enA_28 2   //P1
#define enB_28 3   //P2
//U27
#define in1_27 42  //F1 & F2
#define in2_27 44  //F1 & F2
#define in3_27 46  //RS755 Fan Motor
#define in4_27 48  //RS755 Fan Motor
#define enA_27 4   //F1 & F2
#define enB_27 5   //RS755 Fan Motor
//U30
#define in1_30 A4  //PP1
#define in2_30 A5  //PP1
#define in3_30 A6  //PP2
#define in4_30 A7  //PP2
#define enA_30 6   //PP1
#define enB_30 7   //PP2
//U31
#define in1_31 A8   //PP3
#define in2_31 A9   //PP3
#define in3_31 A10  //PP4
#define in4_31 A11  //PP4
#define enA_31 8    //PP3
#define enB_31 9    //PP4
//**********************************

int water_reservoir_Level;
int Nutrient_A_Notify_Level;
int Nutrient_B_Notify_Level;
float pH_up_Notify_Level;
float pH_down_Notify_Level;
float pH_up_Level;

//******* THRESHOLD VALUES **********

//delay takes value in ms ==== 1000ms = 1 sec
//NutSolReservoir Level
int NutSol_lower_threshold = 0;
int NutSol_upper_threshold = 73;

//TDS Thresholds
int ppm_lower_threshold = 400;
int ppm_upper_threshold = 450;
int NutrientA_delay_6a_first = 100;  //Delay in ms for first time 1.125 mL/L
int NutrientA_delay_6a_other = 10;   //Delay in ms for 0.05 ml/L
int NutrientB_delay_6a_first = 100;  ////Delay in ms for first time 1.125 mL/L
int NutrientB_delay_6a_other = 10;   //delay in ms for 0.05 mL
int five_c_one_delay = 10;           //delay in ms for 0.5 L

//pH Thresholds
float pH_upper_threshold = 6.5;
float pH_lower_threshold = 5.5;

int pH_down_delay_first = 1000;  //Delay in ms to add X mL of pH down solution
int pH_down_delay_other = 10;    //Delay in ms to add 0.05 mL of pH down solution...1000ms = 1sec

int pH_up_delay_first = 1000;  //Delay in ms to add X mL of pH down solution
int pH_up_delay_other = 10;    //Delay in ms to add 0.05 mL of pH up solution...1000ms = 1sec

int pH_up_delay = 1000;    //Delay in ms to add X mL of pH Up for Tip burn disease
int pH_down_delay = 1000;  //Delay in ms to add X mL of pH Down for Tip burn disease

char mode = 'O';  //O - Optimal Mode(default) S - Saver Mode
char health = 'H';
long interval = 1000 * 60 * 15;  //15 minute interval

//Light Intensity Thresholds
int lux_lower_threshold = 5100;
int lux_upper_threshold = 7600;
int light_pwm = 255;  // 0 - 255 : to increase light intensity

//Fan F1 & F2 Speed Control
int F_Speed = 150;  //0-255

int notify_threshold = 20;             //if any container less than this value, send to orange pi
int Nutrient_A_container_height = 73;  //Max distance / ultrasonic sensor max value
int Nutrient_B_container_height = 73;  //Max distance / ultrasonic sensor max value
int pH_down_container_height = 73;
int pH_up_container_height = 73;
int water_reservoir_container_height = 73;

int Nut_Sol_Level = 90;  //NutSol will fill upto this %

//humidity
int humidity_lower_threshold = 50;
int humidity_upper_threshold = 60;

//temperature
int temperature_threshold = 22;

//Brown disease add 1L water
long brown_spot_delay_add_one_L_water = 1000;
//**********************************

// -------Functions-----------
// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) {
    bTemp = bTab[(iFilterLen - 1) / 2];
  } else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


// -------Sensor Functions-----------
int UltraMesaureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration / 58;
  return distance;
}

int readWaterLevel() {
  // int lowerThreshold = 420;  int upperThreshold = 520;
  int val = analogRead(WaterLevelSensorPin);
  return val;
}

float GetTDS() {
  bool b = true;
  while (b) {
    static unsigned long analogSampleTimepoint = millis();
    if (millis() - analogSampleTimepoint > 40U) {  //every 40 milliseconds,read the analog value from the ADC
      analogSampleTimepoint = millis();
      analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);  //read the analog value and store into the buffer
      analogBufferIndex++;
      if (analogBufferIndex == SCOUNT) {
        analogBufferIndex = 0;
      }
    }

    static unsigned long printTimepoint = millis();
    if (millis() - printTimepoint > 800U) {  //every 800 milliseconds,read the analog value from the ADC
      printTimepoint = millis();
      for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
        analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

        // read the analog value more stable by the median filtering algorithm, and convert to voltage value
        averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

        //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
        float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
        //temperature compensation
        float compensationVoltage = averageVoltage / compensationCoefficient;

        //convert voltage value to tds value
        tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;
        b = false;
      }
    }
  }
  return tdsValue;
}

float GetpH() {
  int measurings = 0;
  for (int i = 0; i < samples; i++) {
    measurings += analogRead(pHSensorPin);
    delay(10);
  }
  float volt = 5 / adc_resolution * measurings / samples;
  float pH1 = 7 + ((2.5 - volt) / 0.18);
  return pH1;
}

int getTemperature() {
  int chk = DHT.read11(dhtPin);
  int temperature = DHT.temperature;
  return temperature;
}

int getHumidity() {
  int chk = DHT.read11(dhtPin);
  int humidity = DHT.humidity;
  return humidity;
}

int getLightIntensity() {
  lux = GY302.readLightLevel();
  return lux;
}

float getWaterFlow() {
  int X;
  int Y;
  X = pulseIn(WaterFlow, HIGH);
  Y = pulseIn(WaterFlow, LOW);
  TIME = X + Y;
  FREQUENCY = 1000000 / TIME;
  WATER = FREQUENCY / 7.5;
  LS = WATER / 60;

  if (FREQUENCY <= 0) {
    if (isinf(FREQUENCY)) {
      return 0;
    } else {
      TOTAL = TOTAL + LS;
      return WATER;
    }
  }
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

unsigned long printDateTime(const RtcDateTime& dt) {
  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
             dt.Hour(),
             dt.Minute(),
             dt.Second());
  Serial3.println(datestring);
}

// -------System Setup-----------
void setup() {
  //Start Serial
  Serial.begin(9600);
  Serial3.begin(9600);  //Connection to Orange Pi
  Wire.begin();
  //Initialise BH1750
  GY302.begin();
  //Setup Ultrasonic sensors
  //U6
  pinMode(trigPin6, OUTPUT);
  pinMode(echoPin6, INPUT);
  //U9
  pinMode(trigPin9, OUTPUT);
  pinMode(echoPin9, INPUT);
  //U8
  pinMode(trigPin8, OUTPUT);
  pinMode(echoPin8, INPUT);
  //U11
  pinMode(trigPin11, OUTPUT);
  pinMode(echoPin11, INPUT);
  //U12
  pinMode(trigPin12, OUTPUT);
  pinMode(echoPin12, INPUT);
  //U10
  pinMode(trigPin10, OUTPUT);
  pinMode(echoPin10, INPUT);

  //TDS
  pinMode(TdsSensorPin, INPUT);

  //Actuators
  //Light
  pinMode(lights, OUTPUT);
  //Solenoids
  pinMode(solenoid24, OUTPUT);
  pinMode(solenoid35, OUTPUT);
  pinMode(solenoid36, OUTPUT);

  //Motor Driver
  //U28
  pinMode(in1_28, OUTPUT);
  pinMode(in2_28, OUTPUT);
  pinMode(in3_28, OUTPUT);
  pinMode(enA_28, OUTPUT);
  pinMode(enB_28, OUTPUT);

  //U27
  pinMode(in1_27, OUTPUT);
  pinMode(in2_27, OUTPUT);
  pinMode(in3_27, OUTPUT);
  pinMode(enA_27, OUTPUT);
  pinMode(enB_27, OUTPUT);

  //U30
  pinMode(in1_30, OUTPUT);
  pinMode(in2_30, OUTPUT);
  pinMode(in3_30, OUTPUT);
  pinMode(enA_30, OUTPUT);
  pinMode(enB_30, OUTPUT);

  //U31
  pinMode(in1_31, OUTPUT);
  pinMode(in2_31, OUTPUT);
  pinMode(in3_31, OUTPUT);
  pinMode(enA_31, OUTPUT);
  pinMode(enB_31, OUTPUT);

  //Water Flow Sensor
  pinMode(WaterFlow, INPUT);

  //RTC DS1302
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  printDateTime(compiled);
  Serial3.println();

  if (!Rtc.IsDateTimeValid()) {
    // Common Causes:
    //    1) first time you ran and the device wasn't running yet
    //    2) the battery on the device is low or even missing

    Serial3.println("RTC lost confidence in the DateTime!");
    Rtc.SetDateTime(compiled);
  }

  if (Rtc.GetIsWriteProtected()) {
    Serial3.println("RTC was write protected, enabling writing now");
    Rtc.SetIsWriteProtected(false);
  }

  if (!Rtc.GetIsRunning()) {
    Serial3.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();
  if (now < compiled) {
    Serial3.println("RTC is older than compile time!  (Updating DateTime)");
    Rtc.SetDateTime(compiled);
  } else if (now > compiled) {
    Serial3.println("RTC is newer than compile time. (this is expected)");
  } else if (now == compiled) {
    Serial3.println("RTC is the same as compile time! (not expected but all is fine)");
  }

  previousDay = now.Day();
  start_day = now.Day();

  //***************************************************
  //SYSTEM SETUP
  //COPY FROM BELOW ONCE DONE from line 437 - 564
  //Send to Orange Pi
  //2a Open Solenoid U24 and Close solenoid U35
  //If opposite is happening then write 'LOW' instead of 'HIGH'
  digitalWrite(solenoid24, HIGH);
  digitalWrite(solenoid35, LOW);
  //2b Turn ON the water pump(P1) to add 15 Liters(X L) of Water to NutSol Reservoir from Water Reservoir
  int NutSol = UltraMesaureDistance(trigPin10, echoPin10);
  int NutSolLevel = map(NutSol, NutSol_lower_threshold, NutSol_upper_threshold, 100, 0);
  while (NutSolLevel < Nut_Sol_Level) {
    // Turn on water pump P1
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28
    digitalWrite(in1_28, HIGH);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, HIGH);  //Dont change this. If set to LOW pump will not work

    NutSol = UltraMesaureDistance(trigPin10, echoPin10);
    NutSolLevel = map(NutSol, NutSol_lower_threshold, NutSol_upper_threshold, 100, 0);
  }
  if (NutSolLevel > Nut_Sol_Level) {
    //Turn Pump P1 off.
    digitalWrite(in1_28, LOW);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, LOW);
  }

  // 3. Turn on pump P2 to detect water flow
  // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
  digitalWrite(in3_28, HIGH);
  digitalWrite(in4_28, LOW);
  digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
  delay(5000);
  if (getWaterFlow() <= 0) {
    // Send to OrangePi
    Serial.println("No Water Flow Detected");
    //Turn Pump P2 OFF
    digitalWrite(in3_28, LOW);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, LOW);
  } else {
    //Turn Pump P2 OFF
    digitalWrite(in3_28, LOW);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, LOW);
  }

  // 4 Measure PPM(TDS Sensor)
  ppm = GetTDS();
  // 5
  while (ppm < ppm_lower_threshold) {
    //Turn on Peristaltic Pump PP3
    //If pump PP3 is moving in opposite direction than desired then interchange HIGH and LOW in in1_31 and in2_31
    digitalWrite(in1_31, HIGH);
    digitalWrite(in2_31, LOW);
    digitalWrite(enA_31, HIGH);  //Dont change this. If set to LOW pump will not work

    delay(NutrientA_delay_6a_first);  //Delay to add 1.125 mL(first time)

    // Turn PP3 Off
    digitalWrite(in1_31, LOW);
    digitalWrite(in2_31, LOW);
    digitalWrite(enA_31, LOW);

    //Turn on Peristaltic Pump PP4
    // If pump PP4 is moving in opposite direction than desired then interchange HIGH and LOW in in3_31 and in4_31
    digitalWrite(in3_31, HIGH);
    digitalWrite(in4_31, LOW);
    digitalWrite(enB_31, HIGH);  //Dont change this. If set to LOW pump will not work

    delay(NutrientB_delay_6a_first);  //Delay to add 1.125 mL Nutrient B to NutSol

    // Turn PP4 Off
    digitalWrite(in3_31, LOW);
    digitalWrite(in4_31, LOW);
    digitalWrite(enB_31, LOW);

    //Turn On Propellor (RS755 Fan)
    //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
    digitalWrite(in3_27, HIGH);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, HIGH);

    delay(5000);  //5 seconds

    //Turn Off Propellor (RS755 Fan)
    digitalWrite(in3_27, LOW);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, LOW);

    // 3. Turn on pump P2 to detect water flow
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
    digitalWrite(in3_28, HIGH);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
    delay(5000);
    if (getWaterFlow() <= 0) {
      // Send to OrangePi
      Serial.println("No Water Flow Detected");
      //Turn Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    } else {
      //Turn Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    }
    // 4
    ppm = GetTDS();
  }

  while (ppm > ppm_upper_threshold) {
    //Turn on Pump P1
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28n
    digitalWrite(in1_28, HIGH);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, HIGH);

    delay(five_c_one_delay);  //Delay to add o.5L of Water to NutSol

    // Turn P1 OFF
    digitalWrite(in1_28, LOW);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, LOW);

    // Turn On Propellor (RS755 Fan)
    //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
    digitalWrite(in3_27, HIGH);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, HIGH);  //Don't change this

    delay(5000);  //5 seconds

    // Turn Off Propellor (RS755 Fan)
    digitalWrite(in3_27, LOW);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, LOW);

    // 3. Turn on pump P2 to detect water flow
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
    digitalWrite(in3_28, HIGH);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
    delay(5000);
    if (getWaterFlow() <= 0) {
      // Send to OrangePi
      Serial.println("No Water Flow Detected");
      //Turn Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    } else {
      //Turn Pump P2 OFF
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);  //Dont change this. If set to LOW pump will not work
    }

    // 4
    ppm = GetTDS();
  }
  ppm = GetTDS();

  pH = GetpH();
  while (pH > pH_upper_threshold) {
    //Turn PP2 On
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
    digitalWrite(in3_30, HIGH);
    digitalWrite(in4_30, LOW);
    digitalWrite(enB_30, HIGH);  //Dont change this. If set to LOW pump will not work

    delay(pH_down_delay_first);  //Delay in ms to add X mL of pH down

    digitalWrite(in3_30, LOW);
    digitalWrite(in4_30, LOW);
    digitalWrite(enB_30, LOW);

    // Turn On Propellor (RS755 Fan)
    //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
    digitalWrite(in3_27, HIGH);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, HIGH);

    delay(5000);  //5 seconds

    //Turn Off Propellor (RS755 Fan)
    digitalWrite(in3_27, LOW);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, LOW);

    pH = GetpH();
  }

  while (pH < pH_lower_threshold) {
    //Turn PP1 On
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
    digitalWrite(in1_30, HIGH);
    digitalWrite(in2_30, LOW);
    digitalWrite(enA_30, HIGH);  //Dont change this. If set to LOW pump will not work

    delay(pH_up_delay_first);  //Delay in ms to add X mL of pH up

    digitalWrite(in1_30, LOW);
    digitalWrite(in2_30, LOW);
    digitalWrite(enA_30, LOW);

    // Turn On Propellor (RS755 Fan)
    //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
    digitalWrite(in3_27, HIGH);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, HIGH);

    delay(5000);  //5 seconds

    // Turn Off Propellor (RS755 Fan)
    digitalWrite(in3_27, LOW);
    digitalWrite(in4_27, LOW);
    digitalWrite(enB_27, LOW);

    pH = GetpH();
  }
  pH = GetpH();

  // 11 - Operating Mode
  if (mode == 'S') {
    // Pump P2 On for 15 mins and OFF for 15 mins
    long t = 1;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      t++;
    }
    if (t % 2 == 0) {
      //Pump P2 ON
      digitalWrite(in3_28, HIGH);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, HIGH);
    } else {
      //Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    }

    //Turn On light at 4AM and OFF at 10PM
    RtcDateTime now = Rtc.GetDateTime();
    if (now.Hour() > 4 && now.Hour() < 22) {
      currentDay = now.Day();
      if (currentDay - previousDay == 1) {
        previousDay = currentDay;
        percent = (1 * light_pwm) / 100;
        light_pwm = light_pwm - percent;
        if (light_pwm <= 0) {
          light_pwm = 255;
        }
        if (light_pwm >= 255) {
          light_pwm = 255;
        }
      }
      //Turn Lights ON
      analogWrite(lights, light_pwm);

    } else {
      digitalWrite(lights, LOW);
    }
  } else if (mode == 'O') {
    //Pump P2 ON
    digitalWrite(in3_28, HIGH);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, HIGH);

    RtcDateTime now = Rtc.GetDateTime();
    currentDay = now.Day();
    if (currentDay - previousDay == 1) {
      previousDay = currentDay;
      percent = (1 * light_pwm) / 100;
      light_pwm = light_pwm - percent;
      if (light_pwm <= 0) {
        light_pwm = 255;
      }
      if (light_pwm >= 255) {
        light_pwm = 255;
      }
    }
    //Turn Lights ON
    analogWrite(lights, light_pwm);
  }

  //14.
  humidity = getHumidity();
  while (humidity > humidity_upper_threshold) {
    //Turn On fan F1 & F2
    digitalWrite(in1_27, HIGH);
    digitalWrite(in2_27, LOW);
    digitalWrite(enA_27, HIGH);

    humidity = getHumidity();
  }

  while (humidity < humidity_lower_threshold) {
    //Close Solenoid 24 and Open solenoid 35
    //Change LOW to HIGH and vice-versa if opposite is happening in solenoids
    digitalWrite(solenoid24, LOW);
    digitalWrite(solenoid35, HIGH);

    //Turn ON misting pump P1
    digitalWrite(in1_28, HIGH);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, HIGH);

    delay(5000);

    //Turn OFF misting pump P1
    digitalWrite(in1_28, LOW);
    digitalWrite(in2_28, LOW);
    digitalWrite(enA_28, LOW);

    humidity = getHumidity();
  }

  humidity = getHumidity();
  if ((humidity > humidity_lower_threshold) && (humidity < humidity_upper_threshold)) {
    // Open solenoid 24 and close solenoid 35
    digitalWrite(solenoid24, HIGH);
    digitalWrite(solenoid35, LOW);

    //Turn On fan F1 & F2
    digitalWrite(in1_27, HIGH);
    digitalWrite(in2_27, LOW);
    analogWrite(enA_27, F_Speed);  //Reduce F speed
  }

  //15
  temperature = getTemperature();
  while (temperature > temperature_threshold) {
    //Increase Speed of fans F1 and F2
    //F1 & F2
    digitalWrite(in1_27, HIGH);
    digitalWrite(in2_27, LOW);
    digitalWrite(enA_27, HIGH);

    temperature = getTemperature();
  }
  if (temperature < temperature_threshold) {
    //Turn On fan F1 & F2 at lower speed
    digitalWrite(in1_27, HIGH);
    digitalWrite(in2_27, LOW);
    analogWrite(enA_27, F_Speed);  //Reduce F speed
  }

  //Send Sensor Values to Orange Pin
  Serial.print("Temp ");
  Serial.print(temperature);
  Serial.print(",");
  Serial.print("Humidity ");
  Serial.print(humidity);
  Serial.print(",");
  Serial.print("PPM ");
  Serial.print(ppm);
  Serial.print(",");
  Serial.print("pH ");
  Serial.print(pH);
  Serial.print(",");
  Serial.print("Flow ");
  Serial.print(getWaterFlow());
  Serial.print(",");
  Serial.print("lux ");
  Serial.print(getLightIntensity());
  Serial.print(",");
  Serial.print("NutSol Level ");
  Serial.print(NutSolLevel);
  Serial.print(",");
  Serial.print("Water Reservoir Level ");
  Serial.print(water_reservoir_Level);
  Serial.print(",");
  Serial.print("Nut A Level ");
  Serial.print(Nutrient_A_Notify_Level);
  Serial.print(",");
  Serial.print("Nut B Level ");
  Serial.print(Nutrient_B_Notify_Level);
  Serial.print(",");
  Serial.print("pH up Level ");
  Serial.print(pH_down_Notify_Level);
  Serial.print(",");
  Serial.print("pH down Level ");
  Serial.print(pH_down_Notify_Level);
  Serial.println();
  Serial.println("All parameters setup");
  //***************************************************
}

// -------Control Loop-----------
void loop() {

  //NOTIFICATIONS
  /*
	U6 - pH up Level
	U9 - pH down Level
	U8 - Nutrient A Level
	U11 - Nutrient B Level
	U12 - Water Reservoir Level
	U10 - Nut Sol Level
	*/
  Nutrient_A_Notify_Level = UltraMesaureDistance(trigPin8, echoPin8);
  Nutrient_A_Notify_Level = map(Nutrient_A_Notify_Level, 0, Nutrient_A_container_height, 100, 0);
  if (Nutrient_A_Notify_Level < notify_threshold) {
    //Send to Orange Pi
    Serial.println("Your Nutrient A Solution is Running Out! Kindly refill its container");
  }

  Nutrient_B_Notify_Level = UltraMesaureDistance(trigPin11, echoPin11);
  Nutrient_B_Notify_Level = map(Nutrient_B_Notify_Level, 0, Nutrient_B_container_height, 100, 0);
  if (Nutrient_B_Notify_Level < notify_threshold) {
    //Send to Orange Pi
    Serial.println("Your Nutrient B Solution is Running Out! Kindly refill its container");
  }

  pH_down_Notify_Level = UltraMesaureDistance(trigPin9, echoPin9);
  pH_down_Notify_Level = map(pH_down_Notify_Level, 0, pH_down_container_height, 100, 0);
  if (pH_down_Notify_Level < notify_threshold) {
    //Send to Orange Pi
    Serial.println("Your pH Down Formula is Running Out! Kindly refill its container.");
  }

  pH_up_Level = UltraMesaureDistance(trigPin6, echoPin6);
  pH_up_Level = map(pH_up_Level, 0, pH_up_container_height, 100, 0);
  if (pH_up_Notify_Level < notify_threshold) {
    //Send to Orange Pi
    Serial.println("Your pH Down Formula is Running Out! Kindly refill its container.");
  }

  water_reservoir_Level = UltraMesaureDistance(trigPin12, echoPin12);
  water_reservoir_Level = map(water_reservoir_Level, 0, water_reservoir_container_height, 100, 0);
  if (water_reservoir_Level < notify_threshold) {
    //Send to Orange Pi
    Serial.println("Your Water Reservoir is Running Out! Kindly refill its reservoir.");
  }

  if (getWaterFlow() <= 0) {
    //Send to Orange Pi
    Serial.println("The System has been clogged. Kindly check your System.");
  }


  // Program Logic
  RtcDateTime now = Rtc.GetDateTime();
  if (now.Hour() % 2 == 0) {
    //2a Open Solenoid U24 and Close solenoid U35
    //If opposite is happening then write 'LOW' instead of 'HIGH'
    digitalWrite(solenoid24, HIGH);
    digitalWrite(solenoid35, LOW);
    //2b Turn ON the water pump(P1) to add 15 Liters(X L) of Water to NutSol Reservoir from Water Reservoir
    int NutSol = UltraMesaureDistance(trigPin10, echoPin10);
    int NutSolLevel = map(NutSol, NutSol_lower_threshold, NutSol_upper_threshold, 100, 0);
    while (NutSolLevel < Nut_Sol_Level) {
      // Turn on water pump P1
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28
      digitalWrite(in1_28, HIGH);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, HIGH);  //Dont change this. If set to LOW pump will not work

      NutSol = UltraMesaureDistance(trigPin10, echoPin10);
      NutSolLevel = map(NutSol, NutSol_lower_threshold, NutSol_upper_threshold, 100, 0);
    }
    if (NutSolLevel > Nut_Sol_Level) {
      //Turn Pump P1 off.
      digitalWrite(in1_28, LOW);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, LOW);
    }

    // 3. Turn on pump P2 to detect water flow
    // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
    digitalWrite(in3_28, HIGH);
    digitalWrite(in4_28, LOW);
    digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
    delay(5000);
    if (getWaterFlow() <= 0) {
      // Send to OrangePi
      Serial.println("No Water Flow Detected");
      //Turn Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    } else {
      //Turn Pump P2 OFF
      digitalWrite(in3_28, LOW);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, LOW);
    }

    // 4 Measure PPM(TDS Sensor)
    ppm = GetTDS();
    // 5
    while (ppm < ppm_lower_threshold) {
      //Turn on Peristaltic Pump PP3
      //If pump PP3 is moving in opposite direction than desired then interchange HIGH and LOW in in1_31 and in2_31
      digitalWrite(in1_31, HIGH);
      digitalWrite(in2_31, LOW);
      digitalWrite(enA_31, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(NutrientA_delay_6a_other);  //Delay to add 0.05mL (other) Nutrient A to NutSol

      // Turn PP3 Off
      digitalWrite(in1_31, LOW);
      digitalWrite(in2_31, LOW);
      digitalWrite(enA_31, LOW);

      //Turn on Peristaltic Pump PP4
      // If pump PP4 is moving in opposite direction than desired then interchange HIGH and LOW in in3_31 and in4_31
      digitalWrite(in3_31, HIGH);
      digitalWrite(in4_31, LOW);
      digitalWrite(enB_31, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(NutrientB_delay_6a_other);  //Delay to add 0.05mL (other) Nutrient A to NutSol

      // Turn PP4 Off
      digitalWrite(in3_31, LOW);
      digitalWrite(in4_31, LOW);
      digitalWrite(enB_31, LOW);

      //Turn On Propellor (RS755 Fan)
      //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
      digitalWrite(in3_27, HIGH);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, HIGH);

      delay(5000);  //5 seconds

      //Turn Off Propellor (RS755 Fan)
      digitalWrite(in3_27, LOW);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, LOW);

      // 3. Turn on pump P2 to detect water flow
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
      digitalWrite(in3_28, HIGH);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
      delay(5000);
      if (getWaterFlow() <= 0) {
        // Send to OrangePi
        Serial.println("No Water Flow Detected");
        //Turn Pump P2 OFF
        digitalWrite(in3_28, LOW);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, LOW);
      } else {
        //Turn Pump P2 OFF
        digitalWrite(in3_28, LOW);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, LOW);
      }
      // 4
      ppm = GetTDS();
    }

    while (ppm > ppm_upper_threshold) {
      //Turn on Pump P1
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28n
      digitalWrite(in1_28, HIGH);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, HIGH);

      delay(five_c_one_delay);  //Delay to add o.5L of Water to NutSol

      // Turn P1 OFF
      digitalWrite(in1_28, LOW);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, LOW);

      // Turn On Propellor (RS755 Fan)
      //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
      digitalWrite(in3_27, HIGH);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, HIGH);  //Don't change this

      delay(5000);  //5 seconds

      // Turn Off Propellor (RS755 Fan)
      digitalWrite(in3_27, LOW);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, LOW);

      // 3. Turn on pump P2 to detect water flow
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
      digitalWrite(in3_28, HIGH);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, HIGH);  //Dont change this. If set to LOW pump will not work
      delay(5000);
      if (getWaterFlow() <= 0) {
        // Send to OrangePi
        Serial.println("No Water Flow Detected");
        //Turn Pump P2 OFF
        digitalWrite(in3_28, LOW);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, LOW);
      } else {
        //Turn Pump P2 OFF
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_28 and in4_28
        digitalWrite(in3_28, LOW);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, LOW);  //Dont change this. If set to LOW pump will not work
      }

      // 4
      ppm = GetTDS();
    }
    ppm = GetTDS();

    pH = GetpH();
    while (pH > pH_upper_threshold) {
      //Turn PP2 On
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
      digitalWrite(in3_30, HIGH);
      digitalWrite(in4_30, LOW);
      digitalWrite(enB_30, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(pH_down_delay_other);  //Delay in ms to add X mL of pH down / o.o5 for other

      digitalWrite(in3_30, LOW);
      digitalWrite(in4_30, LOW);
      digitalWrite(enB_30, LOW);

      // Turn On Propellor (RS755 Fan)
      //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
      digitalWrite(in3_27, HIGH);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, HIGH);

      delay(5000);  //5 seconds

      //Turn Off Propellor (RS755 Fan)
      digitalWrite(in3_27, LOW);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, LOW);

      pH = GetpH();
    }

    while (pH < pH_lower_threshold) {
      //Turn PP1 On
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
      digitalWrite(in1_30, HIGH);
      digitalWrite(in2_30, LOW);
      digitalWrite(enA_30, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(pH_up_delay_other);  //Delay in ms to add X mL of pH up / 0.05mL for other

      digitalWrite(in1_30, LOW);
      digitalWrite(in2_30, LOW);
      digitalWrite(enA_30, LOW);

      // Turn On Propellor (RS755 Fan)
      //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
      digitalWrite(in3_27, HIGH);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, HIGH);

      delay(5000);  //5 seconds

      // Turn Off Propellor (RS755 Fan)
      digitalWrite(in3_27, LOW);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, LOW);

      pH = GetpH();
    }
    pH = GetpH();

    // 11 - Operating Mode
    if (mode == 'S') {
      // Pump P2 On for 15 mins and OFF for 15 mins
      long t = 1;
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis > interval) {
        t++;
      }
      if (t % 2 == 0) {
        //Pump P2 ON
        digitalWrite(in3_28, HIGH);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, HIGH);
      } else {
        //Pump P2 OFF
        digitalWrite(in3_28, LOW);
        digitalWrite(in4_28, LOW);
        digitalWrite(enB_28, LOW);
      }

      //Turn On light at 4AM and OFF at 10PM
      RtcDateTime now = Rtc.GetDateTime();
      if (now.Hour() > 4 && now.Hour() < 22) {
        currentDay = now.Day();
        if (currentDay - previousDay == 1) {
          previousDay = currentDay;
          percent = (1 * light_pwm) / 100;
          light_pwm = light_pwm - percent;
          if (light_pwm <= 0) {
            light_pwm = 255;
          }
          if (light_pwm >= 255) {
            light_pwm = 255;
          }
        }
        //Turn Lights ON
        analogWrite(lights, light_pwm);

      } else {
        digitalWrite(lights, LOW);
      }
    } else if (mode == 'O') {
      //Pump P2 ON
      digitalWrite(in3_28, HIGH);
      digitalWrite(in4_28, LOW);
      digitalWrite(enB_28, HIGH);

      RtcDateTime now = Rtc.GetDateTime();
      currentDay = now.Day();
      if (currentDay - previousDay == 1) {
        previousDay = currentDay;
        percent = (1 * light_pwm) / 100;
        light_pwm = light_pwm - percent;
        if (light_pwm <= 0) {
          light_pwm = 255;
        }
        if (light_pwm >= 255) {
          light_pwm = 255;
        }
      }
      //Turn Lights ON
      analogWrite(lights, light_pwm);
    }

    //14.
    humidity = getHumidity();
    while (humidity > humidity_upper_threshold) {
      //Turn On fan F1 & F2
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      digitalWrite(enA_27, HIGH);

      humidity = getHumidity();
    }

    while (humidity < humidity_lower_threshold) {
      //Close Solenoid 24 and Open solenoid 35
      //Change LOW to HIGH and vice-versa if opposite is happening in solenoids
      digitalWrite(solenoid24, LOW);
      digitalWrite(solenoid35, HIGH);

      //Turn ON misting pump P1
      digitalWrite(in1_28, HIGH);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, HIGH);

      delay(5000);

      //Turn OFF misting pump P1
      digitalWrite(in1_28, LOW);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, LOW);

      humidity = getHumidity();
    }

    humidity = getHumidity();
    if ((humidity > humidity_lower_threshold) && (humidity < humidity_upper_threshold)) {
      // Open solenoid 24 and close solenoid 35
      digitalWrite(solenoid24, HIGH);
      digitalWrite(solenoid35, LOW);

      //Turn On fan F1 & F2
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      analogWrite(enA_27, F_Speed);  //Reduce F speed
    }

    //15
    temperature = getTemperature();
    while (temperature > temperature_threshold) {
      //Increase Speed of fans F1 and F2
      //F1 & F2
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      digitalWrite(enA_27, HIGH);

      temperature = getTemperature();
    }
    if (temperature < temperature_threshold) {
      //Turn On fan F1 & F2 at lower speed
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      analogWrite(enA_27, F_Speed);  //Reduce F speed
    }

    //Send Sensor Values to Orange Pin
    Serial.print("Temp ");
    Serial.print(temperature);
    Serial.print(",");
    Serial.print("Humidity ");
    Serial.print(humidity);
    Serial.print(",");
    Serial.print("PPM ");
    Serial.print(ppm);
    Serial.print(",");
    Serial.print("pH ");
    Serial.print(pH);
    Serial.print(",");
    Serial.print("Flow ");
    Serial.print(getWaterFlow());
    Serial.print(",");
    Serial.print("lux ");
    Serial.print(getLightIntensity());
    Serial.print(",");
    Serial.print("NutSol Level ");
    Serial.print(NutSolLevel);
    Serial.print(",");
    Serial.print("Water Reservoir Level ");
    Serial.print(water_reservoir_Level);
    Serial.print(",");
    Serial.print("Nut A Level ");
    Serial.print(Nutrient_A_Notify_Level);
    Serial.print(",");
    Serial.print("Nut B Level ");
    Serial.print(Nutrient_B_Notify_Level);
    Serial.print(",");
    Serial.print("pH up Level ");
    Serial.print(pH_down_Notify_Level);
    Serial.print(",");
    Serial.print("pH down Level ");
    Serial.print(pH_down_Notify_Level);
    Serial.println();

    //Read from Orange Pi
    //Expected Input
    //<mode, health, ppm lower threshold (int), ppm upper threshold (int), pH lower threshold (float), pH upper threshold (float)>
    //<O, H, 400, 450, 5.5, 6.5>
    recvWithStartEndMarkers();
    if (newData == true) {
      // this temporary copy is necessary to protect the original data
      // because strtok() used in parseData() replaces the commas with \0
      strcpy(tempChars, receivedChars);
      parseData();
      processParsedData();
      newData = false;
    } else {
      //default values
      mode = 'O';
      health = 'H';

      RtcDateTime now = Rtc.GetDateTime();
      if (start_day - now.Day() >= 8) {
        ppm_lower_threshold = 600;
        ppm_upper_threshold = 850;
      } else {
        ppm_lower_threshold = 400;
        ppm_upper_threshold = 450;
      }

      pH_lower_threshold = 5.5;
      pH_upper_threshold = 6.5;
    }
  }  //2 hrs bracket

  //18
  if (now.Hour() % 4 == 0) {
    //a heck Water FLow
    if (getWaterFlow() <= 0) {
      //Send to Orange Pi
      Serial.println("Water Flow Not Detected");
    }
    //b Check PPM
    ppm = GetTDS();
    if (!(ppm > ppm_lower_threshold)) {
      if (!(ppm < ppm_upper_threshold)) {
        //Send to Orange Pi
        Serial.println("PPM not in desired range");
      }
    }
    //c Check pH
    pH = GetpH();
    if (!(pH > pH_lower_threshold)) {
      if (!(pH < pH_upper_threshold)) {
        //Send to Orange Pi
        Serial.println("pH not in desired range");
      }
    }
    //d Check light
    lux = getLightIntensity();

    //e Check Humidity
    humidity = getHumidity();
    if (!(humidity > humidity_lower_threshold)) {
      if (!(humidity < humidity_upper_threshold)) {
        //Send to Orange Pi
        Serial.println("Humidity not in desired range");
      }
    }

    //f Check temperature
    temperature = getTemperature();
    if (temperature > temperature_threshold) {
      //Send to Orange Pi
      Serial.println("Temperature not in desired range");
    }
  }

  switch (health) {
    case 'T':
      // Tipburn (Calcium) is detected
      // Lower the lights by 10%
      percent = (10 * light_pwm) / 100;
      light_pwm = light_pwm - percent;
      if (light_pwm <= 0) {
        light_pwm = 255;
      }
      if (light_pwm >= 255) {
        light_pwm = 255;
      }
      //Turn Lights ON
      analogWrite(lights, light_pwm);

      //Increase fans speed
      float t = getTemperature();
      while (t < 22) {
        digitalWrite(in1_27, HIGH);
        digitalWrite(in2_27, LOW);
        digitalWrite(enA_27, HIGH);  //Increase F speed
        t = getTemperature();
      }
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      analogWrite(enA_27, F_Speed);

      float pHT = GetpH();
      while (pHT < 5.8) {
        //pH Up
        //Turn PP1 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in1_30, HIGH);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_up_delay);  //Delay in ms to add X mL of pH up

        digitalWrite(in1_30, LOW);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHT = GetpH();
      }
      while (pHT > 6.3) {
        //pH down
        //Turn PP2 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in3_30, HIGH);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_down_delay);  //Delay in ms to add X mL of pH down

        digitalWrite(in3_30, LOW);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHT = GetpH();
      }
      break;
    case 'B':
      // Brown Spots (Boron Toxicity)  is detected

      // Turn on water pump P1 to add 1L water
      // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28
      digitalWrite(in1_28, HIGH);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(brown_spot_delay_add_one_L_water);  //delay in ms to add 1L of water in NutSol Reservoir 1000 ms = 1 sec

      // Turn off pump P1
      digitalWrite(in1_28, LOW);
      digitalWrite(in2_28, LOW);
      digitalWrite(enA_28, LOW);

      //Adjust PPM
      RtcDateTime now = Rtc.GetDateTime();
      if (start_day - now.Day() >= 8) {
        ppm_lower_threshold = 600;
        ppm_upper_threshold = 850;
      } else {
        ppm_lower_threshold = 400;
        ppm_upper_threshold = 450;
      }

      int ppmB = GetTDS();
      if (ppmB < ppm_lower_threshold) {
        //Add 0.05ml of Nutrient A
        //Turn on Peristaltic Pump PP3
        //If pump PP3 is moving in opposite direction than desired then interchange HIGH and LOW in in1_31 and in2_31
        digitalWrite(in1_31, HIGH);
        digitalWrite(in2_31, LOW);
        digitalWrite(enA_31, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(NutrientA_delay_6a_other);  //Delay to add 0.05mL (other) Nutrient A to NutSol

        // Turn PP3 Off
        digitalWrite(in1_31, LOW);
        digitalWrite(in2_31, LOW);
        digitalWrite(enA_31, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);  //Don't change this

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);
      }
      if (ppmB > ppm_upper_threshold) {
        //Add 500 mL of water
        //Turn on Pump P1
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28n
        digitalWrite(in1_28, HIGH);
        digitalWrite(in2_28, LOW);
        digitalWrite(enA_28, HIGH);

        delay(five_c_one_delay);  //Delay to add o.5L of Water to NutSol

        // Turn P1 OFF
        digitalWrite(in1_28, LOW);
        digitalWrite(in2_28, LOW);
        digitalWrite(enA_28, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);  //Don't change this

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);
      }

      //Adjust pH
      float pHB = GetpH();
      while (pHB < 5.8) {
        //pH Up
        //Turn PP1 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in1_30, HIGH);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_up_delay);  //Delay in ms to add X mL of pH up

        digitalWrite(in1_30, LOW);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHB = GetpH();
      }
      while (pHB > 6.3) {
        //pH down
        //Turn PP2 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in3_30, HIGH);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_down_delay);  //Delay in ms to add X mL of pH down

        digitalWrite(in3_30, LOW);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHB = GetpH();
      }
      break;

    case 'Y':
      //Yellowing & Wilting (Nitrogen) is detected

      //Add 0.05 ml/L of Nutrient B
      //Turn on Peristaltic Pump PP4
      // If pump PP4 is moving in opposite direction than desired then interchange HIGH and LOW in in3_31 and in4_31
      digitalWrite(in3_31, HIGH);
      digitalWrite(in4_31, LOW);
      digitalWrite(enB_31, HIGH);  //Dont change this. If set to LOW pump will not work

      delay(NutrientB_delay_6a_other);  //Delay to add 1.125 mL Nutrient B to NutSol

      // Turn PP4 Off
      digitalWrite(in3_31, LOW);
      digitalWrite(in4_31, LOW);
      digitalWrite(enB_31, LOW);

      //Turn On Propellor (RS755 Fan)
      //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
      digitalWrite(in3_27, HIGH);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, HIGH);

      delay(5000);  //5 seconds

      //Turn Off Propellor (RS755 Fan)
      digitalWrite(in3_27, LOW);
      digitalWrite(in4_27, LOW);
      digitalWrite(enB_27, LOW);


      //Adjust PPM
      //RtcDateTime now = Rtc.GetDateTime();
      if (start_day - now.Day() >= 8) {
        ppm_lower_threshold = 600;
        ppm_upper_threshold = 850;
      } else {
        ppm_lower_threshold = 400;
        ppm_upper_threshold = 450;
      }

      int ppmY = GetTDS();
      if (ppmY < ppm_lower_threshold) {
        //Add 0.05ml of Nutrient A
        //Turn on Peristaltic Pump PP3
        //If pump PP3 is moving in opposite direction than desired then interchange HIGH and LOW in in1_31 and in2_31
        digitalWrite(in1_31, HIGH);
        digitalWrite(in2_31, LOW);
        digitalWrite(enA_31, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(NutrientA_delay_6a_other);  //Delay to add 0.05mL (other) Nutrient A to NutSol

        // Turn PP3 Off
        digitalWrite(in1_31, LOW);
        digitalWrite(in2_31, LOW);
        digitalWrite(enA_31, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);  //Don't change this

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);
      }
      if (ppmY > ppm_upper_threshold) {
        //Add 500 mL of water
        //Turn on Pump P1
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in1_28 and in2_28n
        digitalWrite(in1_28, HIGH);
        digitalWrite(in2_28, LOW);
        digitalWrite(enA_28, HIGH);

        delay(five_c_one_delay);  //Delay to add o.5L of Water to NutSol

        // Turn P1 OFF
        digitalWrite(in1_28, LOW);
        digitalWrite(in2_28, LOW);
        digitalWrite(enA_28, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);  //Don't change this

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);
      }

      //Adjust pH
      float pHY = GetpH();
      while (pHY < 5.8) {
        //pH Up
        //Turn PP1 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in1_30, HIGH);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_up_delay);  //Delay in ms to add X mL of pH up

        digitalWrite(in1_30, LOW);
        digitalWrite(in2_30, LOW);
        digitalWrite(enA_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHY = GetpH();
      }
      while (pHY > 6.3) {
        //pH down
        //Turn PP2 On
        // If pump is moving in opposite direction than desired then interchange HIGH and LOW in in3_30 and in4_30
        digitalWrite(in3_30, HIGH);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, HIGH);  //Dont change this. If set to LOW pump will not work

        delay(pH_down_delay);  //Delay in ms to add X mL of pH down

        digitalWrite(in3_30, LOW);
        digitalWrite(in4_30, LOW);
        digitalWrite(enB_30, LOW);

        // Turn On Propellor (RS755 Fan)
        //Put LOW in place of HIGH if you want to change direction in in3_27 and in4_27
        digitalWrite(in3_27, HIGH);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, HIGH);

        delay(5000);  //5 seconds

        // Turn Off Propellor (RS755 Fan)
        digitalWrite(in3_27, LOW);
        digitalWrite(in4_27, LOW);
        digitalWrite(enB_27, LOW);

        pHY = GetpH();
      }
      break;

    case 'N':
      //Gray White Spots is detected
      //Increase Fans Speed
      //Increase fans speed
      int tN = getTemperature();
      while (tN < 22) {
        digitalWrite(in1_27, HIGH);
        digitalWrite(in2_27, LOW);
        digitalWrite(enA_27, HIGH);  //Increase F speed
        tN = getTemperature();
      }
      digitalWrite(in1_27, HIGH);
      digitalWrite(in2_27, LOW);
      analogWrite(enA_27, F_Speed);
      break;

    case 'H':
      break;
    default:
      break;
  }

  //RtcDateTime now = Rtc.GetDateTime();
  if (start_day - now.Day() == 28) {
    Serial.println("Hey Grower! Time to Harvest");
  }
}

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

//============

//<mode, health, ppm lower threshold (int), ppm upper threshold (int), pH lower threshold (float), pH upper threshold (float)>
//<O, H, 400, 450, 5.5, 6.5>
void parseData() {  // split the data into its parts

  char* strtokIndx;  // this is used by strtok() as an index

  //mode
  strtokIndx = strtok(tempChars, ",");   // get the first part - the string
  strcpy(modeFromOrangePi, strtokIndx);  // copy it to modeFromOrangePi

  //health
  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  strcpy(healthFromOrangePi, strtokIndx);  // copy it to healthFromOrangePi

  //ppm lower threshold
  strtokIndx = strtok(NULL, ",");                    // this continues where the previous call left off
  ppmlowerthresholdFromOrangePi = atoi(strtokIndx);  // convert this part to an integer

  //ppm upper threshold
  strtokIndx = strtok(NULL, ",");                    // this continues where the previous call left off
  ppmupperthresholdFromOrangePi = atoi(strtokIndx);  // convert this part to an integer

  //pH lower threshold
  strtokIndx = strtok(NULL, ",");
  pHlowerthresholdFromOrangePi = atof(strtokIndx);  // convert this part to a float

  //pH upper threshold
  strtokIndx = strtok(NULL, ",");
  pHupperthresholdFromOrangePi = atof(strtokIndx);  // convert this part to a float
}

//============

void processParsedData() {
  mode = modeFromOrangePi;
  health = healthFromOrangePi;
  ppm_lower_threshold = ppmlowerthresholdFromOrangePi;
  ppm_upper_threshold = ppmupperthresholdFromOrangePi;
  pH_lower_threshold = pHlowerthresholdFromOrangePi;
  pH_upper_threshold = pHupperthresholdFromOrangePi;
}