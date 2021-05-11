#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <LiquidCrystal_I2C.h>
#include <ezBuzzer.h>

Adafruit_ADS1115 ads(0x48);
LiquidCrystal_I2C lcd2(0x26, 16, 2);
//ezBuzzer buzz(buzzer);

int i;

int SensorSwitch = 2;
int buzzer = 12;

int16_t adc0;
int16_t adcROOM_max;
int16_t adcROOM_data1;
int16_t adcROOM_data2;

float roomReadings;
float sum = 0;

int16_t currentRead;
int16_t previousRead = 0;

float Vin = 4.92;

float Voltage = 0.0;
int thermistor_25 = 10000;
float refCurrent = 0.0001;
float resistance = 0;

float MASKpoll = 0.00;
float MASK = 0.00;
float ROOM = 0.00;
float Tempe = 0.00;

int BPS = 0; //breaths per cyclePoll
int nBPS = 0;

int prevBPS = 0;

int BPM = 0; //breaths per minute
int nBPM = 0;
int fBPM = 99;

int BPMcounter = 0;


unsigned long currentTime;
unsigned long timerStart;

unsigned long timerPollStart;

unsigned long buzzerSound1;

const unsigned long maskPoll_1 = 500;
const unsigned long maskPoll_2 = 200;
unsigned long maskPollTime = 0;

const unsigned long maskTimer = 60000;

const unsigned long buzzerSoundRpd = 1;
//const unsigned long buzzerSoundSlow = 50;
//const unsigned long buzzerSoundHlt = 80;

unsigned long previousTime_1 = 0;
unsigned long previousTime_2 = 0;
unsigned long previousTime_3 = 0;
unsigned long timerCheck = 0;

unsigned long buzzerTime = 0;

unsigned long cycleEnd = 0;

int16_t data = 0;
int16_t mx = 0;

void setup(void) {

  Serial.begin(9600);

  pinMode(SensorSwitch, OUTPUT);
  pinMode(buzzer, OUTPUT);
  lcd2.begin();

  //  lcd2.backlight();
  //  lcd2.clear();
  //  lcd2.setCursor(4,0);
  //  lcd2.print("Initiating...");

  ads.begin();
}

void loop(void) {
  BPM = 0;
  BPS = 0;

  //buzzerRpd();

  //bpmCheck();
  buzzerOn();
  currentTime = millis();
  if ( (currentTime - previousTime_1) >= maskPoll_1) { //500

    getMaskPollingTemp();
    outputPollSerial();
    //    outputPollLCDtest();
    outputPollLCD();

    previousRead = 0;

    previousTime_1 = currentTime;

    switchToRoom();
    readRoomTemp();
    //    outputPollLCDtest();
    outputPollLCD();

    //  delay(5000);
    switchToMask();
    readMaskTemp();

    //bpmCheck();


    //    ROOM = 0;
    sum = 0;
    //    roomReadings = 0;

  }
  //bpmCheck();
  delay(1);
  buzzerOff();
  //buzzerRpd3();
}



void getADC() {
  adc0 = ads.readADC_SingleEnded(0); // Read ADC value from ADS1115
}

void getTempe() {
  Voltage = (currentRead * Vin) / 65535; // Replace 5.0 with whatever the actual Vcc of your Arduino is
  resistance = (Voltage / refCurrent); // Using Ohm's Law to calculate resistance of thermistor
  float ln = log(resistance / thermistor_25); // Log of the ratio of thermistor resistance and resistance at 25 deg. C

  // Using the Steinhart-Hart Thermistor Equation to calculate temp in Kelvin (K)
  float kelvin = 1 / (0.0033540170 + (0.00025617244 * ln) + (0.0000021400943 * ln * ln) + (-0.000000072405219 * ln * ln * ln));

  Tempe = kelvin - 273.15; // Converting Kelvin to Celcuis
}


void getRoomTemp() {
  //  adcROOM_max = ads.readADC_SingleEnded(0); // Read ADC value from ADS1115
  Voltage = (adcROOM_data1 * Vin) / 65535; // Replace 5.0 with whatever the actual Vcc of your Arduino is
  resistance = (Voltage / refCurrent); // Using Ohm's Law to calculate resistance of thermistor
  float ln = log(resistance / thermistor_25); // Log of the ratio of thermistor resistance and resistance at 25 deg. C

  // Using the Steinhart-Hart Thermistor Equation to calculate temp in Kelvin (K)
  float kelvin = 1 / (0.0033540170 + (0.00025617244 * ln) + (0.0000021400943 * ln * ln) + (-0.000000072405219 * ln * ln * ln));

  ROOM = kelvin - 273.15; // Converting Kelvin to Celcuis
}

void readRoomTemp() {
  for (i = 0; i < 15; i++)
  {
    adcROOM_data1 = ads.readADC_SingleEnded(0);

    getRoomTemp();

    Serial.print("RoomTemp ");
    Serial.println(ROOM);

    sum = sum + ROOM;

  }

  Serial.print("SUM ");
  Serial.println(sum);
  roomReadings = sum / 15;
  Serial.print("Room ");
  Serial.println(roomReadings);
}




void getMaskPollingTemp() {
  adc0 = ads.readADC_SingleEnded(0); // Read ADC value from ADS1115
  Voltage = (adc0 * Vin) / 65535; // Replace 5.0 with whatever the actual Vcc of your Arduino is
  resistance = (Voltage / refCurrent); // Using Ohm's Law to calculate resistance of thermistor
  float ln = log(resistance / thermistor_25); // Log of the ratio of thermistor resistance and resistance at 25 deg. C

  // Using the Steinhart-Hart Thermistor Equation to calculate temp in Kelvin (K)
  float kelvin = 1 / (0.0033540170 + (0.00025617244 * ln) + (0.0000021400943 * ln * ln) + (-0.000000072405219 * ln * ln * ln));

  MASKpoll = kelvin - 273.15; // Converting Kelvin to Celcuis
}

void getMaskTemp() {
  //currentRead = ads.readADC_SingleEnded(0); // Read ADC value from ADS1115
  Voltage = (currentRead * Vin) / 65535; // Replace 5.0 with whatever the actual Vcc of your Arduino is
  resistance = (Voltage / refCurrent); // Using Ohm's Law to calculate resistance of thermistor
  float ln = log(resistance / thermistor_25); // Log of the ratio of thermistor resistance and resistance at 25 deg. C

  // Using the Steinhart-Hart Thermistor Equation to calculate temp in Kelvin (K)
  float kelvin = 1 / (0.0033540170 + (0.00025617244 * ln) + (0.0000021400943 * ln * ln) + (-0.000000072405219 * ln * ln * ln));

  MASK = kelvin - 273.15; // Converting Kelvin to Celcuis
}

void readMaskTemp() {

  Serial.println("Reading Mask Temp");

  currentRead = ads.readADC_SingleEnded(0);

  getMaskTemp();
  outputMaskSerial();
  //outputMaskLCDtest();
  outputMaskLCD();

  if (((MASKpoll - ROOM) > 0.2) && ((MASK - MASKpoll) > 0.1)) {
    timerStart = millis();
    Serial.print("Start: ");
    Serial.println(timerStart);

    do {
      Serial.print("Previous Time_2: ");
      Serial.println(previousTime_2);
      timerPollStart = millis();

      if ( (timerPollStart - previousTime_2) >= maskPoll_2) {
        maskPollTime = timerPollStart - previousTime_2;

        Serial.print("maskPolling: ");
        Serial.println(timerPollStart);

        Serial.print("Actual Poll time: ");
        Serial.println(maskPollTime);

        mx = ads.readADC_SingleEnded(0);
        currentRead = mx;
        Serial.print("currentRead: ");
        Serial.println(mx);

        while ( (currentRead < previousRead) ) { // <
          Serial.print("previousRead at start of while loop: ");
          Serial.println(previousRead);
          nBPS = BPS + 1;
          BPS = nBPS;

          Serial.println("setting previousRead to currentRead");
          previousRead = currentRead;

          Serial.println("Reading again currentRead inside while loop");
          currentRead = ads.readADC_SingleEnded(0);
          Serial.print("Breath ADC: ");
          Serial.println(currentRead);
          getMaskTemp();
          outputMaskSerial();
          outputMaskLCD();
          //outputMaskLCDtest();

          //previousRead = currentRead;
          Serial.print("previous after Breath ADC: ");
          Serial.println(previousRead);

        }

        previousRead = currentRead;
        Serial.print("previousRead after testing while loop: ");
        Serial.println(previousRead);

        Serial.print("fBPS ");
        Serial.println(BPS);

        previousTime_2 = timerPollStart;
        previousTime_3 = timerStart;
      }
      //Serial.print("maskPollingEnd: ");
      //Serial.println(millis());

      if (((BPS - prevBPS) >= 3) ) {//&& ((BPS - prevBPS) <5) ) { //&& ((currentRead - previousRead) >= 3)) {
        nBPM = BPM + 1;
        BPM = nBPM;
      }
      Serial.print("nBPM: ");
      Serial.println(nBPM);
      Serial.print("BBPM: ");
      Serial.println(BPM);
      //breathCheck()
      //      if (nBPM == BPM) {
      //        BPMcounter++;
      //
      //        Serial.print("BPMcounter: ");
      //        Serial.println(BPMcounter);
      //
      //        if (BPMcounter == 120) {
      //          Serial.println("returning...");
      //          lcd2.clear();
      //          return;
      //        }
      //      }

      //BPS = 0;

      fBPM = BPM;

      Serial.print("BPM ");
      Serial.println(fBPM);

      getMaskTemp();

      Serial.println("Last Mask Output to Serial");
      outputMaskSerial();
      Serial.println("Last Mask Output to LCD, then set prevBPS to fBPS");
      outputMaskLCD();
      //outputMaskLCDtest();

      prevBPS = BPS;
      Serial.print("PrevBPS: ");
      Serial.println(prevBPS);

      timerCheck = millis();

    }
    while ( (timerCheck - timerStart) < maskTimer);

    Serial.print("End: ");
    Serial.println(previousTime_2);

    //clear oBPM portion
    lcd2.setCursor(8, 1);
    lcd2.print("       ");

    //display oBPM again with new values
    lcd2.setCursor(8, 1);
    lcd2.print("oBPM:");
    lcd2.print(fBPM);

    //clear current BPM portion
    lcd2.setCursor(2, 1);
    lcd2.print("      ");

    BPS = 0;
    BPM = 0;

  }
}

void outputMaskSerial() {
  Serial.print("Msk ADC: "); // Print ADC value to Serial Monitor
  Serial.print(currentRead);
  Serial.print("\tMsk Temp: "); // Print temperature to Serial Monitor in Celcius
  Serial.print(MASK, 7);
  Serial.print("\tMsk Voltage: ");
  Serial.print(Voltage, 7);
  Serial.print("\tMsk Res: ");
  Serial.println(resistance, 7);
}


void outputPollLCD() {
  //  lcd2.setCursor(0, 0);
  //  //  lcd2.print("R=");
  //  lcd2.print(roomReadings);
  //  lcd2.print("C");

  lcd2.setCursor(0, 0);
  lcd2.print("Stat: ");

  //  lcd2.setCursor(8, 0);
  //  lcd2.print("M=");
  //  lcd2.print(MASKpoll);
  //  //  lcd2.print("C");

  lcd2.setCursor(1, 1);
  lcd2.print("BPM:");
  lcd2.print(BPM);
}

void outputMaskLCD() {
  //  lcd2.setCursor(0, 0);
  //  //  lcd2.print("R=");
  //  lcd2.print(roomReadings);
  //  lcd2.print("C");

  lcd2.setCursor(0, 0);
  lcd2.print("Stat: ");

  //  lcd2.setCursor(8, 0);
  //  lcd2.print("M=");
  //  lcd2.print(MASK);
  //  lcd2.print("C");

  lcd2.setCursor(1, 1);
  lcd2.print("BPM:");
  lcd2.print(BPM);
}


void outputPollLCDtest() {
  lcd2.setCursor(0, 0);
  //  lcd2.print("R=");
  lcd2.print(roomReadings);
  lcd2.print("C");

  //  lcd2.setCursor(0, 0);
  //  lcd2.print("Stat: ");

  lcd2.setCursor(8, 0);
  lcd2.print("M=");
  lcd2.print(MASK);
  lcd2.print("C");

  lcd2.setCursor(1, 1);
  lcd2.print("BPM:");
  lcd2.print(BPM);

}

void outputMaskLCDtest() {
  lcd2.setCursor(0, 0);
  //  lcd2.print("R=");
  lcd2.print(roomReadings);
  lcd2.print("C");

  //  lcd2.setCursor(0, 0);
  //  lcd2.print("Stat: ");

  lcd2.setCursor(8, 0);
  lcd2.print("M=");
  lcd2.print(MASK);
  lcd2.print("C");

  lcd2.setCursor(1, 1);
  lcd2.print("BPM:");
  lcd2.print(BPM);
}

void outputPollSerial() {
  Serial.print("MskPoll ADC: "); // Print ADC value to Serial Monitor
  Serial.print(adc0);
  Serial.print("\tMskPoll Temp: "); // Print temperature to Serial Monitor in Celcius
  Serial.print(MASKpoll, 7);
  Serial.print("\tMskPoll Voltage: ");
  Serial.print(Voltage, 7);
  Serial.print("\tMskPoll Res: ");
  Serial.println(resistance, 7);
}



void switchToRoom() {
  digitalWrite(SensorSwitch, LOW);
}

void switchToMask() {
  digitalWrite(SensorSwitch, HIGH);
}

void bpmCheck() {
  if ((fBPM < 99)) {

    if ((fBPM >= 12) && (fBPM <= 19)) {
      lcd2.setCursor(8, 0);
      lcd2.print("       ");
      //  lcd2.print("R=");
      lcd2.setCursor(8, 0);
      lcd2.print("Normal");
      //    buzzerRpd();

    }
    else if ((fBPM > 19)) {
      lcd2.setCursor(8, 0);
      lcd2.print("       ");
      //  lcd2.print("R=");
      lcd2.setCursor(8, 0);

      lcd2.print("Rapid");
      buzzerRpd();
    }
    else if ((fBPM < 12) && (fBPM > 3)) {
      lcd2.setCursor(8, 0);
      lcd2.print("       ");
      //  lcd2.print("R=");
      lcd2.setCursor(8, 0);
      lcd2.print("Slow");
      //      buzzerSlow();
    }
    else if ((fBPM == 0)) {
      lcd2.setCursor(8, 0);
      lcd2.print("       ");
      //  lcd2.print("R=");
      lcd2.setCursor(8, 0);
      lcd2.print("No Brth");
      buzzerRpd();
    }
    else if ((fBPM < 3)) {
      lcd2.setCursor(8, 0);
      lcd2.print("       ");
      //  lcd2.print("R=");
      lcd2.setCursor(8, 0);
      lcd2.print("Halted");
      buzzerRpd();
    }
  }

}

void buzzerRpd() {
  //for (int u = 0; u < 50; u++)
  buzzerSound1 = millis();

  do {
    digitalWrite(buzzer, HIGH);
    buzzerTime = buzzerSound1;

    buzzerSound1 = millis();

  } while (( buzzerSound1 - buzzerTime ) < buzzerSoundRpd);
  digitalWrite(buzzer, LOW);

  //  {
  //    buzzerSound1 = millis();
  //    if ( (buzzerSound1 - buzzerTime) >= buzzerSound2) {
  //      digitalWrite(buzzer, HIGH);
  //      buzzerTime = buzzerSound1;
  //    }
  //    buzzerSound1 = millis();
  //    if ( (buzzerSound1 - buzzerTime) >= buzzerSound2) {
  //      digitalWrite(buzzer, LOW);
  //      buzzerTime = buzzerSound1;
  //    }
  //  }
}

void buzzerRpd2() {
  for (int i = 0; i < 255; i++) { //do this 255 times
    analogWrite(buzzer, i); //raise the voltage sent out of the pin by 1
    //delay(10); //wait 10 milliseconds to repeat
  }

  for (int i = 255; i > 0; i--) { // do this 255 times
    analogWrite(buzzer, i); //lower the voltage sent out of the pin by 1
    //delay(10); //wait 10 milliseconds to repeat
  }
}

void buzzerRpd3() {
  //for (int u = 0; u < 50; u++)
  //buzzerSound1 = millis();

  do {
    digitalWrite(buzzer, HIGH);
    buzzerTime = millis();

    //buzzerSound1 = millis();

  } while (( buzzerTime - currentTime ) <= buzzerSoundRpd);
  digitalWrite(buzzer, LOW);

}

void buzzerOn() {
  digitalWrite(buzzer, HIGH);
}

void buzzerOff() {
  digitalWrite(buzzer, LOW);
}
