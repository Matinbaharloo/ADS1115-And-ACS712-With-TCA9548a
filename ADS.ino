/**
   TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino

   Based on https://playground.arduino.cc/Main/I2cScanner/

*/

#include "Wire.h"



#include <ErriezDHT22.h>

// Connect DTH22 DAT pin to Arduino DIGITAL pin
#if defined(ARDUINO_ARCH_AVR) || defined(ARDUINO_SAM_DUE)
#define DHT22_PIN      18
#elif defined(ESP8266) || defined(ESP32)
#define DHT22_PIN      4 // GPIO4 (Labeled as D2 on some ESP8266 boards)
#else
#error "May work, but not tested on this target"
#endif

#include "ADS1X15.h"



ADS1115 ADS(0x48);

// Create DHT22 sensor object
DHT22 dht22 = DHT22(DHT22_PIN);

// Function prototypes
void printTemperature(int16_t temperature);
void printHumidity(int16_t humidity);
#define TCAADDR 0x70
// Select I2C BUS
void TCA9548A(uint8_t bus) {
  Wire.beginTransmission(0x70);  // TCA9548A address
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
  Serial.print(bus);
}

void tcaselect(uint8_t i) {
  if (i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}




void ADS_INIT(void)
{
  ADS.begin();
  delay(100);

}
void AHT_INIT(void)
{
  Serial.println(F("DHT22 temperature and humidity sensor example\n"));

  // Initialize DHT22
  dht22.begin();
}
void AHT_READ(void)
{
  // Check minimum interval of 2000 ms between sensor reads
  if (dht22.available()) {
    // Read temperature from sensor (blocking)
    int16_t temperature = dht22.readTemperature();

    // Read humidity from sensor (blocking)
    int16_t humidity = dht22.readHumidity();

    // Print temperature
    printTemperature(temperature);

    // Print humidity
    printHumidity(humidity);
  }
  else
  {
    Serial.println("ERROR");
    delay(2000);
  }
}
void ADS_READ(void)
{

  ADS.setGain(1);

  int16_t val_0 = ADS.readADC(0);
  int16_t val_1 = ADS.readADC(1);
  int16_t val_2 = ADS.readADC(2);
  int16_t val_3 = ADS.readADC(3);

  float f = ADS.toVoltage(1);  //  voltage factor

  //  Serial.print("\tAnalog0: "); Serial.print(val_0); Serial.print('\t'); Serial.println(val_0 * f, 3);
  //  Serial.print("\tAnalog1: "); Serial.print(val_1); Serial.print('\t'); Serial.println(val_1 * f, 3);
  //  Serial.print("\tAnalog2: "); Serial.print(val_2); Serial.print('\t'); Serial.println(val_2 * f, 3);
  //  Serial.print("\tAnalog3: "); Serial.print(val_3); Serial.print('\t'); Serial.println(val_3 * f, 3);
  //  Serial.println();





//  float inputVoltage = 3.3;
//  float r = 10;
//  float ntcR25 = 3;
//  float ntcBeta = 3988;
//  float minVoltage = 0.000;
//  float maxVoltage = 4.096;
//  float maxValue = 32767;
////  int value = analogRead(A0);
//  float measuredVoltage = val_1 / maxValue * (maxVoltage - minVoltage) + minVoltage;
//  float rNtc = measuredVoltage * r / (inputVoltage - measuredVoltage);
//  float temperature = 1 / (log(rNtc / ntcR25) / ntcBeta + 1 / (273.15 + 25)) - 273.15;

  float AcsValue=0.0,Samples=0.0,AvgAcs=0.0,AcsValueF=0.0;
  AcsValueF = (2.5 - (val_2 * (5 / 32767)) )/0.1;
  float voltage = val_2*(4.096/32767.0);
#define UP_Element_Sensor 1.959
#define DOWN_1_Element_Sensor 2
#define DOWN_2_Element_Sensor 2
#define Steam_Element_Sensor 1.98
#define ALL_Device_Sensor 1.9746

//   Convert the voltage to current
  float current = (voltage-(DOWN_2_Element_Sensor))/(0.1); // 

  //Serial.println(x,10);

//  Serial.print("\tAnalog0: "); Serial.println(val_1); Serial.print("\trNtc: "); Serial.println(rNtc); Serial.print("\ttemperature "); Serial.println(temperature);
  Serial.print("\tAnalog0: "); Serial.println(val_2); Serial.print("\tcurrent: "); Serial.println(current);  Serial.print("\tval_0 * f "); Serial.println(voltage);
  delay(500);
}

// standard Arduino setup()
void setup()
{
  while (!Serial);
  //  delay(1000);

  Wire.begin();

  Serial.begin(115200);
  Serial.println("\nTCAScanner ready!");

  for (uint8_t t = 0; t < 8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr <= 127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX);
      }
    }
  }
  Serial.println("\ndone");
  pinMode(11, OUTPUT);   // sets the pin as output

  TCA9548A(3);
  ADS_INIT();
  //  AHT_INIT();


}

void loop()
{
  ADS_READ();
  //  AHT_READ();
//  analogWrite(11,127);// analogRead values go from 0 to 1023, analogWrite values from 0 to 255
//pwm();
}



void printTemperature(int16_t temperature)
{
  // Check valid temperature value
  if (temperature == ~0) {
    // Temperature error (Check hardware connection)
    Serial.println(F("Temperature: Error"));
  } else {
    // Print temperature
    Serial.print(F("Temperature: "));
    Serial.print(temperature / 10);
    Serial.print(F("."));
    Serial.print(temperature % 10);

    // Print degree Celsius symbols
    // Choose if (1) for normal or if (0) for extended ASCII degree symbol
    if (1) {
      // Print *C characters which are displayed correctly in the serial
      // terminal of the Arduino IDE
      Serial.println(F(" *C"));
    } else {
      // Note: Extended characters are not supported in the Arduino IDE and
      // displays ?C. This is displayed correctly with other serial terminals
      // such as Tera Term.
      // Degree symbol is ASCII code 248 (decimal).
      char buf[4];
      snprintf_P(buf, sizeof(buf), PSTR(" %cC"), 248);
      Serial.println(buf);
    }
  }
}

void printHumidity(int16_t humidity)
{
  // Check valid humidity value
  if (humidity == ~0) {
    // Humidity error (Check hardware connection)
    Serial.println(F("Humidity: Error"));
  } else {
    // Print humidity
    Serial.print(F("Humidity: "));
    Serial.print(humidity / 10);
    Serial.print(F("."));
    Serial.print(humidity % 10);
    Serial.println(F(" %"));
  }

  Serial.println();
}
