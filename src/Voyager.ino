/* -----------------------------------------------------------
This example shows a lot of different features. As configured here
it will check for a good GPS fix every 10 minutes and publish that data
if there is one. If not, it will save you data by staying quiet. It also
registers 3 Particle.functions for changing whether it publishes,
reading the battery level, and manually requesting a GPS reading.
---------------------------------------------------------------*/

// Getting the library

#include "AssetTracker.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include "DS18.h"
#include "neopixel.h"
#include <Adafruit_VC0706.h>
#include "Serial4/Serial4.h"


// IMPORTANT: Set pixel COUNT, PIN and TYPE
#define PIXEL_PIN D3
#define PIXEL_COUNT 16
#define PIXEL_TYPE SK6812RGBW
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
Adafruit_VC0706 cam = Adafruit_VC0706(&Serial4);


DS18 sensor(D2, true);
float airTemp = 0, waterTemp = 0, caseTemp = 0;


Adafruit_INA219 battLoad;
Adafruit_INA219 sysLoad;

float battShuntvoltage = 0;
float battBusvoltage = 0;
float battCurrent_mA = 0;
float battLoadvoltage = 0;

float sysShuntvoltage = 0;
float sysBusvoltage = 0;
float sysCurrent_mA = 0;
float sysLoadvoltage = 0;




int transmittingData = 1;
long lastPublish = 0;


AssetTracker t = AssetTracker();
FuelGauge fuel;
CellularSignal sig = Cellular.RSSI();





void setup() {
    // Sets up all the necessary AssetTracker bits
    t.begin();

    // Enable the GPS module. Defaults to off to save power.
    // Takes 1.5s or so because of delays.
    t.gpsOn();
    t.antennaExternal();
    battLoad.begin(0x41);
    sysLoad.begin(0x40);
    // Opens up a Serial port so you can listen over USB
    Serial.begin(9600);

    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    colorAll(strip.Color(128,0,128), 0);

    uint32_t currentFrequency;

    //These three functions are useful for remote diagnostics. Read more below.
    Particle.function("tmode", transmitMode);
    Particle.function("batt", batteryStatus);
    Particle.function("gps", gpsPublish);


    // Try to locate the camera
    if (cam.begin()) {
      Serial.println("Camera Found:");
    } else {
      Serial.println("No camera found?");
      return;
    }
    // Print out the camera version information (optional)
    char *reply = cam.getVersion();
    if (reply == 0) {
      Serial.print("Failed to get version");
    } else {
      Serial.println("-----------------");
      Serial.print(reply);
      Serial.println("-----------------");
    }
}

// loop() runs continuously
void loop() {
    // You'll need to run this every loop to capture the GPS output
    t.updateGPS();

    // if the current time - the last time we published is greater than your set delay...
    if (millis()-lastPublish > 10*1000) {
        // Remember when we published
        lastPublish = millis();



        updateCurrent(" ");
        Serial.println("Cellular");
        Serial.println(sig);

        String pubAccel = String::format("%d,%d,%d", t.readX(), t.readY(), t.readZ());
        Serial.println(pubAccel);
        //Particle.publish("A", pubAccel, 60, PRIVATE);

        Serial.println("Updating Temps");
        updateTemperatures();
        Serial.printf("%.2f,%.2f,%.2f", airTemp, waterTemp, caseTemp );
        Serial.println("");




        Serial.println(t.preNMEA());

        // GPS requires a "fix" on the satellites to give good data,
        // so we should only publish data if there's a fix
        // if (t.gpsFix()) {
        //     // Only publish if we're in transmittingData mode 1;
        //     if (transmittingData) {
        //         // Short publish names save data!
        //         Particle.publish("G", t.readLatLon(), 60, PRIVATE);
        //     }
        //     // but always report the data over serial for local development
        //     Serial.println(t.readLatLon());
        // }
    }
}

// Allows you to remotely change whether a device is publishing to the cloud
// or is only reporting data over Serial. Saves data when using only Serial!
// Change the default at the top of the code.
int transmitMode(String command) {
    transmittingData = atoi(command);
    return 1;
}

// Actively ask for a GPS reading if you're impatient. Only publishes if there's
// a GPS fix, otherwise returns '0'
int gpsPublish(String command) {
    if (t.gpsFix()) {
        Particle.publish("G", t.readLatLon(), 60, PRIVATE);
        return 1;
    } else {
      return 0;
    }
}
void updateTemperatures(){
  int count = 0; //Safety check against infinite loop;
  while(sensor.read() && count < 10){
      uint8_t addr[8];
      sensor.addr(addr);
      if (addr[7] == 142){
        airTemp = sensor.fahrenheit();
      }
      else if (addr[7] == 231){
        waterTemp = sensor.fahrenheit();
      }
      else if (addr[7] == 169){
        caseTemp = sensor.fahrenheit();
      }
      if (sensor.searchDone()){
        break;
      }
      count++;
      delay(250);
    }


}


void printDebugInfo() {
  // If there's an electrical error on the 1-Wire bus you'll get a CRC error
  // Just ignore the temperature measurement and try again
  if (sensor.crcError()) {
    Serial.print("CRC Error ");
  }

  // Print the sensor type
  const char *type;
  switch(sensor.type()) {
    case WIRE_DS1820: type = "DS1820"; break;
    case WIRE_DS18B20: type = "DS18B20"; break;
    case WIRE_DS1822: type = "DS1822"; break;
    case WIRE_DS2438: type = "DS2438"; break;
    default: type = "UNKNOWN"; break;
  }
  Serial.print(type);

  // Print the ROM (sensor type and unique ID)
  uint8_t addr[8];
  sensor.addr(addr);
  Serial.printf(
    " ROM=%02X%02X%02X%02X%02X%02X%02X%02X",
    addr[0], addr[1], addr[2], addr[3], addr[4], addr[5], addr[6], addr[7]
  );

  // Print the raw sensor data
  uint8_t data[9];
  sensor.data(data);
  Serial.printf(
    " data=%02X%02X%02X%02X%02X%02X%02X%02X%02X",
    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]
  );
}


int updateCurrent(String command){
  sysShuntvoltage = sysLoad.getShuntVoltage_mV();
  sysBusvoltage = sysLoad.getBusVoltage_V();
  sysCurrent_mA = sysLoad.getCurrent_mA();
  sysLoadvoltage = sysBusvoltage + (sysShuntvoltage / 1000);

  battShuntvoltage = battLoad.getShuntVoltage_mV();
  battBusvoltage = battLoad.getBusVoltage_V();
  battCurrent_mA = battLoad.getCurrent_mA();
  battLoadvoltage = battBusvoltage + (battShuntvoltage / 1000);

  Serial.println("System");
  Serial.print("Bus Voltage:   "); Serial.print(sysBusvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(sysShuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(sysLoadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(sysCurrent_mA); Serial.println(" mA");
  Serial.println("");

  Serial.println("Battery");
  Serial.print("Bus Voltage:   "); Serial.print(battBusvoltage); Serial.println(" V");
  Serial.print("Shunt Voltage: "); Serial.print(battShuntvoltage); Serial.println(" mV");
  Serial.print("Load Voltage:  "); Serial.print(battLoadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(battCurrent_mA); Serial.println(" mA");
  Serial.println("");

}



int batteryStatus(String command){
    // Publish the battery voltage and percentage of battery remaining
    // if you want to be really efficient, just report one of these
    // the String::format("%f.2") part gives us a string to publish,
    // but with only 2 decimal points to save space
    Particle.publish("B",
          "v:" + String::format("%.2f",fuel.getVCell()) +
          ",c:" + String::format("%.2f",fuel.getSoC()),
          60, PRIVATE
    );
    // if there's more than 10% of the battery left, then return 1
    if (fuel.getSoC()>10){ return 1;}
    else { return 0;}
}


// Set all pixels in the strip to a solid color, then wait (ms)
void colorAll(uint32_t c, uint8_t wait) {
  uint16_t i;

  for(i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  delay(wait);
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
