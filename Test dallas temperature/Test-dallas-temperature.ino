SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

// This #include statement was automatically added by the Particle IDE.
#include "spark-dallas-temperature.h"

// This #include statement was automatically added by the Particle IDE.
#include "OneWire.h"

// System Definitions
#define ONE_WIRE_BUS D4
#define SAMPLE_INTERVAL   2000     // Sample every 3 seconds

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

// Global Variables
int nextSampleTime;
int blueLed = D2;
int statusLed = D7;

DeviceAddress outsideThermometer, insideThermometer;

// function to print a device address
void printAddress(DeviceAddress deviceAddress);
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

/*Timer timer1(SAMPLE_INTERVAL, getTemps);*/

void setup(void)
{
    Serial.begin(9600);
    pinMode(statusLed, OUTPUT);
    pinMode(blueLed, OUTPUT);
    Particle.function("ledBleu", setBlueLed);

    // Startup the sensors
    sensors.begin();
    delay(2000);

    // Locate devices on the bus
    Serial.printlnf("Locating devices...Found %d devices", sensors.getDeviceCount());
    sensors.getAddress(insideThermometer, 0);
    sensors.getAddress(outsideThermometer, 1);

    // Show the addresses we found on the bus
    Serial.print("Device 0 Address: ");
    printAddress(insideThermometer);
    Serial.print("Device 1 Address: ");
    printAddress(outsideThermometer);

    sensors.setResolution(10);
    // Set the resolution to 11 bit (Each Dallas/Maxim device is capable of several different resolutions)
    for (int i=0;i<2;i++){
        /*Serial.printlnf("Setting sensor 0 resolution to 10 - %d", sensors.setResolution(outsideThermometer, 10));*/
        Serial.printlnf("Device 0 Resolution: %d", sensors.getResolution(outsideThermometer));

        /*sensors.setResolution(insideThermometer, 10);*/
        Serial.printlnf("Device 1 Resolution: %d", sensors.getResolution(insideThermometer));
        delay (200);
    }

    Serial.print("Current IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();

    delay(2000); // let everything power up for a bit
    nextSampleTime = 0;  // Start the first sample immediately
    /*timer1.start();*/
}

// This wrapper is in charge of calling
// mus be defined like this for the lib work

void loop(void)
{
    // Check if we need to start the next sample
    if (millis() > nextSampleTime)
    {
        nextSampleTime = millis() + SAMPLE_INTERVAL - 1;  // set the time for next sample
        getTemps();

        //If you want to power down between sends to save power (ie batteries).
        //Serial.println("System sleeping for 45 seconds . . .");
        //delay(1000);                        //Without the delay it jumps to sleep too fast
        //System.sleep(SLEEP_MODE_DEEP,44);   //sleep measured in seconds
    }
}

void getTemps(){
        float outsideTempC, insideTempC;
        int i;
        digitalWrite(statusLed, LOW);
        // call sensors.requestTemperatures() to issue a global temperature
        // request to all devices on the bus
        sensors.requestTemperatures(); // Send the command to get temperatures

        for (i = 0; i < 5; i++){
            outsideTempC = sensors.getTempC(outsideThermometer);
            if (outsideTempC > -127 && outsideTempC < 85){
                break;
            }
        }
        Serial.printlnf("Now = %d", millis());
        Serial.printlnf("Outdoor Temperature C: %5.2f, try= %d", outsideTempC, i + 1);

        // Publish the temperature online
        String myOutsideStr(outsideTempC, 3); // Cast the float into a string
        Spark.publish("OutTempC", myOutsideStr);

        for (i = 0; i < 5; i++){
            insideTempC = sensors.getTempC(insideThermometer);
            if (insideTempC > -127 && insideTempC < 85){
                break;
            }
        }
        Serial.printlnf("Indoor Temperature C: %5.2f, try= %d", insideTempC, i + 1);

        // Publish the temperature online
        String myInsideStr(insideTempC, 3); // Cast the float into a string
        Spark.publish("inTmpInfo", myInsideStr);
        Serial.println("");
        digitalWrite(statusLed, HIGH);
}

int setBlueLed(String command){
    if (command == "On"){
        digitalWrite(blueLed, LOW);
        return LOW;
    } else if (command == "Off"){
        digitalWrite(blueLed, HIGH);
        return HIGH;
    }
}
