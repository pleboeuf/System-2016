// Code for Arduino Pro mini in US100 Robuste sensors

#include <SoftwareSerial.h>

// Firmware version
#define FirmwareVersion 1.0.0   //Version .

#define MaxHeatingPowerPercent 70 // Puissance maximale appliqué sur la résistance de chauffage
#define HeatingSetPoint 15        // Température cible à l'intérieur du boitier
#define ExceedRangeUS100 9999     // Distance maximale valide pour le captgeur
#define numReadings 10            // Number of readings to average for filtering
#define sensorNotTiltedRedLED HIGH      // Corresponding Red LED control level
#define sensorNotTiltedGreenLED LOW      // Corresponding GreenLED control level
#define sensorNotTiltedBlueLED HIGH      // Corresponding GreenLED control level
#define sensorTiltedRedLED  LOW         // Corresponding Red LED control level
#define sensorTiltedGreenLED  HIGH         // Corresponding GreenLED control level
#define sensorTiltedBlueLED  HIGH         // Corresponding GreenLED control level

const byte rxPin = 2;       // SoftwareSerial RX pin
const byte txPin = 4;       // SoftwareSerial TX pin
const byte TiltSW = 3;      // Tilt Switch input pin
const byte RGBled_Red = 6;  // RGB Status led RED pin
const byte RGBled_Green = 7;// RGB Status led GREEN pin
const byte RGBled_Blue = 8; // RGB Status led BLUE pin
const byte heater = 10;     // PWM input to Heating transistor
const byte Led = 13;        // Activity LED

int TempUS100 = 0;
int allTempReadings[numReadings];
int HeatingLimitPercent = MaxHeatingPowerPercent; // Puissance maximale appliqué sur la résistance de chauffage

int LedState = LOW;
int HeatingPower = 0;
bool Tilt = HIGH;
bool TempRequest = false;

unsigned long Period = 500;
unsigned long nextSampleTime = millis() + Period;

// set up a new serial object to communicate with the US100
SoftwareSerial US100 (rxPin, txPin);

// Setup
void setup() {
        // Set the data rate for the SoftwareSerial port
        US100.begin(9600);        // Start SoftwareSerial port
        Serial.begin(9600);       // Start Serial port

        // Set I/O pin mode
        pinMode(13, OUTPUT);      // Set Activity LED pin to OUTPUT
        pinMode(heater, OUTPUT);  // Set heater pin to OUTPUT
        pinMode(TiltSW, INPUT);   // Set tilt switch pin to INPUT
        pinMode(RGBled_Red, OUTPUT); // Set RGBled_Red as OUTPUT
        pinMode(RGBled_Green, OUTPUT); // Set RGBled_Green as OUTPUT
        pinMode(RGBled_Blue, OUTPUT); // Set RGBLed_Blue as OUTPUT

        // Initialize the RGB Led to BLUE
        digitalWrite(RGBled_Red, HIGH);
        digitalWrite(RGBled_Green, HIGH);
        digitalWrite(RGBled_Blue, LOW);
}

// Main program loop
void loop() {
        // Monitor the data on the serial ports
        checkSerialPorts();
        // Do additional processing every "Period" ms
        unsigned long LoopTime = millis();
        if (LoopTime > nextSampleTime) {
                nextSampleTime = LoopTime + Period - 1;

                // First Adjust heating as required
                simpleThermostat(HeatingSetPoint);

                // Then flash the Arduino Led to show activities
                LedState = !LedState;
                digitalWrite(Led, LedState);
        }
}

// Monitor the serial ports taking account the TILT status of the sensor
// Intercept the temperature data from the US100 sensor for use bay the thermostat
void checkSerialPorts(){
        char oneByteFromSerial = 0; // One byte buffer for data comming from Serial port
        char oneByteFromUS100 = 0; // One byte buffer for data comming from US100 port
        int Temp45 = 0;       // Raw temperature reading
        bool Tilt = checkTiltSW();
        // Forward data from Serial to US100
        if (Serial.available()) {
                oneByteFromSerial = Serial.read();  // Read one byte from the Serial port
                US100.write(oneByteFromSerial);     // Forward it to the US100 sensor
                // Find if its a request for temperature
                // Writing a 0X50 byte request the US100 to send the temperature
                if(oneByteFromSerial == 0X50) {
                        TempRequest = true;         // Yes, it's a request for temperature
                } else {
                        TempRequest = false;        // No, must be a request for distance.
                }
        }
        // Forward data from US100 to Serial
        if (US100.available()) {
                oneByteFromUS100 = US100.read(); // Read one byte from the US100
                if (!Tilt) {
                        // Sensor is NOT tilted, forward data byte as is
                        Serial.write(oneByteFromUS100);
                } else {
                        // Sensor IS Tilted
                        if (!TempRequest) {
                                // It's not a temperature request. Return "0" to indicate invalid data in case of tilt
                                Serial.write("0");
                        } else {
                                // Its a temperature request, let the data go throught
                                Serial.write(oneByteFromUS100);
                        }
                }
                // Intercept the temperature readings from the US100 and average it
                if(TempRequest) {
                        Temp45 = oneByteFromUS100;
                        if((Temp45 > 1) && (Temp45 < 130)) // Verify if its within acceptable range
                        {
                                TempUS100 = AvgTempReading(Temp45 - 45) / numReadings; //Conversion en température réelle et filtrage
                        } else {
                                TempUS100 = HeatingSetPoint - 5; // Force operation of the heater
                                HeatingLimitPercent = 50; // at 50% power
                        }
                }
        }
}

// Check the state of the TILT Switch and adjust the color of the status LED
// Return the Tilt Status
bool checkTiltSW(){
        bool tiltStatus = !digitalRead(TiltSW); // Note: LOW input indicate sensor IS tilted
        if (!tiltStatus) {
                // Sensor is NOT tilted
                digitalWrite(RGBled_Red, sensorNotTiltedRedLED); // Turn led OFF
                digitalWrite(RGBled_Green, sensorNotTiltedGreenLED); // Turn led ON
                digitalWrite(RGBled_Blue, sensorNotTiltedBlueLED); // Turn led OFF
        } else {
                // If sensor IS Tilted
                digitalWrite(RGBled_Red, sensorTiltedRedLED); // Turn led ON
                digitalWrite(RGBled_Green, sensorTiltedGreenLED); // Turn led OFF
                digitalWrite(RGBled_Blue, sensorTiltedBlueLED); // Turn led OFF
        }
        return tiltStatus;
}

// Filtre par moyenne mobile pour les température
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgTempReading(int thisReading){
        long Avg = 0;
        for (int i = 1; i < numReadings; i++) {
                allTempReadings[i-1] = allTempReadings[i]; //Shift all readings
                Avg += allTempReadings[i-1]; //Total of readings except the last one
        }
        allTempReadings[numReadings - 1] = thisReading; //Current reading in the last position
        Avg += thisReading; //total including the last one
        return (Avg); // Avg sera divisé par numReadings au moment de la publication
}

// Imprémentation d'un thermostat simple ON/OFF
// La routine ne fonctionne que si un capteur de température interne est trouvé
int simpleThermostat(double setPoint){
        if (TempUS100 != 99) {
                if ((double)TempUS100 < (setPoint - 0.5)) {
                        HeatingPower =  256 *  HeatingLimitPercent /100;
                } else if (TempUS100 > (setPoint + 0.5)) {
                        HeatingPower =  0;
                }
        }
        analogWrite(heater, HeatingPower);
        return HeatingPower;
}
