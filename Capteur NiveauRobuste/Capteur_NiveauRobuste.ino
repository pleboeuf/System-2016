// This #include statement was automatically added by the Particle IDE.
#include "spark-dallas-temperature.h"
#include "OneWire.h"

// Photon Pin	Fonction		                    Location                |Capteur Standard	|Capteur Robuste	|Capteur On / Off	|Commun
//      D6	     SSR Relay 	                         Ext.	                        	                   	                  O
//      A0	     Signal moteur opto couplé - IAC5A	 Onboard	                    	                   	                  O
//      A1	     Signal moteur par Current xformer	 Ext.                           	                                      O
//      A2	     Valve 1A	                         Ext.                          O                      O
//      A3	     Valve 1B	                         Ext.	                       O	                  O
//      A4	     Valve 2A	                         Ext.	                       O	                  O
//      A5	     Valve 2B	                         Ext.	                       O	                  O
//      D2 (pwm)	RGB Led Red	                     Onboard	                   O	                  O	                  O	             O
//      D1 (pwm)	RGB Led Green	                 Onboard	                   O	                  O	                  O	             O
//      D0 (pwm)	RGB Led Blue	                 Onboard	                   O	                  O	                  O	             O
//      D7	Activity Blue LED	                     Onboard	                   O	                  O	                  O	             O
//      3V3	US-100 Pin 1 - Vcc	                     Onboard	                   O
//      Tx	US-100 Pin 2 - Trig /  Tx	             Onboard	                   O
//      Rx	US-100 Pin 3 - Echo / Rx	             Onboard	                   O
//      Gnd	US-100 Pin 4 - Gnd	                     Onboard	                   O
//      Gnd	US-100 Pin 5 - Gnd	                     Onboard	                   O
//      3V3	XL-Max Sonar MB7389 Pin 1 - Nc or High	 Ext.	                        	                  O
//      D5	XL-Max Sonar MB7389 Pin 2 - Pulse out	 Ext.	                        	                  O
//      Tx	XL-Max Sonar MB7389 Pin 4 - Trig 20uS	 Ext.	                                              O
//      Rx	XL-Max Sonar MB7389 Pin 5 - Serial out	 Ext.	                        	                  O
//      Gnd	XL-Max Sonar MB7389 Pin 7 - Gnd	         Ext.	                                              O
//      D4	Dallas 18b20 Temperature sensor	         Onboard / Ext.	               O	                  O	                  O	             O
//      D3	Heating circuit. Power control	         Onboard	                     	                  O
//      Vbat	Memory backup battery CR2032	     Onboard	                   O	                  O	                  O	             O
// the US-100 module WITH jumper cap on the back.


// the US-100 module WITH jumper cap on the back.

/*
Tous les événements seront stocké dans dans un buffer circulaire en espace mémoire protégé par batterie.
De cette façon les 300 derniers événements seront conservés en cas de perte de réseau.
Afin d'optimiser l'espace requis, les données seront de type entier (integer ou long).
Les nom d'événements sont remplacer par leur index dans l'array eventName.
Les données sont organisé dans une structure "Event" et stocké dans un array de ces structures.
9 bytes sont utilisés par événement. On peut donc sauver environ 4000 / 9 = 440 événements.
Les événements sont stocké au fur et à mesure de leur production. Il seront publié séquenciellement
indépendamment de leur production.
*/

// SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));


#define minute 60000UL            // 60000 millisecond per minute
#define second 1000UL             // 1000 millisecond per sesond
#define unJourEnMillis (24 * 60 * 60 * second)
#define baseLoopTime  206      //Estimated loop time in millisecond
#define debounceDelay 50    // Debounce time for valve position readswitch
#define fastSampling  1000UL   // in milliseconds
#define slowSampling  5000UL    // in milliseconds
#define numReadings 10           // Number of readings to average for filtering
#define minDistChange 2.0 * numReadings      // Minimum change in distance to publish an event (1/16")
#define minTempChange 0.5 * numReadings      // Minimum temperature change to publish an event
#define maxRangeUS100 2500 // Distance maximale valide pour le captgeur
#define maxRangeMB7389 4999 // Distance maximale valide pour le captgeur
#define ONE_WIRE_BUS D4 //senseur sur D4
#define DallasSensorResolution 9 // Résolution de lecture de température
#define MaxHeatingPowerPC 50 // Puissance maximale appliqué sur la résistance de chauffage
#define HeatingSetPoint 30 // Température cible à l'intérieur du boitier


// Nom des indices du tableau eventName
#define evPompe_T1 0
#define evPompe_T2 1
#define evUS100Distance 2
#define evUS100Temperature 3
#define evOutOfRange 4
#define evValve1OpenSensor 5
#define evValve1CloseSensor 6
#define evValve2OpenSensor 7
#define evValve2CloseSensor 8
#define evRelais 9
#define evVacuum 10
#define evDebit 11
#define evVolume 12
#define evPressionAtmos 13
#define evTempInterne 14
#define evTempExterne 15
#define evHeating 16
#define evMB7389Distance 17
#define evBootTime 18
#define evNewGenSN 19


// Variables lié aux événements
String eventName[] = {
    "sonde/Pompe/T1",
    "sonde/Pompe/T2",
    "sonde/US100/Distance",
    "sonde/US100/Temperature_US100",
    "sonde/Hors portée: ",
    "sonde/Valve1/OpenSensor",
    "sonde/Valve1/CloseSensor",
    "sonde/Valve2/OpenSensor",
    "sonde/Valve2/CloseSensor",
    "sortie/Relais",
    "sonde/Vacuum",
    "sonde/Débit",
    "calcul/Volume",
    "sonde/Pression Atmosphérique",
    "sonde/DS18B20/Température interne",
    "sonde/DS18B20/Température externe",
    "sortie/Chauffage boitier",
    "sonde/MB7389/Distance",
    "Boot timestamp",
    "NewGenSN"
    };

// Structure définissant un événement
typedef struct Event{
  uint16_t noSerie; // Le numéro de série est généré automatiquement
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int16_t eData;  // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
                  // multiplié d'abord la donnée par un facteur (1000 par ex.) en convertir en entier.
                  // Il suffira de divisé la données au moment de la réception de l'événement.
  unsigned long NewGenTimeStamp; // Contient le timestamp initiale de cette génération de no serie
  unsigned long eTime; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
};
const int buffSize = 300; // Nombre max d'événements que l'on peut sauvegarder
retained unsigned int buffLen = 0;
retained unsigned int writePtr = 0;
retained unsigned int readPtr = 0;
retained struct Event eventBuffer[buffSize];
String DomainName = "brunelle/";
String DeptName = "prod/";

// Pin pour l' I/O
int RGBled_Red = D0;
int RGBled_Green = D1;
int RGBLed_Blue = D2;
int led = D7; // Feedback led
int ssrRelay = D6; // Solid state relay
int RelayState = false;
int motorState = A0; // input pour Pompe marche/arrêt
int heater = D3; //Contrôle le transistor du chauffage

// Variables liés à la pompe
bool PumpOldState = true;
bool PumpCurrentState = true;
unsigned long changeTime = 0;

// Variables liés aux valves
int ValvePos_pin[] = {A5, A4, A3, A2};
bool ValvePos_state[] = {true, true, true, true};
int ValvePos_Name[] = {evValve1OpenSensor, evValve1CloseSensor, evValve2OpenSensor, evValve2CloseSensor};

// Variables liés à la mesure de Température
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
int TempUS100 = 0;
int prev_TempUS100 = 0;
int prev_TempExterne = 999;
int prev_TempInterne = 999;
int allTempReadings[numReadings];
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20Sensors(&oneWire);
DeviceAddress enclosureThermometer, outsideThermometer;
int ds18b20Count = 0;
bool validTempExterne = false;
bool validTempInterne = false;

int HeatingPower = 0;
int prev_HeatingPower = 64;

// Variables liés à la mesure de distance
int dist_mm  = 0;
int prev_dist_mm = 0;
int allDistReadings[numReadings];

// Variables liés au temps
unsigned long lastPublish = millis();
unsigned long now;
unsigned long lastSync = millis();
unsigned int samplingInterval = fastSampling;
unsigned long nextSampleTime = 0;
int maxPublishInterval = 2;
unsigned long maxPublishDelay = maxPublishInterval * minute;
unsigned long lastTime = 0UL;
time_t NewGenTimeStamp;

// Variables liés aux publications
char publishString[buffSize];
retained uint16_t noSerie = 0; //Mettre en Backup RAM
int pumpEvent = 0;
bool connWasLost = false;

// Autre variables
bool MB7389Valid = false;
String Dist_MB7389Str;
int MB7389latestReading = 0;
int R = 82;
int CR = 13;

/*
   handler to receive the module name
*/
String myDeviceName = "";
void nameHandler(const char *topic, const char *data) {
    myDeviceName =  String(data);
    /*Serial.println("received " + String(topic) + ": " + String(data));*/
}

/*
// Create a class and handler to mimic the state of the RGB LED on the Photon
*/
class ExternalRGB {
  public:
    ExternalRGB(pin_t r, pin_t g, pin_t b) : pin_r(r), pin_g(g), pin_b(b) {
      pinMode(pin_r, OUTPUT);
      pinMode(pin_g, OUTPUT);
      pinMode(pin_b, OUTPUT);
      RGB.onChange(&ExternalRGB::LEDhandler, this);
    }

    void LEDhandler(uint8_t r, uint8_t g, uint8_t b) {
      analogWrite(pin_r, 255 - r); // 255 - r pour common cathode
      analogWrite(pin_g, 255 - g); // 255 - g pour common cathode
      analogWrite(pin_b, 255 - b); // 255 - b pour common cathode
    }

    private:
      pin_t pin_r;
      pin_t pin_g;
      pin_t pin_b;
};

// Connect an external RGB LED to D0, D1 and D2 (R, G, and B)
ExternalRGB myRGB(RGBled_Red, RGBled_Green, RGBLed_Blue);

/*
// Attach interrupt handler to pin A0 to monitor pump Start/Stop
*/
class PumpState_A1 {
  public:
    PumpState_A1() {
      attachInterrupt(A1, &PumpState_A1::A1Handler, this, CHANGE);
    }
    void A1Handler() {
      System.ticksDelay(50000*System.ticksPerMicrosecond()); // Debounce 50 milliseconds
      Serial.print("Pompe ");
      // enregistre l'état et le temps
      PumpCurrentState = digitalRead(A1);
      changeTime = millis();
      if (PumpCurrentState == false){
        Serial.println("On");
      } else {
        Serial.println("Off");
      }
    }
};

PumpState_A1 pumpState; // Instantiate the class A0State

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

// Initialisation
void setup() {
// connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
    Serial.begin(115200); // Pour débug
    Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
    Serial1.halfduplex(true); // Ce capteur envoie seulement des données sésie dans une seule direction
                          // On peut utiliser la pin RX pour contrôler son fonctionnement
                          // RX = LOW pour arrêter le capteur, RX = HIGH pour le démarrer
    delay(2000); // Pour partir le moniteur série pour débug

// WiFi.setCredentials("BoilerHouse", "Station Shefford");
// WiFi.setCredentials("PumpHouse", "Station Laporte");


// Enregistrement des fonctions et variables disponible par le nuage
    Serial.println("Enregistrement des variables et fonctions\n");
    Particle.variable("relayState", RelayState);
    Particle.variable("DS18B20Cnt", ds18b20Count);
    Particle.function("relay", toggleRelay);
    Particle.function("pubInterval", setPublishInterval);
    Particle.function("reset", remoteReset);
    Particle.function("replay", replayEvent);

    // Attendre la connection au nuage
        Serial.println("En attente... ");
        if (waitFor(Particle.connected, 10000)) {
            if(Particle.connected()){
                Serial.println("Connecté au nuage. :)");
                //Quel est mon nom?
                    delay(1000);
                    Particle.subscribe("spark/", nameHandler);
                    Particle.publish("spark/device/name");

                // Attendre la réception du nom
                    while(myDeviceName.length() == 0){
                        delay(1000);
                        Serial.print(".");
                    }
                    Serial.println("\nMon nom est: " + myDeviceName + "\n");
            } else {
                Serial.println("Pas de connexion au nuage. :( ");
            }
        }

    delay(1000UL);

// initialisation des capteurs de températures
    initDS18B20Sensors();

// Initialisation des I/O digital
    pinMode(led, OUTPUT);
    pinMode(ssrRelay, OUTPUT);
    pinMode(heater, OUTPUT);
    HeatingPower =  256 * MaxHeatingPowerPC / 100; // Valeur de PWM de chauffage
    analogWrite(heater, HeatingPower); //Désactiver le chauffage
    digitalWrite(led, LOW);
    digitalWrite(ssrRelay, LOW);
    PumpCurrentState = digitalRead(A0);
    for (int i=0; i <= 3; i++) {
        pinMode(ValvePos_pin[i], INPUT_PULLUP);
    }

    for (int i = 0; i<=numReadings; i++){ // Init readings array
        allDistReadings[i] = 0;
        allTempReadings[i] = 0;
        /*Serial.println(allReadings[i]);*/
    }

    Time.zone(-4);
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);

    lastPublish = millis(); //Initialise le temps initial de publication
    changeTime = lastPublish; //Initialise le temps initial de changement de la pompe
    time_t bootTimeStamp = Time.now();
    Serial.printlnf("Boot timestamp: %u", bootTimeStamp);

    /*
    Note:
    La position de eTime et de eData sont inversé dans ce cas-ci
    eTime est de type int et ne peux contenir le timestamp
    */
    pushToPublishQueue(evBootTime, millis(), bootTimeStamp);
}

/*
    Le capteur de distance MB7389 fonctionne en continu.
    Cette routine reçoit les données séries et met le résultats dans une variable
     Pour utilisation par la routine de measure
*/

void serialEvent1()
{
    char c = Serial1.read();

// Début de séquence
    if (c == R){
        MB7389Valid = true;
        Dist_MB7389Str = "";
        return;
    }

// Fin de séquence
    if (c == CR){
        MB7389Valid = false;
        MB7389latestReading = Dist_MB7389Str.toInt();
        // Serial.println("Dist_MB7389Str= " + Dist_MB7389Str);
        return;
     }

// Accumule des données
    if (MB7389Valid == true){
        Dist_MB7389Str += c;
    }
}

/*
    Boucle principale
*/
void loop(){
    if (millis() > nextSampleTime) {
        nextSampleTime = millis() + samplingInterval - 1;
        readAllSensors();
        simpleThermostat(HeatingSetPoint);
    }
    CheckValvePos();
    /*readDS18b20temp();*/
    Particle.process();
}

/*
    Routine principale de lecture des capteurs
*/

void initDS18B20Sensors(){
    // Configuration des capteurs de température DS18B20
    float insideTempC, outsideTempC;

        ds18b20Sensors.begin();
        delay(300);
        ds18b20Count = ds18b20Sensors.getDeviceCount();
        ds18b20Sensors.setWaitForConversion(true);
        Serial.printlnf("DS18B20 trouvé: %d", ds18b20Count);

        if (ds18b20Count == 1){
            Serial.println("Configuration de 1 ds18b20");

            ds18b20Sensors.getAddress(enclosureThermometer, 0);
            printAddress(enclosureThermometer);
            ds18b20Sensors.setResolution(enclosureThermometer, DallasSensorResolution);
            Serial.printlnf("Device 0 Resolution: %d", ds18b20Sensors.getResolution(enclosureThermometer));
            ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); //requête de lecture
            insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
            Serial.printlnf("Test device 0 enclosureThermometer = %f", insideTempC);

        } else if (ds18b20Count == 2){
            Serial.println("Configuration de 2 ds18b20");

            ds18b20Sensors.getAddress(enclosureThermometer, 0); // capteur Index 0
            printAddress(enclosureThermometer);
            ds18b20Sensors.setResolution(enclosureThermometer, DallasSensorResolution);
            Serial.printlnf("Device 0 Resolution: %d", ds18b20Sensors.getResolution(enclosureThermometer));
            ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); //requête de lecture
            insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
            Serial.printlnf("Test device 0 enclosureThermometer = %f", insideTempC);

            ds18b20Sensors.getAddress(outsideThermometer, 1); // capteur Index 1
            printAddress(outsideThermometer);
            ds18b20Sensors.setResolution(outsideThermometer, DallasSensorResolution);
            Serial.printlnf("Device 1 Resolution: %d", ds18b20Sensors.getResolution(outsideThermometer));
            ds18b20Sensors.requestTemperaturesByAddress(outsideThermometer); //requête de lecture
            outsideTempC = ds18b20Sensors.getTempC(outsideThermometer);
            Serial.printlnf("Test device 1 outsideThermometer = %f", outsideTempC);

            pushToPublishQueue(evTempInterne, (int) insideTempC, now);
            pushToPublishQueue(evTempExterne, (int) outsideTempC, now);
        }
        Serial.println();
        delay(2000UL);
}

void readAllSensors() {
    digitalWrite(led, LOW);
    now = millis();
    readDS18b20temp();
    ReadDistance_MB7389();
    digitalWrite(led, HIGH);

    // Publication de l'état de la pompe s'il y a eu changement
    if (PumpCurrentState != PumpOldState){
      PumpOldState = PumpCurrentState;
      if (PumpCurrentState == true){
        pumpEvent = evPompe_T1;
      } else {
        pumpEvent = evPompe_T2;
      }
      pushToPublishQueue(pumpEvent, PumpCurrentState, changeTime);
    }
// Pour permettre la modification de maxPublishDelay par le nuage
    maxPublishDelay = maxPublishInterval * minute;

// Publie au moins une fois à tous les "maxPublishDelay" millisecond
    now = millis();
    if (now - lastPublish > maxPublishDelay){
            lastPublish = now;
            pushToPublishQueue(evUS100Distance, (int)(dist_mm / numReadings), now);
            if (ds18b20Count == 1){
                pushToPublishQueue(evTempInterne, (int)prev_TempInterne, now);
            } else if (ds18b20Count == 2){
                pushToPublishQueue(evTempInterne, (int)prev_TempInterne, now);
                pushToPublishQueue(evTempExterne, (int)prev_TempExterne, now);
            }
            samplingInterval = slowSampling;   // Les mesure sont stable, réduire la fréquence de mesure.
        }

// Synchronisation du temps avec Particle Cloud une fois par jour
    if (millis() - lastSync > unJourEnMillis) {
        Particle.syncTime();
        lastSync = millis();
    }
// Publié les événements se trouvant dans le buffer
    if(buffLen > 0){
        Serial.printlnf("BufferLen = %u, Cloud = %s", buffLen, (Particle.connected() ? "true" : "false"));
        bool success = publishQueuedEvents();
        Serial.printlnf("Publishing = %u, Status: %s", readPtr - 1, (success ? "Fait" : "Pas Fait"));
    }
//    Serial.println("call readDS18b20temp");
    // killTime(samplingInterval); // Maintient la boucle sur une base de 1 second
}

// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
void ReadDistance_US100(){
    int currentReading;
    Serial1.flush();                                // clear receive buffer of serial port
    Serial1.write(0X55);                            // trig US-100 begin to measure the distance
    delay(100);                                     // delay 100ms to wait result
    if(Serial1.available() >= 2)                    // when receive 2 bytes
    {
        HighLen = Serial1.read();                   // High byte of distance
        LowLen  = Serial1.read();                   // Low byte of distance
        currentReading = HighLen*256 + LowLen;          // Combine the two bytes
//        Len_mm  = (HighLen*256 + LowLen)/25.4;    // Calculate the distance in inch
        if((currentReading > 1) && (currentReading < maxRangeUS100)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
            dist_mm = AvgDistReading(currentReading); // Average the distance readings
            Serial.printlnf("Dist.: %dmm, now= %d, lastPublish= %d, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, WiFi.RSSI());
            if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
                    lastPublish = now;                               // reset the max publish delay counter.
                    pushToPublishQueue(evUS100Distance, (int)(dist_mm / numReadings), now);
                    prev_dist_mm = dist_mm;
                    samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
                }

        } else {
            /*Particle.publish("Hors portée: ","9999",60,PRIVATE);*/
            pushToPublishQueue(evOutOfRange, 9999, now);
            Serial.print("Hors portée: ");             // output distance to serial monitor
            Serial.print(currentReading, DEC);
            Serial.println("mm ");
            delay(2000);
        }
    } else {
        Serial.println("Données non disponible");
    }
}

// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
void ReadDistance_MB7389(){
    int lastReading = MB7389latestReading;
    Serial.printlnf("MB7389latestReading: %d", MB7389latestReading);
    if((lastReading > 1) && (lastReading < maxRangeMB7389)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
        dist_mm = AvgDistReading(lastReading); // Average the distance readings
        Serial.printlnf("Dist.: %dmm, now= %d, lastPublish= %d, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, WiFi.RSSI());
        if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
                lastPublish = now;                               // reset the max publish delay counter.
                pushToPublishQueue(evMB7389Distance, (int)(dist_mm / numReadings), now);
                prev_dist_mm = dist_mm;
                samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
            }

    } else {
        pushToPublishQueue(evOutOfRange, 9999, now);
        Serial.print("Hors portée: ");             // output distance to serial monitor
        Serial.print(lastReading, DEC);
        Serial.println("mm ");
    }
 }

// This routine read the temperature from the US-100 sensor.
// Note: A varue of 45 MUST be substracted from the readings to obtain real temperature.
void Readtemp_US100(){
    int Temp45 = 0;
    Serial1.flush();       // clear receive buffer of serial port
    Serial1.write(0X50);   // trig US-100 begin to measure the temperature
    delay(200);            // delay 100ms to wait result
    if(Serial1.available() >= 1)            //when receive 1 bytes
    {
        Temp45 = Serial1.read();     //Get the received byte (temperature)
        if((Temp45 > 1) && (Temp45 < 130))   //the valid range of received data is (1, 130)
        {
            TempUS100 = AvgTempReading(Temp45 - 45);
            Serial.printlnf("Temp.:%dC now= %d, lastPublish= %d",  (int)(TempUS100/ numReadings), now, lastPublish);
            if (abs(TempUS100 - prev_TempUS100) > minTempChange){         // Publish event in case of a change in temperature
                    lastPublish = now;                               // reset the max publish delay counter.
                    pushToPublishQueue(evUS100Temperature, (int)(TempUS100/ numReadings), now);
                    prev_TempUS100 = TempUS100;
                    samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
                }
        }
    }
}

// Return a running average of the last "numReadings" distance reading to filter noise.
int AvgDistReading(int thisReading){
    long AvgDist = 0;
    for (int i = 1; i < numReadings; i++){
        allDistReadings[i-1] = allDistReadings[i]; //Shift all readings
        Serial.print(allDistReadings[i-1]);
        Serial.print(" ");
       AvgDist += allDistReadings[i-1]; //Total of readings except the last one
    }
    allDistReadings[numReadings-1] = thisReading; //Current reading in the last position
    Serial.print(allDistReadings[numReadings-1]);
    Serial.print("   ");
    AvgDist += thisReading; //including the last one
    Serial.print(" AvgDist= ");
    Serial.println(AvgDist / numReadings);
    return (AvgDist); // Avg sera divisé par numReadings au moment de la publication
}

// This routine return a running average of the last "numReadings" temperature reading to filter noise.
int AvgTempReading(int thisReading){
    long AvgTemp = 0;
    for (int i = 1; i < numReadings; i++){
        allTempReadings[i-1] = allTempReadings[i]; //Shift all readings
        Serial.print(allTempReadings[i-1]);
        Serial.print(" ");
        AvgTemp += allTempReadings[i-1]; //Total of readings except the last one
    }
    allTempReadings[numReadings - 1] = thisReading; //Current reading in the last position
    Serial.print(allTempReadings[numReadings-1]);
    Serial.print("   ");
    AvgTemp += thisReading; //total including the last one
    Serial.print(" AvgTemp= ");
    Serial.println(AvgTemp);
    return (AvgTemp); // Avg sera divisé par numReadings au moment de la publication
}

double readDS18b20temp(){
    float insideTempC;
    float outsideTempC;
    int i;
    int maxTry = 3;
    Particle.process();
    if (ds18b20Count > 0){
        if (ds18b20Count > 1){
            // Un capteur à l'intérieur du boitier et un à l'extérieur
            Serial.printlnf("lecture de 2 capteurs");

            ds18b20Sensors.requestTemperaturesByIndex(0); //requête de lecture
            for (i = 0; i < maxTry; i++){
                insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
                if (isValidDs18b20Reading(insideTempC)) break;
            }
            if (isValidDs18b20Reading(insideTempC)){
                // Si la mesure est valide
                if (abs(insideTempC - prev_TempInterne) >= 1){
                    // Publier s'il y a eu du changement
                    pushToPublishQueue(evTempInterne, (int) insideTempC, now);
                    prev_TempInterne = insideTempC;
                }
                Serial.printlnf("DS18b20 interne: %f, try= %d", insideTempC, i + 1);
                validTempInterne = true;
            } else {
                Serial.printlnf("DS18B20 interne: Erreur de lecture");
                validTempInterne = false;
                insideTempC = 99;
            }

            ds18b20Sensors.requestTemperaturesByIndex(1); //requête de lecture
            for (i = 0; i < maxTry; i++){
                outsideTempC = ds18b20Sensors.getTempC(outsideThermometer); // 5 tentatives de lecture au maximum
                if (isValidDs18b20Reading (outsideTempC)) break;
            }
            if (isValidDs18b20Reading (outsideTempC)){
                // Si la mesure est valide
                if (abs(outsideTempC - prev_TempExterne) >= 1){
                    // Publier s'il y a eu du changement
                    pushToPublishQueue(evTempExterne, (int) outsideTempC, now);
                    prev_TempExterne = outsideTempC;
                }
                Serial.printlnf("DS18b20 externe: %f, try= %d", outsideTempC, i + 1);
                validTempExterne = true;
            } else {
                // Si la measure est invalide
                Serial.printlnf("DS18B20 externe: Erreur de lecture");
                validTempExterne = false;
                outsideTempC = 99;
            }

            delay(100UL);
            return (outsideTempC);

         } else {
            // Un capteur à l'intérieur du boitier seulement
            Serial.printlnf("lecture de 1 capteur");
            /*ds18b20Sensors.requestTemperaturesByAddress(enclosureThermometer); //requête de lecture*/
            ds18b20Sensors.requestTemperaturesByIndex(0); //requête de lecture
            for (i = 0; i < 5; i++){
                insideTempC = ds18b20Sensors.getTempC(enclosureThermometer);
                if (isValidDs18b20Reading(insideTempC)){
                    break;
                }
            }
            if (isValidDs18b20Reading(insideTempC)){
                if (abs(insideTempC - prev_TempInterne) >= 1){
                    pushToPublishQueue(evTempInterne, (int) insideTempC, now);
                    prev_TempInterne = insideTempC;
                }
                Serial.printlnf("DS18b20 interne: %f", insideTempC);
                validTempInterne = true;
            } else {
                Serial.printlnf("DS18B20 interne: Erreur de lecture");
                validTempInterne = false;
                insideTempC = 99;
            }
            return (insideTempC);
         }

    }
}

bool isValidDs18b20Reading(float reading){
    if (reading > -127 && reading < 85){
        return true;
    } else {
        return false;
    }
}
// Imprémentation d'un thermostat simple ON/OFF
// La routine ne fonctionne que si un capteur de température interne est trouvé
int simpleThermostat(double setPoint){
    Particle.process();
    // executer la fonction de thermostat si on a un capteur de température
    if (ds18b20Count > 1){
        // executer la fonction de thermostat si la température interne est valide
        if (validTempInterne == true){
            if (prev_TempInterne < (setPoint - 0.5)){
                HeatingPower =  256 * MaxHeatingPowerPC /100;
            } else if (prev_TempInterne > (setPoint + 0.5)){
                HeatingPower =  0;
            }
        } else if(validTempInterne == false){
        // Si non mettre le chauffage à 1/4 de puissance pour éviter le gel.
            HeatingPower =  0.5 * (256 * MaxHeatingPowerPC /100); // Chauffage fixe au 1/4 de la puissance
        }

        analogWrite(heater, HeatingPower);
        Serial.printlnf("HeatingPower= %d", HeatingPower);
        if (HeatingPower != prev_HeatingPower){
            pushToPublishQueue(evHeating, HeatingPower, now);
            prev_HeatingPower = HeatingPower;
        }
    }
    return HeatingPower;
}

// Check the state of the valves position reedswitch
void CheckValvePos(){
    bool valveCurrentState;
    String stateStr;

    for (int i=0; i <= 3; i++) {
        valveCurrentState = digitalRead(ValvePos_pin[i]);
        if (ValvePos_state[i] != valveCurrentState)
        {
            delay(debounceDelay);  // Debounce time
            // time_t time = Time.now();
            valveCurrentState = digitalRead(ValvePos_pin[i]);
            ValvePos_state[i] = valveCurrentState;
            now = millis();
            if (valveCurrentState == true){
                stateStr = "Ouvert";
            } else {
                stateStr = "Fermé";
            }
            Serial.println(DomainName + DeptName + eventName[ValvePos_Name[i]] + ": " + stateStr );
            pushToPublishQueue(ValvePos_Name[i], valveCurrentState, now);
        }
    }
}

// Active ou désactive le relais SSR
int toggleRelay(String command) {
    if (command=="on" || command=="1") {
        RelayState = HIGH;
        digitalWrite(ssrRelay, RelayState);
        /*Particle.publish("Relais", "on", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, RelayState, now);
        return 1;
    }
    else if (command=="off" || command=="0") {
        RelayState = LOW;
        digitalWrite(ssrRelay, RelayState);
        /*Particle.publish("Relais", "off", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, RelayState, now);
         return 0;
    }
    else {
        return -1;
    }
}

// Pour modifier l'interval de publication par défault
int setPublishInterval(String command){
    unsigned long newInterval;
    newInterval = command.toInt();
    if (newInterval > 0){
        maxPublishInterval = command.toInt();
        return 1;
    } else {
        return -1;
    }
}

// Pour reseter le capteur à distance au besoin
int remoteReset(String command) {
    if(command == "device"){
        System.reset();
    } else if (command == "serialNo"){
        NewGenTimeStamp = Time.now();
        Serial.printlnf("Nouvelle génération de no de série", NewGenTimeStamp);
        /*
        Note:
        La position de eTime et de eData sont inversé dans ce cas-ci
        eTime est de type int et ne peux contenir le timestamp
        */
        /*noSerie = 0; // reset serial no to 0*/
        pushToPublishQueue(evNewGenSN, millis(), NewGenTimeStamp);
    }
}

// Permet de demander un of des événements manquants
bool replayEvent(String command){
    return true;
}

// Formattage standard pour les données sous forme JSON
String makeJSON(unsigned long numSerie, int eData, unsigned long eTime, String eName){
    /*char* formattedEName = String::format("%c", eName.c_str());*/
    sprintf(publishString,"{\"noSerie\": %u,\"eTime\": %lu,\"eData\":%d,\"eName\": \"%s\"}", numSerie, eTime, eData, eName.c_str());
    Serial.println(publishString);
    return publishString;
}

// Publie les événement et gère les no. de série et le stockage des événements
bool pushToPublishQueue(int eventNamePtr, int eData, unsigned long eTime){
  struct Event thisEvent;
  thisEvent = {noSerie, eventNamePtr, eData, eTime};
  Serial.println(">>>> pushToPublishQueue:::");
  writeEvent(thisEvent); // Pousse les données dans le buffer circulaire
  noSerie++;
  if (noSerie > 65535){
     noSerie = 0;
    }
}

// Publie un événement stocké en mémoire
bool publishQueuedEvents(){
    bool publishSuccess = false;
    struct Event thisEvent = {};
    Serial.println("<<<< publishQueuedEvents:::");
    thisEvent = peekEvent();
    if (sizeof(thisEvent) == 0){
        return publishSuccess; // Rien à publié
    }
    if(Particle.connected()){
        if (connWasLost){
          connWasLost =  false;
          delay(2000); // Gives some time to avoid loosing events
        }
        publishSuccess = Particle.publish(DomainName + DeptName + eventName[thisEvent.namePtr], makeJSON(thisEvent.noSerie, thisEvent.eData, thisEvent.eTime, DomainName + DeptName + eventName[thisEvent.namePtr]), 60, PRIVATE);
        if (publishSuccess){
        readEvent(); // Avance le pointeur de lecture
        }
    } else {
     connWasLost = true;
    }
  return publishSuccess;
}

// Sauvegarde d'un événement en mémoire
bool writeEvent(struct Event thisEvent){
  if (readPtr == (writePtr + 1) % buffSize){
    return false; // Le buffer est plein, ne rien sauver.
  }
  eventBuffer[writePtr] = thisEvent;
  writePtr = (writePtr + 1) % buffSize; // avancer writePtr
  buffLen = writePtr - readPtr;
  //pour debug
  Serial.print("-------> " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  Serial.printlnf(": writeEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                     writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return true;
}

// Lecture d'un événement en mémoire
struct Event readEvent(){
  struct Event thisEvent = {};
  if (readPtr == writePtr){
    Serial.printlnf("<------- readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, *** buffer vide ***",
                                      writePtr, readPtr, buffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[readPtr];
  readPtr = (readPtr + 1) % buffSize;
  buffLen = writePtr - readPtr;
  //pour debug
  Serial.print("<------- " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  Serial.printlnf(": readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}

// Lecture d'un événement en mémoire sans avancé le pointeur
struct Event peekEvent(){
  struct Event thisEvent = {};
  if (readPtr == writePtr){
    Serial.printlnf(" ------- peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, *** buffer vide ***",
                                      writePtr, readPtr, buffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[readPtr];
  //buffLen = writePtr - readPtr;
  //pour debug
  Serial.print(" ------- " + eventName[thisEvent.namePtr]);
  Serial.printlnf(": peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}
