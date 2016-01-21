// This #include statement was automatically added by the Particle IDE.
/*#include "spark-dallas-temperature.h"
#include "OneWire.h"*/


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
De cette façon les 100 à 150 dernier événements seront conservé en cas de perte de réseau.
Afin d'optimiser l'espace requis, les données seront de type entier (integer ou long).
Les nom d'événements sont remplacer par leur index dans l'array eventName.
Les données sont organisé dans une structure "Event" et stocké dans un array de ces structures.
9 bytes sont utilisés par événement. On peut donc sauver environ 4000 / 9 = 440 événements.
Les événements sont stocké au fur et à mesure de leur production. Il seront publié séquenciellement
indépendamment de leur production.
*/

/*SYSTEM_THREAD(ENABLED);*/
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));


#define minute 60000UL            // 60000 millisecond per minute
#define second 1000UL             // 1000 millisecond per sesond
#define unJourEnMillis (24 * 60 * 60 * second)
#define baseLoopTime  206      //Estimated loop time in millisecond
#define debounceDelay 50    // Debounce time for valve position readswitch
#define fastSampling  1000UL   // in milliseconds
#define slowSampling  2000UL   // in milliseconds
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
#define evValve1_OpenSensorState 5      //Active low
#define evValve1_CloseSensorState 6     //Active low
#define evValve2_OpenSensorState 7      //Active low
#define evValve2_CloseSensorState 8     //Active low
#define evRelais 9
#define evVacuum 10
#define evDebit 11
#define evVolume 12
#define evPressionAtmos 13
#define evTempInterne 14
#define evTempExterne 15
#define evHeating 16
#define evMB7389Distance 17
#define evNewGenSN 18
#define evBootTimestamp 19


// Variables lié aux événements
String eventName[] = {
    "sonde/Pompe/T1",
    "sonde/Pompe/T2",
    "sonde/US100/Distance",
    "sonde/US100/Temperature",
    "sonde/Hors portée: ",
    "sonde/Valve1/OpenSensor",
    "sonde/Valve1/CloseSensor",
    "sonde/Valve2/OpenSensor",
    "sonde/Valve2/CloseSensor",
    "sortie/Relais",
    "sonde/Vacuum",
    "sonde/flowmeter/Débit",
    "calcul/Volume",
    "sonde/Pression Atmosphérique",
    "sonde/DS18B20/Température interne",
    "sonde/DS18B20/Température externe",
    "sortie/Chauffage boitier",
    "sonde/MB7389/Distance",
    "NewGenSN",
    "Boot timestamp"
    };

// Structure définissant un événement
typedef struct Event{
  uint16_t noSerie; // Le numéro de série est généré automatiquement
  uint32_t eGenTS; // Timestamp du début d'une série de noSerie.
  uint32_t eTime; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int16_t eData;   // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
                   // multiplié d'abord la donnée par un facteur (1000 par ex.) en convertir en entier.
                   // Il suffira de divisé la données au moment de la réception de l'événement.

};
// Variable relié à l'opération du buffer circulaire
const int buffSize = 250; // Nombre max d'événements que l'on peut sauvegarder
retained unsigned int buffLen = 0;
retained unsigned int writePtr = 0;
retained unsigned int readPtr = 0;
retained unsigned int replayPtr = 0;
unsigned int replayBuffLen = 0;
retained struct Event eventBuffer[buffSize];

// Name space utilisé pour les événements
// DomainName/DeptName/FunctionName/SubFunctionName/ValueName
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
int ValvePos_pin[] = {A2, A3, A4, A5};
bool ValvePos_state[] = {true, true, true, true};
int ValvePos_Name[] = {evValve1_OpenSensorState, evValve1_CloseSensorState, evValve2_OpenSensorState, evValve2_CloseSensorState};

// Variables liés à la mesure de Température
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
int TempUS100 = 0;
int prev_TempUS100 = 0;
int prev_TempExterne = 99;
int prev_TempInterne = 99;
int allTempReadings[numReadings];
/*OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature ds18b20Sensors(&oneWire);
DeviceAddress enclosureThermometer, outsideThermometer;*/
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
unsigned long lastPublish;
unsigned long now;
unsigned long lastSync = millis();
unsigned int samplingInterval = fastSampling;
unsigned long nextSampleTime = 0;
int maxPublishInterval = 2;
unsigned long maxPublishDelay = maxPublishInterval * minute;
unsigned long lastTime = 0UL;
time_t newGenTimestamp = Time.now();

// Variables liés aux publications
char publishString[buffSize];
retained uint16_t noSerie = 0; //Mettre en Backup RAM
int pumpEvent = 0;
bool connWasLost = false;

// Autre variables
/*bool MB7389Valid = false;
String Dist_MB7389Str;
int MB7389latestReading = 0;
const int R = 82;
const int CR = 13;*/


/*
// handler to receive the module name
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

/*// function to print a device address
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
}*/


void setup() {
// connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
    Serial.begin(115200);
    Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
    delay(3000UL); // Pour partir le moniteur série
// Scanning Wifi access point
    WiFiAccessPoint aps[20];
    int found = WiFi.scan(aps, 20);
    for (int i=0; i<found; i++) {
        WiFiAccessPoint& ap = aps[i];
        Serial.printlnf("SSID: %s, Security: %d, Channel: %d, RSSI: %d", ap.ssid, ap.security, ap.channel, ap.rssi);
        if (ap.ssid == "BoilerHouse"){
            WiFi.setCredentials("BoilerHouse", "Station Shefford");
        } else if (ap.ssid == "PumpHouse"){
            WiFi.setCredentials("PumpHouse", "Station Laporte");
        } else if (ap.ssid == "PL-Net"){
            Serial.printlnf("Setting credential for: %s", ap.ssid);
            WiFi.setCredentials("PL-Net", "calvin et hobbes");
        }
    }

// Enregistrement des fonctions et variables disponible par le nuage
    if (WiFi.ready()) {
            Serial.println("Enregistrement des variables et fonctions\n");
            Particle.variable("relayState", RelayState);
            /*Particle.variable("DS18B20Cnt", ds18b20Count);*/

            Particle.function("relay", toggleRelay);
            Particle.function("pubInterval", setPublishInterval);
            Particle.function("reset", remoteReset);
            Particle.function("replay", replayEvent);
    }

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

// Initialisation
    pinMode(led, OUTPUT);
    pinMode(ssrRelay, OUTPUT);
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
    time_t bootTime = Time.now();
    pushToPublishQueue(evBootTimestamp, millis(), bootTime);

    lastPublish = millis(); //Initialise le temps initial de publication
    changeTime = lastPublish; //Initialise le temps initial de changement de la pompe
}

/*
    Le capteur de distance MB7389 fonctionne en continu.
    Cette routine reçoit les données séries et met le résultats dans une variable
     Pour utilisation par la routine de measure
*/

/*void serialEvent1()
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
}*/

/*
    Boucle principale
*/
void loop(){
    if (millis() > nextSampleTime) {
        nextSampleTime = millis() + samplingInterval - 1;
        readAllSensors();
        /*simpleThermostat(HeatingSetPoint);*/
    }
    CheckValvePos();
    /*readDS18b20temp();*/
    Particle.process();
}

void readAllSensors() {
    digitalWrite(led, LOW); // Pour indiqué le début de la prise de mesure
    now = millis();
// ********* Appeler ici les fonctions de mesure pour le capteur concerné. ***********
//
// Les fonctions doivent mettre leur résultats dans la queue de publication avec la function
// pushToPublishQueue(). Si le cloud est dispomible les événements seront publiés plus loin
// dans la boucle
//
    /*Readtemp_US100();
    ReadDistance_US100();*/
//
    delay(100UL);
// ************************************************************************************
    digitalWrite(led, HIGH); // Pour indiqué la fin de la prise de mesure

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

// Publication au moins une fois à tous les "maxPublishDelay" millisecond
    now = millis();
    if (now - lastPublish > maxPublishDelay)
        {
            lastPublish = now;
            /*pushToPublishQueue(evVacuum, vacuumValue, now);*/
            /*samplingInterval = slowSampling;   // Les mesure sont stable, réduire la fréquence de mesure excepté pour les pompes.*/
        }

// Synchronisation du temps avec Particle Cloud une fois par jour
    if (millis() - lastSync > unJourEnMillis) {
        Particle.syncTime();
        lastSync = millis();
    }
// Publication des événements se trouvant dans le buffer
    if(buffLen > 0){
        Serial.printlnf("Buffer = %u, Cloud = %s", buffLen, (Particle.connected() ? "true" : "false")); // Pour debug
        bool success = publishQueuedEvents();
        Serial.printlnf("Publishing = %u, Status: %s", readPtr - 1, (success ? "Fait" : "Pas Fait")); // Pour debug
    } else if (replayBuffLen > 0){
        bool success = replayQueuedEvents();
        Serial.printlnf("replayBuffLen = %u, Replay = %u, Status: %s", replayBuffLen, replayPtr, (success ? "Fait" : "Pas Fait"));
    }
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

// Cette routine lit la température sur le capteur US-100.
// Note: La valeur 45 DOIT être soustraite pour obtenir la température réelle.
void Readtemp_US100(){
    int Temp45 = 0;
    Serial1.flush();                // S'assurer que le buffer du port serie 1 est vide.
    Serial1.write(0X50);            // Demander la lecture de température sur le US-100 en envoyant 50 (Hex)
    delay(100);                     // Attente du résult
    if(Serial1.available() >= 1)    // lors de la réception d'au moins 1 bytes
    {
        Temp45 = Serial1.read();     // Lire la température brut
        if((Temp45 > 1) && (Temp45 < 130))   // Vérifier si la valeur est acceptable
        {
            TempUS100 = AvgTempReading(Temp45 - 45); //Conversion en température réelle et filtrage
            Serial.printlnf("Temp.:%dC now= %d, lastPublish= %d",  (int)(TempUS100/ numReadings), now, lastPublish); // Pour debug
            if (abs(TempUS100 - prev_TempUS100) > minTempChange){    // Vérifier s'il y a eu changement depuis la dernière publication
                    lastPublish = now;                     // Remise à zéro du compteur de délais de publication
                    pushToPublishQueue(evUS100Temperature, (int)(TempUS100 / numReadings), now); //Publication
                    prev_TempUS100 = TempUS100;                      //
                    samplingInterval = fastSampling;   // Augmenter la vitesse d'échantillonnage puisqu'il y a eu changement
                }
        }
    }
}

// Filtre par moyenne mobile pour les distances
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgDistReading(int thisReading){
    long Avg = 0;
    for (int i = 1; i < numReadings; i++){
        allDistReadings[i-1] = allDistReadings[i]; //Shift all readings
       Avg += allDistReadings[i-1]; //Total of readings except the last one
    }
    allDistReadings[numReadings-1] = thisReading; //Current reading in the last position
    Avg += thisReading; //including the last one
    return (Avg); // Avg sera divisé par numReadings au moment de la publication
}

// Filtre par moyenne mobile pour les température
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgTempReading(int thisReading){
    long Avg = 0;
    for (int i = 1; i < numReadings; i++){
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
                stateStr = "/Open";
            } else {
                stateStr = "/Undefined";
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

// Pour resetter le capteur à distance au besoin
int remoteReset(String command) {
    /*Serial.println("Resetting...");*/
    if (command == "device"){
        System.reset();
// ou juste les numéros de série.
    } else if (command == "serialNo") {
        newGenTimestamp = Time.now();
        Serial.printlnf("Nouvelle génération de no de série maintenant.", newGenTimestamp);
        /*noSerie = 0;*/
        pushToPublishQueue(evNewGenSN, -1, millis());
    }
}

/*
typedef struct Event{
  uint16_t noSerie; // Le numéro de série est généré automatiquement
  uint32_t eSnGen; // Timestamp du début d'une série de noSerie.
  uint32_t eTime; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int16_t eData;   // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
*/
// Formattage standard pour les données sous forme JSON
String makeJSON(uint16_t numSerie, uint32_t eGenTS, uint32_t eTime, int eData, String eName){
    /*char* formattedEName = String::format("%c", eName.c_str());*/
    /*sprintf(publishString,"{\"noSerie\": %u,\"eTime\": %lu,\"eData\":%d,\"eName\": \"%s\"}", numSerie, eTime, eData, eName.c_str());*/
    sprintf(publishString,"{\"noSerie\": %u,\"eGenTS\": %lu,\"eTime\": %lu,\"eData\":%d,\"eName\": \"%s\"}", numSerie, eGenTS, eTime, eData, eName.c_str());
    Serial.printlnf ("makeJSON: %s",publishString);
    return publishString;
}

// Publie les événement et gère les no. de série et le stockage des événements
bool pushToPublishQueue(int eventNamePtr, int eData, uint32_t eTime){
  struct Event thisEvent;
  thisEvent = {noSerie, newGenTimestamp, eTime, eventNamePtr, eData};
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
    thisEvent = peekEvent(readPtr);
    if (sizeof(thisEvent) == 0){
        return publishSuccess; // Rien à publié
    }
    if(Particle.connected()){
        if (connWasLost){
          connWasLost =  false;
          delay(2000); // Gives some time to avoid loosing events
        }
        publishSuccess = Particle.publish(DomainName + DeptName + eventName[thisEvent.namePtr],
                                            makeJSON(thisEvent.noSerie, thisEvent.eGenTS, thisEvent.eTime, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr]), 60, PRIVATE);
        if (publishSuccess){
        readEvent(); // Avance le pointeur de lecture
        }
    } else {
     connWasLost = true;
    }
  return publishSuccess;
}

// Publie un événement stocké en mémoire
bool replayQueuedEvents(){
    bool publishSuccess = false;
    struct Event thisEvent = {};
    Serial.println("&&&& replayQueuedEvents:::");
    thisEvent = peekEvent(replayPtr);
    if (sizeof(thisEvent) == 0){
        return publishSuccess; // Rien à publié
    }
    if(Particle.connected()){
        if (connWasLost){
          connWasLost =  false;
          delay(2000); // Gives some time to avoid loosing events
        }
        publishSuccess = Particle.publish(DomainName + "replay/" + eventName[thisEvent.namePtr],
                                            makeJSON(thisEvent.noSerie, thisEvent.eGenTS, thisEvent.eTime, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr]), 60, PRIVATE);
                                            //makeJSON(uint16_t numSerie, uint32_t eGenTS, uint32_t eTime, int eData, String eName){
        if (publishSuccess){
        replayReadEvent(); // Avance le pointeur de lecture
        }
    } else {
     connWasLost = true;
    }
  return publishSuccess;
}

// Permet de demander un replay des événements manquants
// Initialise les paramètres pour la routine replayQueuedEvents()
int replayEvent(String command){
    int targetSerNo = command.toInt();
    Serial.printlnf("?????? Demande de replay Event no: %d,  writePtr= %u, readPtr= %u, replayBuffLen= %u",
                    targetSerNo, writePtr, readPtr, replayBuffLen);

    if (replayBuffLen > 0){
        return -1; // La requête précédente n'est pas encore complête - Attendre
    }
    // Validation de la demande
    if (targetSerNo >= 0){ // Le numéro recherché doit être plus grand que 0
        // Validation
        if (targetSerNo >= noSerie){ // et plus petit que le numéro de série courant
            return false;
        }
        if ((noSerie - targetSerNo) > buffSize){ // Il y a 300 événement au maximum dans le buffer
            /*targetSerNo = noSerie - buffSize;*/
            return false;
        }
        // Calcul de la position de l'événement dans le buffer
        int tmpPtr = readPtr - (noSerie - targetSerNo);  // Position dans le buffer du premier événement à faire un playback
        if (tmpPtr < 0){
            replayPtr =  tmpPtr + buffSize;
        } else {
            replayPtr =  tmpPtr;
        }
        // calcul de la longueur du replayBuffLen
        if ((readPtr - replayPtr) < 0){
            replayBuffLen = readPtr - replayPtr + buffSize;
        } else {
            replayBuffLen = readPtr - replayPtr;
        }
        Serial.printlnf("?????? Accepté pour replay Event no: %d, ReplayPtr= %u, writePtr= %u, readPtr= %u, replayBuffLen= %u, No de série courant= %u",
                        targetSerNo, replayPtr, writePtr, readPtr, replayBuffLen, noSerie);
        return 0;
    } else {
        return -1;
    }
}

// Sauvegarde d'un événement en mémoire
bool writeEvent(struct Event thisEvent){
  if (readPtr == (writePtr + 1) % buffSize){
    return false; // Le buffer est plein, ne rien sauver.
  }
  eventBuffer[writePtr] = thisEvent;
  writePtr = (writePtr + 1) % buffSize; // avancer writePtr
  if ((writePtr - readPtr) < 0){
      buffLen = writePtr - readPtr + buffSize;
  } else {
      buffLen = writePtr - readPtr;
  }
   //pour debug
  Serial.print("W------> " + DomainName + DeptName + eventName[thisEvent.namePtr]);
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
  if ((writePtr - readPtr) < 0){
      buffLen = writePtr - readPtr + buffSize;
  } else {
      buffLen = writePtr - readPtr;
  }
  //pour debug
  Serial.print("<R------ " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  Serial.printlnf(": readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}

// Lecture d'un événement en mémoire
struct Event replayReadEvent(){
  struct Event thisEvent = {};
  if (replayPtr == writePtr){
    Serial.printlnf("\n<--&&--- replayReadEvent:: writePtr= %u, replayPtr= %u, replayBuffLen= %u, *** replay buffer vide ***",
                                      writePtr, replayPtr, replayBuffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[replayPtr];
  replayPtr = (replayPtr + 1) % buffSize; // increment replay pointer
  if ((writePtr - replayPtr) < 0){
      replayBuffLen = writePtr - replayPtr + buffSize;
  } else {
      replayBuffLen = writePtr - replayPtr;
  }

  //pour debug
  Serial.print("<------- " + DomainName + "replay/" + eventName[thisEvent.namePtr]);
  Serial.printlnf(": readEvent:: writePtr= %u, replayPtr= %u, replayBuffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, replayPtr, replayBuffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}

// Lecture d'un événement en mémoire sans avancé le pointeur
struct Event peekEvent(uint16_t peekReadPtr){
  struct Event thisEvent = {};
  if (peekReadPtr == writePtr){
    Serial.printlnf(" ------- peekEvent:: writePtr= %u, peekReadPtr= %u, buffLen= %u, *** buffer vide ***",
                                      writePtr, peekReadPtr, buffLen);
    return thisEvent; // événement vide
  }
  thisEvent = eventBuffer[peekReadPtr];
   //pour debug
  Serial.print(" ------- " + eventName[thisEvent.namePtr]);
  Serial.printlnf(": peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, peekReadPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}

// check buffer pointers and length consistency
void checkPtrState(){
    uint16_t tmp;
// check readPtr
    tmp = readPtr;
    if (readPtr > buffSize) {
        if (writePtr < buffSize){
            readPtr = writePtr;
        }
    }
    Serial.printlnf("Checked readPtr --> was:%u, now:%u", tmp, readPtr);
// check writePtr
    tmp = writePtr;
    if (writePtr > buffSize) {
        if (readPtr < buffSize){
            writePtr = readPtr;
        }
    }
    Serial.printlnf("Checked writePtr --> was:%u, now:%u", tmp, writePtr);
// check buffLen
    tmp = buffLen;
    if (buffLen > buffSize){
        // inconsistance: reset pointers to 0
        readPtr  = 0;
        writePtr = 0;
    }
// calcul le buffLen
    if ((writePtr - readPtr) < 0){
        buffLen = writePtr - readPtr + buffSize;
    } else {
        buffLen = writePtr - readPtr;
    }
    Serial.printlnf("Checked buffLen --> was:%u, now:%u", tmp, buffLen);
}
