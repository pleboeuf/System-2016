// This #include statement was automatically added by the Particle IDE.
#include "spark-dallas-temperature.h"
#include "OneWire.h"

//SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));


// Paramètres pour la compilation conditionnelle
#define US100 1
#define MB7389 2
#define DISTANCESENSOR MB7389   //Pour compilation conditionnelle du serial handler: US100. MB7389, None
#define PUMPMOTORDETECT false   //Pour compilation conditionnelle de la routin e d'interruption
#define HASDS18B20SENSOR true   //Pour le code spécifique au captgeur de température DS18B20
#define HASHEATING true         //Pour le chauffage du boitier
//

// General definitions
#define minute 60000UL            // 60000 millisecond per minute
#define second 1000UL             // 1000 millisecond per sesond
#define unJourEnMillis (24 * 60 * 60 * second)
#define debounceDelay 50    // Debounce time for valve position readswitch
#define fastSampling  1000UL   // in milliseconds
#define slowSampling  2000UL    // in milliseconds
#define numReadings 10           // Number of readings to average for filtering
#define minDistChange 2.0 * numReadings      // Minimum change in distance to publish an event (1/16")
#define minTempChange 0.5 * numReadings      // Minimum temperature change to publish an event
#define maxRangeUS100 2500 // Distance maximale valide pour le captgeur
#define maxRangeMB7389 4999 // Distance maximale valide pour le captgeur
#define ONE_WIRE_BUS D4 //senseur sur D4
#define DallasSensorResolution 9 // Résolution de lecture de température
#define MaxHeatingPowerPercent 50 // Puissance maximale appliqué sur la résistance de chauffage
#define HeatingSetPoint 30 // Température cible à l'intérieur du boitier


// Nom des indices du tableau eventName
#define evPompe_T1 0
#define evPompe_T2 1
#define evDistance 2
#define evUS100Temperature 3
#define evOutOfRange 4
#define evValve1_OpenSensorState 5
#define evValve1_CloseSensorState 6
#define evValve2_OpenSensorState 7
#define evValve2_CloseSensorState 8
#define evRelais 9
#define evVacuum 10
#define evFlowmeterFlow 11
#define evFlowmeterVolume 12
#define evAtmosPressure 13
#define evEnclosureTemp 14
#define evAmbientTemp 15
#define evHeatingPowerLevel 16
#define evNewGenSN 17
#define evBootTimestamp 18

// Table des nom d'événements
String eventName[] = {
  "pump/T1",  // Pump state. Pump start
  "pump/T2", // Pump state. Pump stop
  "sensor/level", // Tank level. Post processing required for display
  "sensor/sensorTemp", // Temperature read on the US100 senson
  "sensor/outOfRange ", // Level sensor is out of max range
  "sensor/openSensorV1", // Valve 1 open position sensor state. Active LOW
  "sensor/closeSensorV1", // Valve 1 close position sensor state. Active LOW
  "sensor/openSensorV2", // Valve 2 open position sensor state. Active LOW
  "sensor/closeSensorV2", // Valve 2 close position sensor state. Active LOW
  "output/ssrRelayState", // Output ssrRelay pin state. Active LOW
  "sensor/vacuum", // Vacuum rsensor eading
  "sensor/flowmeterValue", // Flowmeter reading. Not used
  "computed/flowmeterVolume", // Volume computed from flowmert readings. Not used
  "sensor/atmPressure", // Atmospheric pressure
  "sensor/enclosureTemp", // Temperature inside device enclosure.
  "sensor/ambientTemp", // Ambient temperature read by remote probe.
  "output/enclosureHeating", // Value of PWM output to heating resistor.
  "device/NewGenSN", // New generation of serial numbers for this device
  "device/boot" // Device boot or reboot timestamp
  };

// Structure définissant un événement
typedef struct Event{
  uint16_t noSerie; // Le numéro de série est généré automatiquement
  uint32_t timeStamp; // Timestamp du début d'une génération de noSerie.
  uint32_t timer; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int16_t eData;   // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
                   // multiplié d'abord la donnée par un facteur (1000 par ex.) en convertir en entier.
                   // Il suffira de divisé la données au moment de la réception de l'événement.
};

// Variable relié à l'opération du buffer circulaire
const int buffSize = 190; // Nombre max d'événements que l'on peut sauvegarder
retained struct Event eventBuffer[buffSize];
retained unsigned int buffLen = 0;
retained unsigned int writePtr = 0;
retained unsigned int readPtr = 0;
retained unsigned int replayPtr = 0;
unsigned int replayBuffLen = 0;
retained unsigned int savedEventCount = 0;

// Name space utilisé pour les événements
// DomainName/DeptName/FunctionName/SubFunctionName/ValueName
String DomainName = "";
String DeptName = "";

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
int ValvePos_Name[] = {evValve1_OpenSensorState, evValve1_CloseSensorState, evValve2_OpenSensorState, evValve2_CloseSensorState};

// Variables liés à la mesure de Température
unsigned int US100HighByte = 0;
unsigned int US100LowByte  = 0;
int TempUS100 = 0;
int prev_TempUS100 = 0;
int allTempReadings[numReadings];

#if HASDS18B20SENSOR
  OneWire oneWire(ONE_WIRE_BUS);
  DallasTemperature ds18b20Sensors(&oneWire);
  DeviceAddress enclosureThermometer, outsideThermometer;
  int ds18b20Count = 0;
  bool validTempExterne = false;
  bool validEnclosureTemp = false;
  int prev_EnclosureTemp = 99;
  int prev_TempExterne = 99;
#endif

int HeatingPower = 0;
int prev_HeatingPower = 64;

// Variables liés à la mesure de distance
int dist_mm  = 0;
int prev_dist_mm = 0;
int allDistReadings[numReadings];

// Variables liés au temps
unsigned long lastPublish = millis();
unsigned long now;
unsigned long lastRTCSync = millis();
unsigned int samplingInterval = fastSampling;
unsigned long nextSampleTime = 0;
unsigned long lastTime = 0UL;

// Variables liés aux publications
char publishString[buffSize];
retained time_t newGenTimestamp = 0;
retained uint16_t noSerie = 0; //Mettre en Backup RAM
int maxPublishInterval = 2;
unsigned long maxPublishDelay = maxPublishInterval * minute;
int pumpEvent = 0;
bool connWasLost = false;


// Variable associé au capteur MB7389 seulement
#if DISTANCESENSOR == MB7389
  bool MB7389Valid = false;
  String Dist_MB7389Str;
  int MB7389latestReading = 0;
  const int R = 82;
  const int CR = 13;
#endif


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


#if PUMPMOTORDETECT
/*
  Attach interrupt handler to pin A0 to monitor pump Start/Stop
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
#endif



#if HASDS18B20SENSOR
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
#endif



void setup() {
// connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
    Serial.begin(115200); // Pour débug
    Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud

    #if DISTANCESENSOR == MB7389
      Serial1.halfduplex(true); // Ce capteur envoie seulement des données sésie dans une seule direction
                                // On peut utiliser la pin RX pour contrôler son fonctionnement
                                // RX = LOW pour arrêter le capteur, RX = HIGH pour le démarrer
    #endif
    delay(300UL); // Pour partir le moniteur série pour débug

// Enregistrement des fonctions et variables disponible par le nuage
    Serial.println("Enregistrement des variables et fonctions\n");
    Particle.variable("relayState", RelayState);
    #if HASDS18B20SENSOR
        Particle.variable("DS18B20Cnt", ds18b20Count);
    #endif
    Particle.function("relay", toggleRelay);
    Particle.function("set", remoteSet);
    Particle.function("reset", remoteReset);
    Particle.function("replay", replayEvent);

    delay(3000); // Pour partir le moniteur série

// Attendre la connection au nuage
    Serial.println("En attente... ");
    if (waitFor(Particle.connected, 10000)) {
      delay(1000);
      Serial.print(".");
    }
    if(Particle.connected()){
        Serial.println("Connecté au nuage. :)");
    } else {
        Serial.println("Pas de connexion au nuage. :( ");
    }

    delay(1000UL);
    if (savedEventCount == 0){ // If there's no accumulated event
      remoteReset("serialNo"); // the non
    }

// check that all buffer pointers are ok.
    checkPtrState();
    replayBuffLen = 0;
    replayPtr = readPtr;

// initialisation des capteurs de températures
    #if HASDS18B20SENSOR
        initDS18B20Sensors();
    #endif

// Initialisation des pin I/O
    pinMode(led, OUTPUT);
    pinMode(ssrRelay, OUTPUT);
    #if HASHEATING
        pinMode(heater, OUTPUT);
        HeatingPower =  256 * MaxHeatingPowerPercent / 100; // Valeur de PWM de chauffage
        analogWrite(heater, HeatingPower); //Désactiver le chauffage
    #endif
    digitalWrite(led, LOW);
    digitalWrite(ssrRelay, LOW);

    PumpCurrentState = digitalRead(A0);
    for (int i=0; i <= 3; i++) {
        pinMode(ValvePos_pin[i], INPUT_PULLUP);
    }

    for (int i = 0; i<=numReadings; i++){ // Init readings array
        allDistReadings[i] = 0;
        allTempReadings[i] = 0;
    }

    Time.zone(-4);
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);
    Particle.syncTime();
    pushToPublishQueue(evBootTimestamp, 0,  millis());

    lastPublish = millis(); //Initialise le temps initial de publication
    changeTime = lastPublish; //Initialise le temps initial de changement de la pompe
}


/*
    Le capteur de distance MB7389 fonctionne en continu.
    Cette routine reçoit les données séries et met le résultats dans une variable
     Pour utilisation par la routine de measure
*/

#if DISTANCESENSOR == MB7389
  void serialEvent1(){
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
          //Serial.println("Dist_MB7389Str= " + Dist_MB7389Str);
          return;
       }

  // Accumule des données
      if (MB7389Valid == true){
          Dist_MB7389Str += c;
      }
  }
#endif


/*
    Boucle principale
*/
void loop(){
  if (millis() > nextSampleTime) {
      nextSampleTime = millis() + samplingInterval - 1;
      readAllSensors();
      #if HASHEATING
        simpleThermostat(HeatingSetPoint);
      #endif
  }
  CheckValvePos();
  Particle.process();
}

// Read the sensors attached to the device
void readAllSensors() {
  digitalWrite(led, LOW); // Pour indiqué le début de la prise de mesure
  now = millis();

  #if HASDS18B20SENSOR
      readDS18b20temp();
  #endif

  #if DISTANCESENSOR == US100
      Readtemp_US100();
      ReadDistance_US100();
  #endif

  #if DISTANCESENSOR == MB7389
      ReadDistance_MB7389();
  #endif

  digitalWrite(led, HIGH); // Pour indiqué la fin de la prise de mesure

  #if PUMPMOTORDETECT
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
  #endif
// Pour permettre la modification de maxPublishDelay par le nuage
  maxPublishDelay = maxPublishInterval * minute;

// Publie au moins une fois à tous les "maxPublishDelay" millisecond
  now = millis();
  if (now - lastPublish > maxPublishDelay){

    lastPublish = now;
    #if DISTANCESENSOR == US100
      pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
      pushToPublishQueue(evUS100Temperature, (int)(TempUS100/ numReadings), now);
    #endif

    #if DISTANCESENSOR == MB7389
      pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
    #endif

    #if HASDS18B20SENSOR
      if (ds18b20Count == 1){
          pushToPublishQueue(evEnclosureTemp, (int)prev_EnclosureTemp, now);
      } else if (ds18b20Count == 2){
          pushToPublishQueue(evEnclosureTemp, (int)prev_EnclosureTemp, now);
          pushToPublishQueue(evAmbientTemp, (int)prev_TempExterne, now);
      }
    #endif
    samplingInterval = slowSampling;   // Les mesure sont stable, réduire la fréquence de mesure.
  }

// Synchronisation du temps avec Particle Cloud une fois par jour
  if (millis() - lastRTCSync > unJourEnMillis) {
      Particle.syncTime();
      lastRTCSync = millis();
  }
// Publier les événements se trouvant dans le buffer
  if(buffLen > 0){
    Serial.printlnf("BufferLen = %u, Cloud = %s", buffLen, (Particle.connected() ? "true" : "false")); // Pour debug
    bool success = publishQueuedEvents();
    Serial.printlnf("Publishing = %u, Status: %s", readPtr - 1, (success ? "Fait" : "Pas Fait")); // Pour debug
  } else if (replayBuffLen > 0){
    bool success = replayQueuedEvents();
    Serial.printlnf("replayBuffLen = %u, Replay = %u, Status: %s", replayBuffLen, replayPtr, (success ? "Fait" : "Pas Fait"));
  }
}

#if HASDS18B20SENSOR
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

        pushToPublishQueue(evEnclosureTemp, (int) insideTempC, now);
        pushToPublishQueue(evAmbientTemp, (int) outsideTempC, now);
    }
    Serial.println();
    delay(2000UL);
  }
#endif

#if DISTANCESENSOR == US100
// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
  void ReadDistance_US100(){
      int currentReading;
      Serial.println("Distance meas routine: ReadDistance_US100");
      Serial1.flush();                                // clear receive buffer of serial port
      Serial1.write(0X55);                            // trig US-100 begin to measure the distance
      delay(100);                                     // delay 100ms to wait result
      if(Serial1.available() >= 2)                    // when receive 2 bytes
      {
          US100HighByte = Serial1.read();                   // High byte of distance
          US100LowByte  = Serial1.read();                   // Low byte of distance
          currentReading = US100HighByte*256 + US100LowByte;          // Combine the two bytes
  //        Len_mm  = (US100HighByte*256 + US100LowByte)/25.4;    // Calculate the distance in inch
          if((currentReading > 1) && (currentReading < maxRangeUS100)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
              dist_mm = AvgDistReading(currentReading); // Average the distance readings
              Serial.printlnf("Dist.: %dmm, now= %d, lastPublish= %d, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, WiFi.RSSI());
              if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
                  lastPublish = now;                               // reset the max publish delay counter.
                  pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
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
#endif

#if DISTANCESENSOR == MB7389
// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
  void ReadDistance_MB7389(){
      int currentReading = MB7389latestReading;
      Serial.printlnf("MB7389latestReading: %d", MB7389latestReading);
      if((currentReading > 1) && (currentReading < maxRangeMB7389)){       // normal distance should between 1mm and 2500 mm (1mm, 2,5m)
          dist_mm = AvgDistReading(currentReading); // Average the distance readings
          Serial.printlnf("Dist.: %dmm, now= %d, lastPublish= %d, RSSI= %d", (int)(dist_mm / numReadings), now, lastPublish, WiFi.RSSI());
          if (abs(dist_mm - prev_dist_mm) > minDistChange){         // Publish event in case of a change in temperature
                  lastPublish = now;                               // reset the max publish delay counter.
                  pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
                  prev_dist_mm = dist_mm;
                  samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
              }

      } else {
          pushToPublishQueue(evOutOfRange, 9999, now);
          Serial.print("Hors portée: ");             // output distance to serial monitor
          Serial.print(currentReading, DEC);
          Serial.println("mm ");
      }
   }
#endif

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

#if HASDS18B20SENSOR
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
                  if (abs(insideTempC - prev_EnclosureTemp) >= 1){
                      // Publier s'il y a eu du changement
                      pushToPublishQueue(evEnclosureTemp, (int) insideTempC, now);
                      prev_EnclosureTemp = insideTempC;
                  }
                  Serial.printlnf("DS18b20 interne: %f, try= %d", insideTempC, i + 1);
                  validEnclosureTemp = true;
              } else {
                  Serial.printlnf("DS18B20 interne: Erreur de lecture");
                  validEnclosureTemp = false;
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
                      pushToPublishQueue(evAmbientTemp, (int) outsideTempC, now);
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
                  if (abs(insideTempC - prev_EnclosureTemp) >= 1){
                      pushToPublishQueue(evEnclosureTemp, (int) insideTempC, now);
                      prev_EnclosureTemp = insideTempC;
                  }
                  Serial.printlnf("DS18b20 interne: %f", insideTempC);
                  validEnclosureTemp = true;
              } else {
                  Serial.printlnf("DS18B20 interne: Erreur de lecture");
                  validEnclosureTemp = false;
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
#endif

#if HASHEATING
// Imprémentation d'un thermostat simple ON/OFF
// La routine ne fonctionne que si un capteur de température interne est trouvé
int simpleThermostat(double setPoint){
    Particle.process();
    // executer la fonction de thermostat si on a un capteur de température
    if (ds18b20Count > 1){
        // executer la fonction de thermostat si la température interne est valide
        if (validEnclosureTemp == true){
            if (prev_EnclosureTemp < (setPoint - 0.5)){
                HeatingPower =  256 *  MaxHeatingPowerPercent /100;
            } else if (prev_EnclosureTemp > (setPoint + 0.5)){
                HeatingPower =  0;
            }
        } else if(validEnclosureTemp == false){
        // Si non mettre le chauffage à 1/4 de puissance pour éviter le gel.
            HeatingPower =  0.5 * (256 *  MaxHeatingPowerPercent /100); // Chauffage fixe au 1/4 de la puissance
        }

        analogWrite(heater, HeatingPower);
        Serial.printlnf("HeatingPower= %d", HeatingPower);
        if (HeatingPower != prev_HeatingPower){
            pushToPublishQueue(evHeatingPowerLevel, HeatingPower, now);
            prev_HeatingPower = HeatingPower;
        }
    }
    return HeatingPower;
}
#endif

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
int remoteSet(String command){
  String token;
  String data;
  int sep = command.indexOf(",");
  if (sep > 0){
    token = command.substring(0, sep);
    data = command.substring(sep + 1);
  } else {
    return -1; //Fail
  }

  if (token == "MaxPublishDelay"){
    unsigned long newInterval;
    newInterval = data.toInt();
    if (newInterval > 0){
        maxPublishInterval = data.toInt();
        Serial.printlnf("Now publishing at %d minutes interval", maxPublishInterval);
        return 0;
    } else {
        return -1;
    }
  } else if (token == "MaxHeatingPower"){
    return -1;
  }
  return -1;
}

// Pour resetter le capteur à distance au besoin
int remoteReset(String command) {
  if (command == "device"){
    System.reset();
// ou juste les numéros de série.
  } else if (command == "serialNo") {
    newGenTimestamp = Time.now();
    noSerie = 0;
    savedEventCount = 0;
    Serial.printlnf("Nouvelle génération de no de série maintenant: %lu", newGenTimestamp);
    pushToPublishQueue(evNewGenSN, -1, millis());
  }
}

/*
typedef struct Event{
  uint16_t noSerie; // Le numéro de série est généré automatiquement
  uint32_t eSnGen; // Timestamp du début d'une série de noSerie.
  uint32_t timer; // Temps depuis la mise en marche du capteur. Overflow après 49 jours.
  uint8_t namePtr; // Pointeur dans l'array des nom d'événement. (Pour sauver de l'espace NVRAM)
  int16_t eData;   // Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
*/
// Formattage standard pour les données sous forme JSON
String makeJSON(uint16_t numSerie, uint32_t timeStamp, uint32_t timer, int eData, String eName, bool replayFlag){
    /*char* formattedEName = String::format("%c", eName.c_str());*/
    /*sprintf(publishString,"{\"noSerie\": %u,\"timer\": %lu,\"eData\":%d,\"eName\": \"%s\"}", numSerie, timer, eData, eName.c_str());*/
    sprintf(publishString,"{\"noSerie\": %u,\"generation\": %lu,\"timestamp\": %lu,\"timer\": %lu,\"eData\":%d,\"eName\": \"%s\",\"replay\":%d}",
                              numSerie, newGenTimestamp, timeStamp, timer, eData, eName.c_str(), replayFlag);
    Serial.printlnf ("makeJSON: %s",publishString);
    return publishString;
}

// Publie les événement et gère les no. de série et le stockage des événements
bool pushToPublishQueue(uint8_t eventNamePtr, int eData, uint32_t timer){
  struct Event thisEvent;
  thisEvent = {noSerie, Time.now(), timer, eventNamePtr, eData};
  Serial.println(">>>> pushToPublishQueue:::");
  writeEvent(thisEvent); // Pousse les données dans le buffer circulaire
  noSerie++;
  if (noSerie > 65535){
     noSerie = 0;
    }
  return true;
}

// Publie un événement stocké en mémoire
bool publishQueuedEvents(){
    bool publishSuccess = false;
    struct Event thisEvent = {};
    bool replayFlag = false; // not a replay
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
                                            makeJSON(thisEvent.noSerie, thisEvent.timeStamp, thisEvent.timer, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr], replayFlag), 60, PRIVATE);
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
    bool replayFlag = true; // This is a replay
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
        publishSuccess = Particle.publish(DomainName + DeptName + eventName[thisEvent.namePtr],
                                            makeJSON(thisEvent.noSerie, thisEvent.timeStamp, thisEvent.timer, thisEvent.eData, DomainName + DeptName + eventName[thisEvent.namePtr], replayFlag), 60, PRIVATE);
                                            //makeJSON(uint16_t numSerie, uint32_t timeStamp, uint32_t timer, int eData, String eName){
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
// Format de la string de command: "Target SN, Target generation Id"
int replayEvent(String command){
  time_t targetGen;
  uint16_t targetSerNo;
  int sep = command.indexOf(",");
  if (sep > 0){
    targetSerNo = command.substring(0, sep).toInt();
    targetGen = command.substring(sep + 1).toInt();
  } else {
    return -1; //Fail
  }
  Serial.printlnf("?????? Demande de replay Event SN: %u, génération: %u,  writePtr= %u, readPtr= %u, replayBuffLen= %u",
                  targetSerNo, targetGen, writePtr, readPtr, replayBuffLen);
  if (replayBuffLen > 0){
      return -2; // Replay en cour - Attendre
  }
  // Validation de la demande
  if (targetSerNo >= 0 && targetGen > 0){ // Le numéro recherché doit être plus grand que 0
    // Validation
    if (targetGen != newGenTimestamp){
      Serial.printlnf("??????? Error -99: targetGen= %lu, targetSerNo= %u", targetGen, targetSerNo);
      return -99; // "invalid generation id"
    }
    if (targetSerNo >= noSerie){ // et plus petit que le numéro de série courant
        return -1;
    }
    if ((noSerie - targetSerNo) > savedEventCount){ // Il y a 250 événement au maximum dans le buffer
        /*targetSerNo = noSerie - buffSize;*/
        return savedEventCount; //
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
    return 0; //success
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
  if (savedEventCount < buffSize) { // increment the number of saved events up until buffer length
    savedEventCount++;

  } // Number of stored events in the buffer.
   //pour debug
  Serial.print("W------> " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  Serial.printlnf(": writeEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
                                     writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
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
  Serial.printlnf(": readEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
  return thisEvent;
}

// Re-lecture d'un événement en mémoire
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
  Serial.print("<------- " + DomainName + DeptName + eventName[thisEvent.namePtr]);
  Serial.printlnf(": readEvent:: writePtr= %u, replayPtr= %u, replayBuffLen= %u, noSerie: %u, eData: %u, timer: %u",
                                    writePtr, replayPtr, replayBuffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
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
  Serial.printlnf(": peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, timer: %u",
                                    writePtr, peekReadPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.timer);
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
