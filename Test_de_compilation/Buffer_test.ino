
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

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
retained unsigned int savedEventCount = 0;
retained time_t newGenTimestamp = 0;
retained uint16_t noSerie = 0; //Mettre en Backup RAM
unsigned int replayBuffLen = 0;

// Name space utilisé pour les événements
// DomainName/DeptName/FunctionName/SubFunctionName/ValueName
String DomainName = "";
String DeptName = "";

char publishString[buffSize];
bool connWasLost = false;




void setup(){
  Serial.begin(115200);
}

void loop(){

  for (int i=0; i<buffSize; i++){
    pushToPublishQueue(evVacuum, i, millis());
    delay(500);
  }

  for (int i=0; i<buffSize; i++){
    readEvent();
    delay(500);
  }

}

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
