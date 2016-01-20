/*
*
*/

#include fonctionsDeBase.h

SYSTEM_THREAD(ENABLED);
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

const int buffSize = 300; // Nombre max d'événements que l'on peut sauvegarder
unsigned int buffLen = 0;
retained unsigned int writePtr = 0;
retained unsigned int readPtr = 0;
retained struct Event eventBuffer[buffSize];

// Pin pour l' I/O
int RGBled_Red = D0;
int RGBled_Green = D1;
int RGBLed_Blue = D2;
int led = D7; // Feedback led
int ssrRelay = D6; // Solid state relay
int ssrRelayState = false;
int motorState = A0; // input pour Pompe marche/arrêt

// Variables liés à la pompe
/*bool PumpOldState = true;
bool PumpCurrentState = true;
unsigned long changeTime = 0;*/

// Variables liés aux valves
/*int ValvePos_pin[] = {A2, A3, A4, A5};
bool ValvePos_state[] = {true, true, true, true};
int ValvePos_Name[] = {evValve_A, evValve_B, evValve_C, evValve_D};*/

// Variables liés à la mesure de Température
/*unsigned int HighLen = 0;
unsigned int LowLen  = 0;
int Temp = 0;
int prev_Temp = 0;
int allTempReadings[numReadings];*/

// Variables liés à la mesure de distance
int dist_mm  = 0;
int prev_dist_mm = 0;
int allDistReadings[numReadings];

// Variables liés au temps
unsigned long lastPublish;
unsigned long now;
unsigned long lastSync = millis();
unsigned int samplingInterval = fastSampling;
int maxPublishInterval = 2;
unsigned long maxPublishDelay = maxPublishInterval * minute;
unsigned long lastTime = 0UL;

// Variables liés aux publications
char publishString[buffSize];
retained unsigned noSerie = 0; //Mettre en Backup RAM
int pumpEvent = 0;
bool connWasLost = false;

// Autre variables
//String myDeviceName = "";
/*
// handler to receive the module name
*/
/*void nameHandler(const char *topic, const char *data) {
    myDeviceName =  String(data);
    Serial.println("received " + String(topic) + ": " + String(data));
}*/

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
      analogWrite(pin_r, r); // 255 - r pour common cathode
      analogWrite(pin_g, g); // 255 - g pour common cathode
      analogWrite(pin_b, b); // 255 - b pour common cathode
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
/*class PumpInputState {
  public:
    PumpInputState() {
      attachInterrupt(motorState, &PumpInputState::A0Handler, this, CHANGE);
    }
    void A0Handler() {
      System.ticksDelay(50000*System.ticksPerMicrosecond()); // Debounce 50 milliseconds
      Serial.print("Pompe ");
      // enregistre l'état et le temps
      PumpCurrentState = digitalRead(A0);
      changeTime = millis();
      if (PumpCurrentState == false){
        Serial.println("On");
      } else {
        Serial.println("Off");
      }
    }
};

PumpInputState pumpState; // Instantiate the class PumpInputState*/

void setup() {
// connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
//    Serial.begin(115200);
    Serial1.begin(9600);  // Le capteur US-100 fonctionne à 9600 baud
    delay(3000); // Pour partir le moniteur série
// Attendre la connection au nuage
    Serial.println("En attente... ");
    if (waitFor(Particle.connected, 10000)) {
        if(Particle.connected()){
            Serial.println("Connecté au nuage. :)");
            /*//Quel est mon nom?
                delay(3000);
                Particle.subscribe("spark/", nameHandler);
                Particle.publish("spark/device/name");

            // Attendre la réception du nom
                while(myDeviceName.length() == 0){
                    delay(1000);
                    Serial.print(".");
                }
                Serial.println("\nMon nom est: " + myDeviceName + "\n");*/
        } else {
            Serial.println("Pas de connexion au nuage. :( ");
        }
    }


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
// Fonction et variable disponible par le nuage
    Particle.function("pubInterval", setPublishInterval);
    Particle.function("relay", toggleRelay);
    Particle.variable("relayState", &ssrRelayState, INT);

    Time.zone(-4);
    Time.setFormat(TIME_FORMAT_ISO8601_FULL);

    lastPublish = millis(); //Initialise le temps initial de publication
    changeTime = lastPublish; //Initialise le temps initial de changement de la pompe
}

void loop() {
    digitalWrite(led, HIGH); // Pour indiqué le début de la prise de mesure
    now = millis();
// ********* Appeler ici les fonctions de mesure pour le capteur concerné. ***********
//
// Les fonctions doivent mettre leur résultats dans la queue de publication avec la function
// pushToPublishQueue(). Si le cloud est dispomible les événements seront publiés plus loin
// dans la boucle
//
//    Readtemp_US100();
//    ReadDistance_US100();
//
// ************************************************************************************
    digitalWrite(led, LOW); // Pour indiqué la fin de la prise de mesure

// Publication de l'état de la pompe s'il y a eu changement
    /*if (PumpCurrentState != PumpOldState){
      PumpOldState = PumpCurrentState;
      if (PumpCurrentState == true){
        pumpEvent = evPompe_T1;
      } else {
        pumpEvent = evPompe_T2;
      }
      pushToPublishQueue(pumpEvent, PumpCurrentState, changeTime);
    }*/
// Pour permettre la modification de maxPublishDelay par le nuage
    maxPublishDelay = maxPublishInterval * minute;

// Publication au moins une fois à tous les "maxPublishDelay" millisecond
    now = millis();
    if (now - lastPublish > maxPublishDelay)
        {
            lastPublish = now;
// ********* Publier ici les mesures pour le capteur concerné. ***********
//
//            pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
//            pushToPublishQueue(evTemperature_US100, (int)(Temp/ numReadings), now);
//
// ************************************************************************************
            samplingInterval = slowSampling;   // Les mesure sont stable, réduire la fréquence de mesure.
        }

// Synchronisation du temps avec Particle Cloud une fois par jour
    if (millis() - lastSync > unJourEnMillis) {
        Particle.syncTime();
        lastSync = millis();
    }
// Publication des événements se trouvant dans le buffer
    if(buffLen > 0){
        Serial.printlnf("BufferLen = %u, Cloud = %s", buffLen, (Particle.connected() ? "true" : "false")); // Pour debug
        bool success = publishQueuedEvents();
        Serial.printlnf("Publishing = %u, Status: %s", readPtr - 1, (success ? "Fait" : "Pas Fait")); // Pour debug
    }
    killTime(samplingInterval); // Maintient la boucle sur une base de 1 second
}

// Cette routine mesure la distance entre la surface de l'eau et le capteur ultason
/*void ReadDistance_US100(){
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
                    pushToPublishQueue(evDistance, (int)(dist_mm / numReadings), now);
                    prev_dist_mm = dist_mm;
                    samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
                }

        } else {
            pushToPublishQueue(evOutOfRange, 9999, now);
            Serial.print("Hors portée: ");             // output distance to serial monitor
            Serial.print(currentReading, DEC);
            Serial.println("mm ");
            delay(2000);
        }
    } else {
        Serial.println("Données non disponible");
    }
}*/

// Cette routine lit la température sur le capteur US-100.
// Note: La valeur 45 DOIT être soustraite pour obtenir la température réelle.
/*void Readtemp_US100(){
    int Temp45 = 0;
    Serial1.flush();                // S'assurer que le buffer du port serie 1 est vide.
    Serial1.write(0X50);            // Demander la lecture de température sur le US-100 en envoyant 50 (Hex)
    delay(100);                     // Attente du résult
    if(Serial1.available() >= 1)    // lors de la réception d'au moins 1 bytes
    {
        Temp45 = Serial1.read();     // Lire la température brut
        if((Temp45 > 1) && (Temp45 < 130))   // Vérifier si la valeur est acceptable
        {
            Temp = AvgTempReading(Temp45 - 45); //Conversion en température réelle et filtrage
            Serial.printlnf("Temp.:%dC now= %d, lastPublish= %d",  (int)(Temp/ numReadings), now, lastPublish); // Pour debug
            if (abs(Temp - prev_Temp) > minTempChange){    // Vérifier s'il y a eu changement depuis la dernière publication
                    lastPublish = now;                     // Remise à zéro du compteur de délais de publication
                    pushToPublishQueue(evTemperature_US100, (int)(Temp / numReadings), now); //Publication
                    prev_Temp = Temp;                      //
                    samplingInterval = fastSampling;   // Augmenter la vitesse d'échantillonnage puisqu'il y a eu changement
                }
        }
    }
}*/

// Filtre par moyenne mobile pour les distances
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
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

// Filtre par moyenne mobile pour les température
// Note: Il s'agit en fait de la somme des x dernière lecture.
//       La division se fera au moment de la publication
int AvgTempReading(int thisReading){
    long AvgTemp = 0;
    for (int i = 1; i < numReadings; i++){
        allTempReadings[i-1] = allTempReadings[i]; //Shift all readings
        Avg += allTempReadings[i-1]; //Total of readings except the last one
    }
    allTempReadings[numReadings - 1] = thisReading; //Current reading in the last position
    AvgTemp += thisReading; //total including the last one
    return (AvgTemp); // Avg sera divisé par numReadings au moment de la publication
}

// Check the state of the valves position reedswitch
/*void CheckValvePos(){
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
            Serial.println(eventName[ValvePos_Name[i]] + ": " + stateStr );
            pushToPublishQueue(ValvePos_Name[i], valveCurrentState, now);
        }
    }
}*/

void killTime(unsigned long interval){
    for (unsigned long i=0 ; i < 2 * interval ; i++){
        /*CheckValvePos();*/
        if (i == 0){
            delay(0.5 * second - baseLoopTime); // Delay required to make the loop approx. samplingInterval seconds.
        } else {
            delay(0.5 * second);
        }
     }
}

// Active ou désactive le relais SSR
int toggleRelay(String command) {
    if (command=="on" || command=="1") {
        ssrRelayState = HIGH;
        digitalWrite(ssrRelay, ssrRelayState);
        /*Particle.publish("Relais", "on", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, ssrRelayState, now);
        return 1;
    }
    else if (command=="off" || command=="0") {
        ssrRelayState = LOW;
        digitalWrite(ssrRelay, ssrRelayState);
        /*Particle.publish("Relais", "off", 60, PRIVATE);*/
        pushToPublishQueue(evRelais, ssrRelayState, now);
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

// Formattage standard pour les données sous forme JSON
String makeJSON(unsigned long numSerie, int eData, unsigned long eTime){
    sprintf(publishString,"{'noSerie': %u,'eTime': %u,'eData': %d}", numSerie, eTime, eData);
    /*Serial.println(publishString);*/
    return publishString;
}

// Publie les événement et gère les no. de série et le stockage des événements
bool pushToPublishQueue(int eventNamePtr, int eData, unsigned long eTime){
  struct Event thisEvent;
  thisEvent = {noSerie, eventNamePtr, eData, eTime};
  Serial.println(">>>> pushToPublishQueue:::");
  writeEvent(thisEvent); // Pousse les données dans le buffer circulaire
  noSerie += 1;
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
        publishSuccess = Particle.publish(eventName[thisEvent.namePtr], makeJSON(thisEvent.noSerie, thisEvent.eData, thisEvent.eTime), 60, PRIVATE);
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
  Serial.print("-------> " + eventName[thisEvent.namePtr]);
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
  Serial.print("<------- " + eventName[thisEvent.namePtr]);
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
  buffLen = writePtr - readPtr;
  //pour debug
  Serial.print(" ------- " + eventName[thisEvent.namePtr]);
  Serial.printlnf(": peekEvent:: writePtr= %u, readPtr= %u, buffLen= %u, noSerie: %u, eData: %u, eTime: %u",
                                    writePtr, readPtr, buffLen, thisEvent.noSerie, thisEvent.eData, thisEvent.eTime);
  return thisEvent;
}
