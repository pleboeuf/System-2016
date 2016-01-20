/*
********************************************************************************
* @file:   fonctionsDeBase.h
* @Auteur: Pierre Leboeuf
* @Version V0.6.0
* @Date    Novenbre 2015
* @brief   Header pour les fonctions de base d'un capteur.
********************************************************************************

Tous les événements sont stocké dans dans un buffer circulaire en mémoire
protégé par batterie. De cette façon les 100 à 150 dernier événements seront
conservé en cas de perte de réseau. Afin d'optimiser l'espace requis, les données
seront de type entier (integer ou long). Les nom d'événements sont remplacer par
leur index dans l'array eventName. Les données sont organisé dans une
structure "Event" et stocké dans un array de ces structures.
13 bytes sont utilisés par événement. On peut donc sauver environ
4096 / 13 = 315 événements. Les événements sont stocké au fur et à mesure de
leur production. Il seront publié séquenciellement indépendamment de leur
production.
*/

#include "application.h"

#ifndef FONCTION_DE_BASE_H
#define FONCTION_DE_BASE_H

#define minute 60000UL            // 60000 millisecond per minute
#define second 1000UL             // 1000 millisecond per sesond
#define unJourEnMillis (24 * 60 * 60 * second)
#define baseLoopTime  206      //Estimated loop time in millisecond
//#define debounceDelay 50    // Debounce time for valve position readswitch
#define fastSampling  1   // En secondes
#define slowSampling  5    // En secondes
#define numReadings 10           // Nonbre de lecture pour la moyenne mobile
/*#define minDistChange 1.7 * numReadings      // Minimum change in distance to publish an event (1/16")
#define minTempChange 0.5 * numReadings      // Minimum temperature change to publish an event
#define maxRangeUS100 2500 // Distance maximale valide pour le captgeur*/

// Nom des indices du tableau eventName
#define evPompe_T1 0
#define evPompe_T2 1
#define evDistance 2
#define evTemperature_US100 3
#define evOutOfRange 4
#define evValve_A 5
#define evValve_B 6
#define evValve_C 7
#define evValve_D 8
#define evRelais 9
#define evVacuum 10
#define evDebit_A 11
#define evVolume_A 12
#define evDebit_B 13
#define evVolume_B 14
#define evPressionAtmospherique 15
#define evTempInterne 16
#define evTempExterne 17

// Variables lié aux événements
String eventName[] = {
  "Pompe T1",
  "Pompe T2",
  "Distance",
  "Temperature US100",
  "Hors portée: ",
  "Valve A",
  "Valve B",
  "Valve C",
  "Valve D",
  "Relais",
  "Vacuum",
  "Débit A",
  "Volume A",
  "Débit B",
  "Volume B",
  "Pression Atmosphérique",
  "Température interne",
  "Température externe"
};

// Structure définissant un événement
struct Event{
  int16_t noSerie; // Le numéro de série est généré automatiquement
  uint8_t namePtr; // Pointeur dans l'array des noms d'événement. (Pour sauver de l'espace NVRAM)
  unsigned long eDate; // Date et heure de l'horloge interne (RTC). Format UNIX TS
  unsigned long eTime; /* Temps depuis la mise en marche du capteur.
                          Overflow après 49 jours. En millisecondes*/
  int16_t eData;   /* Données pour cet événement. Entier 16 bits. Pour sauvegarder des données en point flottant
                     multiplié d'abord la donnée par un facteur (1000 par ex.) en convertir en entier.
                     Il suffira de divisé la données au moment de la réception de l'événement. */
};
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
// int dist_mm  = 0;
// int prev_dist_mm = 0;
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
// class ExternalRGB {
//   public:
//     ExternalRGB(pin_t r, pin_t g, pin_t b) : pin_r(r), pin_g(g), pin_b(b) {
//       pinMode(pin_r, OUTPUT);
//       pinMode(pin_g, OUTPUT);
//       pinMode(pin_b, OUTPUT);
//       RGB.onChange(&ExternalRGB::LEDhandler, this);
//     }
//
//     void LEDhandler(uint8_t r, uint8_t g, uint8_t b) {
//       analogWrite(pin_r, r); // 255 - r pour common cathode
//       analogWrite(pin_g, g); // 255 - g pour common cathode
//       analogWrite(pin_b, b); // 255 - b pour common cathode
//     }
//
//     private:
//       pin_t pin_r;
//       pin_t pin_g;
//       pin_t pin_b;
// };

// Connect an external RGB LED to D0, D1 and D2 (R, G, and B)
// ExternalRGB myRGB(RGBled_Red, RGBled_Green, RGBLed_Blue);


class SensorPub {
    public:

        SensorPub();

        void
            begin();
        int
            AvgDistReading(int thisReading), // Filtre par moyenne mobile pour les distances
            AvgTempReading(int thisReading), // Average readings
            toggleRelay(String command), // Active ou désactive le relais SSR
            setPublishInterval(String command); // Pour modifier l'interval de publication par défault

        bool
            pushToPublishQueue(int eventNamePtr, int eData, unsigned long eTime), // Publie les événement et gère les no. de série et le stockage des événements
            publishQueuedEvents(), // Publie un événement stocké en mémoire
            writeEvent(struct Event thisEvent); // Sauvegarde d'un événement en mémoire
        struct
            Event readEvent(), // Lecture d'un événement en mémoire
            Event peekEvent(); // Lecture d'un événement en mémoire sans avancé le pointeur

    private:

        String makeJSON(unsigned long numSerie, int eData, unsigned long eTime); // Formattage standard pour les données sous forme JSON
}


#ifndef FONCTION_DE_BASE_H
