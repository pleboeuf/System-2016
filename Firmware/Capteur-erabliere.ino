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
//      3V3	XL-Max Sonar MB7092 Pin 1 - Nc or High	 Ext.	                        	                  O
//      D5	XL-Max Sonar MB7092 Pin 2 - Pulse out	 Ext.	                        	                  O
//      Tx	XL-Max Sonar MB7092 Pin 4 - Trig 20uS	 Ext.	                                              O
//      Rx	XL-Max Sonar MB7092 Pin 5 - Serial out	 Ext.	                        	                  O
//      Gnd	XL-Max Sonar MB7092 Pin 7 - Gnd	         Ext.	                                              O
//      D4	Dallas 18b20 Temperature sensor	         Onboard / Ext.	               O	                  O	                  O	             O
//      D3	Heating circuit. Power control	         Onboard	                     	                  O
//      Vbat	Memory backup battery CR2032	     Onboard	                   O	                  O	                  O	             O
// the US-100 module WITH jumper cap on the back.


// the US-100 module WITH jumper cap on the back.
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL));

#define minute 60000UL            // 60000 millisecond per minute
#define second 1000UL             // 1000 millisecond per sesond
#define unJourEnMillis (24 * 60 * 60 * second)
#define baseLoopTime  208      //Estimated loop time in millisecond
#define debounceDelay 20    // Debounce time for valve position readswitch
#define fastSampling  1   // in second
#define slowSampling  5    // in second
#define minDistChange 1.66       // Minimum change in distance to publish an event (1/16")
#define minTempChange 0.5      // Minimum temperature change to publish an event
#define numReadings 10           // Number of readings to average for filtering
#define myTimeFormat TIME_FORMAT_ISO8601_FULL

// Pin to use for I/O
int led = D7; // Feedback led
int ssrRelay = D6; // Solid state relay
int ssrRelayState = false;
int motorState = A0; // Input to

// Variable relié au valves
int ValvePos_pin[] = {A2, A3, A4, A5};
bool ValvePos_state[] = {true, true, true, true};
String ValvePos_pinName[] = {"1A", "1B", "2A", "2B"};

// Variable relié à la mesure de Température
unsigned int HighLen = 0;
unsigned int LowLen  = 0;
float Temp = 0;
float prev_Temp = 0;
char Temp_str[20];
float allTempReadings[numReadings];

// Variable relié à la mesure de distance
float dist_mm  = 0;
float prev_dist_mm = 0;
char dist_mm_str[20];
float allDistReadings[numReadings];

// Variable relié au temps
unsigned long T0;
unsigned long now;
unsigned long lastSync = millis();
unsigned int samplingInterval = fastSampling;
int maxPublishInterval = 2;
unsigned long maxPublishDelay = maxPublishInterval * minute;
unsigned long lastTime = 0UL;

// Variable relié au publication
char publishString[100];
retained unsigned noSerie = 0; //Mettre en Backup RAM

// Autre variable
String myDeviceName = "";

void handler(const char *topic, const char *data) {
    myDeviceName =  String(data);
    Serial.println("received " + String(topic) + ": " + String(data));
}

void setup() {
  // connect RX to Echo/Rx (US-100), TX to Trig/Tx (US-100)
    Serial.begin(115200);                           // Set serial debug speed
    Serial1.begin(9600);                          // set the sensor serial baudrate.
// Attendre la connection au nuage
    int i = 0;
    while(!Particle.connected()){
         Serial.println("En attente... " + String(5 - i));
         Serial.println("Connecté au nuage. Good!");
    }
//Quel est mon nom?
    Particle.subscribe("spark/", handler);
    Particle.publish("spark/device/name");
// Attendre la réception du nom
    while(myDeviceName.length() == 0){
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nMon nom est: " + myDeviceName + "\n");
// Initialisation
    pinMode(led, OUTPUT);
    pinMode(ssrRelay, OUTPUT);
    digitalWrite(led, LOW);
    digitalWrite(ssrRelay, LOW);
    for (int i=0; i <= 3; i++) {
        pinMode(ValvePos_pin[i], INPUT_PULLUP);
    }

    for (int i = 0; i<=numReadings; i++){ // Init readings array
        allDistReadings[i] = 0;
        allTempReadings[i] = 0;
        // Serial.println(allReadings[i]);
    }
    // Particle.variable("dist", &dist_mm, INT);
    // Particle.variable("temp", &Temp, INT);
    Particle.function("relay", toggleRelay);
    Particle.function("pubInterval", setPublishInterval);
    Particle.variable("ssrRelay", &ssrRelayState, INT);

    Time.zone(-4);
    Time.setFormat(myTimeFormat);

    // for test only
    // Serial.println(Time.format(Time.now(), "%Y-%m-%dT%H:%M:%S%z"));
    // Serial.println(Time.getFormat());
    // Serial.println(Time.format(Time.now(), TIME_FORMAT_ISO8601_FULL));
    // Serial.println(Time.getFormat());

    T0 = millis();
}

void loop() {
    digitalWrite(led, HIGH);
    ReadDistance();
    Readtemp();
    digitalWrite(led, LOW);
    maxPublishDelay = maxPublishInterval * minute; // Pour permettre la modification par le nuage
    now = millis();
    if (now - T0 > maxPublishDelay)                 // Publish at least every maxPublishDelay millisecond
        {
            T0 = now;
            // time_t time = Time.now();
            Particle.publish("Distance", String(dist_mm), 60, PRIVATE);
            makeJSON(dist_mm);
            Particle.publish("Temperature",String(Temp),60,PRIVATE);
            samplingInterval = slowSampling;   //Measurements are stable, reduce the sampling frequency
        }
    // Request time synchronization from the Particle Cloud
    if (millis() - lastSync > unJourEnMillis) {
        Particle.syncTime();
        lastSync = millis();
    }
    killTime(samplingInterval);
    // upTime();
}

void ReadDistance(){
    int lastReading;
    Serial1.flush();                                // clear receive buffer of serial port
    Serial1.write(0X55);                            // trig US-100 begin to measure the distance
    delay(100);                                     // delay 100ms to wait result
    if(Serial1.available() >= 2)                    // when receive 2 bytes
    {
        HighLen = Serial1.read();                   // High byte of distance
        LowLen  = Serial1.read();                   // Low byte of distance
        lastReading = HighLen*256 + LowLen;          // Combine the two bytes
//        Len_mm  = (HighLen*256 + LowLen)/25.4;    // Calculate the distance in inch
        if((lastReading > 1) && (lastReading < 4500))       // normal distance should between 1mm and 4500 mm (1mm, 4,5m)
        {
            dist_mm = AvgDistReading(lastReading); // Average the distance readings
            Serial.print("Distance 3: ");             // output distance to serial monitor
            Serial.print(dist_mm, DEC);
            // sprintf(dist_mm_str, "%d", dist_mm);
            Serial.print("mm ");
            Serial.print(", T: ");                  //output temperature to serial monitor
            Serial.print(Temp, DEC);
            Serial.print("C.");
            Serial.print("now= ");  Serial.print(now);  Serial.print(",  T0= ");  Serial.println(T0);
            if ( (abs(dist_mm - prev_dist_mm) > minDistChange) || ( abs(Temp - prev_Temp) > minTempChange) )          // Publish event in case of a change in temperature
                {
                    T0 = now;                               // reset the max publish delay counter.
                    // time_t time = Time.now();
                    // Time.format(time, myTimeFormat)
                    Particle.publish("Distance", String(dist_mm), 60, PRIVATE);
                    prev_dist_mm = dist_mm;
                    makeJSON(dist_mm);
                    Particle.publish("Temperature", String(Temp), 60, PRIVATE);
                    prev_Temp = Temp;
                    Serial.print("Événement: Changement distance ou de température - ");
                    Serial.println(String(dist_mm));
                    samplingInterval = fastSampling;   //Measurements NOT stable, increase the sampling frequency
                }
        } else {
            Particle.publish("Hors portée: ","9999",60,PRIVATE);
            Serial.print("Hors portée: ");             // output distance to serial monitor
            Serial.print(lastReading, DEC);
            // sprintf(dist_mm_str, "%d", dist_mm);
            Serial.println("mm ");
            delay(2000);
        }
    } else {
        Serial.println("Données non disponible");
    }
}

// This routine read the temperature from the US-100 sensor.
// Note: A varue of 45 MUST be substracted from the readings to obtain real temperature.
void Readtemp(){
    int Temp45 = 0;
    Serial1.flush();       // clear receive buffer of serial port
    Serial1.write(0X50);   // trig US-100 begin to measure the temperature
    delay(100);            // delay 100ms to wait result
    if(Serial1.available() >= 1)            //when receive 1 bytes
    {
        Temp45 = Serial1.read();     //Get the received byte (temperature)
        if((Temp45 > 1) && (Temp45 < 130))   //the valid range of received data is (1, 130)
        {
            Temp = AvgTempReading(Temp45 - 45);
            sprintf(Temp_str, "%f", Temp);
        }
    }
}

// Return a running average of the last "numReadings" distance reading to filter noise.
float AvgDistReading(int thisReading){
    double Avg = 0;
    for (int i = 1; i < numReadings; i++){
        allDistReadings[i-1] = allDistReadings[i]; //Shift all readings
        // Serial.print(allDistReadings[i-1]);
        // Serial.print(" ");
       Avg += allDistReadings[i-1]; //Total of readings except the last one
    }
    allDistReadings[numReadings-1] = thisReading; //Current reading in the last position
    // Serial.println(allDistReadings[numReadings-1]);
    Avg += thisReading; //including the last one
    return (Avg / numReadings);
}

// This routine return a running average of the last "numReadings" temperature reading to filter noise.
float AvgTempReading(int thisReading){
    double Avg = 0;
    for (int i = 1; i < numReadings; i++){
        allTempReadings[i-1] = allTempReadings[i]; //Shift all readings
        Avg += allTempReadings[i-1]; //Total of readings except the last one
    }
    allTempReadings[numReadings - 1] = thisReading; //Current reading in the last position
    Avg += thisReading; //total including the last one
    return (Avg / numReadings);
}

// Check the state of the valves position reedswitch
void CheckValvePos(){
    bool currentState;
    String stateStr;

    for (int i=0; i <= 3; i++) {
        currentState = digitalRead(ValvePos_pin[i]);
        if (ValvePos_state[i] != currentState)
        {
            delay(debounceDelay);  // Debounce time
            // time_t time = Time.now();
            currentState = digitalRead(ValvePos_pin[i]);
            ValvePos_state[i] = currentState;
            if (currentState == true){
                stateStr = "Ouvert";
            } else {
                stateStr = "Fermé";
            }
            Serial.println("Pin " + ValvePos_pinName[i] + ": " + stateStr );
            Particle.publish("Valve " + ValvePos_pinName[i], stateStr, 60, PRIVATE);
        }
    }
}

void killTime(unsigned long interval){
    for (unsigned long i=0 ; i < interval ; i++){
        CheckValvePos();
        if (i == 0){
            delay(1 * second - baseLoopTime); // Delay required to make the loop approx. samplingInterval seconds.
        } else {
            delay( 1 * second);
        }
     }
}

int toggleRelay(String command) {
    if (command=="on" || command=="1") {
        ssrRelayState = HIGH;
        digitalWrite(ssrRelay, ssrRelayState);
        Particle.publish("Relais", "on", 60, PRIVATE);
        return 1;
    }
    else if (command=="off" || command=="0") {
        ssrRelayState = LOW;
        digitalWrite(ssrRelay, ssrRelayState);
        Particle.publish("Relais", "off", 60, PRIVATE);
         return 0;
    }
    else {
        return -1;
    }
}

void upTime(){
    unsigned long now = millis();
    //Every 1 hour publish uptime
    if (now-lastTime> (60 * minute)) {
        lastTime = now;
        // now is in milliseconds
        unsigned nowSec = now/1000U;
        unsigned sec = nowSec%60;
        unsigned min = (nowSec%3600)/60;
        unsigned hours = (nowSec%86400)/3600;
        sprintf(publishString,"{'Hours': %u,'Minutes': %u,'Seconds': %u}",hours,min,sec);
        sprintf(publishString,"{'Hours': %u}",hours);
        Particle.publish("Uptime",publishString, PRIVATE);
    }
}

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

String makeJSON(float eData){
    sprintf(publishString,"{'noSerie': %u,'iDate': %u,'eData': %f}", noSerie, Time.now(), eData);
    noSerie += 1;
    /*Serial.println(publishString);*/
    return publishString;
}
