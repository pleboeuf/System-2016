// **** Photon Sleep test ****

// #include "Particle.h"
#define sleepTimeInMinutes 2 // Duration of sleep
#define resetCycleCountPin  B0 // Connect to Vcc to reset the cycle counter
#define myEventName "Usage" // Usage data

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

unsigned long waitOTA;
long sleepTimeInSec = 0;
int cycleNumber     = 0;
int statusLed       = D7;
int addrCycleNo     = 0;
int addrPrev_aw_Time = 10;
int lastDay         = 0;

char publishStr[45];
CellularSignal sig;

float voltage     = 0;
float prev_voltage = 0;
float soc         = 0;
float prev_soc    = 0;

int signalRSSI    = 0;
int prev_RSSI     = 0;
int signalQuality = 0;
int prev_QUAL     = 0;

int      txPrec   = 0;      // Previous tx data count
int      rxPrec   = 0;      // Previous rx data count
int      deltaTx  = 0;      // Difference tx data count
int      deltaRx  = 0;      // Difference rx data count

uint32_t w_time   = 0; // Wakeup time in ms
uint32_t s_time   = 0; // Go to sleep time in ms
float aw_time     = 0; // Awake time in sec
float prev_aw_time = 0; // Awake time in sec

// System event handler to update firmware before sleep
void handle_firmware_update_events(system_event_t event, int param)
{
    Serial1.printlnf("\ngot event %d with value %d\n", event, param);
    waitOTA = millis();
    for(uint32_t ms = millis(); millis() - ms < 1000; Particle.process());
}

// called once on startup
void setup() {
    w_time = millis();
    Time.zone(-4);
    pinMode(resetCycleCountPin, INPUT_PULLDOWN);
    if (digitalRead(resetCycleCountPin) == TRUE) {
            cycleNumber = 0;
            EEPROM.put(addrCycleNo, cycleNumber); // Line used to initialized cycleNumber in EEPROM. Comment out after used.
            prev_aw_time = 0;
            EEPROM.put(addrPrev_aw_Time, prev_aw_time);
    }
    EEPROM.get(addrCycleNo, cycleNumber); // Get the cycleNumber from non-volatile storage
    EEPROM.get(addrPrev_aw_Time, prev_aw_time); // Get the previous awake time for publication

    pinMode(statusLed, OUTPUT);
    Serial1.begin(115200);
    /*Serial1.printlnf("\n\nPhoton DEEP SLEEP Test.  ##### Sleeping for %d min. Delay before sleep: %d ms #####", sleepTimeInMinutes, awakeDelay);*/

    Particle.connect();
    waitUntil(Particle.connected)
    syncRTC();

    readCellularData(); // Initial readings

    // Register events listener for OTA update
    System.on(firmware_update_pending, handle_firmware_update_events);
}

void loop() {

    Serial1.printf("%d\t", cycleNumber);
    Serial1.printf(Time.timeStr().c_str());

    readBattery();
    Serial1.printf("\t%2.4f\t%3.3f", voltage, soc);
    checkSignal(); // Print signal RSSI and quality
    publishData(); // Publish a dummy message for test

    cycleNumber += 1;
    EEPROM.put(addrCycleNo, cycleNumber); // Save it for next iteration

    prev_aw_time = aw_time;
    EEPROM.put(addrPrev_aw_Time, prev_aw_time); // Save it for next iteration

    readCellularData(); // After publish
    Serial1.printf("\t%d \t%d ", deltaTx, deltaRx);
    s_time = millis(); // Sleep time is now
    aw_time = (float)(s_time - w_time) / 1000;
    Serial1.printlnf("\t%3.3f", aw_time);
    for(uint32_t ms = millis(); millis() - ms < 100; Particle.process()); // Delay to complete printing

    uint32_t dt = (sleepTimeInMinutes - Time.minute() % sleepTimeInMinutes) * 60 - Time.second(); // wake at next time boundary +30 seconds
    System.sleep(SLEEP_MODE_DEEP, dt, SLEEP_NETWORK_STANDBY); // Deep sleep SLEEP_NETWORK_STANDBY - Network shutdown and processor in stand-by mode. RESET on wake.
}

void publishData() {
    bool success;
    // Publish No, voltage, SOC, RSSI, QUALITY. Tx & Rx count, Awake time for the previous cycle
    sprintf(publishStr, "%d, %2.4f, %3.3f, %d, %d, %d, %d, %3.3f",
            cycleNumber - 1, prev_voltage, prev_soc, signalRSSI, signalQuality, deltaTx, deltaRx, prev_aw_time);
    success = Particle.publish(myEventName, publishStr, PRIVATE);
    /*for(uint32_t ms = millis(); millis() - ms < 1000; Particle.process()); // Gives some time to publish*/
    uint32_t start = millis();
    while (millis() - start < 5000UL) {
            Particle.process(); // Wait a second to complete the publish
    }
    if(!success){
      Serial1.print(" -- Publish failed! -- ");
    }
}

void readBattery() {
    FuelGauge fuel;
    prev_voltage = voltage;
    prev_soc = soc;
    voltage = fuel.getVCell();
    soc = fuel.getSoC();
}

void readCellularData() {
    CellularData data;
    if (!Cellular.getDataUsage(data)) {
          Serial1.print("Error! Not able to get data.");
    }
    else {
          deltaTx = data.tx_session - txPrec;
          deltaRx = data.rx_session - rxPrec;
          txPrec = data.tx_session;
          rxPrec = data.rx_session;
    }
}

void checkSignal() {
    prev_RSSI = signalRSSI;
    prev_QUAL = signalQuality;
    sig = Cellular.RSSI();
    signalRSSI = sig.rssi;
    signalQuality = sig.qual;
}

void syncRTC(){
  if (Time.year() < 2000){   // a new day calls for a sync
      Particle.connect();
      if (waitFor(Particle.connected, 1*60000)){
        Particle.syncTime();
        for(uint32_t ms = millis(); millis() - ms < 1000; Particle.process()); //wait for answer
        /*lastDay = Time.day();*/
      }
  }
}
