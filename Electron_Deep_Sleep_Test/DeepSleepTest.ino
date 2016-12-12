// **** Electron Sleep test ****
/*
*** Test conditions ***
Sleep mode: Stop mode + SLEEP_NETWORK_STANDBY, use SETUP BUTTON (20) to wake
Publish: NO_ACK
Delay loop: 500 ms for publish and print
Sleep duration: See #define sleepTimeInMinutes
*/

#include "Particle.h"
#include "math.h"
#define Version 0.5.0
#define resetCycleCountPin  B0 // Connect to Vcc to reset the cycle counter
/*#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)*/
#define myEventName "Usage" // Usage data
#define TimeBoundaryOffset 0 // Wake at time boundary plus some seconds

#define sleepTimeInMinutes 60  // Duration of sleep

SYSTEM_MODE(SEMI_AUTOMATIC);
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));

/*int addr = 0;*/
char publishStr[45];
int signalRSSI;
int signalQuality;
retained int lastDay     = 0;
retained int cycleNumber = 0;

int      txPrec   = 0;      // Previous tx data count
int      rxPrec   = 0;      // Previous rx data count
int      deltaTx  = 0;      // Difference tx data count
int      deltaRx  = 0;      // Difference rx data count
uint32_t start    = 0;
uint32_t w_time   = 0; // Wakeup time in ms
uint32_t s_time   = 0; // Go to sleep time in ms
float aw_time  = 0; // Awake time in sec

unsigned long lastSync = millis();

// called once on startup
void setup() {
    //Serial1.printlnf("System version: %s", System.version().c_str());
    Time.zone(-4);
    pinMode(D1, INPUT);
    pinMode(resetCycleCountPin, INPUT_PULLDOWN);
    if (digitalRead(resetCycleCountPin) == TRUE) {
            cycleNumber = 0;
            /*EEPROM.put(addr, cycleNumber); // Line used to initialized cycleNumber in EEPROM. Comment out after used.*/
    }

    //Serial1.begin(115200);
    //Serial1.printlnf("\n\n  ****** Electron Cellular STOP MODE Sleep Test. Publish N0_ACK! ****** Sleeping for %d min.", sleepTimeInMinutes);
    w_time = millis();
    /*EEPROM.get(addr, cycleNumber); // Get the cycleNumber from non-volatile storage*/
    //Serial1.printf("Cycle no.:\t%d\t", cycleNumber);
    //Serial1.println(Time.timeStr());
    Particle.connect(); // This imply Cellular.on() and Cellular.connect()
    waitUntil(Particle.connected);
    readCellularData("Initial data \t", TRUE);
    //Serial1.println("\n");
}

void loop() {
  if (Time.day() != lastDay || Time.year() < 2000){   // a new day calls for a sync
      Particle.connect();
      //Serial1.println("Sync time");
      if(waitFor(Particle.connected, 1*60000)){
        Particle.syncTime();
        start = millis();
        while (millis() - start < 1000UL) {
                Particle.process(); // Wait a second to received the time.
        }
        lastDay = Time.day();
      }
  }

    cycleNumber += 1;
    /*EEPROM.put(addr, cycleNumber); // Save it for next iteration*/

    waitUntil(Particle.connected);

    readCellularData("Before publish \t", TRUE);
    readBattery(); // print the battery voltage and state of charge
    checkSignal(); // Print signal RSSI and quality
    publishData(); // Publish a message indicating battery status

    s_time = millis(); //Sleep time is now
    aw_time = (float)(s_time - w_time) / 1000;
    //Serial1.printf("Awake\t%03.2f\tsec.", aw_time);

// sleeps duration corrected to next
    uint32_t dt = (sleepTimeInMinutes - Time.minute() % sleepTimeInMinutes) * 60 - Time.second() + TimeBoundaryOffset; // wake at next time boundary +30 seconds
    System.sleep(BTN, FALLING, dt, SLEEP_NETWORK_STANDBY); // use SETUP BUTTON (20) to wake

    w_time = millis();
    /*String ts = String::format("\nNo.:\t%d\t %s\t", cycleNumber, Time.timeStr().c_str());
    Serial1.printf(ts);*/
}

void publishData() {
    FuelGauge fuel;
    // Publish voltage and SOC, RSSI, QUALITY. plus Tx & Rx count
    sprintf(publishStr, "%d, %02.4f, %03.3f, %d, %d, %d, %d, %03.3f",
            cycleNumber - 1, fuel.getVCell(), fuel.getSoC(), signalRSSI, signalQuality, deltaTx, deltaRx, aw_time);
    Particle.publish(myEventName, publishStr, PRIVATE, NO_ACK);
    start = millis();
    while (millis() - start < 500UL) {
            Particle.process(); // Wait a second to received the time.
    }

}

void readBattery() {
    FuelGauge fuel;
    String batValue = "Battery\t" + String(fuel.getVCell()) + "V\t" + String(fuel.getSoC()) + "%%\t";
    //Serial1.printf( batValue );
}

void readCellularData(String s, bool prtFlag) {
    CellularData data;
    if (!Cellular.getDataUsage(data)) {
            Serial1.print("Error! Not able to get data.");
    }
    else {
          deltaTx = data.tx_session - txPrec;
          deltaRx = data.rx_session - rxPrec;
          if (prtFlag) {
          }
          txPrec = data.tx_session;
          rxPrec = data.rx_session;
    }
}

void checkSignal() {
    CellularSignal sig = Cellular.RSSI();
    signalRSSI = sig.rssi;
    signalQuality = sig.qual;
    String s = "RSSI.QUALITY: \t" + String(signalRSSI) + "\t" + String(signalQuality) + "\t";
    //Serial1.print(s);
}
