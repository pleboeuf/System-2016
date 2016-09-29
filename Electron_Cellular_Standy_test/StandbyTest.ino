// **** Electron Sleep test ****
//#include "cellular_hal.h"
#include "Particle.h"
#include "math.h"
#define sleepTimeInMinutes 5 // Duration of sleep
/*#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)*/

SYSTEM_MODE(SEMI_AUTOMATIC);

int addr = 0;
char publishStr[45];
int cycleNumber = 0;
float vacuum[4];
uint32_t start;
uint32_t txPrec = 0; // Previous tx data value
uint32_t rxPrec = 0; // Previous rx data value
uint32_t w_time = 0; // Wakeup time in ms
uint32_t s_time = 0; // Go to sleep time in ms
uint32_t aw_time = 0; // Awake time in sec

// Functions prototype
void readBattery();
void readCellularData();
void checkSignal();


unsigned long lastSync = millis();

// called once on startup
void setup() {
    Time.zone(-4);
    pinMode(D1, INPUT);
    pinMode(B0, INPUT_PULLDOWN);
    /*bool rstCntPin = digitalRead(B0);*/
    if (digitalRead(B0) == HIGH){
      cycleNumber = 0;
      EEPROM.put(addr, cycleNumber); // Line used to initialized cycleNumber in EEPROM. Comment out after used.
    }
    vacuum[0] = 19.5; vacuum[1] = 19.8; vacuum[2] = 20.1; vacuum[3] = 15.6;

    Serial1.begin(115200);
    Serial1.printlnf("\n\n  ****** Electron Cellular Deep Sleep Test. Publish with ACK! ****** Sleeping for %d min.", sleepTimeInMinutes);
    w_time = millis();
    EEPROM.get(addr, cycleNumber); // Get the cycleNumber from non-volatile storage
    Serial1.printf("Cycle no.:\t%d\t", cycleNumber);
    Serial1.println(Time.timeStr());
    cycleNumber += 1;
    EEPROM.put(addr, cycleNumber); // Save it for next iteration
    /*Serial1.printlnf("Initial wake time: \t%d", w_time);
    Serial1.println("Connecting Cellular...");*/
    /*Cellular.on(); // May be not necessary
    Cellular.connect(); // May be not necessary
    Serial1.println("Connecting to the cloud...");*/
    Particle.connect(); // This imply Cellular.on() and Cellular.connect()
    waitUntil(Particle.connected);
    readCellularData("Initial data \t");
}

void loop() {

    waitUntil(Particle.connected);
    /*readCellularData("Before publish \t");*/

    readBattery(); // print the battery voltage and state of charge
    checkSignal(); // Print signal RSSI and quality
    publishData(); // Publish a message indicating battery status

// Wait for completion of Publish and print before sleep
    while (millis() - start < 3000UL) {
      Particle.process();
    }
    readCellularData("After publish \t");
    s_time = millis(); //Sleep time is now
    Serial1.printlnf("Go to sleep time: \t%d", s_time);
    aw_time = round((s_time - w_time) / 1000);
    Serial1.printlnf("Awake for %d sec.", aw_time);

// sleeps duration corrected for awake time
    System.sleep(SLEEP_MODE_DEEP, sleepTimeInMinutes * 60 - aw_time - 4);
    w_time = millis();
    Serial1.printf("\nCycle no.:\t%d\t", cycleNumber);
    Serial1.println(Time.timeStr());
    Serial1.printlnf("Wake time: \t%d", w_time);
    readCellularData("After sleep \t");
}

void publishData() {
  FuelGauge fuel;
  // Publish voltage and SOC plus 4 dummy value
  sprintf(publishStr, "%d, %02.4fV, %03.2f, %03.1f, %03.1f, %03.1f, %03.1f",
          cycleNumber - 1, fuel.getVCell(), fuel.getSoC(), vacuum[0], vacuum[1], vacuum[2], vacuum[3]);
  Particle.publish("VA-1-2-3-4", publishStr, PRIVATE, NO_ACK);
  /*cycleNumber += 1;*/
  start = millis();
  while (millis() - start < 1000UL) {
    Particle.process();
  }
}

void readBattery() {
    FuelGauge fuel;
    String value = "Battery\t" + String(fuel.getVCell()) + "V\t" + String(fuel.getSoC()) + "%%";
    Serial1.printlnf( value );
}

void readCellularData(String s) {
    CellularData data;
    if (!Cellular.getDataUsage(data)) {
        Serial1.print("Error! Not able to get data.");
    }
    else {
        Serial1.printlnf(s + "CID: \t%d SESSION TX: \t%d \tRX: \t%d TOTAL TX: \t%d \tRX: \t%d Delta TX: \t%d \tRX: \t%d",
            data.cid,
            data.tx_session, data.rx_session,
            data.tx_total, data.rx_total,
            data.tx_session - txPrec, data.rx_session - rxPrec);
        txPrec = data.tx_total;
        rxPrec = data.rx_session;
  }
}

void checkSignal() {
  CellularSignal sig = Cellular.RSSI();
  String s = "RSSI.QUALITY: \t" + String(sig.rssi) + "\t" + String(sig.qual);
  Serial1.println(s);
}

//4.15V, 90.15, 19.1, 19.1, 19.1, 19.1
