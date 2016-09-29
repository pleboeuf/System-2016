#include "Particle.h"

SYSTEM_MODE(AUTOMATIC);
char publishStr[20];
int sleepInterval = 30;
uint32_t start;

void publishData() {

  FuelGauge fuel;
  sprintf(publishStr, "%02.2f %03.2f", fuel.getVCell(), fuel.getSoC());
  Particle.publish("VA-1-2-3-4", publishStr, PRIVATE, NO_ACK);
  start = millis();
  while (millis() - start < 1000UL) {
    Particle.process();
  }

} //void publishData()

void readCellularData(String s) {
    CellularData data;
    if (!Cellular.getDataUsage(data)) {
        Serial1.print("Error! Not able to get data.");
    }
    else {
        Serial1.printlnf(s + "CID: \t%d SESSION TX: \t%d \tRX: \t%d TOTAL TX: \t%d \tRX: \t%d",
            data.cid,
            data.tx_session, data.rx_session,
            data.tx_total, data.rx_total);
  }
}

void checkSignal() {
  CellularSignal sig = Cellular.RSSI();
  String s = "RSSI.QUALITY: " + String(sig.rssi) + String(",") + String(sig.qual);
  Serial1.println(s);
}

void setup() {
  Serial1.begin(115200);
  Serial1.println("\n\nElectron Cellular Standby Test!")
  Serial1.printlnf("Initial wake time: \t%d", millis);

  pinMode(D1, INPUT);
  pinMode(WKP, INPUT);
} //setup()

void loop() {

  publishData();
  readCellularData();

  System.sleep(D1, RISING, 60 * sleepInterval);

} //loop
