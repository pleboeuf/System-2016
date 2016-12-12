// This will publish the values for the preceding cycle.
#define myEventName "Usage" // Usage data
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

char publishStr[64];
int      SleepTimeMin = 5;
int      SleepTimeSec = SleepTimeMin * 60;
int      pubDelay = 1000UL;
int      txPrec   = 0;      // Previous tx data count
int      rxPrec   = 0;      // Previous rx data count
int      deltaTx  = 0;      // Difference tx data count
int      deltaRx  = 0;      // Difference rx data count
uint32_t start    = 0;
uint32_t w_time   = 0; // Wakeup time in ms
float    aw_time  = 0; // Awake time in sec
int      prev_voltage  = 0;
int      prev_soc = 0;
float    prev_aw_time  = 0; // Awake time in sec

int      cycleNumber = 0;

void setup()
{
    Serial1.begin(115200);
    pinMode(D7, OUTPUT);
    digitalWrite(D7, LOW);
    readCellularData();
}

void loop()
{
    if (cycleNumber > 0 && cycleNumber % 10 == 0){
      SleepTimeMin = SleepTimeMin * 2; // Extend the sleep time every 60 cycles
      /*if(SleepTimeMin > 20){SleepTimeMin = 20;} // Maximum 20 min sleep*/
    }
    digitalWrite(D7,HIGH); // Visual indication of code execution
    waitUntil(Particle.connected);

    CellularSignal sig = Cellular.RSSI();
    readCellularData();
    sprintf(publishStr, "%d, %2.4f, %3.3f, %d, %d, %d, %d, %3.3f",
            cycleNumber++, fuel.getVCell(), fuel.getSoC(), sig.rssi, sig.qual, deltaTx, deltaRx, prev_aw_time);
    aw_time = (float)((millis() - w_time)) / 1000;
    Particle.publish(myEventName, publishStr, 60, PRIVATE);
    prev_aw_time = aw_time;
    for(uint32_t ms = millis(); millis() - ms < pubDelay; Particle.process());
    uint32_t dt = (SleepTimeMin - Time.minute() % SleepTimeMin) * 60 - Time.second(); // wake at next time boundary

    digitalWrite(D7,LOW); // Visual indication of code execution
    System.sleep(D0, RISING, dt, SLEEP_NETWORK_STANDBY);
    w_time = millis(); // Wakeup time
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

void readFuelGauge(){
    FuelGauge fuel;

}
