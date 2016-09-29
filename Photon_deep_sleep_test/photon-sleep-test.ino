// **** Photon Sleep test ****

// #include "Particle.h"
#define sleepTimeInMinutes 1 // Duration of sleep
#define awakeDelay 3000 // Delay just befor sleep

SYSTEM_MODE(SEMI_AUTOMATIC);
/*SYSTEM_THREAD(ENABLED)*/

unsigned long waitOTA;
long sleepTimeInSec = 0;
int cycleNumber = 0;
int statusLed = D7;

void sendStream();
void checkSignal();

// System event handler to update firmware before sleep
void handle_firmware_update_events(system_event_t event, int param)
{
    Serial1.printlnf("got event %d with value %d", event, param);
    waitOTA = millis();
    while (millis() - waitOTA < 30000)
       Spark.process();
}

// called once on startup
void setup() {
    Time.zone(-4);
    pinMode(D0, INPUT);
    pinMode(statusLed, OUTPUT);

    Serial1.begin(115200);
    Serial1.printlnf("\n\nPhoton STOP mode Sleep Test.  ##### Sleeping for %d min. Delay before sleep: %d ms #####", sleepTimeInMinutes, awakeDelay);
    Serial1.printlnf("Initial wake-up time(ms): \t%d", millis());

    Particle.connect(); // Start network connection
    waitUntil(Particle.connected); // Block until the cloud is connected
    Particle.syncTime(); // Synchronize time with network
    delay(1000); // Gives some time for the cloud to respond

    // listen for Wi-Fi Listen events and Firmware Update events
    System.on(firmware_update_pending, handle_firmware_update_events);
}

void loop() {

    Serial1.printf("Cycle no.:\t%d\t", cycleNumber);
    Serial1.println(Time.timeStr());

    if(Particle.connected == false){
      Particle.connect();
      waitUntil(Particle.connected);
    }
    checkSignal(); // Print signal RSSI and quality
    sendStream(cycleNumber); // Publish a dummy message for test

    /*waitUntil(Particle.connected); // Block until the cloud is connected*/
    Serial1.printlnf("Sleep time(ms): \t%d", millis());
    delay(awakeDelay); // Time required for event to show up in tbe Particle console
    sleepTimeInSec = sleepTimeInMinutes * 60;
    /*dummySleep(sleepTimeInSec); // Simulate sleep with delay loop. For test purpose*/
    /*System.sleep(SLEEP_MODE_DEEP, sleepTimeInSec); // Deep sleep - Network shutdown and processor in stand-by mode. RESET on wake.*/
    System.sleep(D0, RISING, sleepTimeInSec); // Device in STOP mode. NO RESET on wake. Preserve SRAM and registers.

    Serial1.printlnf("\nWake-up time(ms): \t%d", millis());
    cycleNumber += 1;
}

void sendStream(int cycleCount) {
    bool success;

    digitalWrite(statusLed, HIGH);
    String data = "This is a 50 bytes long message. Cycle number = " + String(cycleCount);
    Serial1.println("Publishing!: " + data);

    success = Particle.publish("TestEvent", data, PRIVATE);
    if (!success) {
      Serial1.println("Publish FAILED!! ");
    }
    digitalWrite(statusLed, LOW);
}

void checkSignal() {
  String s = "RSSI: \t" + String(WiFi.RSSI());
  Serial1.println(s);
}

// Do nothing for a period equivalent to sleep
void dummySleep(long sleepTime){
  for (int i = 0; i <= sleepTime; i++){
    delay(982UL);
  }
}
