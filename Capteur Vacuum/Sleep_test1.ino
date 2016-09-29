// First sleep test of the Electron
// Define the pins we're going to call pinMode on
int led = D6;  // You'll need to wire an LED to this one to see it blink.
int led2 = D7; // This one is the built-in tiny one to the right of the USB jack
int power = A5; // Power for the photoresistor
int photoresistor = A0; // Photoresistor input
int analogvalue; // Photoresistor voltage value

// This routine runs only once upon reset
void setup() {
  // Initialize D0 + D7 pin as output
  // It's important you do this here, inside the setup() function rather than outside it or in the loop function.
  pinMode(led, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(photoresistor,INPUT);
  pinMode(power,OUTPUT);
  Serial.begin(115200);
}

void blinkOnce(int blinkDelay){
  digitalWrite(led, HIGH);   // Turn ON the LED pins
  digitalWrite(led2, LOW);
  /*Serial.print(LOW);*/
  delay(blinkDelay);               // Wait for 1000mS = 1 second
  digitalWrite(led, LOW);    // Turn OFF the LED pins
  digitalWrite(led2, HIGH);
  /*Serial.print(HIGH);*/
  delay(blinkDelay);               // Wait for 1 second in off mode
}

// This routine gets called repeatedly, like once every 5-15 milliseconds.
// Spark firmware interleaves background CPU activity associated with WiFi + Cloud activity with your code.
// Make sure none of your code delays or blocks for too long (like more than 5 seconds), or weird things can happen.
void loop() {
  int temp1 = 500;
  int temp2 = 250;
  blinkOnce(temp1);
  blinkOnce(temp1);
  digitalWrite(power, HIGH);
  blinkOnce(temp1);
  analogvalue = analogRead(photoresistor);
  digitalWrite(power, LOW);
  Serial.print("analogvalue: ");
  Serial.println(analogvalue * 3.3 / 4096);
  /*System.sleep(SLEEP_MODE_DEEP, 30UL);*/
  /*System.sleep(D0, RISING, 30UL);*/
  blinkOnce(temp2);
}
