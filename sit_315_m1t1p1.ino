const int pirPin = A0;   // PIR sensor pin
const int redLedPin = 7; // Red LED pin
const int greenLedPin = 6; // Green LED pin
const int buzzerPin = 10; // Buzzer pin

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Initialize PIR sensor pin as input
  pinMode(pirPin, INPUT);
  
  // Initialize LED pins and buzzer pin as output
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Read the state of the PIR sensor
  int pirState = analogRead(pirPin);
  
  // Check if motion is detected
  if (pirState > 512) { // Adjust the threshold as needed
    // Motion detected
    digitalWrite(redLedPin, HIGH); // Turn on red LED
    digitalWrite(greenLedPin, LOW); // Turn off green LED
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
    Serial.println("Motion detected");
  } else {
    // No motion
    digitalWrite(redLedPin, LOW); // Turn off red LED
    digitalWrite(greenLedPin, HIGH); // Turn on green LED
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
    Serial.println("No motion");
  }
  
  delay(500); // Wait for half a second
}
