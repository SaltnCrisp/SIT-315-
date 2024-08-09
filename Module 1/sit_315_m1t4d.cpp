#include <avr/io.h>
#include <avr/interrupt.h>

const int pirPin1 = 8;   // PIR sensor 1 pin (PCINT0 - Pin 8)
const int pirPin2 = 9;   // PIR sensor 2 pin (PCINT1 - Pin 9)
const int redLedPin = 7; // Red LED pin
const int greenLedPin = 6; // Green LED pin
const int buzzerPin = 10; // Buzzer pin
const int timerLedPin = 13; // LED controlled by Timer1

volatile bool motionDetected = false;
volatile unsigned long lastMotionTime = 0;
const unsigned long motionTimeout = 2000; // 5 seconds timeout

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("System Initialized");

  // Initialize PIR sensor pins as input with pull-up resistors
  pinMode(pirPin1, INPUT_PULLUP);
  pinMode(pirPin2, INPUT_PULLUP);
  
  // Initialize LED pins and buzzer pin as output
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(timerLedPin, OUTPUT);

  // Set initial states
  digitalWrite(redLedPin, LOW);
  digitalWrite(greenLedPin, HIGH);
  digitalWrite(buzzerPin, LOW);
  digitalWrite(timerLedPin, LOW);

  // Enable Pin Change Interrupts for the pins
  PCICR |= (1 << PCIE0);   // Enable Pin Change Interrupt for PCINT0 to PCINT7 (Port B)
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1); // Enable PCINT0 (Pin 8) and PCINT1 (Pin 9)

  // Configure Timer1 for 1-second intervals
  cli(); // Disable global interrupts
  TCCR1A = 0; // Set entire TCCR1A register to 0
  TCCR1B = 0; // Set entire TCCR1B register to 0
  TCNT1 = 0;  // Initialize counter value to 0
  OCR1A = 15624; // Set compare match register for 1Hz increments (1 second)
  TCCR1B |= (1 << WGM12); // Turn on CTC mode (Clear Timer on Compare)
  TCCR1B |= (1 << CS12) | (1 << CS10); // Set prescaler to 1024
  TIMSK1 |= (1 << OCIE1A); // Enable Timer Compare Interrupt
  sei(); // Enable global interrupts
}

void loop() {
  // If motion was detected, check if the timeout has passed
  if (motionDetected && (millis() - lastMotionTime >= motionTimeout)) {
    motionDetected = false;
    handleMotion();
  }
}

// Pin Change Interrupt Service Routine (ISR) for PIR sensors
ISR(PCINT0_vect) {
  // Check which pin triggered the interrupt
  if (!(PINB & (1 << PB0)) || !(PINB & (1 << PB1))) { // PIR1 or PIR2 detected motion
    motionDetected = true;
    lastMotionTime = millis();
    handleMotion();
  }
}

// Timer1 Compare Match Interrupt Service Routine (ISR)
ISR(TIMER1_COMPA_vect) {
  // Toggle the LED every second
  digitalWrite(timerLedPin, !digitalRead(timerLedPin));

}

void handleMotion() {
  if (motionDetected) {
    // Motion detected
    digitalWrite(redLedPin, HIGH); // Turn on red LED
    digitalWrite(greenLedPin, LOW); // Turn off green LED
    digitalWrite(buzzerPin, HIGH); // Turn on buzzer
    Serial.println("Motion detected");
  } else {
    // No motion detected
    digitalWrite(redLedPin, LOW); // Turn off red LED
    digitalWrite(greenLedPin, HIGH); // Turn on green LED
    digitalWrite(buzzerPin, LOW); // Turn off buzzer
    Serial.println("No motion");
  }
}
