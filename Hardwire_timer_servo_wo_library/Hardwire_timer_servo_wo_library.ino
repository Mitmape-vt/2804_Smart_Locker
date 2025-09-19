// Output Check
void setup() {
  // Set pin 9 as output
  pinMode(9, OUTPUT);

  // Configure Timer1 for Fast PWM, mode 14 
  TCCR1A = (1 << WGM11) | (1 << COM1A1);  // Fast PWM, clear OC1A on compare match
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8

  ICR1 = 39999;   // TOP → 20 ms period at 16 MHz / 8 prescaler
  OCR1A = 3000;   // default pulse width (≈1.5 ms = 90°)
}

void loop() {
  // Sweep forward
  for (int angle = 0; angle <= 180; angle++) {
    setServoAngle(angle);
    delay(15);
  }
  // Sweep backward
  for (int angle = 180; angle >= 0; angle--) {
    setServoAngle(angle);
    delay(15);
  }
}

// Map angle (0–180) to pulse width (1000–2000 µs)
void setServoAngle(int angle) {
  int pulseWidth = map(angle, 0, 180, 1000, 2000); // microseconds
  OCR1A = pulseWidth * 2;  // Timer1 ticks = 0.5 µs at prescaler 8
}
