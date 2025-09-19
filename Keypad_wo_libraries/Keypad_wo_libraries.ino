// Pin definitions
const byte rowPins[4] = {2, 3, 4, 5}; 
const byte colPins[3] = {6, 7, 8};    

// Key mapping for the keypad
char keys[4][3] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};

void setup() {
  Serial.begin(9600);

  // Set rows as outputs
  for (int r = 0; r < 4; r++) {
    pinMode(rowPins[r], OUTPUT);
    digitalWrite(rowPins[r], HIGH); // Keep rows HIGH // Active Low
  }

  // Set columns as inputs with the internal pullup resistor
  for (int c = 0; c < 3; c++) {
    pinMode(colPins[c], INPUT_PULLUP);
  }
}

void loop() {
  for (int r = 0; r < 4; r++) {
    // Activate this row by pulling it LOW
    digitalWrite(rowPins[r], LOW);

    // Check all columns
    for (int c = 0; c < 3; c++) {
      if (digitalRead(colPins[c]) == LOW) { // Key pressed
        Serial.print("Key pressed: ");
        Serial.println(keys[r][c]);
        delay(250); // debounce delay
      }
    }

    // Deactivate row (set HIGH again)
    digitalWrite(rowPins[r], HIGH);
  }
}
