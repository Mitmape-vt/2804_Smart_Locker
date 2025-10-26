// EEPROM functions
void EEPROM_write(unsigned int uiAddress, unsigned char ucData) {
  while (EECR & (1 << EEPE));
  EEAR = uiAddress;
  EEDR = ucData;
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress) {
  while (EECR & (1 << EEPE));
  EEAR = uiAddress;
  EECR |= (1 << EERE);
  return EEDR;
}

// pin assignments
const byte colPins[3] = {8, 7, 6}; // keypad columns
const byte AROW = A0;               // analog input row line

const int GREEN_LED  = 3; // unlocked
const int YELLOW_LED = 4; // recording
const int RED_LED    = 5; // locked
const int BUZZER_PIN = 9; // piezo buzzer

const int ROW_ADC_TARGETS[4] = {133, 254, 367, 512}; // target adc values
const int ROW_TOL = 30; // adc tolerance range

char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// state variables and global variables
bool recording = false;
bool unlocked  = false;
int addr = 0;
bool match = true;  // tracks if keys so far matched stored password

// keypad functions
int identifyRow(int adc){
  for (int r = 0; r < 4; r++){
    if (abs(adc - ROW_ADC_TARGETS[r]) <= ROW_TOL)
      return r;
  }
  return -1; 
}

void setColumn(int colId){
  for (int i = 0; i < 3; i++)
    pinMode(colPins[i], INPUT_PULLUP);
  if (colId >= 0 && colId < 3){
    pinMode(colPins[colId], OUTPUT);
    digitalWrite(colPins[colId], LOW);
  }
  delayMicroseconds(300);
}
// read key function
char readKey(){
  for (int c = 0; c < 3; c++){
    setColumn(c);
    int adc = analogRead(AROW);
    int row = identifyRow(adc);
    if (row != -1){
      delay(20);
      int adc2 = analogRead(AROW);
      int row2 = identifyRow(adc2);
      if (row2 == row){
        while (true){
          int a = analogRead(AROW);
          int rr = identifyRow(a);
          if (rr != row) break;
          delay(5);
        }
        setColumn(-1);
        return keys[row][c];
      }
    }
  }
  setColumn(-1);
  return '\0';
}

// led control conditional statements so that one led is on at a time
void setLED(bool red, bool yellow, bool green) {
  digitalWrite(RED_LED,    red    ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(GREEN_LED,  green  ? HIGH : LOW);
}

// buzzer tones
void playErrorTone() {
  tone(BUZZER_PIN, 440, 400);
  delay(400);
  noTone(BUZZER_PIN);
}
void playSuccessTone() {
  tone(BUZZER_PIN, 880, 300);
  delay(300);
  noTone(BUZZER_PIN);
}
void playRecordTone() {
  tone(BUZZER_PIN, 660, 150);
  delay(150);
  noTone(BUZZER_PIN);
}


void setup(){
  Serial.begin(9600);
  // set led and buzzer to pins then set it to red/locked default state
  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  setLED(true, false, false); // start locked (red)

  for (int c = 0; c < 3; c++)
    pinMode(colPins[c], INPUT);

  analogRead(AROW);

  Serial.println();
  Serial.println("Press * to record new password, * again to stop.");
  Serial.println("# acts as Enter when locked, and Lock when unlocked.");
}

void loop(){
  char key = readKey();
  if (key == '\0') return;

  // if recording variable true
  if (recording) {
    setLED(false, true, false); // led yellow
    if (key == '*') { // end password save on second press of *
      EEPROM_write(addr, '*');
      Serial.println("Password saved. Locked.");
      recording = false;
      addr = 0;
      setLED(true, false, false);
      playRecordTone();
      return;
    }  // save key to address if not lock/enter key #
    else if (key != '#') {
      EEPROM_write(addr, key);
      addr++;
      Serial.print("Stored: "); Serial.println(key);
      playRecordTone();
    }
    delay(150);
    return;
  }

  // unlocked mode
  if (unlocked) {
    if (key == '#') { // if # pressed act as lock key
      Serial.println("System locked.");
      unlocked = false;
      addr = 0;
      match = true;
      setLED(true, false, false);
      return;
    }
    else return;
  }

  // locked mode
  if (key == '*') { // if * press then set recording to true and begin record sequence
    recording = true;
    addr = 0;
    Serial.println("Recording... press * again to stop.");
    setLED(false, true, false);
    playRecordTone();
    return;
  }

  if (key == '#') { // if # key then compare to saved value
    // When Enter pressed, evaluate
    char end = EEPROM_read(addr);
    if (match && end == '*') {
      Serial.println("Unlocked!");
      unlocked = true;
      addr = 0;
      match = true;
      setLED(false, false, true);
      playSuccessTone();
    } else {
      Serial.println("Incorrect password. Locked.");
      addr = 0;
      match = true;
      setLED(true, false, false);
      playErrorTone();
    }
    return;
  }

  // individual input to keypad
  if (key >= '0' && key <= '9') {
    char expected = EEPROM_read(addr);
    if (expected == (char)0xFF) {
      Serial.println("No password saved.");
      addr = 0;
      match = true;
      setLED(true, false, false);
    } else {
      if (key != expected) match = false;
      addr++;
      Serial.print("Entered: "); Serial.println(key);
    }
  }

  delay(150);
}
