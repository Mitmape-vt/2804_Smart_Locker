//EEPROM FUNCTIONS

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

const byte colPins[3] = {6, 7, 8}; // keypad columns
const byte AROW = A0;               // analog input row line

const int GREEN_LED  = 4; // unlocked
const int YELLOW_LED = 3; // recording
const int RED_LED    = 2; // locked

const int ROW_ADC_TARGETS[4] = {133, 254, 367, 512}; // adc target values
const int ROW_TOL = 30; // adc value tolerance

char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

int addr = 0;
int enable = 0;  // 0 = verify/locked, 1 = recording

int identifyRow(int adc){
  for (int r = 0; r < 4; r++){
    if (abs(adc - ROW_ADC_TARGETS[r]) <= ROW_TOL) // find row by reading adc
      return r;
  }
  return -1; // no adc value reading
}

void setColumn(int colId){
  for (int i = 0; i < 3; i++)
    pinMode(colPins[i], INPUT_PULLUP);

  if (colId >= 0 && colId < 3){
    pinMode(colPins[colId], OUTPUT);
    digitalWrite(colPins[colId], LOW); // set read column low when pressed
  }
  delayMicroseconds(300);
}

char readKey(){
  for (int c = 0; c < 3; c++){
    setColumn(c);
    int adc = analogRead(AROW); 
    int row = identifyRow(adc); // find row from adc
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
// conditional to find which led should be on, only 1 at a time
void setLED(bool red, bool yellow, bool green) { 
  digitalWrite(RED_LED,    red    ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(GREEN_LED,  green  ? HIGH : LOW);
}

void setup(){
  Serial.begin(9600);

  pinMode(GREEN_LED, OUTPUT);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  setLED(true, false, false); //default red

  for (int c = 0; c < 3; c++){
    pinMode(colPins[c], INPUT);
  }

  analogRead(AROW);

  Serial.println();
  Serial.println("Press * to begin saving password, press * again to finish.");
  Serial.println("Type password to unlock; # returns to lock mode.");
}

void loop(){
  char key = readKey();
  if (key == '\0') return; // idle

  // recording
  if (enable == 1){
    setLED(false, true, false); // yellow ON

    if (key == '*'){
      EEPROM_write(addr, '*');
      Serial.println("Password saved.");
      enable = 0;
      addr = 0;
      setLED(true, false, false); // set red
    }
    else if (key != '#') {
      EEPROM_write(addr, key);
      addr++;
      Serial.print("Stored: "); Serial.println(key);
    }
    delay(150);
    return;
  }

  // Verify / Locked mode
  if (key >= '0' && key <= '9') {
    Serial.print("Input: "); Serial.println(key);
  }

  if (key == '*'){        // Enter recording mode
    enable = 1;
    addr = 0;
    Serial.println("Recording... press * to stop.");
    setLED(false, true, false); // yellow on
  }
  else if (key == '#'){   // Lock again
    enable = 0;
    addr = 0;
    Serial.println("Locking.");
    Serial.println("Locked.");
    setLED(true, false, false); // red on
  }
  else {                  // Check password
    char expected_val = EEPROM_read(addr);

    if (expected_val == (char)0xFF){
      Serial.println("No password saved.");
      addr = 0;
      setLED(true, false, false); // red on
    }
    else if (key == expected_val){
      addr++;
      if (EEPROM_read(addr) == '*'){
        Serial.println("Unlocked");
        addr = 0;
        setLED(false, false, true); // green on
      }
    } 
    else {
      addr = 0;
      setLED(true, false, false); // red on
    }
  }

  delay(150);
}
