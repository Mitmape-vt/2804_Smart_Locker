void EEPROM_write(unsigned int uiAddress, unsigned char ucData){
  while (EECR & (1<<EEPE));
  EEAR = uiAddress; EEDR = ucData;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
}
unsigned char EEPROM_read(unsigned int uiAddress){
  while (EECR & (1<<EEPE));
  EEAR = uiAddress;
  EECR |= (1<<EERE);
  return EEDR;
}

const byte colPins[3] = {6, 7, 8};    // cols digital
const byte AROW = A0;                  // analog read

const int ROW_ADC_TARGETS[4] =  {133, 254, 367, 512}; // target ADC values
const int ROW_TOL = 90;        // +- values for row 

char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

int addr = 0;
int enable = 0;  // 0 = verify/lock, 1 = record

int identifyRow(int adc){
  for (int r = 0; r < 4; r++){
    if (abs(adc - ROW_ADC_TARGETS[r]) <= ROW_TOL) return r;
  }
  return -1; 
}

void setColumn(int colId){
  for (int i = 0; i < 3; i++){
    pinMode(colPins[i], INPUT); 
  }
  if (colId >= 0 && colId < 3){
    pinMode(colPins[colId], OUTPUT);
    digitalWrite(colPins[colId], LOW);
  }

  delayMicroseconds(300);
}

char readKey(){
  // scan each column; read adc on key press
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

void setup(){
  Serial.begin(9600);

  // Columns start high
  for (int c = 0; c < 3; c++){
    pinMode(colPins[c], INPUT);
  }

  analogRead(AROW);

  Serial.println();
  Serial.println("Press * to begin saving password, press * again to finish.");
  Serial.println("type password to unlock; # returns to lock mode.");
}

void loop(){
  char key = readKey();
  if (key == '\0'){
    // No key, idle
    return;
  }

  // Recording mode
  if (enable == 1){
    if (key == '*'){
      EEPROM_write(addr, '*');
      Serial.println("Password saved.");
      enable = 0; addr = 0;
    } else if (key != '#') {
      EEPROM_write(addr, key);
      addr++;
      Serial.print("Stored: "); Serial.println(key);
    }
    delay(150);
    return;
  }

  if (key >= '0' && key <= '9') {
    Serial.print("Input: ");
    Serial.println(key);
  }

  // Verify / locked mode
  if (key == '*'){
    enable = 1; addr = 0;
    Serial.println("Recording... press * to stop.");
  }
  else if (key == '#'){
    enable = 0; addr = 0;
    Serial.println("Locking.");
    Serial.println("Locked.");
  }
  else {
    char expected_val = EEPROM_read(addr);

    if (expected_val == (char)0xFF){
      Serial.println("No password saved.");
      addr = 0;
    }
    else if (key == expected_val){
      addr++;
      if (EEPROM_read(addr) == '*'){
        Serial.println("Unlocked");
        addr = 0;
      }
    } else {
      addr = 0;
    }
  }

  delay(150);
}
