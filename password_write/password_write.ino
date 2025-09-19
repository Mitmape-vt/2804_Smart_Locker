void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address and Data Registers */
EEAR = uiAddress;
EEDR = ucData;
/* Write logical one to EEMPE */
EECR |= (1<<EEMPE);
/* Start eeprom write by setting EEPE */
EECR |= (1<<EEPE);
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
/* Wait for completion of previous write */
while(EECR & (1<<EEPE))
;
/* Set up address register */
EEAR = uiAddress;
/* Start eeprom read by writing EERE */
EECR |= (1<<EERE);
/* Return data from Data Register */
return EEDR;
}

// Pin definitions
const byte rowPins[4] = {2, 3, 4, 5}; 
const byte colPins[3] = {6, 7, 8};    

int addr = 0;
int enable = 0;

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
        if( (keys[r][c] == 42) && (enable == 0))
          {
            enable=1;
            addr = 0;
            delay (500);
            return;
          
          }
        if (enable == 1)
        {
          Serial.print("Key pressed: ");
          Serial.println(keys[r][c]);
          EEPROM_write(addr, keys[r][c]);
          addr++;
          delay(250); // debounce delay
          if (keys[r][c] == 42)
          {  
            enable = 2;
          }
          
        }
        if (enable == 0)
        {
          if ( keys[r][c] == EEPROM_read(addr))
          {

            addr ++;
            if (EEPROM_read(addr)+1 == 42)
            {
              Serial.println("Unlocked");
            }
          }
          if ( keys[r][c] != EEPROM_read(addr))
          {

            addr = 0;
          }

          
        }
        
      }
    }

    // Deactivate row (set HIGH again)
    digitalWrite(rowPins[r], HIGH);
  }


}

