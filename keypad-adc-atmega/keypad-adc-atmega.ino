// ------------------------------------------------------
// SMART LOCKER
// Timer-based stall detection (2° steps, 20ms per step)
// Revert state when stalled
// Servo power-control code INCLUDED but COMMENTED OUT
// ------------------------------------------------------

// ---------------- PIN DEFINITIONS ----------------
const byte colPins[3] = {8,7,6};
const byte AROW = A0;

const int GREEN_LED  = 3;
const int YELLOW_LED = 4;
const int RED_LED    = 5;

const int BUZZER_PIN = 10;
const int SERVO_PIN  = 9;
const int SERVO_PWR_PIN = 12;   // COMMENTED OUT for now

const int WAKE_PIN   = 2;       // INT0
const int BATTERY_PIN = A1;
const int LOW_BATT_LED = 11;

// keypad ladder
const int ROW_ADC_TARGETS[4] = {133,254,367,512};
const int ROW_TOL = 30;

unsigned long lastActivity = 0;
const unsigned long IDLE_TIMEOUT = 15000UL;

const float LOW_BATT_THRESHOLD = 7.5;
unsigned long lastBlink = 0;

bool recording=false;
bool unlocked=false;    // TRUE = unlocked, FALSE = locked
bool match=true;
int addr=0;
int currentServoAngle=0;

// ------------------------------------------------------
// SERVO POWER CONTROL (COMMENTED OUT)
// ------------------------------------------------------
/*
void servoPowerOn()  { digitalWrite(SERVO_PWR_PIN, HIGH); }
void servoPowerOff() { digitalWrite(SERVO_PWR_PIN, LOW);  }
*/

// ------------------------------------------------------
// SERVO PWM SETUP
// ------------------------------------------------------
void setServoAngleRaw(int angle){
  int pw = map(angle,0,180,1000,2000);
  OCR1A = pw * 2;
  currentServoAngle = angle;
}

// ------------------------------------------------------
// TIMER-BASED STALL DETECTION
// Moves in 2° increments
// 20ms between steps
// revert behavior changed depending on direction
// ------------------------------------------------------

bool moveServoWithStall(int targetAngle, bool lockingMovement)
{
    int startAngle = currentServoAngle;
    int step = (targetAngle > startAngle) ? 2 : -2;

    for (int a = startAngle; (step > 0 ? a <= targetAngle : a >= targetAngle); a += step)
    {
        unsigned long start = millis();
        setServoAngleRaw(a);

        delay(20);  // 20ms per increment

        // --- Stall detection ---
        // If servo hasn't changed position within 20ms → stall
        if ((millis() - start) > 40)
        {
            // ----- Stalled -----
            tone(BUZZER_PIN, 440, 200);
            delay(250);

            if (lockingMovement)
            {
                // If we stalled while LOCKING → revert to UNLOCKED
                setServoAngleRaw(0);
                unlocked = true;
                setLED(false,false,true);
            }
            else
            {
                // If we stalled while UNLOCKING → revert to LOCKED
                setServoAngleRaw(180);
                unlocked = false;
                setLED(true,false,false);
            }

            return false;
        }
    }

    // Successful move
    return true;
}

// ------------------------------------------------------
// LED + BUZZER
// ------------------------------------------------------
void setLED(bool r, bool y, bool g){
  digitalWrite(RED_LED,   r);
  digitalWrite(YELLOW_LED,y);
  digitalWrite(GREEN_LED, g);
}

void playErrorTone()  { tone(BUZZER_PIN,440,300); delay(300); noTone(BUZZER_PIN); }
void playSuccessTone(){ tone(BUZZER_PIN,880,250); delay(250); noTone(BUZZER_PIN); }
void playRecordTone() { tone(BUZZER_PIN,660,150); delay(150); noTone(BUZZER_PIN); }

// ------------------------------------------------------
// EEPROM FUNCTIONS
// ------------------------------------------------------
void EEPROM_write(unsigned int a, unsigned char d) {
  while (EECR & (1<<EEPE));
  EEAR = a;
  EEDR = d;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
}
unsigned char EEPROM_read(unsigned int a) {
  while (EECR & (1<<EEPE));
  EEAR = a;
  EECR |= (1<<EERE);
  return EEDR;
}

// ------------------------------------------------------
// KEYPAD SCAN
// ------------------------------------------------------
int identifyRow(int adc){
  for(int r=0;r<4;r++)
    if(abs(adc-ROW_ADC_TARGETS[r])<=ROW_TOL)
      return r;
  return -1;
}

void setColumn(int c){
  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);
  if(c>=0){
    pinMode(colPins[c],OUTPUT);
    digitalWrite(colPins[c],LOW);
  }
  delayMicroseconds(200);
}

char readKeyNonBlocking() {
  static char lastKey='\0';
  static char confirmedKey='\0';
  static unsigned long stableSince=0;

  char current='\0';

  for(int c=0;c<3;c++){
    setColumn(c);
    int row=identifyRow(analogRead(AROW));
    if(row!=-1){
      current=keys[row][c];
      break;
    }
  }
  setColumn(-1);

  if(current!=lastKey){
    lastKey=current;
    stableSince=millis();
    return '\0';
  }

  if(current!='\0' && millis()-stableSince>25){
    if(confirmedKey=='\0'){
      confirmedKey=current;
      return current;
    }
  }

  if(current=='\0') confirmedKey='\0';

  return '\0';
}

// ------------------------------------------------------
// BATTERY CHECK
// ------------------------------------------------------
void checkBattery(){
  float vBatt = (analogRead(BATTERY_PIN)*5.0/1023.0)*((100.0+82.0)/82.0);
  static bool low=false;

  if(!low && vBatt<LOW_BATT_THRESHOLD) low=true;
  if(low && vBatt>8.0) low=false;

  if(!low){
    digitalWrite(LOW_BATT_LED,LOW);
    return;
  }

  if(millis()-lastBlink>12000UL){
    lastBlink=millis();
    for(int i=0;i<2;i++){
      digitalWrite(LOW_BATT_LED,HIGH); delay(80);
      digitalWrite(LOW_BATT_LED,LOW);  delay(80);
    }
  }
}

// ------------------------------------------------------
// SETUP
// ------------------------------------------------------
void setup(){
  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);

  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(LOW_BATT_LED,OUTPUT);

  pinMode(WAKE_PIN,INPUT_PULLUP);
  pinMode(BATTERY_PIN,INPUT);

  // pinMode(SERVO_PWR_PIN,OUTPUT); // COMMENTED OUT

  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);

  // Timer1 setup
  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999;

  setServoAngleRaw(0);
  setLED(true,false,false);

  lastActivity=millis();
  lastBlink=millis();
}

// ------------------------------------------------------
// LOOP
// ------------------------------------------------------
void loop(){
  char key = readKeyNonBlocking();

  if(key!='\0'){
    lastActivity=millis();

    // RECORD MODE
    if(recording){
      setLED(false,true,false);

      if(key=='*'){
        EEPROM_write(addr,'*');
        recording=false; addr=0;
        setLED(true,false,false);
        playRecordTone();
        return;
      }
      if(key!='#'){
        EEPROM_write(addr,key);
        addr++;
        playRecordTone();
      }
      delay(120);
      return;
    }

    // LOCKED → UNLOCK request
    if(!unlocked && key=='#'){
      char end = EEPROM_read(addr);
      if(match && end=='*'){
        addr=0; match=true;

        bool ok = moveServoWithStall(180, false);

        if(ok){
          unlocked = true;
          setLED(false,false,true);
          playSuccessTone();
        }

      } else {
        addr=0; match=true;
        setLED(true,false,false);
        playErrorTone();
      }
      return;
    }

    // UNLOCKED → LOCK request
    if(unlocked && key=='#'){
      bool ok = moveServoWithStall(0, true);

      if(ok){
        unlocked=false;
        setLED(true,false,false);
        playSuccessTone();
      }
      return;
    }

    // START PROGRAMMING
    if(key=='*'){
      recording=true; addr=0;
      setLED(false,true,false);
      playRecordTone();
      return;
    }

    // NUMBERS
    if(key>='0' && key<='9'){
      char expected = EEPROM_read(addr);
      if(expected==(char)0xFF){
        addr=0; match=true;
        setLED(true,false,false);
      } else {
        if(key!=expected) match=false;
        addr++;
      }
    }
  }

  checkBattery();

  if(millis()-lastActivity > IDLE_TIMEOUT){
    // You can re-enable sleep later
  }
}