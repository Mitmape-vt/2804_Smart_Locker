// ===============================================================
// LOW POWER SMART LOCKER + TIMER-BASED STALL DETECTION (OPTION B1)
// Sleep via INT0, all registers manually defined (no includes)
// Servo power MOSFET preserved but NOT actively used (as requested)
// Stall detection ONLY during LOCKING. If stalled → revert instantly.
// ===============================================================


// --------------------------------------------------------------
// ---------------------- REGISTER MACROS -----------------------
// --------------------------------------------------------------
#define _MMIO_BYTE(mem_addr) (*(volatile unsigned char *)(mem_addr))
#define _SFR_MEM8(mem_addr)  _MMIO_BYTE(mem_addr)
#define _SFR_IO8(io_addr)    _MMIO_BYTE((io_addr) + __SFR_OFFSET)

#ifndef __SFR_OFFSET
#define __SFR_OFFSET 0x20
#endif

#define DIDR0  _SFR_MEM8(0x7E)
#define ADMUX  _SFR_MEM8(0x7C)
#define ADCSRA _SFR_MEM8(0x7A)
#define ADEN   7

#define SMCR   _SFR_IO8(0x33)
#define _SLEEP_CONTROL_REG SMCR
#define _BV(bit) (1 << (bit))
#define SE 0

#define set_sleep_mode(mode) \
  do { _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~(_BV(1)|_BV(2)|_BV(3))) | (mode)); } while (0)

#define sleep_enable()  (_SLEEP_CONTROL_REG |= (unsigned char)_BV(SE))
#define sleep_disable() (_SLEEP_CONTROL_REG &= (unsigned char)~_BV(SE))
#define sleep_cpu()     __asm__ __volatile__("sleep" "\n\t" ::)

#define sei()  __asm__ __volatile__("sei" ::: "memory")
#define cli()  __asm__ __volatile__("cli" ::: "memory")

// INT0 registers
#define EICRA _SFR_MEM8(0x69)
#define EIMSK _SFR_MEM8(0x3D)
#define EIFR  _SFR_MEM8(0x3C)
#define ISC01 1
#define INT0  0
#define INTF0 0

// Power Reduction
#define PRR _SFR_MEM8(0x64)

// UART
#define UCSR0B _SFR_MEM8(0xC1)


// --------------------------------------------------------------
// --------------------- PIN DEFINITIONS ------------------------
// --------------------------------------------------------------
const byte colPins[3] = {8,7,6};
const byte AROW = A0;

const int GREEN_LED  = 3;
const int YELLOW_LED = 4;
const int RED_LED    = 5;

const int BUZZER_PIN = 10;
const int SERVO_PIN  = 9;

const int SERVO_PWR_PIN = 12;  // kept but NOT used

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
bool unlocked=false;
bool match=true;
int addr=0;
int currentServoAngle=0;


// --------------------------------------------------------------
// ------------------------ KEYPAD MAP --------------------------
// --------------------------------------------------------------
char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};


// --------------------------------------------------------------
// -------------------------- EEPROM ----------------------------
// --------------------------------------------------------------
void EEPROM_write(unsigned int a, unsigned char d) {
  while (EECR & (1<<EEPE));
  EEAR=a; EEDR=d;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
}
unsigned char EEPROM_read(unsigned int a) {
  while (EECR & (1<<EEPE));
  EEAR=a; EECR |= (1<<EERE);
  return EEDR;
}


// --------------------------------------------------------------
// --------------------------- LEDS ------------------------------
// --------------------------------------------------------------
void setLED(bool r,bool y,bool g){
  digitalWrite(RED_LED, r);
  digitalWrite(YELLOW_LED, y);
  digitalWrite(GREEN_LED, g);
}


// --------------------------------------------------------------
// ------------------------ BUZZER ------------------------------
// --------------------------------------------------------------
void playErrorTone(){ tone(BUZZER_PIN,440,300); delay(300); noTone(BUZZER_PIN); }
void playSuccessTone(){ tone(BUZZER_PIN,880,250); delay(250); noTone(BUZZER_PIN); }
void playRecordTone(){ tone(BUZZER_PIN,660,150); delay(150); noTone(BUZZER_PIN); }


// --------------------------------------------------------------
// ---------------------- SERVO PWM -----------------------------
// --------------------------------------------------------------
void setServoAngle(int angle){
  int pw = map(angle,0,180,1000,2000);
  OCR1A = pw * 2;   // prescaler=8 → 2 counts/us
  currentServoAngle = angle;
}


// --------------------------------------------------------------
// -------------------- KEYPAD FUNCTIONS ------------------------
// --------------------------------------------------------------
int identifyRow(int adc){
  for(int r=0;r<4;r++)
    if(abs(adc-ROW_ADC_TARGETS[r]) <= ROW_TOL)
      return r;
  return -1;
}

void setColumn(int c){
  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);
  if(c>=0){
    pinMode(colPins[c],OUTPUT);
    digitalWrite(colPins[c],LOW);
  }
  delayMicroseconds(250);
}

char readKeyNonBlocking(){
  static char lastKey='\0';
  static char confirmed='\0';
  static unsigned long stable=0;

  char cur='\0';

  for(int c=0;c<3;c++){
    setColumn(c);
    int row=identifyRow(analogRead(AROW));
    if(row!=-1){
      cur = keys[row][c];
      break;
    }
  }
  setColumn(-1);

  if(cur != lastKey){
    lastKey=cur;
    stable=millis();
    return '\0';
  }

  if(cur!='\0' && millis()-stable > 30){
    if(confirmed=='\0'){
      confirmed=cur;
      return cur;
    }
  }

  if(cur=='\0') confirmed='\0';
  return '\0';
}


// --------------------------------------------------------------
// ------------------ BATTERY CHECK -----------------------------
// --------------------------------------------------------------
void checkBattery(){
  float vBatt=(analogRead(BATTERY_PIN)*5.0/1023.0)*((100.0+82.0)/82.0);
  static bool low=false;

  if(!low && vBatt<LOW_BATT_THRESHOLD) low=true;
  if(low && vBatt>8.0) low=false;

  if(!low){
    digitalWrite(LOW_BATT_LED,LOW);
    return;
  }

  if(millis()-lastBlink>12000){
    lastBlink=millis();
    for(int i=0;i<2;i++){
      digitalWrite(LOW_BATT_LED,HIGH); delay(80);
      digitalWrite(LOW_BATT_LED,LOW); delay(80);
    }
  }
}


// --------------------------------------------------------------
// ---------------- TIMER-BASED STALL DETECTION -----------------
// Option B1 – checks each incremental movement
// --------------------------------------------------------------
// CONFIG:
const int STEP_SIZE = 2;            // degrees per step
const int STEP_DELAY = 20;          // ms between steps
const int STALL_TIME_LIMIT = 150;   // ms allowed before stall

// return true = success, false = stalled
bool moveServoWithStall(int startAngle, int targetAngle){
  int dir = (targetAngle > startAngle ? 1 : -1);

  int angle = startAngle;
  unsigned long lastMove = millis();

  while(angle != targetAngle){
    angle += dir * STEP_SIZE;
    if(dir > 0 && angle > targetAngle) angle = targetAngle;
    if(dir < 0 && angle < targetAngle) angle = targetAngle;

    setServoAngle(angle);

    delay(STEP_DELAY);

    if(millis() - lastMove > STALL_TIME_LIMIT){
      return false;  // STALLED
    }

    lastMove = millis();
  }

  return true;
}


// --------------------------------------------------------------
// ----------------------- WAKE IRQ -----------------------------
// --------------------------------------------------------------
ISR(INT0_vect){
  sleep_disable();
}


// --------------------------------------------------------------
// --------------------- SLEEP MODE -----------------------------
// --------------------------------------------------------------
void enterSleepMode(){
  cli();

  setLED(0,0,0);
  digitalWrite(LOW_BATT_LED,0);

  ADCSRA &= ~(1<<ADEN);
  DIDR0 = 0x3F;
  PRR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5)|(1<<6)|(1<<7);
  UCSR0B = 0;

  pinMode(WAKE_PIN,INPUT_PULLUP);

  EICRA = (1<<ISC01);
  EIFR  = (1<<INTF0);
  EIMSK = (1<<INT0);

  set_sleep_mode(0x02);
  sleep_enable();
  sei();
  sleep_cpu();

  // WOKE
  sleep_disable();
  EIMSK = 0;
  PRR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5)|(1<<6)|(1<<7));
  ADCSRA |= (1<<ADEN);

  Serial.begin(9600);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(LOW_BATT_LED,OUTPUT);
  pinMode(BATTERY_PIN,INPUT);
  pinMode(WAKE_PIN,INPUT_PULLUP);
  pinMode(SERVO_PWR_PIN,OUTPUT);

  for(int c=0;c<3;c++)
    pinMode(colPins[c],INPUT_PULLUP);

  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999;

  setServoAngle(0);
  setLED(true,false,false);

  lastActivity=millis();
}


// --------------------------------------------------------------
// -------------------------- SETUP -----------------------------
// --------------------------------------------------------------
void setup(){
  Serial.begin(9600);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(LOW_BATT_LED,OUTPUT);
  pinMode(WAKE_PIN,INPUT_PULLUP);
  pinMode(BATTERY_PIN,INPUT);
  pinMode(SERVO_PWR_PIN,OUTPUT);

  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);

  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999;

  setServoAngle(0);
  setLED(true,false,false);

  lastActivity=millis();
  lastBlink=millis();
}


// --------------------------------------------------------------
// --------------------------- LOOP -----------------------------
// --------------------------------------------------------------
void loop(){

  char key = readKeyNonBlocking();

  if(key!='\0'){
    Serial.print("Key pressed: ");
    Serial.println(key);
  }

  // -----------------------------------------
  // LOCKING / UNLOCKING LOGIC + STALL DETECT
  // -----------------------------------------
  if(key!='\0'){
    lastActivity=millis();

    if(recording){
      setLED(0,1,0);
      if(key=='*'){
        EEPROM_write(addr,'*');
        recording=false; addr=0;
        setLED(1,0,0);
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

    // ------------------- LOCK -------------------
    if(unlocked && key=='#'){
      unlocked=false; addr=0; match=true;

      bool ok = moveServoWithStall(180,0);

      if(!ok){
        // STALLED → REVERT TO UNLOCKED
        moveServoWithStall(0,180);
        unlocked=true;
        setLED(0,0,1);
        playErrorTone();
        return;
      }

      setLED(1,0,0);
      playErrorTone();
      return;
    }

    // ------------------- ENTER RECORD -------------------
    if(key=='*'){
      recording=true; addr=0;
      setLED(0,1,0);
      playRecordTone();
      return;
    }

    // ------------------ UNLOCK -------------------
    if(key=='#'){
      char end=EEPROM_read(addr);
      if(match && end=='*'){
        addr=0; match=true;

        // No stall detection when UNLOCKING
        moveServoWithStall(0,180);

        unlocked=true;
        setLED(0,0,1);
        playSuccessTone();
      } else {
        addr=0; match=true;
        setLED(1,0,0);
        playErrorTone();
      }
      return;
    }

    // ------------------ NUMBER KEYS -------------------
    if(key>='0' && key<='9'){
      char expected=EEPROM_read(addr);
      if(expected==(char)0xFF){
        addr=0; match=true;
        setLED(1,0,0);
      } else {
        if(key!=expected) match=false;
        addr++;
      }
    }
  }

  checkBattery();

  if(millis()-lastActivity > IDLE_TIMEOUT)
    enterSleepMode();
}