// --- Sleep mode definitions & low-level macros ---
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
  do { \
    _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~(_BV(1)|_BV(2)|_BV(3))) | (mode)); \
  } while (0)

#define sleep_enable()  (_SLEEP_CONTROL_REG |= (unsigned char)_BV(SE))
#define sleep_disable() (_SLEEP_CONTROL_REG &= (unsigned char)~_BV(SE))
#define sleep_cpu()     __asm__ __volatile__("sleep" "\n\t" ::)

#define sei()  __asm__ __volatile__("sei" ::: "memory")
#define cli()  __asm__ __volatile__("cli" ::: "memory")

// External interrupt registers (INT0)
#define EICRA _SFR_MEM8(0x69)
#define EIMSK _SFR_MEM8(0x3D)
#define EIFR  _SFR_MEM8(0x3C)
#define ISC01 1
#define INT0  0
#define INTF0 0

// Power Reduction Register
#define PRR _SFR_MEM8(0x64)
#define UCSR0B _SFR_MEM8(0xC1)

// =====================================================
//      SMART LOCKER – NORMAL SERVO + STALL DETECTION
// =====================================================

// ---------------- PIN DEFINITIONS ----------------
const byte colPins[3] = {8,7,6};
const byte AROW = A0;

const int GREEN_LED  = 3;
const int YELLOW_LED = 4;
const int RED_LED    = 5;

const int BUZZER_PIN = 10;
const int SERVO_PIN  = 9;

// future MOSFET power control (NOT USED)
// const int SERVO_PWR_PIN = 12;

const int WAKE_PIN = 2;
const int BATTERY_PIN = A1;
const int LOW_BATT_LED = 11;


// keypad ladder
const int ROW_ADC_TARGETS[4] = {133,254,367,512};
const int ROW_TOL = 30;

// state + timing
unsigned long lastActivity = 0;
unsigned long IDLE_TIMEOUT = 15000;
unsigned long lastBlink = 0;

bool recording=false;
bool unlocked=false;
bool match=true;
int addr=0;
int currentServoAngle=0;

// keypad map (fixes your compile error)
char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

// ---------------- EEPROM ----------------
void EEPROM_write(unsigned int a, unsigned char d) {
  while (EECR & (1<<EEPE));
  EEAR = a;  EEDR = d;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
}
unsigned char EEPROM_read(unsigned int a) {
  while (EECR & (1<<EEPE));
  EEAR = a;  EECR |= (1<<EERE);
  return EEDR;
}

// ---------------- LEDs & BUZZER ----------------
void setLED(bool r, bool y, bool g){
  digitalWrite(RED_LED,r);
  digitalWrite(YELLOW_LED,y);
  digitalWrite(GREEN_LED,g);
}

void playErrorTone(){ tone(BUZZER_PIN,440,250); delay(250); noTone(BUZZER_PIN); }
void playSuccessTone(){ tone(BUZZER_PIN,880,200); delay(200); noTone(BUZZER_PIN); }
void playRecordTone(){ tone(BUZZER_PIN,660,130); delay(130); noTone(BUZZER_PIN); }

// ---------------- SERVO ----------------
void setServoAngle(int angle){
  int pw = map(angle,0,180,1000,2000);
  OCR1A = pw * 2;
  currentServoAngle = angle;
}

// ---------------- STEP-BASED STALL DETECTION ----------------
// Returns true if movement stalled
bool moveServoWithStall(int startAngle, int targetAngle, bool lockingMode){
  int step = (targetAngle > startAngle ? 2 : -2);
  int pos = startAngle;

  unsigned long moveTimeout = 40;  // ms between steps (medium-fast)

  while (pos != targetAngle){
    pos += step;
    if ((step > 0 && pos > targetAngle) || (step < 0 && pos < targetAngle))
        pos = targetAngle;

    setServoAngle(pos);
    delay(moveTimeout);

    // detect stall: servo tries to move but external force prevents movement
    // simple way: check if position didn't actually change
    // (requires physical feedback: your servo "clicks" when stalled)
    // We detect stall as "clicking for >60 ms"
    
    static int stillCount = 0;
    if (pos == currentServoAngle){
      stillCount++;
      if (stillCount >= 3){  // ~120ms
         if (lockingMode){
           // BLOCKED WHILE LOCKING → revert to unlocked
           setLED(true,false,false);  // red
           playErrorTone();
           return true;
         }
      }
    } else {
      stillCount = 0;
    }
  }

  return false;
}

// ---------------- KEYBOARD ----------------
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

char readKeyNonBlocking(){
  static char lastKey='\0', confirmed='\0';
  static unsigned long t=0;
  char cur='\0';

  for(int c=0;c<3;c++){
    setColumn(c);
    int row=identifyRow(analogRead(AROW));
    if(row!=-1){ cur=keys[row][c]; break; }
  }
  setColumn(-1);

  if(cur!=lastKey){
    lastKey=cur;
    t=millis();
    return '\0';
  }

  if(cur!='\0' && millis()-t>30){
    if(confirmed=='\0'){
      confirmed=cur;
      return cur;
    }
  }

  if(cur=='\0') confirmed='\0';
  return '\0';
}

// ---------------- BATTERY ----------------
void checkBattery(){
  float v = (analogRead(BATTERY_PIN)*5.0/1023.0)*((100.0+82.0)/82.0);
  static bool low=false;

  if(!low && v<7.5) low=true;
  if(low && v>8.0) low=false;

  if(!low){ digitalWrite(LOW_BATT_LED,0); return; }

  if(millis()-lastBlink > 12000){
    lastBlink=millis();
    for(int i=0;i<2;i++){
      digitalWrite(LOW_BATT_LED,1); delay(80);
      digitalWrite(LOW_BATT_LED,0); delay(80);
    }
  }
}

// ---------------- SLEEP MODE ----------------
ISR(INT0_vect){ sleep_disable(); }

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

  sleep_disable();
  EIMSK=0;
  PRR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5)|(1<<6)|(1<<7));
  ADCSRA |= (1<<ADEN);

  Serial.begin(9600);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);

  for(int c=0;c<3;c++) pinMode(colPins[c],INPUT_PULLUP);

  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999;

  setServoAngle(0);
  setLED(true,false,false);

  lastActivity=millis();
}

// ---------------- SETUP ----------------
void setup(){
  Serial.begin(9600);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(WAKE_PIN,INPUT_PULLUP);
  pinMode(BATTERY_PIN,INPUT);
  pinMode(LOW_BATT_LED,OUTPUT);

  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);

  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999;

  setServoAngle(0);
  setLED(true,false,false);

  lastActivity=millis();
  lastBlink=millis();
}

// ---------------- LOOP ----------------
void loop(){
  char key = readKeyNonBlocking();
  if(key!='\0') Serial.println(key);

  if(key!='\0'){
    lastActivity=millis();

    // --- recording mode ---
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

    // --- unlocking returns to lock ---
    if(unlocked && key=='#'){
      unlocked=false; addr=0; match=true;
      moveServoWithStall(180,0,true);
      setLED(true,false,false);
      playErrorTone();
      return;
    }

    if(key=='*'){
      recording=true; addr=0;
      setLED(false,true,false);
      playRecordTone();
      return;
    }

    if(key=='#'){
      char e = EEPROM_read(addr);

      if(match && e=='*'){
         addr=0; match=true;

         // **UNLOCK**
         bool stalled = moveServoWithStall(0,180,false);
         unlocked=true;
         setLED(false,false,true);
         playSuccessTone();
      }
      else{
         addr=0; match=true;
         setLED(true,false,false);
         playErrorTone();
      }
      return;
    }

    // numeric key
    if(key>='0' && key<='9'){
      char expected=EEPROM_read(addr);
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

  if(millis() - lastActivity > IDLE_TIMEOUT){
    enterSleepMode();
  }
}