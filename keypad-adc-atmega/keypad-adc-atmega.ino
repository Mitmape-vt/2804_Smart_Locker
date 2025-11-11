
// --- Sleep mode definitions ---
#define DIDR0 _SFR_MEM8(0x7E)
#define _SFR_MEM8(mem_addr) _MMIO_BYTE(mem_addr)
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define ADMUX  _SFR_MEM8(0x7C)
#define ADCSRA _SFR_MEM8(0x7A)
#define ADEN   7
#define sei()  __asm__ __volatile__("sei" ::: "memory")
#define cli()  __asm__ __volatile__("cli" ::: "memory")

#ifdef __cplusplus
#  define ISR(vector, ...) \
   extern "C" void vector(void) __attribute__((signal,__INTR_ATTRS)) __VA_ARGS__; \
   void vector(void)
#else
#  define ISR(vector, ...) \
   void vector(void) __attribute__((signal,__INTR_ATTRS)) __VA_ARGS__; \
   void vector(void)
#endif
#define _SLEEP_CONTROL_REG SMCR
#define SMCR _SFR_IO8(0x33)
#define _SFR_IO8(io_addr) _MMIO_BYTE((io_addr)+__SFR_OFFSET)
#define _BV(bit) (1 << (bit))
#define SM1 2
#define SE 0
#define _SLEEP_ENABLE_MASK _BV(SE)
#define set_sleep_mode(mode) \
  do { \
    _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~(_BV(1)|_BV(2)|_BV(3))) | (mode)); \
  } while(0)
#define sleep_enable() (_SLEEP_CONTROL_REG |= (uint8_t)_SLEEP_ENABLE_MASK)
#define sleep_disable() (_SLEEP_CONTROL_REG &= (uint8_t)(~_SLEEP_ENABLE_MASK))
#define sleep_cpu() __asm__ __volatile__("sleep" "\n\t" ::)

// EEPROM functions
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

// Pin assignments
const byte colPins[3] = {8,7,6};
const byte AROW = A0;
const int GREEN_LED  = 3;
const int YELLOW_LED = 4;
const int RED_LED    = 5;
const int BUZZER_PIN = 10;
const int SERVO_PIN  = 9;
const int SERVO_SENSE_PIN = A1;
const int WAKE_PIN   = 2;
const int BATTERY_PIN = A2;
const int LOW_BATT_LED = 11;

const int ROW_ADC_TARGETS[4] = {133,254,367,512};
const int ROW_TOL = 30;
const unsigned long IDLE_TIMEOUT = 30000; // 30s sleep timer
const float LOW_BATT_THRESHOLD = 7.5;
const unsigned long LOW_BATT_BLINK_INTERVAL = 15000; // every 15s

// state variables
bool recording=false;
bool unlocked=false;
bool match=true;
int addr=0;
unsigned long lastActivity=0;
int currentServoAngle=0;
unsigned long lastBlink=0;
bool lowBattLEDState=false;

// Keypad map
char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

//LED control
void setLED(bool red, bool yellow, bool green) {
  digitalWrite(RED_LED, red ? HIGH : LOW);
  digitalWrite(YELLOW_LED, yellow ? HIGH : LOW);
  digitalWrite(GREEN_LED, green ? HIGH : LOW);
}

// buzzer
void playErrorTone() {
   tone(BUZZER_PIN, 440, 400); delay(400); noTone(BUZZER_PIN); 
   }
void playSuccessTone() {
   tone(BUZZER_PIN, 880, 300); delay(300); noTone(BUZZER_PIN); 
   }
void playRecordTone() {
   tone(BUZZER_PIN, 660, 150); delay(150); noTone(BUZZER_PIN); 
   }

int identifyRow(int adc){
  for(int r=0;r<4;r++){
    if(abs(adc-ROW_ADC_TARGETS[r])<=ROW_TOL) return r;
  }
  return -1;
}
void setColumn(int c){
  for(int i=0;i<3;i++) pinMode(colPins[i],INPUT_PULLUP);
  if(c>=0&&c<3){ pinMode(colPins[c],OUTPUT); digitalWrite(colPins[c],LOW); }
  delayMicroseconds(300);
}
char readKey(){
  for(int c=0;c<3;c++){
    setColumn(c);
    int adc=analogRead(AROW);
    int row=identifyRow(adc);
    if(row!=-1){
      delay(20);
      if(identifyRow(analogRead(AROW))==row){
        while(identifyRow(analogRead(AROW))==row) delay(5);
        setColumn(-1);
        return keys[row][c];
      }
    }
  }
  setColumn(-1);
  return '\0';
}

// servo PWM setup
void setServoAngle(int angle){
  int pw = map(angle,0,180,1000,2000);
  OCR1A = pw*2;
  currentServoAngle = angle;
}

// battery check
void checkBattery() {
  int adc = analogRead(BATTERY_PIN);
  float vBatt = (adc * 5.0 / 1023.0) * ((100.0 + 82.0) / 82.0);

  static bool lowBatt = false;

  if (!lowBatt && vBatt < LOW_BATT_THRESHOLD) lowBatt = true;
  if (lowBatt && vBatt > 8.0) lowBatt = false;

  if (lowBatt) {
    if (millis() - lastBlink >= LOW_BATT_BLINK_INTERVAL) {
      lastBlink = millis();
      lowBattLEDState = !lowBattLEDState;
      digitalWrite(LOW_BATT_LED, lowBattLEDState ? HIGH : LOW);
    }
  } else {
    digitalWrite(LOW_BATT_LED, LOW);
  }
}

volatile bool woke=false;
ISR(INT0_vect){ woke=true; sleep_disable(); }

void enterSleepMode(){
  cli();
  setLED(false,false,false);
  digitalWrite(LOW_BATT_LED, LOW); // turn off low batt LED
  Serial.println("Entering Power-Down sleep...");

  ADCSRA &= ~(1<<ADEN); // disable ADC

  pinMode(WAKE_PIN, INPUT_PULLUP);
  delay(5);
  if(digitalRead(WAKE_PIN)==LOW){
    Serial.println("INT0 low before sleep — waiting...");
    while(digitalRead(WAKE_PIN)==LOW);
  }

  EICRA = (1<<ISC01);
  EIFR = (1<<INTF0);
  EIMSK = (1<<INT0);
  delay(5);
  EIFR = (1<<INTF0);

  set_sleep_mode(0x02);
  sleep_enable();
  sei();
  sleep_cpu(); // sleep

  // wakeup
  sleep_disable();
  EIMSK = 0;
  ADCSRA |= (1<<ADEN);
  Serial.println("Woke from sleep!");
  setLED(true,false,false);
  lastActivity = millis();
}

void setup(){
  Serial.begin(9600);
  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(WAKE_PIN,INPUT_PULLUP);
  pinMode(SERVO_SENSE_PIN,INPUT);
  pinMode(LOW_BATT_LED,OUTPUT);
  pinMode(BATTERY_PIN,INPUT);
  digitalWrite(LOW_BATT_LED,LOW);

  for(int col=0; col<3; col++) pinMode(colPins[col],INPUT);

  // servo pwm
  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999; OCR1A=3000;

  setLED(true,false,false);
  setServoAngle(0);
  lastActivity = millis();
  lastBlink = millis();
}

void loop(){
  char key = readKey();

  if(key!='\0'){
    lastActivity = millis();
    Serial.print("Key: "); Serial.println(key);

    if(recording){
      setLED(false,true,false);
      if(key=='*'){
        EEPROM_write(addr,'*');
        Serial.println("Password saved.");
        recording=false; addr=0;
        setLED(true,false,false); playRecordTone();
        return;
      }
      else if(key!='#'){
        EEPROM_write(addr,key); addr++;
        Serial.print("Stored: "); Serial.println(key);
        playRecordTone();
      }
      delay(150);
      return;
    }

    if(unlocked && key=='#'){
      Serial.println("Locked.");
      unlocked=false; addr=0; match=true;
      setServoAngle(0);
      setLED(true,false,false); playErrorTone();
      return;
    }

    if(key=='*'){
      recording=true; addr=0;
      Serial.println("Recording password...");
      setLED(false,true,false); playRecordTone();
      return;
    }

    if(key=='#'){
      char end=EEPROM_read(addr);
      if(match && end=='*'){
        Serial.println("Unlocked!");
        unlocked=true;
        addr=0; match=true;
        setServoAngle(180);
        setLED(false,false,true); playSuccessTone();
      } else {
        Serial.println("Incorrect password.");
        addr=0; match=true; setLED(true,false,false); playErrorTone();
      }
      return;
    }

    if(key>='0' && key<='9'){
      char expected=EEPROM_read(addr);
      if(expected==(char)0xFF){
        Serial.println("No password saved.");
        addr=0; match=true; setLED(true,false,false);
      } else {
        if(key!=expected) match=false;
        addr++; Serial.print("Entered: "); Serial.println(key);
      }
    }
    delay(150);
  }

  // check battery every loop
  checkBattery();

  // check for inactivity
  if(millis()-lastActivity > IDLE_TIMEOUT){
    enterSleepMode();
    lastActivity = millis();
  }
}
