// sleep mode definitions
#define DIDR0 _SFR_MEM8(0x7E)
#define _SFR_MEM8(mem_addr) _MMIO_BYTE(mem_addr)
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define ADC0D 0
#define ADC1D 1
#define ADC2D 2
#define ADC3D 3
#define ADC4D 4
#define ADC5D 5
#define ADMUX  _SFR_MEM8(0x7C)
#define REFS0  6
#define REFS1  7
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
#define SM0 1
#define SM1 2
#define SM2 3
#define SE 0
#define _SLEEP_ENABLE_MASK _BV(SE)
#define set_sleep_mode(mode) \
  do { \
    _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~(_BV(SM0)|_BV(SM1)|_BV(SM2))) | (mode)); \
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

// pin assignments
const byte colPins[3] = {8,7,6};   // keypad columns
const byte AROW = A0;              // analog row input
const int GREEN_LED  = 3;
const int YELLOW_LED = 4;
const int RED_LED    = 5;
const int BUZZER_PIN = 10;
const int SERVO_PIN  = 9;
const int WAKE_PIN   = 2;

const int ROW_ADC_TARGETS[4] = {133,254,367,512};
const int ROW_TOL = 30;

// state variables

bool recording=false; 
bool unlocked=false;
bool match=true;
int addr=0;
unsigned long lastActivity=0;
const unsigned long IDLE_TIMEOUT=30000; // 30 s idle sleep timer

// keypad mapping
char keys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

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

// row and column identification as well as readkey function
int identifyRow(int adc){
  for(int r=0;r<4;r++){
    if(abs(adc-ROW_ADC_TARGETS[r])<=ROW_TOL){ 
      return r;
    }
  }
  return -1;
}


void setColumn(int c){

  for(int i=0;i<3;i++){

    pinMode(colPins[i],INPUT_PULLUP);
  }
  if(c>=0&&c<3){ 

    pinMode(colPins[c],OUTPUT); digitalWrite(colPins[c],LOW); 
  }
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

// servo angle control
void setServoAngle(int angle){
  int pw=map(angle,0,180,1000,2000);
  OCR1A=pw*2; // 0.5 µs per tick @ prescaler 8
}

// interrupt service routine and sleep_disable()
volatile bool woke=false;
ISR(INT0_vect){
  woke=true;
  sleep_disable();
}

// enter sleep mode and set triggers for falling edge and enabling the interrupt
void enterSleepMode(){
  cli();
  setLED(false,false,false);
  Serial.println("Entering Power-Down sleep...");
  
  // configure wakeup on falling edge trigger
  EICRA = (1 << ISC01);  // falling edge triggers
  EIFR  = (1 << INTF0);  // clear flag
  EIMSK = (1 << INT0);   // enable INT0 interrupt
  
  set_sleep_mode(0x02);  // enter power down mode
  sleep_enable();
  sei();
  sleep_cpu();

  // wakes up here after ISR triggers
  sleep_disable();
  EIMSK = 0;              // turn off int0 to stop from triggering anything else on d2
  Serial.println("Woke from sleep!");
  setLED(true,false,false);
  lastActivity=millis();
}

void setup(){

  Serial.begin(9600);

  pinMode(GREEN_LED,OUTPUT);
  pinMode(YELLOW_LED,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(SERVO_PIN,OUTPUT);
  pinMode(WAKE_PIN,INPUT);

  for(int col = 0; col < 3; col++){
    pinMode(colPins[col],INPUT);
  }

  // Servo PWM setup
  TCCR1A=(1<<WGM11)|(1<<COM1A1);
  TCCR1B=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1=39999; OCR1A=3000;
  
  setLED(true,false,false);
  setServoAngle(0);
  lastActivity=millis();
}

void loop(){
  // adc reading to serial monitor for debugging
  Serial.print("ADC: ");
  Serial.println(analogRead(AROW));

  char key=readKey();

  if(key!='\0'){

    lastActivity=millis();
    Serial.print("Key: "); Serial.println(key);

    if(recording){

      setLED(false,true,false);
      if(key=='*'){

        EEPROM_write(addr,'*');
        Serial.println("Password saved.");

        recording=false; 
        addr=0;
      
        setLED(true,false,false);
        playRecordTone();
        return;

      }
      else if(key!='#'){

        EEPROM_write(addr,key); 
        addr++;

        Serial.print("Stored: "); 
        Serial.println(key);
        playRecordTone();

      }
      delay(150); 
      return;
    }

    if(unlocked){

      if(key=='#'){

        Serial.println("Locked.");

        unlocked=false; 
        addr=0; 
        match=true;

        setServoAngle(0);
        setLED(true,false,false);
        playErrorTone();

      }
      return;
    }

    if(key=='*'){

      recording=true; 
      addr=0;

      Serial.println("Recording password...");
      setLED(false,true,false); 
      playRecordTone(); 
      return;

    }

    if(key=='#'){

      char end=EEPROM_read(addr);
      if(match && end=='*'){

        Serial.println("Unlocked!");
        unlocked=true;

        setServoAngle(180);
        addr=0; 
        match=true;

        setLED(false,false,true); 
        playSuccessTone();

      }
      else{
        Serial.println("Incorrect password.");
        addr=0; 
        match=true;
        setLED(true,false,false); 
        playErrorTone();
      }
      return;
    }

    if(key>='0'&&key<='9'){

      char expected=EEPROM_read(addr);
      if(expected==(char)0xFF){

        Serial.println("No password saved.");
        addr=0; 
        match=true; 
        setLED(true,false,false);
      }else{

        if(key!=expected){ 
          match=false;
        }
        addr++; 
        Serial.print("Entered: "); 
        Serial.println(key);
      }
    }
    delay(150);
  }

  // check for inactivity
  if(millis()-lastActivity> IDLE_TIMEOUT){
    enterSleepMode();
    lastActivity=millis();
  }
}
