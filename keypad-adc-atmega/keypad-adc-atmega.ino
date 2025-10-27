// EEPROM functions
// Servo and ADC Definitions



#define DIDR0 _SFR_MEM8(0x7E)
#define _SFR_MEM8(mem_addr) _MMIO_BYTE(mem_addr)
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define ADC0D 0
#define ADC1D 1
#define ADC2D 2
#define ADC3D 3
#define ADC4D 4
#define ADC5D 5
#define ADMUX _SFR_MEM8(0x7C)
#define REFS0 6
#define REFS1 7
#define ADCSRB _SFR_MEM8(0x7B)
#define ADCSRA _SFR_MEM8(0x7A)
#define ADPS0 0
#define ADPS1 1
#define ADPS2 2
#define ADIE 3
#define ADIF 4
#define ADATE 5
#define ADSC 6
#define ADEN 7
#define ADC_vect_num      21
#define ADC_vect          _VECTOR(21)  /* ADC Conversion Complete */
#ifndef _VECTOR
#define _VECTOR(N) __vector_ ## N
#endif

#if defined(__DOXYGEN__)
/** \def sei()
    \ingroup avr_interrupts

    Enables interrupts by setting the global interrupt mask. This function
    actually compiles into a single line of assembly, so there is no function
    call overhead.  However, the macro also implies a <i>memory barrier</i>
    which can cause additional loss of optimization.

    In order to implement atomic access to multi-byte objects,
    consider using the macros from <util/atomic.h>, rather than
    implementing them manually with cli() and sei().
*/
#define sei()
#else  /* !DOXYGEN */
# define sei()  __asm__ __volatile__ ("sei" ::: "memory")
#endif /* DOXYGEN */

#if defined(__DOXYGEN__)
/** \def cli()
    \ingroup avr_interrupts

    Disables all interrupts by clearing the global interrupt mask. This function
    actually compiles into a single line of assembly, so there is no function
    call overhead.  However, the macro also implies a <i>memory barrier</i>
    which can cause additional loss of optimization.

    In order to implement atomic access to multi-byte objects,
    consider using the macros from <util/atomic.h>, rather than
    implementing them manually with cli() and sei().
*/
#define cli()
#else  /* !DOXYGEN */
# define cli()  __asm__ __volatile__ ("cli" ::: "memory")
#endif /* DOXYGEN */

#ifdef __cplusplus
#  define ISR(vector, ...)            \
    extern "C" void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
    void vector (void)
#else
#  define ISR(vector, ...)            \
    void vector (void) __attribute__ ((signal,__INTR_ATTRS)) __VA_ARGS__; \
    void vector (void)
#endif

// Latest non-blocking readings
volatile uint16_t A2_latest = 0;
volatile uint16_t A3_latest = 0;

// Internal ISR state
static volatile uint8_t adc_chan = 2;      
static volatile uint8_t discard_next = 0; 

// --- Init ADC in free-running mode, alternating A2 <-> A3 ---
/*
static void adc_init_A2_A3_freerun() {
  // Disable digital input buffers on A2/A3 to reduce noise/power
  // A0..A5 correspond to PC0..PC5 DIDR0 bits ADC0D..ADC5D
  DIDR0 |= (1 << ADC2D) | (1 << ADC3D);

  // AVcc reference, start on ADC2 (A2), right-adjusted result
  ADMUX  = (1 << REFS0) | (2 & 0x0F);  // REFS0=AVcc, MUX=2 (A2)
  ADCSRB = 0;                          // Free-running (ADTS=000)

  // Enable ADC, Auto Trigger, Interrupt; prescaler /128 (125 kHz at 16 MHz)
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADIE)
         | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  adc_chan = 2;
  discard_next = 0;

  ADCSRA |= (1 << ADSC); // start conversions
  sei();                 // enable global interrupts
}

ISR(ADC_vect) {
  uint16_t val = ADC; // read 10-bit result (compiler reads ADCL then ADCH)

  if (discard_next) {
    discard_next = 0; // throw away first sample after MUX switch
  } else {
    if (adc_chan == 2) {
      A2_latest = val;
      adc_chan = 3;
      ADMUX = (ADMUX & 0xF0) | 3; // keep REFS bits, set MUX=3 (A3)
      discard_next = 1;
    } else {
      A3_latest = val;
      adc_chan = 2;
      ADMUX = (ADMUX & 0xF0) | 2; // set MUX=2 (A2)
      discard_next = 1;
    }
  }
  
}


static inline void read_A2_A3(uint16_t &a2, uint16_t &a3) {
  uint8_t sreg = SREG; cli();
  a2 = A2_latest;
  a3 = A3_latest;
  SREG = sreg; // restore interrupt state
}
*/
// Sleep mode definitions


#define _SLEEP_CONTROL_REG  SMCR
#define SMCR _SFR_IO8(0x33)
#define _SFR_IO8(io_addr) _MMIO_BYTE((io_addr) + __SFR_OFFSET)
#define _MMIO_BYTE(mem_addr) (*(volatile uint8_t *)(mem_addr))
#define set_sleep_mode(mode) \
    do { \
        _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~(_BV(SM0) | _BV(SM1) | _BV(SM2))) | (mode)); \
    } while(0)
#define _BV(bit) (1 << (bit))
#define bit(b) (1UL << (b))
#define SM0 1
#define SM1 2
#define SM2 3
#define _SLEEP_ENABLE_MASK  _BV(SE)
#define SE 0

#define sleep_enable()             \
do {                               \
  _SLEEP_CONTROL_REG |= (uint8_t)_SLEEP_ENABLE_MASK;   \
} while(0)
#define sleep_cpu()                              \
do {                                             \
  __asm__ __volatile__ ( "sleep" "\n\t" :: );    \
} while(0)
#define sleep_disable()            \
do {                               \
  _SLEEP_CONTROL_REG &= (uint8_t)(~_SLEEP_ENABLE_MASK);  \
} while(0)
 // end sleep mode defs


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
const int BUZZER_PIN = 10; // piezo buzzer

const int ROW_ADC_TARGETS[4] = {133, 254, 367, 512}; // target adc values
const int ROW_TOL = 30; // adc tolerance range
int sleepTimer = 0;

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
          
          if (rr != row) break;  // we break here
          delay(5);
          
        }
        setColumn(-1);
        return keys[row][c];
      }
    }
  } 
  setColumn(-1);
  if (analogRead(AROW) == 1023)
  {
    sleepTimer = sleepTimer + 20;
    delay(20); 
    if ( sleepTimer == 12000)
    {
      setLED(false, false, false);
      set_sleep_mode(0x03);  // currently in Power Save Mode
        /*
        #define SLEEP_MODE_IDLE (0x00<<1)
        #define SLEEP_MODE_ADC (0x01<<1)
        #define SLEEP_MODE_PWR_DOWN (0x02<<1)
        #define SLEEP_MODE_PWR_SAVE (0x03<<1)
        #define SLEEP_MODE_STANDBY (0x06<<1)
        #define SLEEP_MODE_EXT_STANDBY (0x07<<1)
        
        */
      pinMode(2, INPUT); // Set pin 2 as input with pull-up resistor
      attachInterrupt(digitalPinToInterrupt(2), wakeup, FALLING);
      sei();          
      sleep_enable();
      sleep_cpu(); // Arduino goes to sleep here
      return '\0'; // nothing
    }
    
  }
  else {
    sleepTimer = 0;
  }
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

  //servostuff
  pinMode(9, OUTPUT);

  // Configure Timer1 for Fast PWM, mode 14
  TCCR1A = (1 << WGM11) | (1 << COM1A1);                 // clear OC1A on compare
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);    // prescaler = 8

  ICR1  = 39999;   // TOP → 20 ms period at 16 MHz / 8
  OCR1A = 3000;    // default pulse (~1.5 ms = 90°)

   //adc_init_A2_A3_freerun();
}


void loop(){
  Serial.println(analogRead(AROW));
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
    return;f
  }

  if (key == '#') { // if # key then compare to saved value
    // When Enter pressed, evaluate
    char end = EEPROM_read(addr);
    if (match && end == '*') {
      Serial.println("Unlocked!");
      unlocked = true;
      for (int angle = 0; angle <= 180; angle++) {
        setServoAngle(angle);
        
      }
    

    // (Optional) use the latest ADC readings
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
void wakeup() {
  setLED(true, true, true);
  sleep_disable();
}

void setServoAngle(int angle) {
  int pulseWidth = map(angle, 0, 180, 1000, 2000); // microseconds
  OCR1A = pulseWidth * 2;  // Timer1 ticks = 0.5 µs at prescaler 8
  // A2/A3 continue sampling in the ADC ISR with zero blocking.
}
