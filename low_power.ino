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


void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);

  // Prepare to enter the sleep mode
  set_sleep_mode(0x03);  // currently in Power Save Mode
 /*
 #define SLEEP_MODE_IDLE (0x00<<1)
 #define SLEEP_MODE_ADC (0x01<<1)
 #define SLEEP_MODE_PWR_DOWN (0x02<<1)
 #define SLEEP_MODE_PWR_SAVE (0x03<<1)
 #define SLEEP_MODE_STANDBY (0x06<<1)
 #define SLEEP_MODE_EXT_STANDBY (0x07<<1)
 
 */
  sleep_enable();
  sleep_cpu(); // Arduino goes to sleep here

}

void loop() {
  //TODO
}

