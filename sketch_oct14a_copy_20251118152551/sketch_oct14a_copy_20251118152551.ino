
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

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  char pressedbutton = 9;
  int values[] = {3, 7, 1, 9, 4, 5, 2, 0, 8, 6, 4};
  int index = 0;
  for (int i = 0; i < 9; i++){
    encrypt(values, i, i);
  }
  for(int i = 0; i < 10; i++)
  {
    Serial.println(EEPROM_read(i));
  }
  for (int i = 0; i < 9; i++)
  {
    Serial.println(decrypt(values, i));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(decimalToUndecimal(binaryToDecimal(1111000)));
 // encrypt(values[], index, pressedButton);
  
  
}
/*
int decimalToBinary8bit(int num) {
  for (int i = 7; i >= 0; i--) {
    int bit = (num >> i) & 1;
  }
  return bit;
}


*/
void encrypt( int values[], int index, int pressedButton )
{
  EEPROM_write(values[index], pressedButton );  
}

int decrypt (int values[], int index )
{
  return EEPROM_read(values[index]);
}
/*
void encode (char pressedButton, int state,int final_value)
{
  int storedValue = pressedButton - 48;
  int package = 0;

  if (state == 0 )  // in the case that we take the ninth digit for 32 bit EEPROM fusion
  {
    package = 0; 
    fused_digit = 
  }
  else if (digit % 9 = 0 && digit % 36 < 19 )  // in the case that we take the ninth digit for 32 bit EEPROM fusion
  {
    package = 1; 
    fused_digit = decimalToBinary4bit(storedValue);
  }
  else if (digit % 9 = 0 && digit % 36 < 28 )  // in the case that we take the ninth digit for 32 bit EEPROM fusion
  {
    package = 2; 
    fused_digit = decimalToBinary4bit(storedValue);
  }
  else if (digit % 9 = 0)
  {
    package = 3;
    fused_digit = decimalToBinary4bit(storedValue);
  }


  else if (((digit % 18) > 9 && (digit/2 = 1)) || ((digit % 18 < 9) && (digit/2 = 0)))  // in the case the digit is even, this is the second 
  {
     second_digit = storedValue * 11;
     state = 0;

  }
  else (((digit % 18) > 9 && (digit/2 = 0)) || ((digit % 18 < 9) && (digit/2 = 1)))
  {
    first_digit = storedValue;
    state = 1; 
  }  

  int final_value = first_digit + second_digit;
  return final_value;




int binaryToDecimal(long binary) {
  int dec_value = 0, base = 1;
  while (binary > 0) {
    int last_digit = binary % 10;
    if (last_digit > 1) return -1; // invalid binary digit
    dec_value += last_digit * base;
    binary /= 10;
    base *= 2;
  }
  return dec_value;          
}

char* decimalToUndecimal (int decimal)
{
    int significant_dig = decimal/11;  // automatically floor
    int remainder = decimal % 11;  
    char sigdig = significant_dig + 48; // convert integer to character
    char enddig = remainder + 48; 
    static char undec[3]; // two digits + null terminator
    undec[0] = sigdig;
    undec[1] = enddig;
    undec[2] = '\0';

    return undec;

}

char* decode (int regnum)
{
  int sigdig = 0;
  int enddig = 0;
  

  if (reg_content > 128)
  {
    reg_lsb7 = reg_content - 128;
    reg_msb = 1;
  }
  else
  {
    reg_lsb7 = reg_content;
    reg_msb = 0;
  }
  undec = decimalToUndecimal(reg_lsb7);
  return sigdig = undec[0] - 48;
  return enddig = undec[1] - 48;
  return reg_msb;

}
*/

