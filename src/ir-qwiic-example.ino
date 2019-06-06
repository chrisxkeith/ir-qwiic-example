#include <Wire.h>

//The default I2C address for the THING on the SparkX breakout is 0x69. 0x68 is also possible.
#define DEFAULT_ADDRESS 0x69

//Platform specific configurations

//Define the size of the I2C buffer based on the platform the user has
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)

//I2C_BUFFER_LENGTH is defined in Wire.H
#define I2C_BUFFER_LENGTH BUFFER_LENGTH

#elif defined(__SAMD21G18A__)

//SAMD21 uses RingBuffer.h
#define I2C_BUFFER_LENGTH SERIAL_BUFFER_SIZE

#elif __MK20DX256__
//Teensy

#elif ARDUINO_ARCH_ESP32
//ESP32 based platforms

#else

//The catch-all default is 32
#define I2C_BUFFER_LENGTH 32

#endif
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Registers
#define POWER_CONTROL_REGISTER        0x00
#define RESET_REGISTER                0x01
#define FRAMERATE_REGISTER            0x02
#define INT_CONTROL_REGISTER          0x03
#define STATUS_REGISTER               0x04
#define STATUS_CLEAR_REGISTER         0x05
#define AVERAGE_REGISTER              0x07
#define INT_LEVEL_REGISTER_UPPER_LSB  0x08
#define INT_LEVEL_REGISTER_UPPER_MSB  0x09
#define INT_LEVEL_REGISTER_LOWER_LSB  0x0A
#define INT_LEVEL_REGISTER_LOWER_MSB  0x0B
#define INT_LEVEL_REGISTER_HYST_LSB   0x0C
#define INT_LEVEL_REGISTER_HYST_MSB   0x0D
#define THERMISTOR_REGISTER_LSB       0x0E
#define THERMISTOR_REGISTER_MSB       0x0F
#define INT_TABLE_REGISTER_INT0       0x10
#define RESERVED_AVERAGE_REGISTER     0x1F
#define TEMPERATURE_REGISTER_START    0x80

class GridEYE {
  public:

    // Return values

    //By default use the default I2C addres, and use Wire port
    void begin(uint8_t deviceAddress = DEFAULT_ADDRESS, TwoWire &wirePort = Wire);

	float getPixelTemperature(unsigned char pixelAddr);
	int16_t getPixelTemperatureRaw(unsigned char pixelAddr);
	float getPixelTemperatureFahrenheit(unsigned char pixelAddr);
	
	float getDeviceTemperature();
	int16_t getDeviceTemperatureRaw();
	float getDeviceTemperatureFahrenheit();	
	
	void setFramerate1FPS();
	void setFramerate10FPS();
	bool isFramerate10FPS();
	
	void wake();
	void sleep();
	void standby60seconds();
	void standby10seconds();
	
	void interruptPinEnable();
	void interruptPinDisable();
	void setInterruptModeAbsolute();
	void setInterruptModeDifference();
	bool interruptPinEnabled();
	
	bool interruptFlagSet();
	bool pixelTemperatureOutputOK();
	bool deviceTemperatureOutputOK();
	void clearInterruptFlag();
	void clearPixelTemperatureOverflow();
	void clearDeviceTemperatureOverflow();
	void clearAllOverflow();
	void clearAllStatusFlags();
	
	bool pixelInterruptSet(uint8_t pixelAddr);
	
	void movingAverageEnable();
	void movingAverageDisable();
	bool movingAverageEnabled();
	
	void setUpperInterruptValue(float DegreesC);
	void setUpperInterruptValueRaw(int16_t regValue);
	void setUpperInterruptValueFahrenheit(float DegreesF);
	
	void setLowerInterruptValue(float DegreesC);
	void setLowerInterruptValueRaw(int16_t regValue);
	void setLowerInterruptValueFahrenheit(float DegreesF);
	
	void setInterruptHysteresis(float DegreesC);
	void setInterruptHysteresisRaw(int16_t regValue);
	void setInterruptHysteresisFahrenheit(float DegreesF);
	
	float getUpperInterruptValue();
	int16_t getUpperInterruptValueRaw();
	float getUpperInterruptValueFahrenheit();
	
	float getLowerInterruptValue();
	int16_t getLowerInterruptValueRaw();
	float getLowerInterruptValueFahrenheit();
	
	float getInterruptHysteresis();
	int16_t getInterruptHysteresisRaw();
	float getInterruptHysteresisFahrenheit();
	
	void setRegister(unsigned char reg, unsigned char val);
	int16_t getRegister(unsigned char reg, int8_t len);

    void setI2CAddress(uint8_t addr); //Set the I2C address we read and write to
    uint8_t getI2CAddress(); // for diagnostics

  private:
  
    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _deviceAddress; //Keeps track of I2C address. setI2CAddress changes this.

};


//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
void GridEYE::begin(uint8_t deviceAddress, TwoWire &wirePort)
{
  _deviceAddress = deviceAddress;
  _i2cPort = &wirePort;
}

//Change the address we read and write to
void GridEYE::setI2CAddress(uint8_t addr)
{
  _deviceAddress = addr;
}

uint8_t GridEYE::getI2CAddress()
{
  return _deviceAddress;
}

/********************************************************
 * Functions for retreiving the temperature of
 * a single pixel. 
 ********************************************************
 * 
 * getPixelTemperature() - returns float Celsius
 * 
 * getPixelTemperatureFahrenheit() - returns float Fahrenheit
 * 
 * getPixelTemperatureRaw() - returns int16_t contents of
 *    both pixel temperature registers concatinated
 *    
 ********************************************************/

float GridEYE::getPixelTemperature(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE::getPixelTemperatureFahrenheit(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE::getPixelTemperatureRaw(unsigned char pixelAddr)
{

  // Temperature registers are numbered 128-255
  // Each pixel has a lower and higher register
  unsigned char pixelLowRegister = TEMPERATURE_REGISTER_START + (2 * pixelAddr);
  int16_t temperature = getRegister(pixelLowRegister, 2);

  return temperature;

}

/********************************************************
 * Functions for retreiving the temperature of
 * the device according to the embedded thermistor. 
 ******************************************************** 
 * 
 * getDeviceTemperature() - returns float Celsius
 * 
 * getDeviceTemperatureFahrenheit() - returns float Fahrenheit
 * 
 * getDeviceTemperatureRaw() - returns int16_t contents of
 *    both thermistor temperature registers concatinated
 *    
 ********************************************************/

float GridEYE::getDeviceTemperature()
{

  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);

    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float realTemperature = temperature * 0.0625;

  return realTemperature;

}

float GridEYE::getDeviceTemperatureFahrenheit()
{

  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);

    // temperature is reported as 12-bit twos complement
    // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float realTemperatureF = (temperature * 0.0625) * 1.8 + 32;

  return realTemperatureF;

}

int16_t GridEYE::getDeviceTemperatureRaw()
{

  int16_t temperature = getRegister(THERMISTOR_REGISTER_LSB, 2);

  return temperature;

}

/********************************************************
 * Functions for manipulating Framerate
 ******************************************************** 
 * 
 * Internal framerate of the device is always 10fps
 * When operating in 1FPS mode, each frame is an average
 * of 10 readings.
 * 
 * setFramerate1FPS() - sets framerate to 1 Frame per Second
 * 
 * setFramerate10FPS() - sets framerate to 10 Frames per Second
 * 
 * isFramerate10FPS() - returns true if framerate is currently
 *    set to 10 Frames per Second (device default)
 *    
 ********************************************************/

void GridEYE::setFramerate1FPS()
{
    setRegister(FRAMERATE_REGISTER, 1);  
}

void GridEYE::setFramerate10FPS()
{
    setRegister(FRAMERATE_REGISTER, 0);  
}

bool GridEYE::isFramerate10FPS()
{

    if(getRegister(FRAMERATE_REGISTER, 1) == 0){
      return true;
    }else{
      return false;
    }
  
}

/********************************************************
 * Functions for manipulating Operating Mode
 ******************************************************** 
 * 
 * Device defaults to normal mode on reset. 
 * When the device is in standby mode, the temperature
 * register is only updated intermittently.
 * 
 * wake() - returns device to normal mode from any
 *    other state.
 *    
 * sleep() - puts device into sleep mode, temperature
 *    register is not updated
 * 
 * standby60seconds() - puts device into standby mode
 *    with 60 second update frequency
 *    
 * standby10seconds() - puts device into standby mode
 *    with 10 second update frequency
 *    
 ********************************************************/

void GridEYE::wake(){

    setRegister(POWER_CONTROL_REGISTER, 0x00);
  
}

void GridEYE::sleep(){

    setRegister(POWER_CONTROL_REGISTER, 0x10);
  
}

void GridEYE::standby60seconds(){

    setRegister(POWER_CONTROL_REGISTER, 0x20);
  
}

void GridEYE::standby10seconds(){

    setRegister(POWER_CONTROL_REGISTER, 0x21);
  
}

/********************************************************
 * Functions for manipulating Interrupt Control Register
 ******************************************************** 
 * 
 * interruptPinEnable() - Enable INT pin to pull low on 
 *    interrupt flag
 *    
 * interruptPinDisable() - Put INT pin into Hi-Z state
 * 
 * setInterruptModeAbsolute() - Set interrupt mode to
 *    "Absolute Value" mode
 *    
 * setInterruptModeDifference() - Set interrupt mode to
 *    "Difference" mode
 *    
 * interruptPinEnabled() - returns true if the INT pin
 *    is enabled. Returns false if INT pin is in Hi-Z 
 * 
 ********************************************************/

void GridEYE::interruptPinEnable(){

    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue |= (1 << 0);
    
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);   
  
}

void GridEYE::interruptPinDisable(){

    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue &= ~(1 << 0);
    
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
  
}

void GridEYE::setInterruptModeAbsolute(){

    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue |= (1 << 1);
    
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);
   
}

void GridEYE::setInterruptModeDifference(){

    int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
    
    ICRValue &= ~(1 << 1);
    
    setRegister(INT_CONTROL_REGISTER, ICRValue & 0xFF);    
  
}

bool GridEYE::interruptPinEnabled(){

  int16_t ICRValue = getRegister(INT_CONTROL_REGISTER, 1);
  if(ICRValue & (1 << 0)){
    return true;
  }else{
    return false;
  }
  
}

/********************************************************
 * Functions for manipulating Status/Clear Registers
 ******************************************************** 
 * 
 * interruptFlagSet() - returns true if there is an 
 *    interrupt flag in the status register
 *    
 * pixelTemperatureOutputOK() - returns false if temperature
 *    output overflow flag is present in status register
 * 
 * deviceTemperatureOutputOK() - returns false if thermistor
 *    output overflow flag is present in status register
 *    
 * clearInterruptFlag() - clears interrupt flag in the 
 *    status register
 *    
 * clearPixelTemperatureOverflow() - clears temperature
 *    output overflow flag in status register
 *    
 * clearDeviceTemperatureOverflow() - clears thermistor
 *    output overflow flag in status register
 *    
 * clearAllOverflow() - clears both thermistor and 
 *    temperature overflow flags in status register but
 *    leaves interrupt flag untouched
 *    
 * clearAllStatusFlags() - clears all flags in status 
 *    register
 * 
 ********************************************************/

bool GridEYE::interruptFlagSet(){

  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 1)){
    return true;
  }else{
    return false;
  }  
  
}

bool GridEYE::pixelTemperatureOutputOK(){

  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 2)){
    return false;
  }else{
    return true;
  }  
  
}

bool GridEYE::deviceTemperatureOutputOK(){

  int16_t StatRegValue = getRegister(STATUS_REGISTER, 1);
  if(StatRegValue & (1 << 3)){
    return false;
  }else{
    return true;
  }  
  
}

void GridEYE::clearInterruptFlag(){
    
    setRegister(STATUS_CLEAR_REGISTER, 0x02); 
  
}

void GridEYE::clearPixelTemperatureOverflow(){
    
    setRegister(STATUS_CLEAR_REGISTER, 0x04); 
  
}

void GridEYE::clearDeviceTemperatureOverflow(){
    
    setRegister(STATUS_CLEAR_REGISTER, 0x08); 
  
}

void GridEYE::clearAllOverflow(){

    setRegister(STATUS_CLEAR_REGISTER, 0x0C); 
  
}

void GridEYE::clearAllStatusFlags(){

    setRegister(STATUS_CLEAR_REGISTER, 0x0E); 
  
}

/********************************************************
 * Function for reading Interrupt Table Register
 ******************************************************** 
 * 
 * pixelInterruptSet() - Returns true if interrupt flag 
 * is set for the specified pixel
 * 
 ********************************************************/

bool GridEYE::pixelInterruptSet(uint8_t pixelAddr){

  unsigned char interruptTableRegister = INT_TABLE_REGISTER_INT0 + (pixelAddr / 8);
  uint8_t pixelPosition = (pixelAddr % 8);

  int16_t interruptTableRow = getRegister(interruptTableRegister, 1);

  if(interruptTableRow & (1 << pixelPosition)){
    return true;
  }else{
    return false;
  }    

}

/********************************************************
 * Functions for manipulating Average Register
 ******************************************************** 
 * 
 * Moving Average Mode enable and disable are only 
 * referenced in some of the documentation for this 
 * device but not in all documentation. Requires writing
 * in sequence to a reserved register. I'm not sure it 
 * does anything.
 * 
 * movingAverageEnable() - enable "Twice Moving Average" 
 * 
 * movingAverageDisable() - disable "Twice Moving Average"
 * 
 * movingAverageEnabled() - returns true if enabled
 * 
 ********************************************************/

void GridEYE::movingAverageEnable(){

    setRegister(RESERVED_AVERAGE_REGISTER, 0x50); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x45); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x57); 
    setRegister(AVERAGE_REGISTER, 0x20); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x00); 
  
}

void GridEYE::movingAverageDisable(){

    setRegister(RESERVED_AVERAGE_REGISTER, 0x50); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x45); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x57); 
    setRegister(AVERAGE_REGISTER, 0x00); 
    setRegister(RESERVED_AVERAGE_REGISTER, 0x00); 
  
}

bool GridEYE::movingAverageEnabled(){

  int16_t AVGRegValue = getRegister(AVERAGE_REGISTER, 1);
  if(AVGRegValue & (1 << 5)){
    return true;
  }else{
    return false;
  }  
  
}

/********************************************************
 * Functions for manipulating Interrupt Level Register
 ******************************************************** 
 * 
 * setUpperInterruptValue() - accepts float Celsius 
 * 
 * setUpperInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setUpperInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 *  
 * setLowerInterruptValue() - accepts float Celsius 
 * 
 * setLowerInterruptValueRaw() - accepts int16_t register
 *    configuration
 * 
 * setLowerInterruptValueFahrenheit() - accepts float 
 *    Fahrenheit
 * 
 * setInterruptHysteresis() - accepts float Celsius
 * 
 * setInterruptHysteresisRaw() - accepts int16_t register
 *    configuration
 * 
 * setInterruptHysteresisFahrenheit() - accepts float 
 *    Fahrenheit
 *    
 * getUpperInterruptValue() - returns float Celsius 
 * 
 * getUpperInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getUpperInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 *  
 * getLowerInterruptValue() - returns float Celsius 
 * 
 * getLowerInterruptValueRaw() - returns int16_t register
 *    contents
 * 
 * getLowerInterruptValueFahrenheit() - returns float 
 *    Fahrenheit
 * 
 * getInterruptHysteresis() - returns float Celsius
 * 
 * getInterruptHysteresisRaw() - returns int16_t register
 *    contents
 * 
 * getInterruptHysteresisFahrenheit() - returns float 
 *    Fahrenheit   
 * 
 ********************************************************/

void GridEYE::setUpperInterruptValue(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);
  
}

void GridEYE::setUpperInterruptValueRaw(int16_t regValue){
  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, regValue >> 8);
  
}

void GridEYE::setUpperInterruptValueFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_UPPER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_UPPER_MSB, temperature >> 8);
  
}

void GridEYE::setLowerInterruptValue(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);
  
}

void GridEYE::setLowerInterruptValueRaw(int16_t regValue){

  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, regValue >> 8);
  
}

void GridEYE::setLowerInterruptValueFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_LOWER_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_LOWER_MSB, temperature >> 8);
  
}

void GridEYE::setInterruptHysteresis(float DegreesC){

  bool isNegative = false;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);
  
}

void GridEYE::setInterruptHysteresisRaw(int16_t regValue){
  
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, regValue & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, regValue >> 8);
  
}

void GridEYE::setInterruptHysteresisFahrenheit(float DegreesF){

  bool isNegative = false;

  float DegreesC = (DegreesF - 32) / 1.8;

  if(DegreesC < 0){
    DegreesC = abs(DegreesC);
    isNegative = true;
  }
  
  int16_t temperature = 0;  
  temperature = round(DegreesC*4);

  if(isNegative){
    temperature = 0 - temperature;
    temperature |= (1 << 11);
  }
  
  setRegister(INT_LEVEL_REGISTER_HYST_LSB, temperature & 0xFF);
  setRegister(INT_LEVEL_REGISTER_HYST_MSB, temperature >> 8);
  
}

float GridEYE::getUpperInterruptValue()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

int16_t GridEYE::getUpperInterruptValueRaw()
{

  return getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

}

float GridEYE::getUpperInterruptValueFahrenheit()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_UPPER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

float GridEYE::getLowerInterruptValue()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE::getLowerInterruptValueFahrenheit()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE::getLowerInterruptValueRaw()
{

  return getRegister(INT_LEVEL_REGISTER_LOWER_LSB, 2);

}

float GridEYE::getInterruptHysteresis()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesC = temperature * 0.25;

  return DegreesC;

}

float GridEYE::getInterruptHysteresisFahrenheit()
{

  int16_t temperature = getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

  // temperature is reported as 12-bit twos complement
  // check if temperature is negative
  if(temperature & (1 << 11))
  {
    // if temperature is negative, mask out the sign byte and 
    // make the float negative
    temperature &= ~(1 << 11);
    temperature = temperature * -1;
  }

  float DegreesF = (temperature * 0.25) * 1.8 + 32;

  return DegreesF;

}

int16_t GridEYE::getInterruptHysteresisRaw()
{

  return getRegister(INT_LEVEL_REGISTER_HYST_LSB, 2);

}

/********************************************************
 * Functions for setting and getting registers over I2C
 ******************************************************** 
 * 
 * setRegister() - set unsigned char value at unsigned char register
 * 
 * getRegister() - get up to INT16 value from unsigned char register
 * 
 ********************************************************/

void GridEYE::setRegister(unsigned char reg, unsigned char val)
{

    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->write(val);
    _i2cPort->endTransmission();
    
}

// The parameter "len" may only be 1 (to retrieve a byte) or 2 (for a signed 16-bit integer).
int16_t GridEYE::getRegister(unsigned char reg, int8_t len)
{
  int16_t result = 0;     // default 0, if the sensor was not connected

  _i2cPort->beginTransmission(_deviceAddress);
  _i2cPort->write(reg);
  _i2cPort->endTransmission(false);    // 'false' for a repeated start

  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)len);
  if (_i2cPort->available() == len)    // got valid data ?
  {
    if (len == 1)
    {
      result = (int16_t) _i2cPort->read();
    }
    else if (len == 2)
    {
      int lsb = _i2cPort->read();
      int msb = _i2cPort->read();

      // concat bytes
      result = (int16_t) (msb << 8 | lsb);
    }
  }
  return result;
}

// Start CK's code ------- --------- --------- --------- ---------
const String githubHash = "to be replaced manually (and code re-flashed) after 'git push'";

#include <limits.h>

class JSonizer {
  public:
    static void addFirstSetting(String& json, String key, String val) {
        json.concat("\"");
        json.concat(key);
        json.concat("\":\"");
        json.concat(val);
        json.concat("\"");
    }

    static void addSetting(String& json, String key, String val) {
        json.concat(",");
        addFirstSetting(json, key, val);
    }
    
    static String toString(bool b) {
        if (b) {
            return "true";
        }
        return "false";
    }
};

/* to test:
- Set doRunTests to true.
- You may want to temporarily set NUM_QUEUED_MESSAGES down to 4 to make the test finish faster.
- Compile, flash and watch the console output.
- Reset doRunTests back to false and NUM_QUEUED_MESSAGES back to original value if necessary.
- Compile and flash.
*/
bool doRunTests = false;

// Keeps a queue of messages to publish, and publishes them no faster than
// 1 / second to avoid Particle publish() limit, without using calls to delay().
class Publisher {
  public:
    const static int NUM_QUEUED_MESSAGES = 20;
  private:
    const unsigned int WAIT_IN_MILLIS = 1100;     // Give 100 ms extra to increase odds that message will not be discarded.
    String    queued_events[NUM_QUEUED_MESSAGES]; // Circular buffers for queued events and datas.
    String    queued_datas[NUM_QUEUED_MESSAGES];
    int       nextAvailableIndex = 0;             // Can point to the last queued item if the queue is full.
                                                  // The last item will be the one overwritten if the queue is full.
    int       nextToBePublishedIndex = 0;         // If points to empty string, nothing to publish.
    unsigned long lastPublishMillis = 0;

    void publish_(String event, String data) {
      Particle.publish(event, data, 1, PRIVATE);
      lastPublishMillis = millis();
    }

    void incrementIndex(int& i) {
      if (i < NUM_QUEUED_MESSAGES - 1) {
        i++;
      } else {
        i = 0;
      }
    }

    void forDebug(String& s, String& d, String indicator) {
      if (doRunTests) {
          s.concat("(");
          s.concat(indicator);
          s.concat(":");
          s.concat(String(nextToBePublishedIndex));
          s.concat(",");
          s.concat(String(nextAvailableIndex));
          s.concat(")");
          d = String(lastPublishMillis);
          d.concat(",");
          d.concat(String(millis()));
      }
    }

  public:
    Publisher() {
      for (int i = 0; i < NUM_QUEUED_MESSAGES; i++) {
        queued_events[i] = String("");
        queued_datas[i] = String("");
      }
    }

    bool hasEvents() {
      return queued_events[nextToBePublishedIndex].length() > 0;
    }

    void doPublish(String event, String data) {
      if (!hasEvents() && (lastPublishMillis + WAIT_IN_MILLIS < millis())) {
        // Can publish immediately without queuing.
        forDebug(event, data, "i");
        publish_(event, data);
        nextAvailableIndex = 0;
        nextToBePublishedIndex = 0;
      } else {
        queued_events[nextAvailableIndex] = event;
        queued_datas[nextAvailableIndex] = data;
        int i = nextAvailableIndex;
        incrementIndex(i);
        if (queued_events[i].length() == 0) {
          // If there is no data in the next available slot, it's available.
          // Otherwise the queue is full, don't increment.
          incrementIndex(nextAvailableIndex);
        }
      }
    }

    void handlePublish() {
      if (hasEvents() && (lastPublishMillis + WAIT_IN_MILLIS < millis())) {
        forDebug(queued_events[nextToBePublishedIndex], queued_datas[nextToBePublishedIndex], "q");
        publish_(queued_events[nextToBePublishedIndex], queued_datas[nextToBePublishedIndex]);
        queued_events[nextToBePublishedIndex] = String("");
        queued_datas[nextToBePublishedIndex] = String("");
        incrementIndex(nextToBePublishedIndex);
      }
    }
};
Publisher* publisher = new Publisher();

class PublisherTester {
  private:
    void runTests(int testNum, int nPublishes) {
      for (int i = 0; i < nPublishes; i++) {
        String e("test_");
        e.concat(String(testNum));
        e.concat("_");
        e.concat(String(i));
        e.concat("_event");
        publisher->doPublish(e, String("-"));
      }
      while (publisher->hasEvents()) {
        delay(500);
        publisher->handlePublish();
      }
      delay(2000); // give time for the queue to fully flush out.
    }
  public:
    void runAllTests() {
      Particle.publish("Starting test run", "-", 1, PRIVATE);
      delay(2000);        // make sure we are well past publisher->lastPublishMillis.
      runTests(1, 1);     // minimal, nothing queued
      runTests(2, 2);     // one message queued
      runTests(3, publisher->NUM_QUEUED_MESSAGES + 1);  // one immediate + full queue, none dropped
      runTests(4, publisher->NUM_QUEUED_MESSAGES + 2);  // one immediate + full queue, next-to-last dropped
      delay(60000);        // give time to the human to review the console output.
    }
};
PublisherTester* pt = NULL;

bool publishDelay = true;
class Utils {
  public:
    const static int publishRateInSeconds = 5;

    static int setInt(String command, int& i, int lower, int upper) {
        int tempMin = command.toInt();
        if (tempMin > lower && tempMin < upper) {
            i = tempMin;
            return 1;
        }
        return -1;
    }
    static void publish(String event, String data) {
        Particle.publish(event, data, 1, PRIVATE);
        if (publishDelay) {
          delay(1000);
        }
    }
    static void publishJson() {
        String json("{");
        JSonizer::addFirstSetting(json, "githubHash", githubHash);
        JSonizer::addSetting(json, "publishRateInSeconds", String(publishRateInSeconds));
        JSonizer::addSetting(json, "publishDelay", JSonizer::toString(publishDelay));
        json.concat("}");
        publish("Utils json", json);
    }
};

class TimeSupport {
  private:
    const unsigned long ONE_DAY_IN_MILLISECONDS = 24 * 60 * 60 * 1000;
    unsigned long lastSyncMillis = millis(); // store millis() for lastSyncMillis as unsigned long
    String timeZoneString;

    String getSettings() {
        String json("{");
        JSonizer::addFirstSetting(json, "lastSyncMillis", String(lastSyncMillis));
        JSonizer::addSetting(json, "timeZoneOffset", String(timeZoneOffset));
        JSonizer::addSetting(json, "timeZoneString", String(timeZoneString));
        JSonizer::addSetting(json, "internalTime", nowZ());
        json.concat("}");
        return json;
    }

  public:
    int timeZoneOffset;

    TimeSupport(int timeZoneOffset, String timeZoneString) {
        this->timeZoneOffset = timeZoneOffset;
        this->timeZoneString = timeZoneString;
        Time.zone(timeZoneOffset);
        Particle.syncTime();
        lastSyncMillis = millis();
    }

    String timeStrZ(time_t t) {
        String fmt("%a %b %d %H:%M:%S ");
        fmt.concat(timeZoneString);
        fmt.concat(" %Y");
        return Time.format(t, fmt);
    }

    String nowZ() {
        return timeStrZ(Time.now());
    }

    void handleTime() {
        if (millis() - lastSyncMillis > ONE_DAY_IN_MILLISECONDS) {    // If it's been a day since last sync...
                                                                // Request time synchronization from the Particle Cloud
            Particle.syncTime();
            lastSyncMillis = millis();
        }
    }

    int setTimeZoneOffset(String command) {
        timeZoneString = "???";
        return Utils::setInt(command, timeZoneOffset, -24, 24);
    }

    void publishJson() {
        Utils::publish("TimeSupport", getSettings());
    }
};
TimeSupport    timeSupport(-8, "PST");

#include <SparkFunMicroOLED.h>
// https://learn.sparkfun.com/tutorials/photon-oled-shield-hookup-guide
#include <math.h>

class OLEDWrapper {
  public:
    MicroOLED oled;

    OLEDWrapper() {
        oled.begin();    // Initialize the OLED
        oled.clear(ALL); // Clear the display's internal memory
        oled.display();  // Display what's in the buffer (splashscreen)
        delay(1000);     // Delay 1000 ms
        oled.clear(PAGE); // Clear the buffer.
    }

    void display(String title, int font)
    {
      oled.clear(PAGE);
      oled.setFontType(font);
      oled.setCursor(0, 0);
      oled.print(title);
      oled.display();
    }

    bool errShown = false;
    void verify(int xStart, int yStart, int xi, int yi) {
      if (!errShown && (xi >= oled.getLCDWidth() || yi >= oled.getLCDHeight())) {
        String json("{");
        JSonizer::addSetting(json, "xStart", String(xStart));
        JSonizer::addSetting(json, "yStart", String(yStart));
        JSonizer::addSetting(json, "xi", String(xi));
        JSonizer::addSetting(json, "yi", String(yi));
        json.concat("}");
        Utils::publish("super-pixel coordinates out of range", json);
        errShown = true;
      }
    }

   void superPixel(int xStart, int yStart, int xSuperPixelSize, int ySuperPixelSize, int pixelVal,
         int left, int right, int top, int bottom) {
     int pixelSize = xSuperPixelSize * ySuperPixelSize;
     if (pixelVal < 0) {
       pixelVal = 0;
     } else if (pixelVal >= pixelSize) {
       pixelVal = pixelSize - 1;
     }
     for (int xi = xStart; xi < xStart + xSuperPixelSize; xi++) {
       for (int yi = yStart; yi < yStart + ySuperPixelSize; yi++) {
         verify(xStart, yStart, xi, yi);
         int r = rand() % (pixelSize + 1);
/*
         float xfactor;
         if (xi >= xStart + (xSuperPixelSize / 2)) {
           xfactor = left;
         } else {
           xfactor = right;
         }
         float yfactor;
         if (yi >= yStart + (ySuperPixelSize / 2)) {
           yfactor = top;
         } else {
           yfactor = bottom;
         }
         // how much the adjacent super-pixels will affect the current one.
         float weight = ((xfactor + yfactor) / 2.0);

         // width/height size of super-pixel.
         float size = ((xSuperPixelSize + ySuperPixelSize) / 2.0);

         // distance of this pixel from the center of the super-pixel.
         float xdistance = abs(xi - (xStart + (xSuperPixelSize / 2.0)));
         float ydistance = abs(yi - (yStart + (ySuperPixelSize / 2.0)));
         float distance = (xdistance + ydistance) / 2.0;
         int r = rand() * (pixelVal+ (weight * (distance / size)));
*/
         if (r < pixelVal) { // lower value maps to white pixel.
           oled.pixel(xi, yi);
         }
       }
     }
   }

    void publishJson() {
        String json("{");
        JSonizer::addFirstSetting(json, "getLCDWidth()", String(oled.getLCDWidth()));
        JSonizer::addSetting(json, "getLCDHeight()", String(oled.getLCDHeight()));
        json.concat("}");
        Utils::publish("OLED", json);
    }
};
OLEDWrapper oledWrapper;

String thermistor2_test = "300040001347343438323536";
String photon_05        = "19002a001347363336383438";
String photon_06        = "290048001647363335343834";
String photon_08        = "500041000b51353432383931";

class GridEyeSupport {
private:
  String  mostRecentData;
  float   factor = 1.0;

  String getName() {
    String id = System.deviceID();
    if (id.equals(thermistor2_test)) {
      String ret("Stove IR sensor 2");
      return ret;
    }
    int numericId = INT_MIN;
    if (id.equals(photon_05)) {
      numericId = 5;
    }
    if (id.equals(photon_06)) {
      numericId = 6;
    }
    if (id.equals(photon_08)) {
      factor = 1.2;
      numericId = 8;
    }
    String ret("IR heat sensor ");
    ret.concat(String(numericId));
    return ret;
  }

public:
  GridEYE grideye;
  int     mostRecentValue;

  int readValue() {
    float total = 0;
    mostRecentData = String("");
    for (int i = 0; i < 64; i++) {
      int t = (int)(grideye.getPixelTemperature(i) * 9.0 / 5.0 + 32.0);
      total += t;
      if (i > 0) {
        mostRecentData.concat(",");
      }
      mostRecentData.concat(String(t));
    }
    mostRecentValue = (int)(this->factor * total / 64);
    return mostRecentValue;
  }

  int publishData() {
    Utils::publish(getName(), String(mostRecentValue));
    return 1;
  }

  void displayGrid(int lowestTempInF, int highestTempInF) {
    srand(0);
    oledWrapper.oled.clear(PAGE);
    int xSuperPixelSize = 6;
    int ySuperPixelSize = 6;
    int pixelSize = xSuperPixelSize * ySuperPixelSize;
    int pixelVals[64];
    for (int i = 0; i < 64; i++) {
      int t = (int)(grideye.getPixelTemperature(i) * 9.0 / 5.0 + 32.0);
      pixelVals[i] = map(t, lowestTempInF, highestTempInF, 0, pixelSize);
    }
    for (int i = 0; i < 64; i++) {
      int x = (i % 8) * xSuperPixelSize;
      int y = (i / 8) * ySuperPixelSize;
      int left = x;
      int right = x;
      int top = y;
      int bottom = y;
      if (x > 0) {
        left = pixelVals[i - 1];
      }
      if (x < 7) {
        right = pixelVals[i + 1];
      }
      if (y > 0) {
        top = pixelVals[i - 8];
      }
      if (y < 7) {
        top = pixelVals[i + 8];
      }
      // This (admittedly confusing) switcheroo of x and y axes is to make the orientation
      // of the sensor (with logo reading correctly) match the orientation of the OLED.
      oledWrapper.superPixel(y, x, ySuperPixelSize, xSuperPixelSize, pixelVals[i],
          left, right, top, bottom);
    }
    oledWrapper.oled.display();
  }

  void publishJson() {
        String json("{");
        JSonizer::addFirstSetting(json, "address", String((int)grideye.getI2CAddress()));
        JSonizer::addSetting(json, "mostRecentData", mostRecentData);
        json.concat("}");
        Utils::publish("Grideye json", json);
  }
};
GridEyeSupport gridEyeSupport;

class OLEDDisplayer {
  private:
    int   tempToBlinkInF = 85;  // If at this temperature or above, blink the temperature display.
    int   minTempInF = 70;      // degrees F that will display as non-black superpixels.
    int   maxTempInF = 90;
  public:
    bool showTemp = true;
    void display() {
      if (showTemp) {
        int temp = gridEyeSupport.mostRecentValue;
        if (temp >= tempToBlinkInF) {
          oledWrapper.oled.clear(ALL);
          delay(500);
        }
        oledWrapper.display(String(temp), 3);
        delay(1000);
      } else {
        gridEyeSupport.displayGrid(minTempInF, maxTempInF);
      }
    }
    int switchDisp(String command) {
      showTemp = !showTemp;
      publishDelay = showTemp;
      return 1;
    }
    int setDispTemps(String cmd) {
      char paramBuf[20];
      cmd.toCharArray(paramBuf, 20);
      char *pch = strtok(paramBuf, ",");
      this->tempToBlinkInF = atoi(pch);
      pch = strtok(NULL, ",");
      if (pch != NULL) {
        this->minTempInF = atoi(pch);
        pch = strtok(NULL, ",");
        if (pch != NULL) {
          this->maxTempInF = atoi(pch);
        }
      }
      return 1;
    }
    void publishJson() {
      String json("{");
      JSonizer::addFirstSetting(json, "tempToBlinkInF", String(tempToBlinkInF));
      JSonizer::addSetting(json, "minTempInF", String(minTempInF));
      JSonizer::addSetting(json, "maxTempInF", String(maxTempInF));
      json.concat("}");
      Utils::publish("OLEDDisplayer json", json);
    }
};
OLEDDisplayer oledDisplayer;

int setDispTemps(String command) {
  return oledDisplayer.setDispTemps(command);
}

int switchDisp(String command) {
  return oledDisplayer.switchDisp(command);
}

int pubData(String command) {
  return gridEyeSupport.publishData();
}

// getSettings() is already defined somewhere.
int pubSettings(String command) {
    if (command.compareTo("") == 0) {
        Utils::publishJson();
    } else if (command.compareTo("time") == 0) {
        timeSupport.publishJson();
    } else if (command.compareTo("grideye") == 0) {
        gridEyeSupport.publishJson();
    } else if (command.compareTo("oled") == 0) {
        oledWrapper.publishJson();
    } else if (command.compareTo("display") == 0) {
        oledDisplayer.publishJson();
    } else {
        Utils::publish("GetSettings bad input", command);
    }
    return 1;
}

int rawPublish(String command) {
    String  event(command);
    String  data(command);
    int     separatorIndex = command.indexOf(":");
    if (separatorIndex > 0) { // Not zero, need at least one character for event.
        event = command.substring(0, separatorIndex);
        data = command.substring(separatorIndex + 1);
    } else {
        int     separatorIndex = command.indexOf("=");
        if (separatorIndex > 0) { // Not zero, need at least one character for event.
          event = command.substring(0, separatorIndex);
          data = command.substring(separatorIndex + 1);
      }
    }
    Utils::publish(event, data);
    return 1;
}

int getHelp(String command) {
    Utils::publish("Code repository", "https://github.com/chrisxkeith/ir-qwiic-example");
    return 1;
}

void setup() {
  // Start your preferred I2C object
  Wire.begin();
  // Library assumes "Wire" for I2C but you can pass something else with begin() if you like
  gridEyeSupport.grideye.begin();
  Serial.begin(115200);

  Particle.function("publishData", pubData);
  Particle.function("getSettings", pubSettings);
  Particle.function("switchDisplay", switchDisp);
  Particle.function("rawPublish", rawPublish);
  Particle.function("setDispTemps", setDispTemps);
  Particle.function("getHelp", getHelp);
  delay(2000);
  gridEyeSupport.readValue();
  oledDisplayer.display();
  gridEyeSupport.publishData();

  if (System.deviceID().equals(photon_08)) {
    switchDisp(""); // show heat map.
  }
}

int lastPublish = 0;
void loop() {
  if (doRunTests) {
    if (pt == NULL) {
      pt = new PublisherTester();
    }
    pt->runAllTests();
  } else {
    timeSupport.handleTime();
    gridEyeSupport.readValue();
    oledDisplayer.display();
    if ((Time.second() % Utils::publishRateInSeconds) == 0 && Time.second() > lastPublish) {
      gridEyeSupport.publishData();
      lastPublish = Time.second();
    }
  }
}
