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

// Start CK's code 00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
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
    bool    weighted = false;

    OLEDWrapper() {
        oled.begin();    // Initialize the OLED
        oled.clear(ALL); // Clear the display's internal memory
        oled.display();  // Display what's in the buffer (splashscreen)
        delay(1000);     // Delay 1000 ms
        oled.clear(PAGE); // Clear the buffer.
    }

    void display(String title, int font, uint8_t x, uint8_t y) {
        oled.clear(PAGE);
        oled.setFontType(font);
        oled.setCursor(x, y);
        oled.print(title);
        oled.display();
    }

    void display(String title, int font) {
        display(title, font, 0, 0);
    }

    void displayNumber(String s) {
        // To reduce OLED burn-in, shift the digits (if possible) on the odd minutes.
        int x = 0;
        if (Time.minute() % 2) {
            const int MAX_DIGITS = 5;
            if (s.length() < MAX_DIGITS) {
                const int FONT_WIDTH = 12;
                x += FONT_WIDTH * (MAX_DIGITS - s.length());
            }
        }
        display(s, 3, x, 0);
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

         // Value between 1 and pixelSize - 2,
         // so pixelVal of 0 will have all pixels off
         // and pixelVal of pixelSize - 1 will have all pixels on. 
         int r = (rand() % (pixelSize - 2)) + 1;
         if (weighted) {
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
         }
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

    void testPattern() {
      int xSuperPixelSize = 6;	
      int ySuperPixelSize = 6;
      int pixelSize = xSuperPixelSize * ySuperPixelSize; 
      float diagonalDistance = sqrt((float)(xSuperPixelSize * xSuperPixelSize + ySuperPixelSize * ySuperPixelSize));
      float factor = (float)pixelSize / diagonalDistance;
      int pixelVals[64];
      for (int i = 0; i < 64; i++) {
        int x = (i % 8);
        int y = (i / 8);
        pixelVals[i] = (int)(round(sqrt((float)(x * x + y * y)) * factor));
      }
      displayArray(xSuperPixelSize, ySuperPixelSize, pixelVals);
      delay(5000);
      this->weighted = true;
      displayArray(xSuperPixelSize, ySuperPixelSize, pixelVals);
      delay(5000);	
    }

    void displayArray(int xSuperPixelSize, int ySuperPixelSize, int pixelVals[]) {
      oled.clear(PAGE);
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
        superPixel(y, x, ySuperPixelSize, xSuperPixelSize, pixelVals[i],
            left, right, top, bottom);
      }
      oled.display();
    }

    void clear() {
      oled.clear(ALL);
    }
};
OLEDWrapper oledWrapper;

String thermistor_test  = "1c002c001147343438323536";
String photon_02        = "300040001347343438323536";
String photon_05        = "19002a001347363336383438";
String photon_07        = "32002e000e47363433353735";
String photon_08        = "500041000b51353432383931";
String photon_09        = "1f0027001347363336383437";
String photon_10        = "410027001247363335343834";

class SensorData {
  private:
    int     pin;
    String  name;
    double  factor; // apply to get human-readable values, e.g., degrees F
    int     lastVal;

  public:
    SensorData(int pin, String name, double factor) {
        this->pin = pin;
        this->name = name;
        this->factor = factor;
        this->lastVal = INT_MIN;
        pinMode(pin, INPUT);
    }
    
    String getName() { return name; }

    void sample() {
        if (pin >= A0 && pin <= A5) {
            lastVal = analogRead(pin);
        } else {
            lastVal = digitalRead(pin);
        }
    }
    
    int applyFactor(int val) {
        return val * factor;
    }

    int getValue() {
        return applyFactor(lastVal);
    }

    int publishData() {
      Utils::publish(getName(), String(getValue()));
      return 1;
    }
};

class ThermistorSensor {
  private:
    SensorData t1 = SensorData(A0, "Thermistor sensor 1", 0.036);
    SensorData p9 = SensorData(A0, "Thermistor sensor 9", 0.036);

  public:    
    SensorData* getSensor() {
        String id = System.deviceID();
        if (id.equals(thermistor_test)) {
            return &t1;
        }
        if (id.equals(photon_09)) {
            return &p9;
        }
        return NULL;
    }
};
ThermistorSensor thermistorSensor;

class GridEyeSupport {
private:
  GridEYE grideye;
  String  mostRecentData;
  float   factor = 1.0;

  String getName() {
    // Getting the display to react quickly for Maker Faire
    // takes precedence over publishing to the Particle cloud.
    publishDelay = false;
    String id = System.deviceID();
    String location = "Unknown";
    if (id.equals(photon_05)) {
      location = "Faire 1";
    }
    if (id.equals(photon_07)) {
      location = "Fab Lab";
    }
    if (id.equals(photon_08)) {
      location = "Stove";
      publishDelay = true;
    }
    if (id.equals(photon_10)) {
      location = "Faire 2";
    }
    location.concat(" heat sensor");
    return location;
  }

public:
  int     mostRecentValue = INT_MIN;
  boolean enabled = false;

  void begin() {
    if (enabled) {
      grideye.begin();
    }
  }

  // This will take 15-20 seconds if the GridEye isn't connected.
  int readValue() {
    if (enabled) {
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
    }
    return mostRecentValue;
  }

  int publishData() {
    Utils::publish(getName(), String(mostRecentValue));
    return 1;
  }

  void displayGrid(int lowestTempInF, int highestTempInF) {
    if (!enabled) {
      return;
    }
    srand(0);
    int xSuperPixelSize = 6;
    int ySuperPixelSize = 6;
    int pixelSize = xSuperPixelSize * ySuperPixelSize;
    int pixelVals[64];
    for (int i = 0; i < 64; i++) {
      int t = (int)(grideye.getPixelTemperature(i) * 9.0 / 5.0 + 32.0);
      pixelVals[i] = map(t, lowestTempInF, highestTempInF, 0, pixelSize);
    }
    oledWrapper.displayArray(xSuperPixelSize, ySuperPixelSize, pixelVals);
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
    int   tempToBlinkInF = 100;  // If at this temperature or above, blink the temperature display.
    int   minTempInF = 80;      // degrees F that will display as non-black superpixels.
    int   maxTempInF = 90;
    bool  invert = true;

    int getTemp() {
        if (thermistorSensor.getSensor() == NULL) {
          return gridEyeSupport.mostRecentValue;
        }
        return thermistorSensor.getSensor()->getValue();
    }

  public:
    bool showTemp = System.deviceID().equals(thermistor_test) || System.deviceID().equals(photon_05);
    void display() {
      if (showTemp) {
        int temp = getTemp();
        if (temp >= tempToBlinkInF) {
          oledWrapper.oled.invert(invert);
          invert = !invert;
        } else {
            if (!invert) {
                // If going out of "blink" mode, make sure background is black.
                oledWrapper.oled.invert(false);
                invert = true;
            }
        }
        oledWrapper.displayNumber(String(temp));
        delay(500);
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
  if (thermistorSensor.getSensor() == NULL) {
    gridEyeSupport.publishData();
  } else {
    thermistorSensor.getSensor()->publishData();
  }
  return 1;
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

int testPatt(String command) {	
  oledWrapper.testPattern();	
  return 1;	
}

void setup() {
  Particle.publish("Debug", "Started setup...", 1, PRIVATE);
  // Start your preferred I2C object
  Wire.begin();
  // Library assumes "Wire" for I2C but you can pass something else with begin() if you like

  gridEyeSupport.enabled = true;
  gridEyeSupport.begin();

  Serial.begin(115200);

  Particle.function("publishData", pubData);
  Particle.function("getSettings", pubSettings);
  Particle.function("switchDisplay", switchDisp);
  Particle.function("rawPublish", rawPublish);
  Particle.function("setDispTemps", setDispTemps);
  Particle.function("getHelp", getHelp);
  Particle.function("testPattern", testPatt);
  delay(2000);

  if (thermistorSensor.getSensor() == NULL) {
    int now = millis();
    gridEyeSupport.readValue();
    if (millis() - now > 5000) {
      // GridEye probably not connected, will eventually semi-brick the Photon.
      gridEyeSupport.enabled = false;
      gridEyeSupport.mostRecentValue = INT_MIN;
    }
  } else {
    gridEyeSupport.enabled = false;
    gridEyeSupport.mostRecentValue = INT_MIN;
    thermistorSensor.getSensor()->sample();
  }
  oledDisplayer.display();
  pubData("");
  Particle.publish("Debug", "Finished setup...", 1, PRIVATE);
}

int lastPublish = 0;
void loop() {
    timeSupport.handleTime();
    if (thermistorSensor.getSensor() == NULL) {
      gridEyeSupport.readValue();
    } else {
      thermistorSensor.getSensor()->sample();
    }
    oledDisplayer.display();
    int thisSecond = millis() / 1000;
    if ((thisSecond % Utils::publishRateInSeconds) == 0 && thisSecond > lastPublish) {
      pubData("");
      lastPublish = thisSecond;
    }
}
