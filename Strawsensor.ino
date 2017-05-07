#include <Wire.h>

//wiring for MS580314BA01-00
//When looking at the sensor face on silicon side towards you, blue dot in lower right 
//represents pin 1. Count around counter clockwise
//1 - clock (pin 21 on due) with 10k ohm pull up resistor to 3.3v
//2 - gnd
//3 - gnd or 3.3v (chip select bit, see sensor address)
//4 - not used
//5 - 3.3v
//6 - 3.3v
//7 - SDA (pin 20 on due) -I2C communication- 10k ohm pull up resistor to 3.3v
//8 - not used

//now working on two sensors

//Sensor's memory register addresses:
const byte RAW_PRESSURE = 0x00; //3 most significant bits of pressure
const byte RESET = 0x1E; //Can be used to reset from unkown state
const byte CONVERT_PRESS_4096 = 0x48;//reads pressure, 4096 resolution
const byte CONVERT_TEMP_4096 = 0x58;// reads temp
const byte PROM_READ = 0xA2; // starting address of prom read
const byte SENSOR_ADDRESS_1 = 0x76; //address with chip select bit pulled (0x76 high) (0x77 low)
const byte SENSOR_ADDRESS_2 = 0x77; //address with chip select bit pulled (0x76 high) (0x77 low)

//Global pressure offsets
double pressure1_offset, pressure2_offset; 

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(9600); 
  
  sendResetCommand(SENSOR_ADDRESS_1);  
  sendResetCommand(SENSOR_ADDRESS_2);  
  readCalibrationData(SENSOR_ADDRESS_2);
  delay(1000); 
  zeroReadings(); 
  
}

unsigned int SENS, P_OFF, TCS, TCO, T_ref, TEMPSENS;
/* These values are read from the sensor. They are then used to compensate for the pressure read from the sensor
SENSE = pressure sensitivity
P_OFF = pressure offset
TCS = temperature coeff of press sens
TCO = temperature coeff of pressure offset
T_REF = reference temperature
TEMPSENS = temp coeff of the temp
*/

long int reading = 0; 

void loop() {
  double pressure1, pressure2; 
  readPressures(pressure1, pressure2); 
  double pressure_delta = (pressure1 - pressure1_offset) - (pressure2 - pressure2_offset); 
  Serial.println("Pressure delta (mbar): " + String(pressure_delta)); 
  delay(100);
}


void readPressures(double& pressure1, double& pressure2){
  //long int readtime = micros(); 
  long int rawTemp_1 = readI2C(CONVERT_TEMP_4096, 3,true,SENSOR_ADDRESS_1);
  long int rawPress_1 = readI2C(CONVERT_PRESS_4096, 3,true,SENSOR_ADDRESS_1);
  pressure1 = calcPressure(rawPress_1, rawTemp_1)/10.0; 

  long int rawTemp_2 = readI2C(CONVERT_TEMP_4096, 3,true,SENSOR_ADDRESS_2);
  long int rawPress_2 = readI2C(CONVERT_PRESS_4096, 3,true,SENSOR_ADDRESS_2);
  pressure2 = calcPressure(rawPress_2, rawTemp_2)/10.0; 
  //readtime = micros() - readtime; 
  

  //Serial.println("");  
  //Serial.println("Total microseconds for read: " + String(readtime)); 
}

void zeroReadings(){
  readPressures(pressure1_offset, pressure2_offset); 
  Serial.println("Sensors have been zeroed to: " + String(pressure1_offset) + " and " + String(pressure2_offset));
}

long int readI2C(int readAddress, int dataLength, bool ADC_value, byte sensor_address){
  /*readAddress = register to read
  datalength = number of bytes to be read
  ADC_value = whether or not we are reading from the ADC. On the pressure sensor we are required to send a 0x00
  message when reading an ADC value
  */
  
  // step 1: instruct sensor to read 
  Wire.beginTransmission(sensor_address); //address with chip select bit pulled low (0x77 high)
  Wire.write(readAddress);  //set register to pressure reading
  Wire.endTransmission();      // stop transmitting

  // step 2: wait for readings to happen
  delay(10);                   // datasheet suggests at least 8 milliseconds

  if(ADC_value){
    // step 3: instruct sensor to return a particular echo reading
    Wire.beginTransmission(sensor_address); // 
    Wire.write(byte(0x00));      
    Wire.endTransmission();      
  }

  // step 4: request reading from sensor
  int recievedquantity = Wire.requestFrom(sensor_address, 3,false);    // request 3 bytes from slave device #112
  // step 5: receive reading from sensor
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  for(int i = 1; i<dataLength; i++){
    reading = reading << 8; 
    reading |= Wire.read(); 
  }
  return reading;  
}

//TODO: Need to pull calibration data from each sensor for better accuracy. For now shouldnt make too much difference
void readCalibrationData(byte sensor_address){
  //unsigned int SENS, P_OFF, TCS, TCO, T_ref, TEMPSENS;
  SENS = int(readI2C(PROM_READ, 2,false,sensor_address));
  P_OFF =  int(readI2C(PROM_READ+2, 2,false,sensor_address));
  TCS = int(readI2C(PROM_READ+4, 2,false,sensor_address));
  TCO  = int(readI2C(PROM_READ+6, 2,false,sensor_address));
  T_ref  = int(readI2C(PROM_READ+8, 2,false,sensor_address));
  TEMPSENS  = int(readI2C(PROM_READ+10, 2,false,sensor_address));
}

double calcPressure(long int rawPress, long int rawTemp){
   double dt = rawTemp - T_ref*256; 
   double temp = 2000.0 + (double(dt)*double(TEMPSENS))/8388608.0;
   double offset = double(P_OFF)*65536.0 + double(TCO)*dt/128.0;
   double sensitivity = double(SENS)*32768.0 + double(TCS)*dt/256.0;
   double pressure = (double(rawPress)*sensitivity/2097152.0  - offset)/32768.0;
   return pressure; 
}

void sendResetCommand(byte sensor_address){
  Serial.println("Resetting");
  Wire.beginTransmission(sensor_address); //address with chip select bit pulled low (0x77 high)
  Wire.write(RESET);  //set register to pressure reading
  Wire.endTransmission();      // stop transmitting
  delay(250);
}

