#include <SPI.h>
#include <SoftwareSerial.h>

#include <inttypes.h>
#include <SBGC.h>
#include <SBGC_Arduino.h>

SoftwareSerial mySerial(2, 3);  // RX, TX

#define SBGC_CMD_CALIB_ACC 'A'
#define SBGC_CMD_CALIB_GYRO 'g'
#define SBGC_CMD_REALTIME_DATA_3 23
#define U_DEG 3  // read encoder in degrees

#define CS_Pitch 9 //Chip or Slave select 
#define CS_Yaw 10 //Chip of Slave Select

uint8_t temp_bit[1];

const int LED = 13;

uint8_t outgoing_data[13];

// This helper function formats and sends a command to SimpleBGC Serial API
void SBGC_sendCommand(uint8_t cmd, void *data, uint8_t size) {
  uint8_t i, checksum=0;
  // Header
  mySerial.write('>');
  mySerial.write(cmd);
  mySerial.write(size);
  mySerial.write(cmd+size);
  // Body
  for(i=0;i<size;i++) {
    checksum+= ((uint8_t*)data)[i];
    mySerial.write(((uint8_t*)data)[i]);
  }
  mySerial.write(checksum);
} 

//**************************************************************************
// This function makes sure the buffer is clear at the start and end of each loop
void clearBuffer() {
  while(mySerial.available()>0)
    mySerial.read();
}

uint8_t SPI_T (uint8_t msg, int cs){    //Repetive SPI transmit sequence
  uint8_t msg_temp = 0;  //vairable to hold recieved data
  digitalWrite(cs, LOW);    //select spi device
  msg_temp = SPI.transfer(msg);    //send and recieve
  digitalWrite(cs, HIGH);   //deselect spi device
  return (msg_temp);     //return recieved byte
}

float read_encoder(int cs){
  uint8_t recieved = 0xA5;    //just a temp vairable
  uint16_t ABSposition = 0;    //reset position vairable
  
  SPI_T(0x10, cs);   //issue read command
  recieved = SPI_T(0x00, cs);    //issue NOP to check if encoder is ready to send
  while (recieved != 0x10)    // loop while encoder is not ready to send
  {
    recieved = SPI_T(0x00, cs);    //cleck again if encoder is still working
    delay(2);    //wait a bit
  }
  temp_bit[0] = SPI_T(0x00, cs);    //Recieve MSB
  temp_bit[1] = SPI_T(0x00, cs);    // recieve LSB
 
  // bit combining
  temp_bit[0] &= ~ 0xF0;   //mask out the first 4 bits
  
  ABSposition = temp_bit[0] << 8;    //shift MSB to correct ABSposition in ABSposition message
  ABSposition += temp_bit[1];    // add LSB to ABSposition message to complete message
  
  float encoder_value = ABSposition;
  encoder_value = encoder_value * 0.08789;    // aprox 360/4096
  return encoder_value;
}

void initialize_outgoing_data(int motor_speed){
  outgoing_data[0] = 2;   // control mode
  outgoing_data[1] = 0;  // speed roll
  outgoing_data[2] = 0;
  outgoing_data[3] = 0;  // angle roll
  outgoing_data[4] = 0;
  int16_t speed_pitch = motor_speed * SBGC_SPEED_SCALE;
  outgoing_data[5] = speed_pitch;  // speed pitch
  outgoing_data[6] = speed_pitch >> 8;
  outgoing_data[7] = 0;  // angle pitch
  outgoing_data[8] = 0;
  int16_t speed_yaw = motor_speed * SBGC_SPEED_SCALE;  
  outgoing_data[9] = speed_yaw;  // speed yaw
  outgoing_data[10] = speed_yaw >> 8;
  outgoing_data[11] = 0;  // angle yaw
  outgoing_data[12] = 0;  
}

void set_outgoing_data(float el_desired, float az_desired){
  int16_t angle_pitch = SBGC_DEGREE_TO_ANGLE(el_desired);  
  outgoing_data[7] = angle_pitch;  // angle pitch
  outgoing_data[8] = angle_pitch >> 8;
  int16_t angle_yaw = SBGC_DEGREE_TO_ANGLE(az_desired);  
  outgoing_data[11] = angle_yaw;  // angle yaw
  outgoing_data[12] = angle_yaw >> 8;
}

void setup(){
  mySerial.begin(19200);
//  SBGC_Demo_setup(&mySerial);  
  Serial.begin(57600);
  pinMode(LED, OUTPUT);

  pinMode(CS_Pitch, OUTPUT); //Slave Select
  pinMode(CS_Yaw, OUTPUT);
  digitalWrite(CS_Pitch, HIGH);
  digitalWrite(CS_Yaw, HIGH);
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  
  //Calibrate accelerometers
  SBGC_sendCommand(SBGC_CMD_CALIB_ACC,0,0);
  delay(6000);
  //Calibrate gyros
  SBGC_sendCommand(SBGC_CMD_CALIB_GYRO,0,0);
  delay(6000);  
  SBGC_sendCommand(SBGC_CMD_MOTORS_ON,0,0);
  delay(3000);

  initialize_outgoing_data(30);
  set_outgoing_data(0, 0);
  SBGC_sendCommand(67, outgoing_data, 13);
  

//  digitalWrite(LED, HIGH);
//  delay(1000);
//  digitalWrite(LED, LOW);
//  delay(1000);
//  digitalWrite(LED, HIGH);
//  delay(1000);
//  digitalWrite(LED, LOW);
//  delay(1000);
}

//************************************************************//
// Define Gimbal Parameters for main loop
char headerBuf[4];
char dataBuf[44]; // the data buffer is really 63 in size, but we will only read in the data we need
char checkSum[1];
double rollIMU, pitchIMU, yawIMU;
double pitchENC, yawENC;
double driftPitch, driftYaw;
double convertFactor = 0.02197265625;
float yaw_angle, pitch_angle;
float az_final, el_final;

void loop(){
//  Serial.println("looping");
  clearBuffer();
  //send read parameter command
  SBGC_sendCommand(SBGC_CMD_REALTIME_DATA_3,0,0); // send command to read data
  // wait until we have a certain number of bytes ready
  int numBytes = 0;
  while(numBytes<(63))
  {
    numBytes = mySerial.available();
  }
  // read data into arrays
  for (int i = 0; i<4; i++)
  {
    headerBuf[i] = mySerial.read();
  }
  for (int i = 0; i<44;i++)
  {
    dataBuf[i] = mySerial.read();
  }
  
  // perform byte shifting
  int temp_rollIMU  = ((unsigned char)dataBuf[33] << 8) | (unsigned char)dataBuf[32];
  int temp_pitchIMU = ((unsigned char)dataBuf[35] << 8) | (unsigned char)dataBuf[34];
  int temp_yawIMU   = ((unsigned char)dataBuf[37] << 8) | (unsigned char)dataBuf[36];
  clearBuffer();
  rollIMU  =  temp_rollIMU  * convertFactor;
  pitchIMU = -temp_pitchIMU * convertFactor;
  yawIMU   =  temp_yawIMU   * convertFactor;
  
  // recalculate angles using encoder, subtract initial angle
  pitchENC = read_encoder(CS_Pitch);
  if(pitchENC < 180 && pitchENC > 0){
    pitchENC = -pitchENC; 
  }
  else if(pitchENC > 180){
    pitchENC = 360-pitchENC; 
  }
  yawENC = read_encoder(CS_Yaw);
  if(yawENC >= 180){
    yawENC = 360-yawENC;
  }
  else if(yawENC < 180 && yawENC > 0){
    yawENC = -yawENC; 
  }
  
  // calculate drift
  driftPitch = pitchIMU - pitchENC;
  driftYaw   = yawIMU   - yawENC;
  
//  Serial.println("IMU");    
  Serial.println(pitchIMU,2);
  Serial.println(yawIMU,2);   
//  Serial.println("Encoder");
  Serial.println(pitchENC,2);
  Serial.println(yawENC,2);
  
  // reading the desired azimuth angle
//  Serial.println("desired");
  while(!Serial.available());
  float az_d;
  Serial.readBytes((char*)&az_d, sizeof(az_d));

  // reading the desired elevation angle
  while(!Serial.available());
  float el_d;
  Serial.readBytes((char*)&el_d, sizeof(el_d));
  
  az_final = az_d+driftYaw;
  el_final = -(el_d+driftPitch);
//  Serial.println("drift");
  Serial.println(driftYaw);
  Serial.println(driftPitch);
//  Serial.println("final");
  Serial.println(az_final);
  Serial.println(el_final);
  
  // move the motors to the desired position
  set_outgoing_data(el_final, az_final);
  SBGC_sendCommand(67, outgoing_data, 13);
  
  delay(200);
}

