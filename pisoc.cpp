/*
The MIT License (MIT)

Copyright (c) 2016 Embedit Electronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <cstring>
#include "pisoc.hpp"
using namespace std;

std::shared_ptr<int> FileDescriptor::fd = std::make_shared<int>(-1);
unsigned char UART::magic_word = 0xEB;
unsigned char I2C::magic_word = 0xEB;
unsigned char I2C::addr = 0x07;

std::mutex _open;
std::mutex _io;

void UART::Open(std::string fid)
{
    std::lock_guard<std::mutex> lock(_open);
    if (*FileDescriptor::fd  == -1)
    {
	    *FileDescriptor::fd = open(fid.c_str(), O_RDWR | O_NOCTTY);

		if(*FileDescriptor::fd  == -1)
		{
		  perror("Could not open UART ");
		  exit(1);
		}

	    tcgetattr(*FileDescriptor::fd, &this->options);
	    this->options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;  
	    this->options.c_iflag = IGNPAR;
	    this->options.c_oflag = 0;
	    this->options.c_lflag = 0;
	    tcflush(*FileDescriptor::fd, TCIFLUSH);
	    tcsetattr(*FileDescriptor::fd, TCSANOW, &this->options);
    }
    this->uartfd = FileDescriptor::fd; //This will increase our reference count to the shared pointer holding our file descriptor.
}
 
void UART::Close(){
    std::unique_lock<std::mutex> lock(_open);
    if (FileDescriptor::fd.use_count() <= 2) //We are the last reference to the file descriptor. We need to clean up after everone...
    {
    	close(*this->uartfd);
		*FileDescriptor::fd = -1; //We are free to reopen the file normally now.
    }
}
 
int UART::Write(packet_t *packet)
{
 
  uint32_t retVal = this->Read(packet);

  if (retVal != 0xA11600D)
  {
    return -1;
  }
   
  return 0;
}

int UART::inWaiting(){
    int result; 
    ioctl(*this->uartfd, FIONREAD, &result);
    return result;
}

uint32_t UART::Read(packet_t *packet)
{

  union
  {
      uint8_t rx_array[];
      uint32_t response;
  };

  this->tx_array[1] = sizeof(packet);
  memcpy(this->tx_array + 2, packet, this->tx_array[1]);

  {//limiting the scope of our mutex lock.
    std::lock_guard<std::mutex> lock(_io);
    int count = write(*this->uartfd, this->tx_array, this->tx_array[1] + 2);
    packet->value = 0;
    while (this->inWaiting()<4){};
    count = read(*this->uartfd, rx_array, 4);
  }

  return response;
}

 
UART::UART(){
  this->tx_array[0] = this->magic_word;
  this->Open(std::string("/dev/ttyAMA0"));
}
 
UART::UART(std::string fid){
    this->tx_array[0] = this->magic_word;
    this->Open(fid);
}
 
UART::~UART(){
    this->Close();
}


void I2C::Open(std::string fid)
{
    std::lock_guard<std::mutex> lock(_open);
    if (*FileDescriptor::fd  == -1)
    {	
	   *FileDescriptor::fd = open(fid.c_str(), O_RDWR);

      if(*FileDescriptor::fd  == -1)
      {
          perror("Could not open I2C ");
          exit(1);
      }
      
      if (ioctl(*FileDescriptor::fd, I2C_SLAVE, this->addr) < 0)
      {
		  perror("I2C slave address set failed");
		  exit(1);
	  }

    }
    this->handle = FileDescriptor::fd; //This will increase our reference count to the shared pointer holding our file descriptor.
}
 
void I2C::Close(){
    std::unique_lock<std::mutex> lock(_open);
    if (FileDescriptor::fd.use_count() <= 2) //We are the last reference to the file descriptor. We need to clean up after everone...
    {
    	close(*this->handle);
		*FileDescriptor::fd = -1; //We are free to reopen the file normally now.
    }
}
 
int I2C::Write(packet_t *packet)
{
 
  uint32_t retVal = this->Read(packet);

  if (retVal != 0xA11600D)
  {
    return -1;
  }
   
  return 0;
}

bool I2C::data_ready(){
    union
	  {
		  uint8_t rx_array[];
		  uint32_t response;
	  };
    this->tx_array[0] = 0x00;
    write(*this->handle, this->tx_array, 1);
    int count = read(*this->handle, rx_array, 4);
    return response == this->magic_word;
}

uint32_t I2C::Read(packet_t *packet)
{

  union
  {
      uint8_t rx_array[];
      uint32_t response;
  };

  this->tx_array[0] = 0x08;
  memcpy(this->tx_array + 1, packet, 4);

  {//limiting the scope of our mutex lock.
    std::lock_guard<std::mutex> lock(_io);
    this->tx_array[0] = 0x08;
    int count = write(*this->handle, this->tx_array, 5);
    constexpr uint8_t signal[] = {0x00, 0xAC};
    count = write(*this->handle, signal, 2);
    packet->value = 0;
    while (!this->data_ready()){};
    this->tx_array[0] = 0x04;
    write(*this->handle, this->tx_array, 1);
    count = read(*this->handle, rx_array, 4);
  }

  return response;
}

 
I2C::I2C(){
	
  this->tx_array[1] = this->magic_word;
  this->Open(std::string("/dev/i2c-1"));
}
 
I2C::I2C(std::string fid){
    this->tx_array[1] = this->magic_word;
    this->Open(fid);
}
 
I2C::~I2C(){
    this->Close();
}

PWM::PWM(unsigned char pwmChannel):pisoc(){
  this->clockDivider = 8;
  this->src_clk = 24000000;
  switch (pwmChannel){
    case 0: this->address = 9; break;
    case 1: this->address = 10; break;
    case 2: this->address = 11; break;
    case 3: this->address = 12; break;
    case 4: this->address = 13; break;
    case 5: this->address = 14; break;
    case 6: this->address = 15; break;
    case 7: this->address = 16; break;
    default: this->address = 9; break;
  }
  this->packet.address = this->address;
  this->packet.value = 0;
}

void PWM::Start(){
  this->packet.cmd = 0x00;
	pisoc.comms.Write(&this->packet);
}


void PWM::Stop(){
  this->packet.cmd = 0x01;
  pisoc.comms.Write(&this->packet);
}

void PWM::WriteCompare(unsigned int compare){
  this->packet.cmd = 0x0E;
  this->packet.value = compare;

  pisoc.comms.Write(&this->packet);
  this->cmp = compare;
} 

void PWM::WritePeriod(unsigned int period){
  
  if (period<this->cmp)
  {
    WriteCompare(period);
  }
  this->packet.cmd = 0x0C;
  this->packet.value = period;

  if (pisoc.comms.Write(&this->packet) >=0)
  {
    this->per = period;
  }
  
} 

unsigned int PWM::ReadCompare()
{
  unsigned int response;

  this->packet.cmd = 0x0F;

  response = pisoc.comms.Read(&this->packet);
  this->cmp = response;
  return response;
} 

unsigned int PWM::ReadPeriod(){
  unsigned int response; 

  this->packet.cmd = 0x0D;
  response = pisoc.comms.Read(&this->packet);
 
  this->per = response;
  return response;
} 

void  PWM::setClockDivider(unsigned int divider){

  this->packet.cmd = 0xFF;
  this->packet.value = divider;

  this->clockDivider = pisoc.comms.Read(&this->packet) + 1;
} 

unsigned int PWM::getClockDivider(){
  return this->clockDivider;
} 

float PWM::getFrequency(){
  return (this->src_clk/(float)(this->per*this->clockDivider));
}
float PWM::getDutyCycle(){
  return (100.0*this->cmp/(float)this->per);
}

void PWM::setDutyCycle(float dc)
{

  if (dc<0)
  {
    dc = 0;
  }
  else if (dc>100)
  {
    dc = 100;
  }

  this->WriteCompare((unsigned int)(this->per*dc*0.01 + 0.5));
}


PWM::~PWM(){}

Servo::Servo(unsigned char servoChannel):servo_pwm(servoChannel){
  this->packet.value = 0;
  this->min_angle = 0;
  this->max_angle = 180;
  this->min_pulse = 1.0;
  this->max_pulse = 2.0;
  this->pulse_range = 1.0;
  this->angle_range = 180;
  this->servo_pwm.WritePeriod(60000);//set to 50Hz pwm output frequency
  this->servo_pwm.setClockDivider(8);
  this->setAngle(90);
}

void Servo::Start(){
  servo_pwm.Start();
}

void Servo::Stop(){
  servo_pwm.Stop();
}

void Servo::setPulse(float pulse_ms){
  if (pulse_ms<this->min_pulse)
  {
    pulse_ms = this->min_pulse;
  }
  else if (pulse_ms>this->max_pulse)
  {
    pulse_ms = this->max_pulse;
  }
  this->servo_pwm.setDutyCycle(servo_pwm.getFrequency()*0.1*pulse_ms);
  this->pulse = pulse_ms;
  this->angle = this->min_angle + this->angle_range*((float)(pulse_ms - this->min_pulse)/this->pulse_range);
}

void Servo::setAngle(float angle_deg){
  float angle_perc;
  angle_perc = (float)(angle_deg - this->min_angle)/(this->angle_range);
  this->setPulse(this->min_pulse + angle_perc*this->pulse_range);
}
float Servo::getPulse(){
  return this->pulse;
}

float Servo::getAngle(){
  return this->angle;
}
void Servo::changeAngleRange(float minAngleNew, float maxAngleNew){
 float angle_perc;
 angle_perc = (float)(this->angle - this->min_angle)/(this->angle_range);
 this->min_angle= minAngleNew;
 this->max_angle = maxAngleNew;
 this->angle_range = maxAngleNew - minAngleNew;
 this->angle = this->angle_range*angle_perc + this->min_angle;

}

void Servo::changePulseRange(float minPulseNew, float maxPulseNew){
  this->min_pulse = minPulseNew;
  this->max_pulse = maxPulseNew;
  this->pulse_range = maxPulseNew - minPulseNew;
}

void Servo::Move(float delta){
  this->setAngle(this->angle + delta);
}

Servo::~Servo(){}

