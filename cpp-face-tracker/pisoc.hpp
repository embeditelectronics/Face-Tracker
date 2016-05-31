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

#ifndef PISOC_H
#define PISOC_H
     
#include <unistd.h>
#include <stdint.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <stdint.h>
#include <mutex>
#include <errno.h>
#include <memory>

#define BACKEND_PROTOCOL	I2C //TODO: Implement a dynamic way of handling this. This way is not very usbable, but it works for now.
typedef struct Packet
{

  uint8_t address;
  uint8_t cmd;
  union
  {
    unsigned char bytes[];
    unsigned short value;
  };
}packet_t;

class FileDescriptor
{
    public:
        static std::shared_ptr<int> fd;
};

template <class T>
class Pisoc{
	public:
		T comms;
};




class I2C{
public:
    I2C();
    ~I2C();
    I2C(std::string fid);
    int Write( packet_t *packet);
    uint32_t Read(packet_t *packet);
     
private:
    std::shared_ptr<int> handle;
    unsigned char tx_array[32];
    static unsigned char magic_word;
    static unsigned char addr;
    bool data_ready();
    void Open();
    void Open(std::string fid);
    void Close();
};
class UART{
 
public:
    UART();
    ~UART();
    UART(std::string fid);
    int Write( packet_t *packet);
    uint32_t Read(packet_t *packet);
     
private:
    uint32_t baud;
    std::shared_ptr<int> uartfd;
    unsigned char tx_array[60];
    struct termios options;
    static unsigned char magic_word;
    int inWaiting();
    void Open();
    void Open(std::string fid);
    void Close();
};


class PWM{
 private:
  unsigned char address;
  packet_t packet;
  unsigned int clockDivider;
  Pisoc<BACKEND_PROTOCOL> pisoc;
  unsigned int per;
  unsigned int cmp;
  float src_clk;
 public:
  PWM(unsigned char pwmChannel);
  ~PWM();
  void Start();
  void Stop();
  void WriteCompare(unsigned int cmp);
  unsigned int getClockDivider(); 
  void setClockDivider(unsigned int divider);
  unsigned int ReadPeriod();
  unsigned int ReadCompare();
  void  WritePeriod(unsigned int period);
  float getFrequency();
  float getDutyCycle();
  void setDutyCycle(float dc);
};

class Servo{
 private:
  packet_t packet;
  float min_angle;
  float max_angle;
  float min_pulse; 
  float max_pulse;
  float angle;
  float pulse;
  float angle_range;
  float pulse_range;
 public:
  PWM servo_pwm;
  Servo(unsigned char servoChannel);
  ~Servo();
  void Start();
  void Stop();
  void changeAngleRange(float minAngleNew, float maxPulseNew);
  void changePulseRange(float minPulseNew, float maxPulseNew);
  void setAngle(float angle);
  void setPulse(float pulse);
  void Move(float delta);
  float getAngle();
  float getPulse();
};



 
#endif
