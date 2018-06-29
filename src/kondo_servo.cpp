#include"kondo_servo.hpp"

#include<cerrno>
#include<system_error>

#include<fcntl.h>//O_RDWR,O_NOCTTY,O_NDELAY
#include<termios.h>//termios,tcgetattr,cfsetspeed,options
#include<unistd.h>//usleep

//#define DEBUG

kondo_servo::kondo_servo(const unsigned char servo_id):
  fd(file("/dev/ttyAMA0",O_RDWR|O_NOCTTY|O_NDELAY)), id(servo_id)
{
  //https://bitbucket.org/vo/libkondo4/src/af6838734ebae792c9d58044ff4b547780efc347/serial/serial.c?at=default&fileviewer=file-view-default

  struct termios options;
  tcgetattr(fd.descriptor,&options);

  cfsetspeed(&options, B115200);//baudrate

  options.c_iflag &= ~(IXON|IXOFF|IXANY);//flow control disable
  options.c_iflag |= IGNPAR|IGNBRK;//ignore parity error & break signal

  options.c_oflag = 0;

  options.c_cflag |= CLOCAL|CREAD;//local connection | receive
  options.c_cflag |= PARENB;//parity enable
  options.c_cflag &= ~(PARODD|CSTOPB|CSIZE);//parity even | stop 1 bit | size unclearize
  options.c_cflag |= CS8;//8bit

  options.c_lflag &= ~(ICANON|ECHO|ISIG);//disable canonical mode,echo,and so on.

  tcflush(fd.descriptor,TCIOFLUSH);
  tcsetattr(fd.descriptor,TCSANOW,&options);
  tcflush(fd.descriptor,TCIOFLUSH);
}

int kondo_servo::transceive(const char write_buf[],unsigned char read_buf[],unsigned size,unsigned time)
{
  int total = 0;
  fd._write(write_buf);
  for(int t = 0;total < size && t < time;++t){
    total += fd._read(&read_buf[total],size);
    usleep(1000);//very important
  }
  return total;
}

#ifdef DEBUG
#include<iostream>
using std::cout;
using std::endl;
#endif

unsigned short kondo_servo::rotate(unsigned short target_angle)
{
  //target_angle : 4500~11500
  const char wbuf[3] = {'\x80'+id,target_angle>>7,target_angle&'\x7f'};
  unsigned char rbuf[6];
  transceive(wbuf,rbuf,6);

  #ifdef DEBUG
  for(int i = 0; i < 6;++i)cout << static_cast<int>(rbuf[i]) << " ";
  cout << endl;
  #endif

  return (static_cast<unsigned short>(rbuf[4])<<7) + static_cast<unsigned short>(rbuf[5]);
}

char kondo_servo::get_state(const char sc) {
  const char wbuf[2] = {'\xa0'+id,sc};
  unsigned char rbuf[5];//id,sc,rid,rsc,data
  transceive(wbuf,rbuf,5);
  return rbuf[4];
}

char kondo_servo::current(void)
{
  return get_state('\x03');
}

char kondo_servo::temperature(void)
{
  return get_state('\x04');
}

kondo_servo::~kondo_servo()
{
  fd.~file();
}
