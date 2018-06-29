#include "ros/ros.h"
#include "khr_ros/khr_srv.h"

#include <vector>
using std::vector;


#include<fcntl.h>

struct file
{
  const int descriptor;
  file(const char* device, const int flags = O_RDWR);
  ~file();
  int _write(const char buf[])const;
  int _read(unsigned char buf[],unsigned size);
};

#include<unistd.h>//read,write,close
#include<string.h>//strlen
#include<cerrno>
#include<system_error>

#ifdef _debug
#include<iostream>
#define print(var) std::cout << #var << ":" << var << std::endl;
#endif

file::file(const char* device, const int flags) : descriptor(open(device,flags))
{
  if(descriptor < 0){
    throw std::system_error(errno,std::system_category());
  }

  #ifdef _debug
  print(descriptor);
  #endif
}

int file::_read(unsigned char buf[],unsigned size)
{
  #ifdef _debug
  print(sizeof(buf))
  #endif

  return read(descriptor,buf,size);
}

int file::_write(const char buf[]) const
{
  #ifdef _debug
  print(strlen(buf))
  #endif

  return write(descriptor,buf,strlen(buf));
}

file::~file()
{
  #ifdef _debug
  std::cout << "fd " << descriptor << " closed" << std::endl;
  #endif

  close(descriptor);
}

class kondo_servo
{
  const unsigned char id;
  file fd;
  char get_state(char sc);
public:
  kondo_servo(const unsigned char servo_id);

  int transceive(const char write_buf[],unsigned char read_buf[],unsigned size,unsigned time = 3);
  unsigned short rotate(unsigned short target_angle);//2byte
  char current(void);//1byte
  char temperature(void);//1byte

  ~kondo_servo();
};

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






const int N = 3;

//http://wiki.ros.org/roscpp_tutorials/Tutorials/UsingClassMethodsAsCallbacks
struct khr_service
{
  kondo_servo servo[N];
public:

  khr_service():servo{1,2,4}
  {
  }

  bool add(khr_ros::khr_srv::Request  &req,
          khr_ros::khr_srv::Response &res)
  {
    for(int i = 0;i < N;++i)
    {
//      res.angle[i] = req.action[i]*req.action[i];
      const unsigned short r = servo[i].rotate(static_cast<unsigned short>(req.action[i]));
      res.angle[i] = static_cast<int>(r);
    }
    ROS_INFO("request: %d,%d,%d", req.action[0], req.action[1], req.action[2]);
    ROS_INFO("sending back response: [%d,%d,%d]", res.angle[0],res.angle[1],res.angle[2]);
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "khr_server");
  ros::NodeHandle n;

  khr_service ati = khr_service();

  ros::ServiceServer service = n.advertiseService("khr_srv", &khr_service::add, &ati);
  ROS_INFO("Ready khr.");
  ros::spin();

  return 0;
}