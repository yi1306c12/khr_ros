#include"file_descriptor.hpp"

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
