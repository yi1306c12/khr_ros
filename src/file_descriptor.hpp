#include<fcntl.h>

struct file
{
  const int descriptor;
  file(const char* device, const int flags = O_RDWR);
  ~file();
  int _write(const char buf[])const;
  int _read(unsigned char buf[],unsigned size);
};
