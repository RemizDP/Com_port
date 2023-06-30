#include <iostream>
#include <iomanip>


#include </home/d/Documents/COM_port/TApcSerialPort.hpp>
int get_speed(struct termios& s1){
  struct termios* ss= &s1;
  std::cout << "ispeed = "<<cfgetispeed(ss)<<std::endl;
  std::cout << "ospeed = "<<cfgetospeed(ss)<<std::endl;
  return 0;
}
int main(){
  const char *portname = (char*)"/dev/ttyS0";
  struct termios settings={};
  uint32_t Wsize=5000;
  uint32_t Rsize=5000;
  size_t astWritten=0;
  size_t astRead=0;
  uint32_t adwWriteTimeout=100;
  uint32_t adwReadTimeout=1;
  //char quit[size] = ">quit";
  //uint8_t xstr[size]={};
 
  TApcSerialPort P;
  P.file_open(portname);
  int res = P.prepareSettings(settings, B9600);

  
  //while(adwReadTimeout>0){
    printf("WriteTimeout: %d, ReadTimeout: %d\n", adwWriteTimeout, adwReadTimeout);
    int j =0;
    while(j<1) {
      uint8_t wstr[Wsize] = {};
      wstr[0] = 0;

      get_speed(settings);
      int nResult = P.write(wstr, Wsize, adwWriteTimeout, astWritten);
      printf("write res: %d, written: %ld\n", nResult, astWritten);

      uint8_t rstr[Rsize]={};
      usleep(1000);
      nResult = P.read(rstr, Rsize, adwReadTimeout, astRead);
      printf("read res: %d, read: %ld\n", nResult, astRead);
      for(size_t i=0; i< astRead; ++i) {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)rstr[i] << " ";
      }
      std::cout << std::endl << std::endl;
      //memset(rstr, 0, size);
      j++;
      //usleep(1000000);
    }
    std::cout << "================================================================================="<<std::endl;
    adwReadTimeout-=50;
  //}
  
  P.file_close();  

  return 0; 
}
