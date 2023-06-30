#include <iostream>
#include <iomanip>

#include "TApcSerialPort.h"
int main(){
  const std::string portname = "/dev/ttyS0"; 
  TApcSerialPort P;
  P.file_open(portname);
  int res = P.SetDefaultSettings(9600);

  //while(adwReadTimeout>0){
    uint32_t adwWriteTimeout=100;
    uint32_t adwReadTimeout=100;
    std::cout << "WriteTimeout: " << adwWriteTimeout << ", ReadTimeout: " << adwReadTimeout << std::endl;
    int j =0;
    //while(j<1) {
      uint32_t Wsize=1;
      uint8_t wstr[Wsize] = {};
      wstr[0] = 0;


      size_t astWritten=0;
      
      int nResult = P.write(wstr, Wsize, adwWriteTimeout, astWritten);
      std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;

      uint32_t Rsize=100;
      uint8_t rstr[Rsize]={};
      usleep(1000);

      size_t astRead=0;

      nResult = P.read(rstr, Rsize, adwReadTimeout, astRead);
      std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
      for(size_t i=0; i< astRead; ++i) {
        std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)rstr[i] << " ";
      }
      std::cout << std::dec<< std::endl << std::endl;
      //memset(rstr, 0, size);
      j++;
      //usleep(1000000);
    //}
    std::cout << "================================================================================="<<std::endl;
    adwReadTimeout-=50;
  //}
  
  P.file_close();  

  return 0; 
}
