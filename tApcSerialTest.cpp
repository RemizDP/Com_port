#include <iomanip>		// вывод в 16-чной сс
#include <cstring>      // вывод ошибок strerr()
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // poll()
#include <fcntl.h>      // open()
#include <iostream>

#include "tApcSerialPort.h"
#include "tApcSerialTest.h"

int TApcSerialTest::openTest(const std::string astrPortName){
	TApcSerialPort P;
    int nResult = P.file_open(astrPortName);
    if (nResult < 0){
      std::cerr << "openTest failed. Port with name " << astrPortName << " isn't exist" << std::endl;
      return -1;
    }
    std::cout << "openTest passed successfully." << std::endl;
    return 0;
  };
int TApcSerialTest::baudrateTest(const std::string astrPortName, enBaudRate adwBaudRate){
  TApcSerialPort P;
  int nResult = P.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "baudrateTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  nResult = P.configure(adwBaudRate);
  if (nResult < 0){
    std::cerr << "baudrateTest failed. Error from configure" << std::endl;
    return-1;
  }
  struct termios settings = {};
  tcgetattr(P.get_handle(), &settings);
  std::cout << "Write speed = " << cfgetispeed(&settings) << "; Read speed = " << cfgetospeed(&settings) << std::endl;
  return 0;
  };
int TApcSerialTest::sendTest(const std::string astrPortName, enBaudRate adwBaudRate, uint32_t adwWsize, uint32_t adwRsize, uint32_t adwWriteTimeout, uint32_t adwReadTimeout){
  TApcSerialPort P;
  int nResult = P.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "sendTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  nResult = P.configure(adwBaudRate, 1, 10);
  if (nResult < 0){
    std::cerr << "sendTest failed. Error from configure" << std::endl;
    return-1;
  }

  uint8_t wstr[1000] = {};
  //uint8_t wstr[adwWsize] = {};
  //wstr[0] = 1;
  size_t astWritten=0;
  nResult = P.cycle_write(wstr, adwWsize, 1000, adwWriteTimeout, 100, astWritten);
  //nResult = P.write(wstr, adwWsize, adwWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult < 0){
    std::cerr << "sendTest failed. Error from write" << std::endl;
    return-1;
  }

  //uint8_t rstr[1000]={};
  uint8_t rstr[adwRsize]={};
  size_t astRead=0;
  //nResult = P.cycle_read(rstr, adwRsize, 1000, adwReadTimeout, 100, astRead);
  nResult = P.read(rstr, adwRsize, adwReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult < 0){
    std::cerr << "sendTest failed. Error from read" << std::endl;
    return-1;
  }
  for(size_t i=0; i< astRead; ++i) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)rstr[i] << " ";
  }
  std::cout << std::dec<< std::endl << std::endl;
  std::cout << "sendTest passed successfully." << std::endl;
  return 0;
};
int TApcSerialTest::run(const std::string astrPortName){
  enBaudRate aenBaudRate = enBaudRate::b9600;
  int nResult = openTest(astrPortName);
  if (nResult < 0){
    return -1;
  }
  nResult = baudrateTest(astrPortName, aenBaudRate);
  if (nResult < 0){
    return -1;
  }

  uint32_t adwWriteTimeout=10;
  uint32_t adwReadTimeout=10;
  uint32_t adwWsize = 100;
  uint32_t adwRsize = 1000;
  nResult = sendTest(astrPortName, aenBaudRate, adwWsize, adwRsize, adwWriteTimeout, adwReadTimeout);
  if (nResult < 0){
    return -1;
  }
  return 0;
};