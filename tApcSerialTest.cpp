#include <iomanip>		// вывод в 16-чной сс
#include <cstring>      // вывод ошибок
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // набор дескрипторов
#include <fcntl.h>      // open()
#include <iostream>

#include "TApcSerialPort.h"
#include "TApcSerialTest.h"

int TApcSerialTest::openTest(const std::string astrPortName){
	TApcSerialPort P;
    int nResult = P.file_open(astrPortName);
    if (nResult < 0){
      std::cerr << "openTest failed. Port with this name isn't exist" << std::endl;
      return -1;
    }
    std::cout << "openTest passed successfully." << std::endl;
    P.file_close();
    std::cout << "================================================================================"<<std::endl;
    return 0;
  };
int TApcSerialTest::baudrateTest(const std::string astrPortName, baud_t adwBaudRate){
    TApcSerialPort P;
    int nResult = P.file_open(astrPortName);
    if (nResult < 0){
      std::cerr << "baudrateTest failed. Port with this name isn't exist" << std::endl;
      return -1;
    }
    nResult = P.configure_settings(adwBaudRate);
    if (nResult < 0){
      std::cerr << "baudrateTest failed. Error from configure_settings." << std::endl;
      return-1;
    }
    struct termios settings = {};
    tcgetattr(P.get_handle(), &settings);
    std::cout << "Write speed = " << cfgetispeed(&settings) << "; Read speed = " << cfgetospeed(&settings) << std::endl;
    P.file_close();
    std::cout << "================================================================================"<<std::endl;
    return 0;
  };
int TApcSerialTest::sendTest(const std::string astrPortName, baud_t adwBaudRate, uint32_t adwWsize, uint32_t adwRsize, uint32_t adwWriteTimeout, uint32_t adwReadTimeout){
    TApcSerialPort P;
    int nResult = P.file_open(astrPortName);
    if (nResult < 0){
      std::cerr << "sendTest failed. Port with this name isn't exist" << std::endl;
      return -1;
    }
    nResult = P.configure_settings(adwBaudRate);
    if (nResult < 0){
      std::cerr << "sendTest failed. Error from configure_settings." << std::endl;
      return-1;
    }

    uint8_t wstr[adwWsize] = {};
    wstr[0] = 0;
    size_t astWritten=0;
    nResult = P.write(wstr, adwWsize, adwWriteTimeout, astWritten);
    std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;

    uint8_t rstr[adwRsize]={};
    size_t astRead=0;
    nResult = P.read(rstr, adwRsize, adwReadTimeout, astRead);
    
    std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
    for(size_t i=0; i< astRead; ++i) {
      std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)rstr[i] << " ";
    }
    std::cout << std::dec<< std::endl << std::endl;
    std::cout << "sendTest passed successfully." << std::endl;
    P.file_close();
    std::cout << "================================================================================"<<std::endl;
    return 0;
  };
int TApcSerialTest::run(const std::string astrPortName, baud_t adwBaudRate){
    int nResult = openTest(astrPortName);
    if (nResult < 0){
      return -1;
    }

      nResult = baudrateTest(astrPortName, adwBaudRate);
      if (nResult < 0){
        return -1;
    }

    uint32_t adwWriteTimeout=100;
    uint32_t adwReadTimeout=100;
    uint32_t adwWsize = 1;
    uint32_t adwRsize = 100;
    nResult = sendTest(astrPortName, adwBaudRate, adwWsize, adwRsize, adwWriteTimeout, adwReadTimeout);
    if (nResult < 0){
        return -1;
    }
    return 0;
};