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
	TApcSerialPort Port;
    int nResult = Port.file_open(astrPortName);
    if (nResult < 0){
      std::cerr << "openTest failed. Port with name " << astrPortName << " isn't exist" << std::endl;
      return -1;
    }
    std::cout << "openTest passed successfully." << std::endl;
    return 0;
  };

int TApcSerialTest::moveTest(const std::string astrPortName){
  TApcSerialPort Port1;
  int nResult = Port1.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "moveTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }

  enBaudRate adwBaudRate = enBaudRate::b9600;
  nResult = Port1.configure(adwBaudRate);
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from configure" << std::endl;
    return-1;
  }
  
  size_t astExternWsize = 1;
  uint32_t adwExternWriteTimeout = 100;
  uint8_t apWBuf[astExternWsize] = {};
  size_t astWritten=0;
  nResult = Port1.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from write" << std::endl;
    return-1;
  }
  size_t astExternRsize = 100;
  uint32_t adwExternReadTimeout = 100;
  uint8_t apRBuf[astExternRsize]={};
  size_t astRead = 0;
  nResult = Port1.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from read" << std::endl;
    return-1;
  }
  for(size_t i=0; i< astRead; ++i) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
  }
  std::cout << std::dec<< std::endl << std::endl;

  TApcSerialPort Port2 = std::move(Port1);
  std::cout << "PORT1 handle = " << Port1.get_handle() << std::endl;
  std::cout << "PORT2 handle = " << Port2.get_handle() << std::endl;

  nResult = Port1.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from write Port1" << std::endl;
    //return-1;
  }
  nResult = Port1.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from read Port1" << std::endl;
    //return-1;
  }
  /*for(size_t i=0; i< astRead; ++i) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
  }
  std::cout << std::dec<< std::endl << std::endl;*/

  nResult = Port2.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from write Port2" << std::endl;
    //return-1;
  }
  nResult = Port2.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult < 0){
    std::cerr << "moveTest failed. Error from read Port2" << std::endl;
    //return-1;
  }
  for(size_t i=0; i< astRead; ++i) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
  }
  std::cout << std::dec<< std::endl << std::endl;//*/
  std::cout << "moveTest passed successfully." << std::endl;
  std::cout << "================================================================================="<<std::endl;
  return 0;
}

int TApcSerialTest::baudrateTest(const std::string astrPortName){
  TApcSerialPort Port1;
  int nResult = Port1.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "baudrateTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }

  TApcSerialPort Port = std::move(Port1);
  std::cout << "PORT1 handle = " << Port1.get_handle() << std::endl;

  enBaudRate adwBaudRate [8] = {enBaudRate::b1200, enBaudRate::b2400, enBaudRate::b4800, enBaudRate::b9600, enBaudRate::b19200, enBaudRate::b38400, enBaudRate::b57600, enBaudRate::b115200};
  for (int i = 0; i < 8; i++){
    nResult = Port.configure(adwBaudRate[i]);
    if (nResult < 0){
      std::cerr << "baudrateTest failed. Error from configure" << std::endl;
      return-1;
    }
    struct termios settings = {};
    tcgetattr(Port.get_handle(), &settings);
    speed_t ispeed = cfgetispeed(&settings);
    speed_t ospeed = cfgetospeed(&settings);
    int intISpeed = 0;
    int intOSpeed = 0;
    switch (ispeed){
    case B1200:
      intISpeed = 1200;
      break;
    case B2400:
      intISpeed = 2400;
      break;
    case B4800:
      intISpeed = 4800;
      break;
    case B9600:
      intISpeed = 9600;
      break;
    case B19200:
      intISpeed = 19200;
      break;
    case B38400:
      intISpeed = 38400;
      break;
    case B57600:
      intISpeed = 57600;
      break;
    case B115200:
      intISpeed = 115200;
      break;
    default:
      std::cerr<<"Unknown speed"<<std::endl;
      break;
    }
    switch (ospeed){
    case B1200:
      intOSpeed = 1200;
      break;
    case B2400:
      intOSpeed = 2400;
      break;
    case B4800:
      intOSpeed = 4800;
      break;
    case B9600:
      intOSpeed = 9600;
      break;
    case B19200:
      intOSpeed = 19200;
      break;
    case B38400:
      intOSpeed = 38400;
      break;
    case B57600:
      intOSpeed = 57600;
      break;
    case B115200:
      intOSpeed = 115200;
      break;
    default:
      std::cerr<<"Unknown speed"<<std::endl;
      break;
    }

    std::cout << "Write speed = " << intISpeed << "; Read speed = " << intOSpeed << std::endl;
  }
  std::cout << "baudrateTest passed successfully." << std::endl;
  return 0;
};

int TApcSerialTest::cycleTest(const std::string astrPortName){
  TApcSerialPort Port;
  int nResult = Port.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "cycleTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  enBaudRate adwBaudRate = enBaudRate::b9600;
  uint32_t MIN[4] = {0, 1, 5, 10};
  uint32_t TIME[4] = {0, 1, 5, 10};
  for (int i = 0; i < 4; i++){
    for (int j =0; j < 4; j++){
      nResult = Port.configure(adwBaudRate, MIN[i], TIME[j]);
      if (nResult < 0){
        std::cerr << "cycleTest failed. Error from configure" << std::endl;
        return-1;
      }
      for (int k = 0; k < 5; k++){
        std::cout << "Test " << 4*i+j+1 << " MIN = " << MIN[i] << " TIME = " << TIME[j] << std::endl;
        size_t astExternWsize = 100;
        uint32_t adwExternWriteTimeout = 100;
        uint8_t apWBuf[astExternWsize] = {};
        size_t astWsize = 10;
        uint32_t adwWriteTimeout = 10;
        size_t astWritten=0;
        nResult = Port.cycle_write(apWBuf, astWsize, astExternWsize, adwWriteTimeout, adwExternWriteTimeout, astWritten);
        std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
        if (nResult < 0){
          std::cerr << "cycleTest failed. Error from write" << std::endl;
          return-1;
        }
        size_t astExternRsize = 100;
        uint32_t adwExternReadTimeout = 100;
        uint8_t apRBuf[astExternRsize]={};
        size_t astRsize = 10;
        uint32_t adwReadTimeout = 10;
        size_t astRead = 0;
        nResult = Port.cycle_read(apRBuf, astRsize, astExternRsize, adwReadTimeout, adwExternReadTimeout, astRead);
        std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
        if (nResult < 0){
          std::cerr << "cycleTest failed. Error from read" << std::endl;
          return-1;
        }
        for(size_t i=0; i< astRead; ++i) {
          std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
        }
        std::cout << std::dec<< std::endl << std::endl;
        std::cout << "================================================================================="<<std::endl;
      }
    }
  };
  std::cout << "cycleTest passed successfully." << std::endl;
  return 0;  
};

int TApcSerialTest::onceTest(const std::string astrPortName){
  TApcSerialPort Port;
  int nResult = Port.file_open(astrPortName);
  if (nResult < 0){
    std::cerr << "cycleTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  enBaudRate adwBaudRate = enBaudRate::b9600;
  uint32_t MIN[4] = {0, 1, 5, 10};
  uint32_t TIME[4] = {0, 1, 5, 10};
  for (int i = 0; i < 4; i++){
    for (int j = 0; j < 4; j++){
      nResult = Port.configure(adwBaudRate, MIN[i], TIME[j]);
      if (nResult < 0){
        std::cerr << "cycleTest failed. Error from configure" << std::endl;
        return-1;
      }
      for (int k = 0; k < 5; k++){
        std::cout << "Test " << 4*i+j+1 << " MIN = " << MIN[i] << " TIME = " << TIME[j] << std::endl;
        size_t astExternWsize = 100;
        uint32_t adwExternWriteTimeout = 100;
        uint8_t apWBuf[astExternWsize] = {};
        size_t astWritten=0;
        nResult = Port.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
        std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
        if (nResult < 0){
          std::cerr << "onceTest failed. Error from write" << std::endl;
          return-1;
        }

        size_t astExternRsize = 100;
        uint32_t adwExternReadTimeout = 100;
        uint8_t apRBuf[astExternRsize]={};
        size_t astRead = 0;
        nResult = Port.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
        std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
        if (nResult < 0){
          std::cerr << "onceTest failed. Error from read" << std::endl;
          return-1;
        }
        for(size_t i=0; i< astRead; ++i) {
          std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
        }
        std::cout << std::dec<< std::endl << std::endl;
        std::cout << "================================================================================="<<std::endl;
      }
    }
  }
  std::cout << "onceTest passed successfully." << std::endl;
  return 0;
};

int TApcSerialTest::sendTest(const std::string astrPortName){
  int nResult = cycleTest(astrPortName);
  if (nResult < 0){
    std::cerr << "sendTest failed. Error from cycleTest" << std::endl;
    return-1;
  }
  nResult = onceTest(astrPortName);
  if (nResult < 0){
    std::cerr << "sendTest failed. Error from onceTest" << std::endl;
    return-1;
  }
    
  std::cout << "sendTest passed successfully." << std::endl;
  return 0;
};

int TApcSerialTest::useful_portTest(const std::string astrPortName){
  int nResult = openTest(astrPortName);
  if (nResult < 0){
    return -1;
  }
  nResult = moveTest(astrPortName);
  if (nResult<0)
  nResult = baudrateTest(astrPortName);
  if (nResult < 0){
    return -1;
  }

  nResult = sendTest(astrPortName);
  if (nResult < 0){
    return -1;
  }
  return 0;
};

int TApcSerialTest::no_portsTest(){
  const std::string astrPortName = "Any_string_name";
  int nResult = openTest(astrPortName);
  return nResult;
}

int TApcSerialTest::no_data_portTest(const std::string astrPortName){
  int nResult = openTest(astrPortName);
  if (nResult < 0){
    return -1;
  }
  nResult = baudrateTest(astrPortName);
  if (nResult < 0){
    return -1;
  }
  return 0;
}

int TApcSerialTest::run(){
  int nResult = 0;
  /*nResult = no_portsTest();
  std::cout << "Result of no_portsTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  /*nResult = no_data_portTest("/dev/ttyS1");
  std::cout << "Result of no_data_portTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  nResult = useful_portTest("/dev/ttyS0");
  std::cout << "Result of useful_portTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  return 0;
}