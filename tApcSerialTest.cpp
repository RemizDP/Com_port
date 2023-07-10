#include <iomanip>		// вывод в 16-чной сс
#include <cstring>      // вывод ошибок strerr()
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // poll()
#include <fcntl.h>      // open()
#include <iostream>

#include "tApcSerialPort.h"
#include "tApcSerialTest.h"

/*Пытается открыть порт. Если порт не существует и открылся корректно, то успех, в противном случае возврат -1*/
int TApcSerialTest::openTest(const std::string astrPortName){
	TApcSerialPort Port;
    int nResult = Port.file_open(astrPortName);
    if (nResult != 0){
      std::cerr << "openTest failed. Port with name " << astrPortName << " isn't exist" << std::endl;
      return -1;
    }
    std::cout << "openTest passed successfully." << std::endl;
    return 0;
  };

/*Проверка реализации move семантики. Создается экземпляр класса Port1, ей присваивается открываемый порт, производится настройка порта,
 тестовая запись и чтение по этому порту с помощью этой переменной (ожидается успех).
Создается второй экземпляр класса Port2, в который присваивается с помощью move Port1. Выводятся дескрипторы Port1 и Port2. 
Совершается попытка записать и прочитатать переменную Port1 (ожидается неудача),
совершается попытка записать и прочитатать переменную Port2 (ожидается успех)*/
int TApcSerialTest::moveTest(const std::string astrPortName){
  TApcSerialPort Port1;
  int nResult = Port1.file_open(astrPortName);
  if (nResult != 0){
    std::cerr << "moveTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }

  enBaudRate adwBaudRate = enBaudRate::b9600;
  nResult = Port1.configure(adwBaudRate);
  if (nResult != 0){
    std::cerr << "moveTest failed. Error from configure" << std::endl;
    return-1;
  }
  
  size_t astExternWsize = 1;
  uint32_t adwExternWriteTimeout = 100;
  uint8_t apWBuf[astExternWsize] = {};
  size_t astWritten=0;
  nResult = Port1.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult != 0){
    std::cerr << "moveTest failed. Error from write" << std::endl;
    return-1;
  }
  size_t astExternRsize = 100;
  uint32_t adwExternReadTimeout = 100;
  uint8_t apRBuf[astExternRsize]={};
  size_t astRead = 0;
  nResult = Port1.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult != 0){
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
  if (nResult == 0){
    std::cerr << "moveTest failed. Error from writing Port1 after move" << std::endl;
    return -1;
  }
  nResult = Port1.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult == 0){
    std::cerr << "moveTest failed. Error from reading Port1 after move" << std::endl;
    return -1;
  }

  nResult = Port2.write(apWBuf, astExternWsize, adwExternWriteTimeout, astWritten);
  std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
  if (nResult != 0){
    std::cerr << "moveTest failed. Error from write Port2" << std::endl;
    return-1;
  }
  nResult = Port2.read(apRBuf, astExternRsize, adwExternReadTimeout, astRead);
  std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
  if (nResult != 0){
    std::cerr << "moveTest failed. Error from read Port2" << std::endl;
    return-1;
  }
  for(size_t i=0; i< astRead; ++i) {
    std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)apRBuf[i] << " ";
  }
  std::cout << std::dec<< std::endl << std::endl;//*/
  std::cout << "moveTest passed successfully." << std::endl;
  std::cout << "================================================================================="<<std::endl;
  return 0;
}
 
 /*Поочередно выставляются и выводятся разные скорости, производится сравнение скороти ввода и вывода*/
int TApcSerialTest::baudrateTest(const std::string astrPortName){
  TApcSerialPort Port1;
  int nResult = Port1.file_open(astrPortName);
  if (nResult != 0){
    std::cerr << "baudrateTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }

  TApcSerialPort Port = std::move(Port1);
  std::cout << "PORT1 handle = " << Port1.get_handle() << std::endl;

  size_t array_size = 8;
  enBaudRate adwBaudRate [array_size] = {enBaudRate::b1200, enBaudRate::b2400, enBaudRate::b4800, enBaudRate::b9600, enBaudRate::b19200, enBaudRate::b38400, enBaudRate::b57600, enBaudRate::b115200};
  for (int i = 0; i < array_size; i++){
    nResult = Port.configure(adwBaudRate[i]);
    if (nResult != 0){
      std::cerr << "baudrateTest failed. Error from configure" << std::endl;
      return-1;
    }

    std::string speed = {};
    nResult = Port.get_baudrate_from_hardware(speed);
    if (nResult != 0){
      std::cerr << "baudrateTest failed. Error from get_baudrate_from_hardware" << std::endl;
      return -1;
    }

    std::cout << speed << std::endl;
  }
  std::cout << "baudrateTest passed successfully." << std::endl;
  return 0;
};

/*Проверка записи и чтения из порта в цикле с внешним таймаутом. Пока не достигнуто требуемое количество символов или не истек внешний таймаут будет повторяться чтение/запись */
int TApcSerialTest::cycleTest(const std::string astrPortName){
  TApcSerialPort Port;
  int nResult = Port.file_open(astrPortName);
  if (nResult != 0){
    std::cerr << "cycleTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  enBaudRate adwBaudRate = enBaudRate::b9600;
  int array_size =2;
  uint32_t min[array_size] = {0, 5};
  uint32_t time[array_size] = {0, 5};
  for (int i = 0; i < array_size; i++){
    for (int j =0; j < array_size; j++){
      nResult = Port.configure(adwBaudRate, min[i], time[j]);
      if (nResult != 0){
        std::cerr << "cycleTest failed. Error from configure" << std::endl;
        return-1;
      }
      for (int k = 0; k < 5; k++){
        std::cout << "Test " << array_size*i+j+1 << " min = " << min[i] << " time = " << time[j] << std::endl;
        uint32_t adwExternWriteTimeout = 100;
        size_t astWsize = 100;
        uint8_t apWBuf[astWsize] = {};
        uint32_t adwWriteTimeout = 10;
        size_t astWritten=0;
        nResult = Port.cycle_write(apWBuf, astWsize, adwWriteTimeout, adwExternWriteTimeout, astWritten);
        std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
        if (nResult != 0){
          std::cerr << "cycleTest failed. Error from write" << std::endl;
          return-1;
        }
        size_t astRsize = 100;
        uint32_t adwExternReadTimeout = 100;
        uint8_t apRBuf[astRsize]={};        
        uint32_t adwReadTimeout = 10;
        size_t astRead = 0;
        nResult = Port.cycle_read(apRBuf, astRsize, adwReadTimeout, adwExternReadTimeout, astRead);
        std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
        if (nResult != 0){
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

/*Проверка единовременной (одна попытка) записи и единовременного чтения из порта */
int TApcSerialTest::onceTest(const std::string astrPortName){
  TApcSerialPort Port;
  int nResult = Port.file_open(astrPortName);
  if (nResult != 0){
    std::cerr << "cycleTest failed. Port with this name isn't exist" << std::endl;
    return -1;
  }
  enBaudRate adwBaudRate = enBaudRate::b9600;
  int array_size =2;
  uint32_t min[array_size] = {0, 5};
  uint32_t time[array_size] = {0, 5};
  for (int i = 0; i < array_size; i++){
    for (int j = 0; j < array_size; j++){
      nResult = Port.configure(adwBaudRate, min[i], time[j]);
      if (nResult != 0){
        std::cerr << "cycleTest failed. Error from configure" << std::endl;
        return-1;
      }
      for (int k = 0; k < 5; k++){
        std::cout << "Test " << array_size*i+j+1 << " min = " << min[i] << " time = " << time[j] << std::endl;
        size_t astWsize = 100;
        uint32_t adwWriteTimeout = 100;
        uint8_t apWBuf[astWsize] = {};
        size_t astWritten=0;
        nResult = Port.write(apWBuf, astWsize, adwWriteTimeout, astWritten);
        std::cout << "write res: " << nResult << ", written: " << astWritten  << std::endl;
        if (nResult != 0){
          std::cerr << "onceTest failed. Error from write" << std::endl;
          return-1;
        }

        size_t astRsize = 100;
        uint32_t adwReadTimeout = 100;
        uint8_t apRBuf[astRsize]={};
        size_t astRead = 0;
        nResult = Port.read(apRBuf, astRsize, adwReadTimeout, astRead);
        std::cout << "read res: " << nResult << ", read: " << astRead << std::endl;
        if (nResult != 0){
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

//проверка порта с подключенным устройством
int TApcSerialTest::useful_portTest(const std::string astrPortName){
  int nResult = openTest(astrPortName);
  if (nResult != 0){
    return -1;
  }
  nResult = moveTest(astrPortName);
  if (nResult != 0){
    return -1;
  }
  nResult = baudrateTest(astrPortName);
  if (nResult != 0){
    return -1;
  }

  nResult = onceTest(astrPortName);
  if (nResult != 0){
    return -1;
  }

  nResult = cycleTest(astrPortName);
  if (nResult != 0){
    return -1;
  }
  return 0;
};

/* Тест при отсутствии порта. В тесте ожидается ошибка при открытии порта, т.к. порта с таким именем быть не должно. Реализована обратная проверка */
int TApcSerialTest::no_portsTest(){
  const std::string astrPortName = "Any_string_name";
  int nResult = openTest(astrPortName);
  if (nResult == 0){
    std::cout << "no_portTest failed. Port exists." << std::endl;
    return -1;
  }
  std::cout << "no_portTest passed successfully." << std::endl;
  return 0;
}

/*Тест существующего порта, к которому не подключены устройства*/
int TApcSerialTest::port_without_deviceTest(const std::string astrPortName){
  int nResult = openTest(astrPortName);
  if (nResult != 0){
    return -1;
  }
  nResult = baudrateTest(astrPortName);
  if (nResult != 0){
    return -1;
  }
  return 0;
}

/*запуск 3-х групп тестов*/
int TApcSerialTest::run(){
  int nResult = 0;
  nResult = no_portsTest();
  std::cout << "Result of no_portsTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  nResult = port_without_deviceTest("/dev/ttyS1");
  std::cout << "Result of port_without_deviceTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  nResult = useful_portTest("/dev/ttyS0");
  std::cout << "Result of useful_portTest = " << nResult << std::endl;
  std::cout << std::endl << "#################################################################################" << std::endl;//*/
  return 0;
}