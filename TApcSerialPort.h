#pragma once
#include <cerrno>
#include <cstdlib>     
#include <string>
#include <cstring>
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // набор дескрипторов
#include <fcntl.h>      // open()
#include <stdint.h>     // uint8_t и uint32_t

//#include "TApcSerialPort.h"
//speed_t ConvertBaudRate(uint32_t adwBaudRate);
speed_t ConvertBaudRate(uint32_t adwBaudRate){
  speed_t ret = B9600;
  switch (adwBaudRate)
  {
    case 1200:
      ret = B1200;
      break;
    case 2400:
      ret = B2400;
      break;
    case 4800:
      ret = B4800;
      break;
    default:
      ret = B9600;
      break;
  }
  return ret;
}//*/

class TApcSerialPort{

  public:
    /*TApcSerialPort();
    TApcSerialPort(uint32_t fd);
    ~TApcSerialPort();//*/
    /*int get_FH();
    int file_open(const std::string astrPortPathName);
    int file_close();
    int SetDefaultSettings(uint32_t adwBaudRate);
    int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten);
    int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen);
//*/
  TApcSerialPort(){
    m_FileHandle =-1;
  };
  TApcSerialPort(uint32_t fd){
    m_FileHandle =fd;
  };
  ~TApcSerialPort(){};

  int get_FH(){
    return m_FileHandle;
  };//*/

  /* Открытие файла на чтение и запись, терминальное устройство по этому пути 
  не станет терминальным устройством управления процесса, режим синхронного ввода/вывода
  т.е. пока данные не будут физически записаны, write блокирует вызывающий процесс. 
  */
  int file_open(const std::string astrPortPathName){
    m_FileHandle = open(astrPortPathName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_FileHandle < 0) {
      std::cout << "Error opening: "<< astrPortPathName.c_str()<< strerror(errno)<< std::endl;
      return -1;
    }
    std::cout << "Port is opened\n"<< std::endl;
    return 0;
  } 
  
  int file_close(){
    int ret = close(m_FileHandle);
    if (ret<0){
      std::cout << "Error closing: "<< strerror(errno)<< std::endl;
      return -1;
    }
    std::cout << "Port is closed"<< std::endl;
    return 0;
  }//*/

  
  //Настройка флагов взята со StackOverflow
  
  int SetDefaultSettings(uint32_t adwBaudRate){
    struct termios aSettings={};
    aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием 
    aSettings.c_cflag &= ~CSIZE;              // сброс маски размера символов
    aSettings.c_cflag |= CS8;                 // 8-bit символы (установка маски размера)
    //aSettings.c_cflag &= ~PARENB;           // сброс бита четности
    aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
    aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 
    //*/
    /* setup для неканонического режима (в основном отключение функций терминала и спецсимволов терминала, например Сtrl+Z и др.)
    сброс флагов (IGNBRK -- флаг игнорирования режим BREAK (Ctrl+BREAK), 
                  BRKINT -- флаг сброса очередей, 
                  PARMRK -- флаг отслеживания ошибки, 
                  ISTRIP -- флаг удаления 8-го бита, 
                  INLCR -- флаг преобразовывания NL в CR при вводе
                  IGNCR -- флаг игнорирования перевода каретки,
                  ICRNL -- флаг преобразовывания перевода каретки в конец строки при вводе,
                  IXON -- флаг запуска управления потоком данных XON/XOFF при выводе 
                  ) 
    сброс флагов (ECHO -- флаг ECHO,
                  ECHONL -- флаг ECHO новой строки,
                  ICANON -- флаг канонического режима (линии используют специальные символы:
                  EOF, EOL, EOL2, ERASE, KILL, LNEXT, REPRINT, STATUS и WERASE, а также строчную буферизацию) 
                  ISIG -- флаг генерации сигналов при вводе сиволов из INTR, QUIT, SUSP или DSUSP
                  IEXTEN -- флаг включения режима ввода по умолчанию (как и ICANON 
                  должен быть включен для обработки специальных символов EOL2, LNEXT, 
                  REPRINT, WERASE, а также для того, чтобы работал флаг IUCLC.)
                  )
    сброс         OPOST -- флага включени режима вывода по умолчанию (если OPOST бит не установлен,
                  все остальные флаги игнорируются, а символы выводятся дословно).

    значения c_iflag по умолчанию: (BRKINT|ICRNL|IMAXBEL)
    значения c_lflag по умолчанию: (ISIG|ICANON|ECHO|IEXTEN|ECHOE|ECHOKE|ECHOCTL).
    значения c_oflag по умолчанию: (OPOST|ONLCR|ONOEOT).
    */

    //неканонический режим устанавливается функцией 
    cfmakeraw (&aSettings);                 // нет зарезервированного значения для ошибки
    aSettings.c_cflag &= ~PARENB;           // сброс бита четности

    //извлекать байты как только становятся доступны
    aSettings.c_cc[VMIN] = 1;               //минимальное кол-во символов для передачи за раз
    aSettings.c_cc[VTIME] = 1;              //время ожидания (задержка) в децисекундах

    speed_t asptBaudRate = ConvertBaudRate(adwBaudRate);
    int ret = cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
    if (ret!=0){
      std::cout << "Error from cfsetospeed: " << strerror(errno) << std::endl;
      return -1;
    }
    ret = cfsetispeed(&aSettings, asptBaudRate);  //установка скорости ввода
    if (ret!=0){
      std::cout << "Error from cfsetispeed: " << strerror(errno) << std::endl;
      return -1;
    }
    ret = tcsetattr(m_FileHandle, TCSANOW, &aSettings);
    if (ret != 0) {
      std::cout << "Error from tcsetattr: " << strerror(errno) << std::endl;
      return -1;
    }
    return 0; 
  };

  // Пишем в порт (StackOverflow). Таймаут в милисекундах.
  int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten){
    astWritten = 0;               // количество записанных символов 

    struct pollfd fds={};         // для взаимодействия по событиям
    fds.fd=m_FileHandle;
    fds.events = POLLOUT;
    
    int ret = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
    if (ret==-1){
      std::cout << "Error from poll:" << strerror(errno) << std::endl;
      return -1;
    }

    if(fds.revents & POLLOUT){
      astWritten = ::write(m_FileHandle, apBuf, astSize);
      ret = tcdrain(m_FileHandle); // заморозка процессов до окончания записи в память
      if (ret==-1){
      std::cout << "Error from tcdrain:" << strerror(errno) << std::endl;
      return -1;
    }
      //usleep(1000000);
    }
    std::cout << "Write ends successfull, Length = " << astWritten << std::endl;
    memset(apBuf, 0, astSize);  // нет зарезервированного значения для ошибки
    return 0; 
  };

  // Чтение из порта (StackOverflow). Таймаут в милисекундах.
  int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen){
    astRlen = 0; //кол-во прочитанных символов
    struct pollfd fds={};
    fds.fd=m_FileHandle;
    fds.events = POLLIN;

    int ret = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
    if (ret==-1){
      std::cout << "Error from poll:" << strerror(errno) << std::endl;
      return -1;
    }
    //usleep(1000);
    if(fds.revents & POLLIN) {
      astRlen = ::read(m_FileHandle, apBuf, astSize);
        }
    std::cout << "Write ends successfull, Length = " << astRlen << std::endl;
    return 0;
  }//*/
   private:

   int m_FileHandle;
};
