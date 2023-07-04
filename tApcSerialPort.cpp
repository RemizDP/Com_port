#include <cstring>      // вывод ошибок
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // набор дескрипторов
#include <fcntl.h>      // open()
#include <iostream>

#include "TApcSerialPort.h"

TApcSerialPort::TApcSerialPort(){
    m_FileHandle =-1;
  };
TApcSerialPort::TApcSerialPort(uint32_t adwFileHandle){
    m_FileHandle =adwFileHandle;
  };
speed_t ConvertBaudRate(uint32_t adwBaudRate){
  speed_t nResult = B9600;
  switch (adwBaudRate)
  {
    case 1200:
      nResult = B1200;
      break;
    case 2400:
      nResult = B2400;
      break;
    case 4800:
      nResult = B4800;
      break;
    default:
      nResult = B9600;
      break;
  }
  return nResult;
}

int TApcSerialPort::get_handle(){
    return m_FileHandle;
  };

  int TApcSerialPort::file_open(const std::string astrPortPathName){
    if (m_FileHandle>0){
      std::cerr << "Error: port already opened" << std::endl;
      return -1;
    }

  /*O_RDWR -- открытие файла на чтение и запись, 
    O_NOCTTY -- терминальное устройство по этому пути не станет терминальным устройством управления процесса,
    O_SYNC -- режим синхронного ввода/вывода, т.е. пока данные не будут физически записаны, write блокирует вызывающий процесс. 
    https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
  */
    m_FileHandle = open(astrPortPathName.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (m_FileHandle == -1) {
      std::cerr << "Error opening: "<< astrPortPathName.c_str() << " " << strerror(errno)<< std::endl;
      return -1;
    }
    std::cout << "Port is opened\n"<< std::endl;
    return 0;
  } 
  
  int TApcSerialPort::file_close(){
    int nResult = close(m_FileHandle);
    if (nResult == -1){
      std::cerr << "Error closing: "<< strerror(errno)<< std::endl;
      return -1;
    }
    std::cout << "Port is closed"<< std::endl;
    return 0;
  }

  int TApcSerialPort::configure_settings(uint32_t adwBaudRate){
    struct termios aSettings={};
    aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием
    
    // несмотря на создание пустой структуры на всякий случай сброс флагов убирать не стал. 
    aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
    aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 

    /* Настройка флагов взята со StackOverflow 
  https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
  Для неканонического режима (в основном отключение функций терминала и спецсимволов терминала, например Сtrl+Z и др.)
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
    сброс флагов (CSIZE -- флага маски размера символов,
                  PARENB -- флага бита четности)
    установка флага CS8 -- 8-bit символы (установка маски размера)       

    значения c_iflag по умолчанию: (BRKINT|ICRNL|IMAXBEL)
    значения c_lflag по умолчанию: (ISIG|ICANON|ECHO|IEXTEN|ECHOE|ECHOKE|ECHOCTL).
    значения c_oflag по умолчанию: (OPOST|ONLCR|ONOEOT).
    */

    //неканонический режим устанавливается функцией 
    cfmakeraw (&aSettings);                 // нет возвращает ошибки

    //извлекать байты как только становятся доступны
    //параметры влияют на режим чтения: MIN -- на какое минимальное кол-во символов среагирует для чтения
    //TIME -- время задержки (таймаут на символ). 
    //Если MIN == 0, TIME == 0, то режим немедленного возврата, т.е. будут доступны только уже принятые символы (в моем случае не влияет)
    //Если MIN > 0,  TIME == 0, то режим блокирующего чтения
    //Если MIN == 0, TIME > 0, то возврат по крайней мере одного полученного символа или произойдет истечение времени TIME
    //Если MIN > 0, TIME > 0, то посимвольный таймаут для чтения, вернется по меньшей мере MIN символов
    aSettings.c_cc[VMIN] = 1;               //минимальное кол-во символов для передачи за раз
    aSettings.c_cc[VTIME] = 1;              //время ожидания (задержка) в децисекундах

    speed_t asptBaudRate = ConvertBaudRate(adwBaudRate);
    int nResult = cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
    if (nResult == -1){
      std::cerr << "Error from cfsetospeed: " << strerror(errno) << std::endl;
      return -1;
    }
    nResult = cfsetispeed(&aSettings, asptBaudRate);  //установка скорости ввода
    if (nResult == -1){
      std::cerr << "Error from cfsetispeed: " << strerror(errno) << std::endl;
      return -1;
    }

    // применеине вышеуказанных настроек 
    nResult = tcsetattr(m_FileHandle, TCSANOW, &aSettings);
    if (nResult == -1) {
      std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
      return -1;
    }
    return 0; 
  };

  // Пишем в порт (Habr). Таймаут в милисекундах. https://habr.com/ru/companies/ruvds/articles/578432/
  int TApcSerialPort::write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten){
    astWritten = 0;               // количество записанных символов 

    struct pollfd fds={};         // для взаимодействия по событиям
    fds.fd=m_FileHandle;
    fds.events = POLLOUT;
    
    int nResult = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
    if (nResult==-1){
      std::cerr << "Error from poll:" << strerror(errno) << std::endl;
      return -1;
    }
    
    if(fds.revents & POLLOUT){
      astWritten = ::write(m_FileHandle, apBuf, astSize);
      if (astWritten == -1){
        std::cerr << "Error from write" << strerror(errno) << std::endl;
        }
      nResult = tcdrain(m_FileHandle); // ждет, пока все данные вывода, записанные на объект, на который ссылается дескриптор, не будут переданы.
      if (nResult==-1){
      std::cerr << "Error from tcdrain:" << strerror(errno) << std::endl;
      return -1;
    }
      //usleep(1000000);
    }
    else {
      std::cerr << "Writing is impossible: revent doesn't equal POLLOUT " << strerror(errno) << std::endl;
      return -1;
    }
    std::cout << "Write ends successfull, Length = " << astWritten << std::endl;
    memset(apBuf, 0, astSize);  // нет возвращает ошибки
    return 0; 
  };

  // Чтение из порта (Habr). Таймаут в милисекундах. https://habr.com/ru/companies/ruvds/articles/578432/
  int TApcSerialPort::read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen){
    astRlen = 0; //кол-во прочитанных символов
    struct pollfd fds={};
    fds.fd=m_FileHandle;
    fds.events = POLLIN;

    int nResult = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
    if (nResult==-1){
      std::cerr << "Error from poll:" << strerror(errno) << std::endl;
      return -1;
    }
    //usleep(1000);
    if(fds.revents & POLLIN) {
      astRlen = ::read(m_FileHandle, apBuf, astSize);
      if (astRlen == -1){
        std::cerr << "Error from read" << strerror(errno) << std::endl;
        }
      }
    else{
      std::cerr << "Reading is impossible: revent doesn't equal POLLIN " << strerror(errno) << std::endl;
      return -1;
    }
    std::cout << "Read ends successfull, Length = " << astRlen << std::endl;
    return 0;
  }