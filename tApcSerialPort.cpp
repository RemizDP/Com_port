#include <cstring>      // вывод ошибок strerror()
#include <termios.h>    // выставление флагов в структуре  (режим работы  порта)
#include <unistd.h>     // usleep, close(), write(), read();
#include <poll.h>       // poll()
#include <fcntl.h>      // open()
#include <iostream>
#include <chrono>       //steady_clock::now() секундомер в милисекундах

#include "tApcSerialPort.h"



TApcSerialPort::TApcSerialPort():m_FileHandle(-1){};
TApcSerialPort::TApcSerialPort(const TApcSerialPort& astrPortPathName){};
TApcSerialPort& TApcSerialPort::operator=(const TApcSerialPort& ataspPort){return* this;};
TApcSerialPort::~TApcSerialPort(){
  if (m_FileHandle < 0){
    std::cerr << "Closing warning: negative handle. "<< strerror(errno)<< std::endl;
  }
  else{
    close(m_FileHandle);
    std::cout << "Port is closed"<< std::endl;
    std::cout << "================================================================================="<<std::endl;
  }
}

TApcSerialPort::TApcSerialPort(TApcSerialPort&& ataspPort){
  m_FileHandle = ataspPort.get_handle();
  ataspPort.m_FileHandle = -1;
}
TApcSerialPort& TApcSerialPort::operator=(TApcSerialPort&& ataspPort){
  if (&ataspPort == this){
    return *this;
  }
  close(m_FileHandle);
  //close left
  m_FileHandle = ataspPort.get_handle();
  ataspPort.m_FileHandle = -1;
  return* this;
}
int convert_baudrate(enBaudRate aenBaudRate, speed_t& asptBaudRate){
  switch (aenBaudRate)
  {
    case enBaudRate::b1200:
      asptBaudRate = B1200;
      break;
    case enBaudRate::b2400:
      asptBaudRate = B2400;
      break;
    case enBaudRate::b4800:
      asptBaudRate = B4800;
      break;
    case enBaudRate::b9600:
      asptBaudRate = B9600;
      break;
    case enBaudRate::b19200:
      asptBaudRate = B19200;
      break;
    case enBaudRate::b38400:
      asptBaudRate = B38400;
      break;
    case enBaudRate::b57600:
      asptBaudRate = B57600;
      break;
    case enBaudRate::b115200:
      asptBaudRate = B115200;
      break;
    default:
      std::cerr << "Unexpected value: " << (int)aenBaudRate << std::endl;
      return -1;
      break;
  }
  return 0;
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
  //On success, open(), openat(), and creat() return the new file
  //descriptor (a nonnegative integer).  On error, -1 is returned and
  //errno is set to indicate the error. Взято с https://man7.org/linux/man-pages/man2/open.2.html
  //дескриптор РЕАЛЬНО ОТКРЫТОГО файла не может быть отрицательным, но на всякий случай проверка
  if (m_FileHandle < 0){
    std::cerr << "Something went wrong: negative handle after opening: "<< std::endl;
    return -1;
  }
  std::cout << "Port is opened\n"<< std::endl;
  return 0;
} 
int TApcSerialPort::configure(enBaudRate adwBaudRate){
  struct termios aSettings={};
  aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием
  
  // несмотря на создание пустой структуры на всякий случай сброс флагов убирать не стал. 
  aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
  aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 
    /* Настройка флагов взята со StackOverflow 
https://stackoverflow.com/questions/6947413/how-to-open-read-and-write-from-serial-port-in-c
Для неканонического режима (в основном отключение функций терминала и спецсимволов терминала, например Сtrl+Z и др.)
Подробнее о неканоническом режиме: http://www.igce.comcor.ru/non_canon.html  
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
    speed_t asptBaudRate = 0;
  int nResult = convert_baudrate(adwBaudRate, asptBaudRate);
  if (nResult == -1){
    std::cerr << "Error from convert_baudrate " << std::endl;
    return -1;
  }
  nResult = cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
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

  // перегрузка функции configure с возможностью задать min и time
int TApcSerialPort::configure(enBaudRate adwBaudRate, uint32_t adwMin, uint32_t adwTime){
  struct termios aSettings={};
  aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием
  
  // несмотря на создание пустой структуры на всякий случай сброс флагов убирать не стал.
  //aSettings.c_cflag |= CS5;                 // установка маски размера
  aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
  aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 
  cfmakeraw (&aSettings);                 // нет возвращает ошибки
    //извлекать байты как только становятся доступны
  //параметры влияют на режим чтения: MIN -- на какое минимальное кол-во символов среагирует для чтения
  //TIME -- время задержки (таймаут на символ). 
  //Если MIN == 0, TIME == 0, то режим немедленного возврата, т.е. будут доступны только уже принятые символы (в моем случае не влияет)
  //Если MIN > 0,  TIME == 0, то режим блокирующего чтения
  //Если MIN == 0, TIME > 0, то возврат по крайней мере одного полученного символа или произойдет истечение времени TIME
  //Если MIN > 0, TIME > 0, то посимвольный таймаут для чтения, вернется по меньшей мере MIN символов
  aSettings.c_cc[VMIN] = adwMin;               //минимальное кол-во символов для передачи за раз
  aSettings.c_cc[VTIME] = adwTime;              //время ожидания (задержка) в децисекундах
    speed_t asptBaudRate = 0;
  int nResult = convert_baudrate(adwBaudRate, asptBaudRate);
  if (nResult == -1){
    std::cerr << "Error from convert_baudrate " << std::endl;
    return -1;
  }
  nResult = cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
  if (nResult == -1){
    std::cerr << "Error from cfsetospeed: " << strerror(errno) << std::endl;
    return -1;
  }
  nResult = cfsetispeed(&aSettings, asptBaudRate);  //установка скорости ввода
  if (nResult == -1){
    std::cerr << "Error from cfsetispeed: " << strerror(errno) << std::endl;
    return -1;
  };
    // применеине вышеуказанных настроек 
  nResult = tcsetattr(m_FileHandle, TCSANOW, &aSettings);
  if (nResult == -1) {
    std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
};

/*Более гибкая функция настройки порта, позволяет настроить маску размера символов (количество бит в символе), четность, количество стоповых битов.
  Если в 3-х последних параметрах передать нули, то по умолчанию будет установлена схема 8N1
*/
int TApcSerialPort::configure(enBaudRate adwBaudRate, uint32_t adwMin, uint32_t adwTime, uint8_t abNumberOfBits, bool abParityBit, uint8_t abStopBits){
  struct termios aSettings={};
  aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием
  
  // несмотря на создание пустой структуры на всякий случай сброс флагов убирать не стал.
  if ((abStopBits == 0) || (abStopBits == 1)){ 
    aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
    std::cout << "1 stop bit has been set, "; 
  }
  else{
    aSettings.c_cflag |= CSTOPB;             // установка двух стоповых бит
    std::cout << "2 stop bits has been set, ";
  }

  aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 
  cfmakeraw (&aSettings);                 // нет возвращает ошибки

  aSettings.c_cflag &= ~CSIZE;
  if ((abNumberOfBits == 0) || (abNumberOfBits == 8)){
    aSettings.c_cflag |= CS8;
    std::cout << "8-bit characters have been set" << std::endl;
  }
  else if (abNumberOfBits == 5){
    aSettings.c_cflag |= CS5;
    std::cout << "5-bit characters have been set, ";
  }
  else if (abNumberOfBits == 6){
    aSettings.c_cflag |= CS6;
    std::cout << "6-bit characters have been set, ";
  }
  else if (abNumberOfBits == 7){
    aSettings.c_cflag |= CS7;
    std::cout << "7-bit characters have been set, ";
  }
  else {
    aSettings.c_cflag |= CS8;
    std::cout << "8-bit characters have been set, ";
  }

  if (abParityBit == false){
    aSettings.c_cflag &= ~PARENB;
    std::cout << "No parity bit have been set" << std::endl;
  }
  else{
    aSettings.c_cflag |= PARENB;
    std::cout << "Parity bit have been set" << std::endl;
  }
    //извлекать байты как только становятся доступны
  //параметры влияют на режим чтения: MIN -- на какое минимальное кол-во символов среагирует для чтения
  //TIME -- время задержки (таймаут на символ). 
  //Если MIN == 0, TIME == 0, то режим немедленного возврата, т.е. будут доступны только уже принятые символы (в моем случае не влияет)
  //Если MIN > 0,  TIME == 0, то режим блокирующего чтения
  //Если MIN == 0, TIME > 0, то возврат по крайней мере одного полученного символа или произойдет истечение времени TIME
  //Если MIN > 0, TIME > 0, то посимвольный таймаут для чтения, вернется по меньшей мере MIN символов
  aSettings.c_cc[VMIN] = adwMin;               //минимальное кол-во символов для передачи за раз
  aSettings.c_cc[VTIME] = adwTime;             //время ожидания (задержка) в децисекундах
  
  speed_t asptBaudRate = 0;
  int nResult = convert_baudrate(adwBaudRate, asptBaudRate);
  if (nResult == -1){
    std::cerr << "Error from convert_baudrate " << std::endl;
    return -1;
  }
  nResult = cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
  if (nResult == -1){
    std::cerr << "Error from cfsetospeed: " << strerror(errno) << std::endl;
    return -1;
  }
  nResult = cfsetispeed(&aSettings, asptBaudRate);  //установка скорости ввода
  if (nResult == -1){
    std::cerr << "Error from cfsetispeed: " << strerror(errno) << std::endl;
    return -1;
  };
    // применеине вышеуказанных настроек 
  nResult = tcsetattr(m_FileHandle, TCSANOW, &aSettings);
  if (nResult == -1) {
    std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
    return -1;
  }
  return 0;
};

int TApcSerialPort::get_baudrate_from_hardware(std::string& astrSettings){
  struct termios settings = {};
  tcgetattr(m_FileHandle, &settings);
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
    return -1;
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
      std::cerr<<"Unknown speed" << std::endl;
      return -1;
      break;
    }
  astrSettings = "Input baudrate: " + std::to_string(intISpeed) + "; Output baudrate: " + std::to_string(intOSpeed);
  if (intISpeed != intOSpeed){
    std::cerr<<"Different speeds for input and output"<<std::endl;
    return -1;
  }
  return 0;
}

/*Вывод в строку настроек (количества бит на символ, четность, количество стоповых битов, скорости)*/
int TApcSerialPort::get_settings_from_hardware(std::string& astrSettings){
  struct termios settings = {};
  tcgetattr(m_FileHandle, &settings);

  std::string flags = {};

  
  if (settings.c_cflag & CS8){
    flags += "Character size: CS8, ";
  }
  else if (settings.c_cflag & CS7){
    flags += "Character size: CS7, ";
  }
  else if (settings.c_cflag & CS6){
    flags += "Character size: CS6, ";
  }
  else if (settings.c_cflag & CS5){
    flags += "Character size: CS5, ";
  }
  else{
    flags += "Caslmcakc, ";
  }

  if (settings.c_cflag & PARENB){
    flags += "Parity: On, ";
  }
  else{
    flags += "Parity: Off, ";
  }

  if (settings.c_cflag & CSTOPB){
    flags += "Stop bits: 2 ";
  }
  else{
    flags += "Stop bits: 1 ";
  }

  astrSettings = flags;
  flags = {};
  int nResult = get_baudrate_from_hardware(flags);
  if(nResult != 0){
    std::cerr << "Error from get_baudrate_from_hardware" << std::endl;
    return -1;
  }
  astrSettings += flags;
  return 0; 

}

  // Пишем в порт (Habr). Таймаут в милисекундах. https://habr.com/ru/companies/ruvds/articles/578432/
int TApcSerialPort::write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten){
  astWritten = 0;               // количество записанных символов
  if (m_FileHandle < 0){
    std::cerr << "Error: negative handle to write" << std::endl;
    return -1;
  }
   
    struct pollfd fds={};         // для взаимодействия по событиям
  fds.fd=m_FileHandle;
  fds.events = POLLOUT;
  
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  int nResult = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
  if (nResult==-1){
    std::cerr << "Error from poll:" << strerror(errno) << std::endl;
    return -1;
  }
  
  if(!(fds.revents & POLLOUT)){
    std::cerr << "Writing is impossible: No data to write (revent doesn't equal POLLOUT) " << std::endl;
    std::cout << "Write ends successfull, Length = " << astWritten << std::endl;
    memset(apBuf, 0, astSize);  // не возвращает ошибки
    return 0;
  }
  
  astWritten = ::write(m_FileHandle, apBuf, astSize);
  if (astWritten == -1){
    std::cerr << "Error from write" << strerror(errno) << std::endl;
    return -1;
  }
  nResult = tcdrain(m_FileHandle); // ждет, пока все данные вывода, записанные на объект, на который ссылается дескриптор, не будут переданы.
  if (nResult==-1){
    std::cerr << "Error from tcdrain:" << strerror(errno) << std::endl;
    return -1;
  }
  std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
  const auto dur = finish - start;
  //usleep(1000000);
  std::cout << "Write ends successfull, Length = " << astWritten << " Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() << " ms." << std::endl;
  //memset(apBuf, 0, astSize);  // не возвращает ошибки
  return 0; 
};

// Чтение из порта (Habr). Таймаут в милисекундах. https://habr.com/ru/companies/ruvds/articles/578432/
int TApcSerialPort::read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen){
  astRlen = 0; //кол-во прочитанных символов
  if (m_FileHandle < 0){
    std::cerr << "Error: negative handle to read" << std::endl;
    return -1;
  }
  
  struct pollfd fds={};
  fds.fd=m_FileHandle;
  fds.events = POLLIN;

  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  int nResult = poll(&fds, 1, adwTimeout);    // ожидает некоторое событие в файловом дескрипторе
  if (nResult==-1){
    std::cerr << "Error from poll:" << strerror(errno) << std::endl;
    return -1;
  }
  //usleep(1000);
  if(!(fds.revents & POLLIN)) {
    std::cerr << "Reading is impossible: No data to read (revent doesn't equal POLLIN) " << std::endl;
    std::cout << "Read ends successfull, Length = " << astRlen << std::endl;
    return 0;
  }
  astRlen = ::read(m_FileHandle, apBuf, astSize);
  if (astRlen == -1){
    std::cerr << "Error from read" << strerror(errno) << std::endl;
    return -1;
  }
  std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
  const auto dur = finish - start;
  std::cout << "Read ends successfull, Length = " << astRlen << " Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() << " ms." << std::endl;
  return 0;
}

// Запись в порт с внешним таймаутом. Выполняется запись, пока не истек внешний таймаут или не записано запрашиваемое количество символов
int TApcSerialPort::cycle_write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astWritten){
  uint32_t nTimeout = adwExternTimeout;
  astWritten = 0;
  int nResult = 0;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while ((astWritten<astSize)&&(nTimeout>0)){
    size_t astWr = 0;
    if (nTimeout < adwTimeout){
      nResult= write(apBuf, astSize, nTimeout, astWr);
      if (nResult == -1){
        std::cerr << "Error from write" << std::endl;
        return -1;
      }
      nTimeout = 0;
      }
    else{
      nResult= write(apBuf, astSize, adwTimeout, astWr);
      if (nResult == -1){
        std::cerr << "Error from write" << std::endl;
        return -1;
      }
      nTimeout -= adwTimeout;
    }
    astWritten += astWr;
    apBuf += astWr;
  }
  std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
  const auto dur = finish - start;
  std::cout << "Write ends successfull, Total length = " << astWritten << " Total Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() << " ms." << std::endl;
  //memset(apBuf, 0, astExternSize);  // не возвращает ошибки
  return 0;
}

// Чтение из порта с внешним таймаутом. Выполняется чтение, пока не истек внешний таймаут или не прочитано запрашиваемое количество символов
int TApcSerialPort::cycle_read (uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astRlen){
  int nResult = 0;
  uint32_t nTimeout = adwExternTimeout;
  astRlen = 0;
  std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
  while((astRlen<astSize)&&(nTimeout > 0)){
    size_t astRd = 0;
    if (nTimeout < adwTimeout){
      nResult = read(apBuf, astSize, nTimeout, astRd);
      if (nResult == -1){
        std::cerr << "Error from read" << std::endl;
        return -1;
      }
      nTimeout = 0;
    }
    else{
      nResult = read(apBuf, astSize, adwTimeout, astRd);
      if (nResult == -1){
        std::cerr << "Error from read" << std::endl;
        return -1;
      }
      nTimeout -= adwTimeout;
    }
    astRlen += astRd;
    apBuf += astRd;
  }
  std::chrono::steady_clock::time_point finish = std::chrono::steady_clock::now();
  const auto dur = finish - start;
  std::cout << "Read ends successfull, Total length = " << astRlen << " Total Time = " << std::chrono::duration_cast<std::chrono::milliseconds>(dur).count() << " ms." << std::endl;
  //memset(apBuf, 0, astExternSize);  // не возвращает ошибки
  return 0;
}