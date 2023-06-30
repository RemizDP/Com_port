//#define USE_RTS
#include <errno.h>
#include <fcntl.h>      // open(), close(), write(), read()
#include <stdio.h>      
#include <stdlib.h>
#include <cstdlib>     
#include <string>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>       // набор дескрипторов
#include <stdint.h>

class TApcSerialPort{

  public:

  TApcSerialPort(){
    m_FileHandle =-1;
    m_PortPathName =(char*)"Not_opened";
  };
  TApcSerialPort(int fd){
    m_FileHandle =fd;
    m_PortPathName =(char*)"Without_name";
  };
  ~TApcSerialPort(){};

  int get_FH(){
    return m_FileHandle;
  };
  char* get_name(){
    return m_PortPathName;
  }

  /* Открытие файла на чтение и запись, терминальное устройство по этому пути 
  не станет терминальным устройством управления процесса, режим синхронного ввода/вывода
  т.е. пока данные не будут физически записаны, write блокирует вызывающий процесс. 
  */
  int file_open(const char* astrPortPathName){
    m_FileHandle = open(astrPortPathName, O_RDWR | O_NOCTTY | O_SYNC);
    m_PortPathName = (char*)astrPortPathName;
    if (m_FileHandle < 0) {
      printf("Error opening %s: %s\n", m_PortPathName, strerror(errno));
      return -1;
    }
    printf("Port is opened\n");
    return 0;
  } 
  
  int file_close(){
    int ret = close(m_FileHandle);
    if (ret<0)
      printf("Error closing %s: %s\n", (char*)m_PortPathName, strerror(errno));
    else
      printf("Port is closed\n");
    return ret;
  }

  /*
  Настройка флагов взята со StackOverflow
  */
  int applyNonCanonicalPortSettings(struct termios& aSettings){
    if (tcgetattr(m_FileHandle, &aSettings) < 0) {
      printf("Error from tcgetattr (port isn't opened): %s\n", strerror(errno));
      return -1;
    }
    aSettings.c_cflag |= (CLOCAL | CREAD);    // игнорировать управление линиями с помощью модема, включить прием 
    aSettings.c_cflag &= ~CSIZE;              // сброс маски размера символов
    aSettings.c_cflag |= CS8;                 // 8-bit символы (установка маски размера)
    //aSettings.c_cflag &= ~PARENB;           // сброс бита четности
    aSettings.c_cflag &= ~CSTOPB;             // установка только одного стопового бита 
    aSettings.c_cflag &= ~CRTSCTS;            // отключение аппаратного управления потоком 

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
    cfmakeraw (&aSettings);
    aSettings.c_cflag &= ~PARENB;           // сброс бита четности

    //извлекать байты как только становятся доступны
    aSettings.c_cc[VMIN] = 1;               //минимальное кол-во символов для передачи за раз
    aSettings.c_cc[VTIME] = 1;              //время ожидания (задержка) в децисекундах

    if (tcsetattr(m_FileHandle, TCSANOW, &aSettings) != 0) {
      printf("Error from tcsetattr: %s\n", strerror(errno));
      return -1;
    }
    return 0; 
  };

  int applyCanonicalPortSettings(struct termios& aSettings){
    if (tcgetattr(m_FileHandle, &aSettings) < 0) {
      printf("Error from tcgetattr (port isn't opened): %s\n", strerror(errno));
      return -1;
    }
    /* set new port settings for canonical input processing */ 
    aSettings.c_cflag = B9600 | CS8 | CLOCAL;
    aSettings.c_cflag &= ~CSTOPB;
    aSettings.c_cflag &= ~PARENB;
    aSettings.c_iflag = IGNPAR | ICRNL;
    aSettings.c_oflag = 0;
    aSettings.c_lflag = ICANON;

    aSettings.c_cc[VMIN] = 1;               //минимальное кол-во символов для передачи за раз
    aSettings.c_cc[VTIME] = 1;              //время ожидания (задержка) в децисекундах
    if (tcsetattr(m_FileHandle, TCSANOW, &aSettings) != 0) {
      printf("Error from tcsetattr: %s\n", strerror(errno));
      return -1;
    }
    return 0;}

  int prepareSettings(struct termios& aSettings, speed_t asptBaudRate){
    int ret = applyNonCanonicalPortSettings(aSettings);
    ret += cfsetospeed(&aSettings, asptBaudRate);  //установка скорости вывода
    ret += cfsetispeed(&aSettings, asptBaudRate);  //установка скорости ввода
    if (tcsetattr(m_FileHandle, TCSANOW, &aSettings) != 0) {
      printf("Error from tcsetattr: %s\n", strerror(errno));
      return -1;
    }
    //aSettings.c_cflag |= asptBaudRate;
    return ret;
  };

  // Пишем в порт (StackOverflow). Таймаут в милисекундах.
  int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten){
    astWritten = 0;               // количество записанных символов 

    struct pollfd fds={};         // для взаимодействия по событиям
    fds.fd=m_FileHandle;
    fds.events = POLLOUT;
        
    if (poll(&fds, 1, adwTimeout)==-1){  // ожидает некоторое событие в файловом дескрипторе
      return -1;
    }

    if(fds.revents & POLLOUT){
      astWritten = ::write(m_FileHandle, apBuf, astSize);
      tcdrain(m_FileHandle); // заморозка процессов до окончания записи в память
      //usleep(1000000);
    }
    printf("Write ends successfull, Length = %ld\n", astWritten);
    memset(apBuf, 0, astSize);
    return 0; 
  };

  // Чтение из порта (StackOverflow). Таймаут в милисекундах.
  int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen){
    astRlen = 0; //кол-во прочитанных символов
    struct pollfd fds={};
    fds.fd=m_FileHandle;
    fds.events = POLLIN;
    if (poll(&fds, 1, adwTimeout)==-1){
      return -1;
    }
    usleep(1);
    
    if(fds.revents & POLLIN) {
      astRlen = ::read(m_FileHandle, apBuf, astSize);
        }
    printf("Read ends successfull, Length = %ld\n", astRlen);
    return 0;
  }
   private:

   int m_FileHandle;
   char* m_PortPathName;
};
