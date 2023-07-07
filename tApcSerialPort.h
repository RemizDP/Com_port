#pragma once
#include <string>       // работа с типом string

// допустимые значения baudrate было решено ограничить набором enum
enum class enBaudRate : uint8_t {b1200, b2400, b4800, b9600, b19200, b38400, b57600, b115200};

class TApcSerialPort{

  public:
    /* конструктор с инициализацией дескриптора значением -1 (порт закрыт)*/
    TApcSerialPort();
    /* деструктор с закрытием порта. Если дескриптор отрицательный, выводит собщение об ошибке*/
    ~TApcSerialPort();

    /* предназначена для извлечения атрибутов (настроек) по дескриптору 
    с помощью ф-ции tcgetattr или проверки дескриптора*/
    int get_handle();

    /* открытие файла порта на чтение и запись с отключением режима терминала.
    Исторически по последовательному порту к машинам на UNIX подключались терминалы,
    поэтому функции терминала предусмотрены и сейчас*/
    int file_open(const std::string astrPortPathName);


    TApcSerialPort(TApcSerialPort&& ataspPort);
    TApcSerialPort& operator=(TApcSerialPort&& ataspPort);
    /* настройка порта по схеме 8N1, отключение ф-ций терминала с помощью флагов, установка baudrate (в линукс можно выставить разные baudrate для чтения и записи)
    С помощью параметров MIN и TIME можно изменять режим чтения. Предусмотрено 4 таких таких режима:
    Если MIN == 0, TIME == 0, то режим немедленного возврата, т.е. будут доступны только уже принятые символы (в моем случае не влияет, максимальное число возвращаемых символов не изменилось)
    Если MIN > 0,  TIME == 0, то режим блокирующего чтения (не очень интересно,т.к. при отсутствии готовых для чтения символов будет бесконечно висеть в ожидании)
    Если MIN == 0, TIME > 0, то возврат по крайней мере одного полученного символа или произойдет истечение времени TIME
    Если MIN > 0, TIME > 0, то посимвольный таймаут для чтения, вернется по меньшей мере MIN символов
    Подробнее: http://unixwiz.net/techtips/termios-vmin-vtime.html*/
    int configure(enBaudRate adwBaudRate);
    int configure(enBaudRate adwBaudRate, uint32_t adwMin, uint32_t adwTime);

    /* запись в порт и чтение из порта, реализованы с использованием структуры pollfd. Функция poll использует эту структуру,
    ожидает наступление событий (в нашем случае POLLIN -- символы готовы для прочтения с порта; и POLLOUT символы готовы для записи в порт)*/
    int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten);
    int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen);

    /*внешние запись и чтение из порта для снятия ограничения количества записанных/прочитанных данных 
    Если истекает внешний таймаут или достигается внешнее количество записанных/прочитанных символов, то запись/чтение останавливается*/
    int cycle_write(uint8_t* apBuf, size_t astSize, size_t astExternSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astWritten);
    int cycle_read(uint8_t* apBuf, size_t astSize, size_t astExternSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astRlen);
  private:
   int m_FileHandle;
};
