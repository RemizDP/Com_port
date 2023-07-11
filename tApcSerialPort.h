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

    /* открытие файла порта на чтение и запись с отключением режима терминала.
    Исторически по последовательному порту к машинам на UNIX подключались терминалы,
    поэтому функции терминала предусмотрены и сейчас*/
    int file_open(const std::string astrPortPathName);

    /*Реализация move семантики*/
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

    /*Более гибкая функция настройки порта, позволяет настроить маску размера символов (количество бит в символе), четность, количество стоповых битов
    Если в 3-х последних параметрах передать нули, то по умолчанию будет установлена схема 8N1*/
    int configure(enBaudRate adwBaudRate, uint32_t adwMin, uint32_t adwTime, uint8_t abNumberOfBits, bool abParityBit, uint8_t abStopBits);

    /*Вывод baudrate порта, установленного на текущий момент*/
    int get_baudrate_from_hardware(std::string& astrSettings);
    /*Вывод в строку настроек (количества бит на символ, четность, количество стоповых битов, скорости)*/
    int get_settings_from_hardware(std::string& astrSettings);


    /* запись в порт и чтение из порта, реализованы с использованием структуры pollfd. Функция poll использует эту структуру,
    ожидает наступление событий (в нашем случае POLLIN -- символы готовы для прочтения с порта; и POLLOUT символы готовы для записи в порт)*/
    int write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astWritten);
    int read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, size_t& astRlen);

    /*внешние запись и чтение из порта для снятия ограничения количества записанных/прочитанных данных 
    Если истекает внешний таймаут или достигнуто требуемое количество символов, то запись/чтение останавливается*/
    int cycle_write(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astWritten);
    int cycle_read(uint8_t* apBuf, size_t astSize, uint32_t adwTimeout, uint32_t adwExternTimeout, size_t& astRlen);

    /* на всякий случай, возвращает значение дескриптора*/
    int get_handle();
  private:
    /*Запрет использования конструктора копирования, т.к. физический порт копированть нельзя.
    В противном случае могут произойти ошибки (запись или чтение с использованием уже закрытого порта)*/
    TApcSerialPort(const TApcSerialPort& astrPortPathName);
    TApcSerialPort& operator=(const TApcSerialPort& ataspPort);

    int m_FileHandle;
   
};
