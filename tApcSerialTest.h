#include "tApcSerialPort.h" //enBaudRate
class TApcSerialTest{
public:
  /*Проходит 3 нижеуказанных теста*/
  int run();

  /*Тест подходит, если у компьютера нет последовательного порта
  Пытается открыть порт с заданным именем*/
  int no_portsTest();

  /*Подходит, если порт есть, но к нему ничего не подключено.
  Проверяет существование порта, в случае успеха поочередно устанавливает разные скорости передачи и выводит их*/
  int no_data_portTest(const std::string astrPortName);

  /*Подходит, если какое-то устройство подключено к последовательному порту.
  Проверяет существование порта, в случае успеха поочередно устанавливает разные скорости передачи и выводит их.
  В разных режимах пишет в порт и читает с него.*/
  int useful_portTest(const std::string astrPortName);
private:
  //Проверка move оператора присваивания
  int moveTest(const std::string astrPortName);
  //Пытается открыть порт по имени
  int openTest(const std::string astrPortName);
  //Устанавливает разные скорости передачи
  int baudrateTest(const std::string astrPortName);
  //В двух режимах пишет в порт и читает с него 
  int sendTest(const std::string astrPortName);
  //Пишет в порт с учетом внешнего таймаута и размера по 5 раз с разными настройками MIN и TIME
  int cycleTest(const std::string astrPortName);
  //Обычная (единовременная) запись и обычное (единовременное) чтение по 5 раз с разными настройками MIN и TIME
  int onceTest(const std::string astrPortName);
  
};