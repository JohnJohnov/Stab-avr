//Тестовый скетч для проверки коммуникации по последовательному порту
//--https://github.com/JohnJohnov/Stab-avr
//--https://alcodistillers.ru/forum/viewtopic.php?id=1549
//--JohnJohnov-----------------------
//--v0.1-------------------
//--
//-------------------------
#include <Wire.h>
#include <ASOLED.h>
//
#define VERSION "v0.1"  // Версия скетча
#define VERSION_LEN 4   // Длина версии скетча в символах для правильного вывода на дисплей
//
//========================================
//=====Настраиваемые параметры============
//========================================
#define DisplayReset    // Раскомментить, если используется вывод сброса дисплея
//
#define LINE_FREQ 155   // Определяет начальную частоту для фазовой автоподстройки частоты сети (50,08Гц)
#define P_TIME_MAX 100  // Количество полупериодов сети в секунду для отсчета времени
//
//=====Настройки коммуникации по последовательному порту============
#define USE_USART         // Раскомментить для инициализации общения стаба с внешним контроллером 
  #ifdef USE_USART        //
  //
  #define USE_RMVK        // Раскомментить для включения общения с внешним контроллером по протоколу Samovar и/или РМВ-К
  //
    #ifndef USE_RMVK      //
    #define USE_ADprotocol// По умолчанию используется универсальный протокол
    #endif
  #endif
//========================================
//
#define OLED_newADDRESS (0x3C)  //Дисплей OLED 128х64
//
#define pin_OLED_res 2          //  Пин сброса OLED индикатора.
//
// Организуем флаги и индикаторы в структуру
static volatile struct flags {
  unsigned  zero : 1;         // Флаг перехода через ноль
  unsigned  PP : 1;           // Флаг начала очередного полупериода
  unsigned  PVolna : 1;       // Флаг полуволны ((отрицательная = 0, положительная = 1)
  unsigned  sum : 1;          // Флаг готовности насуммированных данных к обработке
  unsigned  Tout : 1;         // Флаг включения ТЭНа (твердотельное реле)
  unsigned  TRelay : 1;       // Флаг включения ТЭНа (контактное реле)
  unsigned  Ulow : 1;         // Флаг невозможности выдать установленный уровень мощности
  unsigned  Udown : 1;        // Флаг аварии сети (действующее напряжение ниже 100В)
  unsigned  NotZero : 1;      // Флаг аварии сети (не детектируются переходы через ноль)
  unsigned  razg : 1;         // Флаг режима "разгон"
  unsigned  razg_on : 1;      // Флаг начала режима "разгон"
  unsigned  butt : 1;         // Флаг опроса кнопок
  unsigned  writable : 1;     // Флаг записи уставок в EEPROM
  unsigned  DisplayOut : 1;   // Флаг вывода на дисплей обновленных данных
  unsigned  dspRefresh : 1;   // Флаг выхода из режима меню / обновления экрана
  unsigned  dspTimeout : 1;   // Флаг истечения времени ожидания выхода из меню
  unsigned  dspMenu : 2;      // Индикатор режима меню
  unsigned  uartUnhold : 1;   // Флаг разрешения передачи данных по USART
  unsigned  uartReport : 1;   // Флаг разрешения отправки данных внешнему контроллеру
  unsigned  uartTimeout : 1;  // Флаг истечения времени приема посылки по USART
} fl = {};  // Инициализируем структуру с нулевыми членами
//
// Организуем счетчики в структуру
static volatile struct counts {
  unsigned  Pnom_count : 3;   // Количество предустановок мощности
  unsigned  Pnom_number : 3;  // Номер активной предустановки мощности
  unsigned  PDMcount : 3;     // Счетчик для перебора уставок мощности ТЭНа
  unsigned  Pcount_tm : 7;    // Счетчик полупериодов для организации отсчета времени
  unsigned  menuWDT : 8;      // Счетчик секунд для организации отсчета ожидания выхода из меню
  unsigned  PPcount_R : 6;    // Счетчик времени шунтирования контактного реле
  unsigned  uartWDT : 4;      // Счетчик секунд для организации отсчета ожидания окончания посылки по USART
} cnt = {}; // Инициализируем структуру с нулевыми членами
//
byte X_position (const byte x, const uint16_t arg = 1); // Функция возвращает начальную позицию по Х для десятичного числа, в зависимости от количества знаков в нём.
byte X_centred (const byte len);    // Функция возвращает начальную позицию по Х для текста длинной len знаков, для размещения оного по центру дисплея.
byte A_to_HEX (const char a);       // Функция переводит символ ASCII в шестнадцатиричную цифру
char HEX_to_A (const byte x);       // Функция переводит шестнадцатиричную цифру в символ ASCII
//ASOLED ASOled;
#define ASOled LD // Заюзаем уже созданный в библиотеке дисплея объект LD
//
static char rawString[21] = "                    ";  //
static byte cnt_rawString = 0;    //
static byte rawStringNumber = 0;  //
const byte inoutStringNumber = 4; //
const byte dspStringNumber = 5;   //
//==============================================================================
//============================ПРОЦЕДУРЫ И ФУНКЦИИ===============================
//==============================================================================
//
// Функция возвращает начальную позицию по Х для десятичного числа, в зависимости от количества знаков в нём.
byte X_position (const byte x, const uint16_t arg) { // arg-выводимое число; х-позиция для arg, если бы оно было однозначно
  byte pix = 6; // Ширина шрифта в пикселях
  if (arg > 999) return pix * (x-3);
  else if (arg > 99) return pix * (x-2);
  else if (arg > 9) return pix * (x-1);
  else return pix * x;
}
//
// Функция возвращает начальную позицию по Х для текста длинной len знаков, для размещения оного по центру дисплея.
byte X_centred (const byte len) { // len - Количество знакомест в тексте
  byte wdt = 128; // Ширина дисплея в пикселях
  byte pix = 6;   // Ширина шрифта в пикселях
  if (len > wdt/pix) return 0;
  else return (wdt - (len * pix))/2;
}
//
// Функция переводит символ ASCII в шестнадцатиричную цифру, при ошибке возвращает 255
byte A_to_HEX (const char a) { // a - символ 0...F
  if (a >= 48 && a <= 57) { // Если а - от 0 до 9
    return byte(a-48);
  }
  else if (a >= 65 && a <= 70) { // Если а - от A до F
    return byte(a-55);
  }
  else if (a >= 97 && a <= 102) { // Если а - от a до f
    return byte(a-87);
  }
  else return 255;
}
//
// Функция переводит шестнадцатиричную цифру в символ ASCII, при ошибке возвращает X
char HEX_to_A (const byte x) {  // x - число, кое необходимо перевести в ASCII-код
  if (x <= 9) {
    return char(x + 48);
  }
  else if (x <= 15) {
    return char(x + 55);
  }
  else return 'X';
}
//
void pp_Delay(const uint16_t pp) {  //===Пауза, измеряется в полупериодах=====
  uint16_t  PPcount = 0;            // счетчик полупериодов
  while (PPcount < pp) {
    if (fl.PP) {
      PPcount++;
      fl.PP = 0;
    }
  }
}
//
#ifdef USE_USART//++++++++++++++++USART initialization++++++++++++++++++++++++++++
//Если задействовано управление регулятором ТЭНа через UART, инициализируем оный
//
void USART_start() {
  Serial.begin(9600, SERIAL_8N1); // Инициализируем USART
}
//
void test_rawString() {
  if (++cnt_rawString == 21) {
    cnt_rawString = 0;
    if (++rawStringNumber == 3) rawStringNumber = 0;
    for (byte i = 0; i < 21; i++) {
      rawString[i] = ' ';
    }
  }
}
//
#endif // USE_USART
//
#ifdef USE_ADprotocol //++++++++++++++++USART++++++++++++++++++++++++++++
//
//Байт "состав данных" b00010111 (основной параметр - мощность в нагрузке, доп. параметр - напряжение сети) в HEX-формате 0x17
static char USART_InfoData[14] = {'T','1','7','0','0','0','0','0','0','0','0','0','0',0x0D};  // Массив готовых данных для передачи внешнему контроллеру
static char USART_SetData[6];  // Массив управляющих символов от внешнего контроллера
//
void USART_parser() { // Парсим управляющую последовательность по универсальному протоколу
  //
  static byte index = 0;
  static byte data_size;
//
  while (Serial.available() > 0) {
    USART_SetData[index] = Serial.read(); // Вычитываем очередной байт
    test_rawString();
    rawString[cnt_rawString] = USART_SetData[index];
    ASOled.printString_6x8(rawString, 0, rawStringNumber);//
    //
    if ( !index || fl.uartTimeout ) {   // Начало
      fl.uartTimeout = 0;               // Сбросим флаг таймаута ожидания окончания посылки
      cnt.uartWDT = 0;                  // Сбросим таймер ожидания окончания посылки
      switch ( USART_SetData[0] ) {     // Ждём первый символ...
        case 'M':
        case 'm': {                     // ...запроса на изменение режима работы
          data_size = 2;
          index=1;
          break;
        }
        case 'P':
        case 'p': {                     // ...запроса на изменение уставки
          data_size = 5;
          index=1;
          break;
        }
        default: {
//          break;
        }
      }
    }
    else {
      if ( USART_SetData[index] == 0x0D ) { // Ждем последнего символа посылки <CR>
        //
        ASOled.printString_6x8(F("                     "), 0, inoutStringNumber); // 21 пробел пустая строка
        ASOled.printString_6x8(F("                     "), 0, dspStringNumber);   // 21 пробел пустая строка
        //
        if ( index == data_size ) {
          switch (index) {
            case 2: { // Парсим запрос на смену режима
              switch ( USART_SetData[1] ) {
                case '0': { // Переход в рабочий режим
                  ASOled.printString_6x8(USART_SetData, 0, inoutStringNumber); //
                  ASOled.printString_6x8(F("-> Переход в раб.реж."), 0, dspStringNumber);   //
                  break;
                }
                case '1': { // Переход в режим разгона
                  ASOled.printString_6x8(USART_SetData, 0, inoutStringNumber); //
                  ASOled.printString_6x8(F("-> Переход в разгон"), 0, dspStringNumber);   //
                  break;
                }
                case '2': { // Отключение нагрузки
                  ASOled.printString_6x8(USART_SetData, 0, inoutStringNumber); //
                  ASOled.printString_6x8(F("-> Останов"), 0, dspStringNumber);   //
                  break;
                }
                default: {
                  break;
                }
              }
              break;
            }
            case 5: { // Парсим запрос на смену уставки
              uint32_t tmp_p = 0;
              byte b;
              for (byte x=1; x <= 4; x++ ) {
                tmp_p *= 16;
                b = A_to_HEX (USART_SetData[x]);
                if (b == 255) {
                  break;
                }
                tmp_p += b;
              }
              if (b != 255) {
                ASOled.printString_6x8(USART_SetData, 0, inoutStringNumber); //
                ASOled.printString_6x8(F("-> Уставка "), 0, dspStringNumber);   //
                ASOled.printNumber((long)tmp_p, X_position (11), dspStringNumber); //
              }
              break;
            }
          }
        index = 0;
        }
        else {
          index = 0;
          ASOled.printString_6x8(USART_SetData, 0, inoutStringNumber); //
          ASOled.printString_6x8(F("-> Неизв.команда"), 0, dspStringNumber);   //
        }
      }
      else if ( index++ == data_size ) {
        index = 0;
      }
    }
  }
}
//
void USART_report() { //=====Отчет внешнему контроллеру по универсальному протоколу=====
  // Отправим
  Serial.write(USART_InfoData, 14);
}
//
#endif  //+++++++++++++++++++++++USART++++++++++++++++++++++++++++
//
#ifdef USE_RMVK //++++++++++++++++RMVK_/_Samovar++++++++++++++++++++++++++++
//
void USART_parser() { // Парсим управляющую последовательность от RMVK_/_Samovar
//
  static String inoutString;
  static byte index = 0;
//
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();// Вычитываем очередной байт
    test_rawString();
    rawString[cnt_rawString] = inChar;
    ASOled.printString_6x8(rawString, 0, rawStringNumber);//
    if ( !index || fl.uartTimeout ) { // Начало посылки
      if ((inChar == 'A') || (byte(inChar) == 0xD0)) { // Ждём первый символ посылки "A" или первый байт UTF-кириллицы из протокола Samovar'a
        inoutString = inChar;
        index=1;
        fl.uartTimeout = 0; // Сбросим флаг таймаута ожидания окончания посылки
        cnt.uartWDT = 0;    // Сбросим таймер ожидания окончания посылки
      }
    }
    else if ( index++ < 13 ) {  // Пока посылка не длиннее 13 символов, считаем её корректной
      if ( inChar == 0x0D ) {   // Ждем последнего символа посылки <CR>
        index = 0;
        ASOled.printString_6x8(F("                     "), 0, inoutStringNumber); // 21 пробел пустая строка
        ASOled.printString_6x8(F("                     "), 0, dspStringNumber);   // 21 пробел пустая строка
        char inoutCharArr[21];
        inoutString.toCharArray(inoutCharArr,21);
        inoutCharArr[20] = ' ';
        ASOled.printString_6x8(inoutCharArr, 0, inoutStringNumber);
        // Парсим строку, поскольку кончилась
        // В протоколе Samovar стандартное начало посылки "АТ" пересылается русскими символами в Юникоде. Баг или фича?
        if (( inoutString == F("AT+VI?")) ||  // Запрос текущего напряжения сети
            ( inoutString == F("АТ+VI?"))) {  // В этой строке "АТ" - русскими символами!
              ASOled.printString_6x8(F("-> 220В"), 0, dspStringNumber);  //
              inoutString = String(220);
        }
        else if (( inoutString == F("АТ+VO?")) || ( inoutString == F("АТ+VS?"))) {  // Запрос текущей мощности от Samovar. В этой строке "АТ" - русскими символами!
          ASOled.printString_6x8(F("-> 3000Вт"), 0, dspStringNumber);//
          inoutString = String(3000);
        }
        else if ( inoutString == F("AT+VO?") ) {  // Запрос текущего напряжения на выходе от РМВ-К
          ASOled.printString_6x8(F("-> 159В"), 0, dspStringNumber);  //
          inoutString = String(159);
        }
        else if ( inoutString == F("AT+VS?") ) {  // Запрос напряжения уставки на выходе от РМВ-К
          ASOled.printString_6x8(F("-> 160В"), 0, dspStringNumber);  //
          inoutString = String(160);
        }
        else if (( inoutString == F("AT+SS?")) ||   // Запрос режима от Samovar
                ( inoutString == F("АТ+SS?"))) {    // В этой строке "АТ" - русскими символами!
          ASOled.printString_6x8(F("-> 0 (раб.режим)"), 0, dspStringNumber); //
          inoutString = String(0);
        }
        else if (( inoutString == F("AT+ON=0")) ||  // Запрос на выключение стабилизатора
                ( inoutString == F("АТ+ON=0"))) {   // В этой строке "АТ" - русскими символами!
          ASOled.printString_6x8(F("-> нет ответа"), 0, dspStringNumber);//
          inoutString = "";
        }
        else if (( inoutString == F("AT+ON=1")) ||  // Запрос на включение режима "Разгон"
                ( inoutString == F("АТ+ON=1"))) {   // В этой строке "АТ" - русскими символами!
          ASOled.printString_6x8(F("-> нет ответа"), 0, dspStringNumber);//
          inoutString = "";
        }
        else if ( inoutString == F("AT+ON?") ) {              // Запрос состояния выхода от РМВ-К
          ASOled.printString_6x8(F("-> ON"), 0, dspStringNumber);//
          inoutString = String("ON");
        }
        else if ( inoutString.substring(0,8) == F("АТ+VS=") ) {  // Запрос на изменение уставки от Samovar. В этой строке "АТ" - русскими символами!
          ASOled.printString_6x8(F("-> нет ответа"), 0, dspStringNumber);//
          inoutString = "";
        }
        else if ( inoutString.substring(0,6) == F("AT+VS=") ) { // Запрос на изменение уставки от РМВ-К
          ASOled.printString_6x8(F("->"), 0, dspStringNumber);//
          ASOled.printNumber((long)inoutString.substring(6).toInt(), X_position (3), dspStringNumber);//
          inoutString = String(inoutString.substring(6).toInt());
        }
        else {  // Неизвестная или закосяченная команда
          ASOled.printString_6x8(F("-> неизв.команда"), 0, dspStringNumber); //
          inoutString = "";
        }
        // 
        if ( inoutString != "" )  {       // Если строка не пустая
            inoutString += char(0x0D);    // Добавляем в конец <CR>
            Serial.print( inoutString );  // Шлём!
        }
      }
      else {  // Еще не конец
        inoutString += inChar;  // Добавляем и это лыко в строку
      }
    }
    else {  // Посылка длинновата, а значит - некорректна, начинаем сначала
      index = 0;
    }
  }
}
//
#endif  //+++++++++++++++++++++++RMVK_/_Samovar++++++++++++++++++++++++++++
//
void Timers_init() { //===============Инициализация таймеров===================
  // Инициализация таймера 2 для формирования импульса нуля Zero
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21); // Счетчик работает в режиме CTC (сброс по совпадению)
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Предделитель на 1024 (сч. - 15.625 кГц/64мкс)
  OCR2A = LINE_FREQ; // Прерывание с удвоенной частотой сети
  TIMSK2 |= (1 << OCIE2A); // Разрешаем прерывания по совпадению с OCR2A
}//=================================Инициализация таймеров===================
//
void Pins_init() { //======Инициализация входов/выходов контроллера=========
#ifdef DisplayReset
  pinMode(pin_OLED_res, OUTPUT);      // Определяем вывод сброса дисплея, как выход
  digitalWrite(pin_OLED_res, LOW);    // Сбрасываем дисплей (!!! НЕ ЗАБЫТЬ ПЕРЕКЛЮЧИТЬ НА ВЫСОКИЙ !!!)
#endif
}
//
ISR(TIMER2_COMPA_vect) { //======Обработчик начала очередного полупериода по таймеру2=========
  //
  fl.PP = 1;              // Ставим флаг окончания очередного полупериода
  // Считаем время
  if (++cnt.Pcount_tm == P_TIME_MAX) {  // Уже секунду суммируем
    cnt.Pcount_tm = 0;
    //
    #ifdef USE_USART
      if (++cnt.uartWDT == 10) {  // Если прошло уже 10 секунд от начала приема посылки по USART
        fl.uartTimeout = 1;       // Установим флаг таймаута ожидания окончания посылки
        cnt.uartWDT = 0;          // Сбросим таймер ожидания окончания посылки
      }
    #endif
    #ifdef USE_ADprotocol
      fl.uartReport = 1;          // пора слать рапорт
    #endif
    //
  }
}//==============================Обработчик начала очередного полупериода по таймеру2=========
//------------------------------------------------------------------------------
void setup() {
// 
  Pins_init();    // Инициализируем входы/выходы
  Timers_init();  // Инициализируем таймеры
  sei();          // Разрешаем глобальные прерывания
  //
  pp_Delay(20);   // Подождем 20 полупериодов
//
#ifdef DisplayReset
  digitalWrite(pin_OLED_res, HIGH); // Разрешаем работу дисплея
#endif
//
  pp_Delay(10);   // Подождем 10 полупериодов для гарантированного разрешения
  //
  ASOled.init();                        // Инициализируем OLED дисплей
  //
  ASOled.clearDisplay();                // Очищаем, иначе некорректно работает для дисплеев на SH1106 (косяк библиотеки)
  ASOled.printString_6x8(F("Тест соединения"), X_centred (15), 0);
  ASOled.printString_6x8(F("по последовательному"), X_centred (20), 1);
  ASOled.printString_6x8(F("порту"), X_centred (5), 2);
  ASOled.printString_6x8(F(VERSION), X_centred (VERSION_LEN), 3);
  ASOled.printString_6x8(F("JohnJohnov"), X_centred (10), 6);
  ASOled.printString_6x8(F("alcodistillers.ru"), X_centred (17), 7);
  //
  pp_Delay(300);        // Подождем 300 полупериодов
  //
  ASOled.clearDisplay();
  fl.dspRefresh = 1;
  rawString[20] = ' ';

#ifdef USE_USART//++++++++++++++++USART initialization++++++++++++++++++++++++++++
//Если задействовано управление регулятором ТЭНа через UART, инициализируем оный
//
USART_start();
fl.uartUnhold = 1;
//
#endif // USE_USART
}
//
void loop() {
// 
  ASOled.printString_6x8(F("Тест посл.порта"), X_centred (15), 7);
  //
  #ifdef USE_ADprotocol
  if (fl.uartReport && fl.uartUnhold) {  //==========Отправка отчета внешнему контроллеру============
    USART_report();
    fl.uartReport = 0;
  } //=========================Отправка отчета внешнему контроллеру============
  #endif
  //
  #ifdef USE_USART
  if (fl.uartUnhold) {
  USART_parser();
  }
  #endif
}
//
