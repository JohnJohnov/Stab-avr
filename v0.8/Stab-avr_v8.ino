//Регулятор ТЭНа полуволнами с программным детектором нуля
//--https://alcodistillers.ru/forum/viewtopic.php?id=1549
//--JohnJohnov---------
//--использован код OldBean----------
//--v0.2-------------------
//--добавлен дисплей-------
//--v0.3-------------------
//--ПИД-подстройка частоты сети по переходу через ноль
//--опрос кнопок----------
//--режим разгона--------
//--v0.4------------------
//--выборки набираются за целое количество периодов
//--v0.5------------------
//--оптимизация
//--v0.6------------------
//--организована корректная обработка отсутствия сети
//--v0.7------------------
//--исправлена ошибка выставления мощности менее 200Вт
//--битовые переменные упакованы в структуры
//--убрано ненужное мерцание символов на дисплее
//--добавлена возможность вернуть установленную мощность после экстренного отключения (идея d.styler)
//--v0.8------------------
//--менюшка при возвращении уст.мощности после экстр.откл.
//--
//-------------------------
#include <Wire.h>
//#include <EEPROM.h>
//#include <avr/eeprom.h>
#include <OzOLED.h>
//
#define VERSION "v0.8"      // Версия скетча
//
//#define Debug             // Раскомментить для дебажения
//#define Serial_out        // Раскомментить для вывода информации на терминал
#define High_level_triac    // Раскомментить если твердотельное реле ТЭНа управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
#define High_level_relay    // Раскомментить если контактное реле ТЭНа управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
//
#define OLED_newADDRESS    (0x3C)   //Дисплей OLED 128х64
//
#define pin_VACin 0       //  Пин входа измеряемого напряжения (A0)
#define pin_TOut 4        //  Пин выхода управления ТЭНом (на твердотельное реле)
#define pin_TRelay 6      //  Пин выхода управления ТЭНом (на контактное реле в режиме максимальной мощности)
#define pin_OLED_res 2    //  Пин сброса OLED индикатора.
#define pin_ZeroOut 5     //  Пин выхода импульса ноля
#define pin_DebugOut 7    //  Пин для отладки
#define pin_TestOut 3     //  Пин для отладки
//
#define pin_butt_1 10     //  Пин кнопки "1". Второй вывод на GND.
#define pin_butt_2 9      //  Пин кнопки "2". Второй вывод на GND.
#define pin_butt_3 12     //  Пин кнопки "3". Второй вывод на GND.
#define pin_butt_4 11     //  Пин кнопки "4". Второй вывод на GND.
#define pin_buttGND 8     //  Пин временного общего провода для подключения кнопок.
//
#define U_ZERO 512              // Значение нуля АЦП для двуполярного сигнала с постоянной составляющей на выходе ОУ
#define LINE_FREQ 155           // Определяет начальную частоту для фазовой автоподстройки частоты сети (50,08Гц)
#define T_ADC 49                // Определяет интервал между запусками АЦП (200 мкс)
//f_OCn = f_clk / 2*N*(1 + X), где N - коэффициент деления предделителя, X- содержимое регистра OCRnA
#define PSUM_MAX 50             // Количество периодов для набора отсчетов АЦП (50 - это за 1 сек, это порядка 5000 отсчетов)
#define ZSUM_MAX (PSUM_MAX*100) // Количество отсчетов АЦП без детекции ноля (за 1 сек порядка 5000 отсчетов)
#define CICLE 200               // Количество полупериодов в полном цикле регулирования (200 полупериодов - 2сек, если устанавливать более 255 - надо менять размерность сопутствующих переменных )
//#define Ku 0.55               // Коэффициент нормирования ((380/512)^2, 380В максимальное амплитудное) для Rail-to-Rail операционника
#define Ku 3                    // Коэффициент нормирования ((380/220)^2, 380В максимальное амплитудное) для стандартно установленного LM358
//
#define Kp 1        // Коэффициент пропорциональности для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Ki 5        // Интегральный коэффициент для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Kd 2        // Дифференциальный коэффициент для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Km 6        // Коэффициент для целочисленной математики (степень двойки для регистрового сдвига)
#define Phase 8     // сдвиг фаз между детекцией ноля и прерыванием таймера (в тиках таймера)
//
#define U_LINE_Q 52900                      // Квадрат номинала сети, для которого указана номинальная мощность ТЭНа (230В)
static volatile uint16_t Pnom = 1800;       // Номинальная мощность ТЭНа (надо бы хранить в EEPROM и устанавливать из менюшки)
#define PDMset_size 3                       // Уменьшенный на единицу размер массива уставок мощности ТЭНа (sizeof(PDMset)/sizeof(PDMset[0])) - 1
static volatile uint8_t PDMset[PDMset_size+1] = {CICLE/3,CICLE/2,2*CICLE/3,CICLE};  // Массив предустановок мощности ТЭНа
//
static volatile uint32_t sum;               // Сумматор квадратов отсчетов АЦП
static volatile uint32_t U_sum = 0;         // Сумма квадратов отсчетов АЦП, готовая для обработки
static volatile uint16_t sc = 0;            // Счетчик просуммированных квадратов
static volatile uint16_t sc_sum = 0;        // Счетчик просуммированных квадратов, готовый к обработке
static volatile uint16_t sc_notzero = 0;    // Счетчик выборок АЦП без перехода через ноль
static volatile uint8_t  pdm = 0;           // Текущий уровень PDM (принимает значения от 0 до CICLE)
static volatile uint32_t Pust = 0;          // Установленная мощность ТЭНа
//  static volatile uint8_t P_step = 0;     // Шаг установки мощности ТЭНа
static volatile uint8_t PDMust = 0;         // PDM, соответствующий установленной мощности ТЭНа
static volatile uint8_t PDMcount = 0;          // Счетчик для перебора уставок мощности ТЭНа
//
static uint16_t U_real = sqrt(U_LINE_Q);    // Среднеквадратичное за секунду
//
static volatile int16_t Uold = 0;           // Буферная переменная для НЧ-фильтрации
static volatile int32_t Udelta = 0;         // Буферная переменная для НЧ-фильтрации
static volatile int16_t lev = 0, err = 0;   // Буферные переменные для реализации PDM методом диффузии (смещения) ошибки
static volatile int8_t ps = 0;              // Буферная переменная для реализации PDM методом диффузии (смещения) ошибки
//
static volatile uint8_t PID_ust = LINE_FREQ;        // Данные для установки регистра сравнения таймера2
static volatile uint16_t PID_reg = PID_ust << Km;   // Функция управления ПИД
static volatile int8_t PVolna = 1;                  // Флаг полуволны (отрицательная = -1, положительная = 1)
static volatile int32_t PID_err_old = 0;            // Разность фаз из предыдущего шага
static volatile int32_t PID_int = 0;                // Интегральная составляющая из предыдущего шага
//
//
static volatile struct flags {
  unsigned  zero : 1;         // Флаг перехода через ноль
  unsigned  PP : 1;           // Флаг начала очередного полупериода
  unsigned  sum : 1;          // Флаг готовности насуммированных данных к обработке
  unsigned  Tout : 1;         // Флаг включения ТЭНа (твердотельное реле)
  unsigned  TRelay : 1;       // Флаг включения ТЭНа (контактное реле)
  unsigned  Ulow : 1;         // Флаг невозможности выдать установленный уровень мощности
  unsigned  Udown : 1;        // Флаг аварии сети (действующее напряжение ниже 100В)
  unsigned  NotZero : 1;      // Флаг аварии сети (не детектируются переходы через ноль)
  unsigned  razg : 1;         // Флаг режима "разгон"
  unsigned  razg_on : 1;      // Флаг начала режима "разгон"
  unsigned  butt : 1;         // Флаг опроса кнопок
  unsigned  DisplayOut : 1;   // Флаг обновления дисплея
  unsigned  dspMenu : 1;      // Флаг режима меню
  unsigned  dspRefresh : 1;   // Флаг выхода из режима меню  
} fl = {0};
//
//
//
//==============================================================================
//============================ПРОЦЕДУРЫ И ФУНКЦИИ===============================
//==============================================================================
//
void ADC_init() { //===============Инициализация АЦП===================
  ADMUX = 0;
  ADMUX |= ( 1 << REFS0);  // Задаем ИОН равный напряжению питания
  ADMUX |= (0 & 0x07);    // Выбираем пин A0 для преобразования
  ADCSRA |= (1 << ADPS2 ) | (1 << ADPS1) | (1 << ADPS0); // предделитель на 128
  //  ADCSRA |= (1 << ADATE); // Включаем автоматическое преобразование
  ADCSRA |= (1 << ADIE);  // Разрешаем прерывания по завершении преобразования
  ADCSRA |= (1 << ADEN);  // Включаем АЦП
  //  ADCSRA |= (1 << ADSC);  // Запускаем преобразование
}//================================Инициализация АЦП===================
//
void Timers_init() { //===============Инициализация таймеров===================
  //---Инициализация таймера 0 для тактирования АЦП -------------
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (1 << WGM01); // Счетчик работает в режиме CTC (сброс по совпадению)
  TCCR0B |= (1 << CS01) | (1 << CS00); // Предделитель на 64 (на счетчик - 250 кГц)
  OCR0A = T_ADC; // Определяет период запуска АЦП
  TIMSK0 |= (1 << OCIE0A); // Разрешаем прерывания по совпадению с OCR0A
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
  pinMode(pin_OLED_res, OUTPUT);      // Определяем вывод сброса дисплея, как выход
  pinMode(pin_TOut, OUTPUT);          // Определяем вывод твердотельного реле, как выход
  pinMode(pin_TRelay, OUTPUT);        // Определяем вывод контактного реле, как выход
  digitalWrite(pin_OLED_res, LOW);    // Сбрасываем дисплей (!!! НЕ ЗАБЫТЬ ПЕРЕКЛЮЧИТЬ НА ВЫСОКИЙ !!!)
#ifdef High_level_triac // управление высоким уровнем
  digitalWrite(pin_TOut, LOW);        // Выключаем ТЭН (твердотельное реле)
#else // управление низким уровнем
  digitalWrite(pin_TOut, HIGH);       // Выключаем ТЭН (твердотельное реле)
#endif
#ifdef High_level_relay // управление высоким уровнем
  digitalWrite(pin_TRelay, LOW);        // Выключаем ТЭН (контактное реле)
#else // управление низким уровнем
  digitalWrite(pin_TRelay, HIGH);       // Выключаем ТЭН (контактное реле)
#endif
  //
  pinMode(pin_buttGND, OUTPUT);      // Определяем временный общий кнопок, как выход
  digitalWrite(pin_buttGND, LOW);    // и устанавливаем на нем низкий уровень
  pinMode(pin_butt_1, INPUT_PULLUP); // Определяем вывод кнопки 1, как вход и подтягиваем его внутренним резюком к VCC
  pinMode(pin_butt_2, INPUT_PULLUP); // Определяем вывод кнопки 2, как вход и подтягиваем его внутренним резюком к VCC
  pinMode(pin_butt_3, INPUT_PULLUP); // Определяем вывод кнопки 3, как вход и подтягиваем его внутренним резюком к VCC
  pinMode(pin_butt_4, INPUT_PULLUP); // Определяем вывод кнопки 4, как вход и подтягиваем его внутренним резюком к VCC
  //
#ifdef Debug
  pinMode(pin_ZeroOut, OUTPUT);       // Определяем вывод импульса ноля, как выход
  digitalWrite(pin_ZeroOut, LOW);     //
  pinMode(pin_DebugOut, OUTPUT);      // Определяем отладочный вывод, как выход
  digitalWrite(pin_DebugOut, LOW);    //
  pinMode(pin_TestOut, OUTPUT);       // Определяем отладочный вывод, как выход
  digitalWrite(pin_TestOut, LOW);     //
#endif
}//========================Инициализация входов/выходов контроллера=========
//
void Razgon_() { //===========Подпрограмма обработки режима разгона================
  static uint8_t   PPcount_R = 0;   // счетчик полупериодов
    if (fl.razg_on &&               // Если включен разгон..
        !fl.TRelay &&               // ..и НЕ включено контактное реле
        (++PPcount_R == 50)) {      // ..и все это длится уже более 500мс,
      fl.TRelay = 1; PPcount_R = 0; // то включим контактное реле и обнулим счетчик
    }
    if (fl.razg &&                  // Если включен максимум для твердотельного реле..
        !fl.razg_on &&              // ..и выключен разгон
        (++PPcount_R == 50)) {      // ..и все это длится уже более 500мс,
      fl.razg = 0; PPcount_R = 0;   // то выключим реле и обнулим счетчик
    }
}//===========Подпрограмма обработки режима разгона================
//
void PDM_() { //===========Подпрограмма управления твердотельным реле ТЭНа================
  if (fl.razg) {
    pdm = CICLE; // В режиме разгона твердотельное всегда открыто
  }
  lev = pdm + err;                // Текущий уровень с учетом ошибки дискретизации, сделанной на предыдущем полупериоде.
  ps += (-1) * PVolna * fl.Tout;  //Текущее значение постоянной составляющей
  if ((lev >= CICLE/2) && ((PVolna * ps) <= 0)) { // Ставим флаг включения ТЭНа
    fl.Tout = 1; err = lev - CICLE;         // и считаем ошибку для следующего полупериода
  }        
  else {
    fl.Tout = 0; err = lev;                     // Снимаем флаг включения ТЭНа и считаем ошибку
  }
}//========================Подпрограмма управления твердотельным реле ТЭНа================
//
void Buttons_() { //==============Опрос кнопок=====================
  static struct buttons {
    unsigned butt_1 : 1;    // текущее состояние кнопки (0 - не нажата)
    unsigned butt_2 : 1;    // текущее состояние кнопки
    unsigned butt_3 : 1;    // текущее состояние кнопки
    unsigned butt_4 : 1;    // текущее состояние кнопки
    unsigned butt : 4;      // код текущей нажатой кнопки
    unsigned last_butt : 4; // код предыдущей нажатой кнопки
  } bt ={0};
  static uint8_t butt_count = 0;  // счетчик для устранения дребезга
  //
  bt.butt_1 = !digitalRead(pin_butt_1);
  bt.butt_2 = !digitalRead(pin_butt_2);
  bt.butt_3 = !digitalRead(pin_butt_3);
  bt.butt_4 = !digitalRead(pin_butt_4);
  if ( (bt.butt_1 + bt.butt_2 + bt.butt_3 + bt.butt_4) == fl.butt ) { // Или нажата одна кнопка или ни одной
    bt.butt = bt.butt_1 + (bt.butt_2 << 1) + (bt.butt_3 << 2) + (bt.butt_4 << 3);
    if ( bt.butt == bt.last_butt ) {
      butt_count++;
    }
    else {
      butt_count = 1;
      bt.last_butt = bt.butt;
    }
  }
  else if (--butt_count < 1) {
    butt_count = 1;
  }
  //
  if ( butt_count == 31 ) { // Есть нажатая кнопка или достаточная пауза после нажатия
    if (fl.dspMenu) { //===========Если мы в меню, то...
      switch (bt.butt) {
        case 1:
          if (++PDMcount > PDMset_size) PDMcount=0; //Перебираем значения уставок мощности ТЭНа
//          fl.butt = 0;            //После нажатия должна быть пауза          
          break;                    //Закончили
        case 2:
        case 4:
        case 8:
          fl.dspMenu = 0;           //Снимаем флаг перехода в меню
          fl.dspRefresh = 1;        //Ставим флаг обновления экрана
          PDMust = PDMset[PDMcount];//Устанавливаем выбранную мощность ТЭНа
          fl.butt = 0;              //После нажатия должна быть пауза
          break;                    //Закончили
        default:
          fl.butt = 1;              // достаточная пауза между нажатиями
      }
    }
    else {  //=====================А если не в меню, то...
      switch (bt.butt) {
        case 1:
          if (PDMust > 0) PDMust--;    //Уменьшаем установленную мощность
          break;
        case 2:
          if (PDMust < CICLE) PDMust++; //Увеличиваем установленную мощность
          break;
        case 4:
          if (PDMust == 0) {    //Если мы не в меню и мощность ТЭНа нулевая, то...
            fl.dspMenu = 1;     //Ставим флаг перехода в меню
            fl.dspRefresh = 1;  //Ставим флаг обновления экрана
          }
          else {                //Иначе...
            if (PDMust != 0) PDMset[PDMset_size] = PDMust; PDMcount = PDMset_size;  //Запоминаем текущую мощность ТЭНа
            PDMust = 0;         //Экстренно выключаем ТЭН
            fl.razg_on = 0;     //Выключаем разгон
            fl.TRelay = 0;      //Выключаем контактное реле
          }
          fl.butt = 0;          //После нажатия должна быть пауза
          break;                
        case 8:
          fl.razg_on = ((!fl.NotZero) & (!fl.Udown) & (!fl.razg_on)); //Триггер режима разгона (гистерезис организован в обработке начала полупериода)
          fl.razg |= fl.razg_on;                      //Если разгон включили, то твердотельное реле на максимум сразу
          fl.TRelay &= fl.razg_on;                    //Если разгон выключили, то контактное реле выключаем сразу
          fl.butt = 0;                                //После нажатия должна быть пауза
          break;
        default:
          fl.butt = 1;  // достаточная пауза между нажатиями
      }
    }
    butt_count = 1;
    bt.butt = 0;
    Pust = Pnom << 1; Pust *= PDMust; Pust /= CICLE;  Pust++; Pust = Pust >> 1; // Считаем Pust с округлением
    fl.DisplayOut = 1; //Обновление информации на дисплее
  }
} //================================Опрос кнопок=====================
//
ISR(TIMER2_COMPA_vect) { //======Обработчик начала очередного полупериода по таймеру2=========
  //
  #ifdef Debug
    PORTD ^=  (1 << pin_TestOut); //ОТЛАДКА
  #endif
  //
  Razgon_();  
  PDM_();
//
#ifdef High_level_triac // управление твердотельным реле высоким уровнем
  if (fl.Tout) {
    PORTD |=  (1 << pin_TOut); // Включаем ТЭН
  }
  else {
    PORTD &=  ~(1 << pin_TOut); // Выключаем ТЭН
  }
#else // управление твердотельным реле низким уровнем
  if (fl.Tout) {
    PORTD &=  ~(1 << pin_TOut); // Включаем ТЭН
  }
  else {
    PORTD |=  (1 << pin_TOut); // Выключаем ТЭН
  }
#endif
#ifdef High_level_relay // управление контактным реле высоким уровнем
  if (fl.TRelay) {
    PORTD |=  (1 << pin_TRelay); // Включаем ТЭН
  }
  else {
    PORTD &=  ~(1 << pin_TRelay); // Выключаем ТЭН
  }
#else // управление контактным реле низким уровнем
  if (fl.TRelay) {
    PORTD &=  ~(1 << pin_TRelay); // Включаем ТЭН
  }
  else {
    PORTD |=  (1 << pin_TRelay); // Выключаем ТЭН
  }
#endif
  //
  fl.PP = 1;
  PVolna *= (-1);
  OCR2A = PID_ust;                          // Грузим новое значение в регистр сравнения
  //
  sei(); // разрешим прерывания
  Buttons_();   // Опрашиваем кнопки
}//==============================Обработчик начала очередного полупериода по таймеру2=========
//------------------------------------------------------------------------------
ISR(TIMER0_COMPA_vect) { //======Обработчик запуска преобразования АЦП по таймеру0=========
  ADCSRA |=  (1 << ADSC); // Запуск преобразования
}//==============================Обработчик запуска преобразования АЦП по таймеру0=========
//------------------------------------------------------------------------------
ISR(ADC_vect) { //===============Обработчик окончания преобразования АЦП===================
#ifdef Debug
  PORTD |=  (1 << pin_DebugOut);  //ОТЛАДКА
#endif
  static uint8_t   Pcount_sc = 0; // счетчик периодов, за которые просуммированы квадраты отсчетов
  register int32_t PID_err;
  register uint16_t val;
  val = ADCL; val |= ((uint16_t)ADCH) << 8; // забрали результат преобразования АЦП
  register uint8_t TM2_current;
  TM2_current = TCNT2;                      // забрали значение из таймера синхронизации с сетью
  register int16_t U;
  U = 0;
  if (val >= U_ZERO) {
    val -= U_ZERO;  // Убираем постоянную составляющую из оцифрованного сигнала
    U += val;
  }
  else {
    val = U_ZERO - val;
    U -= val;
  }
  //===детекция перехода через ноль и ПИД-синхронизация=================================
  Udelta += (U - Uold);       //
  U = Udelta >> 5;          //КИХ ФНЧ 1-го порядка с коэффициентом 1/32
  //  U = (1*Udelta)/32;          //КИХ ФНЧ 1-го порядка с коэффициентом 1/32
  if ((!fl.zero) && (U >= 0) && (Uold <= 0) && (U != Uold)) { //=======переход через ноль детектед=======
#ifdef Debug
    PORTD |=  (1 << pin_ZeroOut);
#endif
   sc_notzero = 0;    // Обнуляем счетчик выборок АЦП без перехода через ноль
   fl.NotZero = 0;    // Снимаем флажок отсутствия детекции перехода через ноль
  //===Проверка насуммированных отсчетов============================
    if (++Pcount_sc == PSUM_MAX) {          // Насуммировали достаточно
    U_sum = sum; fl.sum = 1; sc_sum = sc;   // Насуммированное готово к обработке
    sc = 0; sum = 0; Pcount_sc = 0;         // Сбрасываем счетчик, сумматор и счетчик полупериодов
  }
  //===Проверка насуммированные отсчетов============================
    fl.zero = 1;
    PID_err = (TM2_current + Phase) << Km;         // Разность фаз
    if (PVolna == (-1)) {
      PID_err = PID_err - PID_reg - (1 << Km); // Разность фаз должна быть с соответствующим знаком
    }
    PID_int += (PID_err >> Ki);                                     // Считаем интегральную составляющую
    PID_reg += PID_err >> Kp;                                       // Считаем новую функцию управления
    PID_reg += PID_int;
    PID_reg += ( PID_err - PID_err_old ) >> Kd;
    PID_err_old = PID_err;
    // Готовим данные для записи в регистр сравнения таймера 2
      if ( PID_reg > (255 << Km)) {
        PID_reg = (255 << Km);  // Ограничим сверху
      }
      else {
        if ( PID_reg < (127 << Km)) {
          PID_reg = (127 << Km);  // Ограничим снизу
        }
      }
      PID_err = PID_reg >> (Km - 1);  // ...и правильно округлим
      PID_err++;                      // используя уже не нужную в этой подпрограмме
      PID_ust = PID_err >> 1;         // переменную PID_err
    }
 //
  else { //=======переход через ноль  NOT детектед=======
    fl.zero = 0;
    if (++sc_notzero == ZSUM_MAX) {   // Насуммировали достаточно
      fl.NotZero = 1; sc_notzero = 0;
      PID_ust = LINE_FREQ;
      fl.razg_on = 0;       //выключим режим разгона
      pdm = 0; fl.Tout = 0; //выключим твердотельное реле
      fl.TRelay = 0;        //выключим контактное реле
      U_real = 0; sc = 0; sum = 0; Pcount_sc = 0; // Обнулим счетчик, сумматор, счетчик полупериодов и значение напряжения
      fl.DisplayOut = 1;
    }
   //
#ifdef Debug
    PORTD &=  ~(1 << pin_ZeroOut);
#endif
  }
  //
  Uold = U;                   //
  //===детекция перехода через ноль и ПИД-синхронизация=================================
  //
    sei(); // Следующие фрагменты длительны, но не требуют атомарности; разрешим прерывания
  //===Суммирование квадратов=======================================
  PID_err = val * val;    // Возводим в квадрат выборку АЦП (используем всё ту же переменную PID_err)
  sum += PID_err;       // Суммирование квадратов выборок АЦП
  ++sc;                           // Счетчик выборок АЦП
  //===Суммирование квадратов=======================================
  //
#ifdef Debug
  PORTD &=  ~(1 << pin_DebugOut); //ОТЛАДКА
#endif
}//===============================Обработчик окончания преобразования АЦП===================
//
void setup() {
#ifdef Serial_out
  Serial.begin(57600);
#endif
  Pins_init();    // Инициализируем входы/выходы
  ADC_init();     // Инициализируем АЦП
  Timers_init();  // Инициализируем таймеры
  sei();          // Разрешаем глобальные прерывания
  //
uint16_t  PPcount = 0;     // счетчик полупериодов
  while (PPcount < 50) {
    PPcount += fl.PP;  // Ждем для гарантированного сброса
    fl.PP = 0;
  }
  digitalWrite(pin_OLED_res, HIGH);   // Разрешаем работу дисплея
  while (PPcount < 60) {
    PPcount += fl.PP;  // Ждем для гарантированного разрешения
    fl.PP = 0;
  }
  OzOled.init();                      // Инициализируем OLED дисплей
  //
  OzOled.printString("Regulator_TENa", 1, 0);
  OzOled.printString(VERSION, 6, 1);
  OzOled.printString("by  JohnJohnov", 1, 4);
  OzOled.printString("alcodistillers", 1, 6);
  PPcount = 0;
  while (PPcount < 200) {
    PPcount += fl.PP;  // Пережидаем переходные процессы и любуемся заставкой
    fl.PP = 0;
  }
  OzOled.clearDisplay();
  fl.dspRefresh = 1;
#ifdef Serial_out
  Serial.print("Regulator_TENa "); Serial.println(VERSION);
#endif
}
//
void loop() {
  //
  if (fl.sum) { //==========Обработка данных от АЦП и корректировка выдаваемой мощности============
    U_sum /= sc_sum;   //Ненормированный квадрат среднеквадратичного
    U_sum *= Ku;        //Нормированный квадрат среднеквадратичного
    register uint32_t tmp; // Величины великоваты, чтобы попасть в размерность приходится считать аккуратно
    //    pdm = U_LINE_Q*PDMust/(U_sum);
    tmp = U_LINE_Q << 1;
    tmp *= PDMust;
    tmp /= U_sum;
    tmp++;
    tmp = tmp >> 1;
    //
    if (tmp > CICLE || fl.razg) { // Следим, чтобы pdm не превышала CICLE
      pdm = CICLE;
      fl.Ulow = !fl.razg; // Или напряжение сети не позволяет выдать установленный уровень мощности или разгон
    }
    else {
      fl.Ulow = 0;
      pdm = tmp;
    }
    //
    U_real = sqrt(U_sum); //Среднеквадратичное
    if ( U_real < 100 ) { //Действующее напряжение сети ниже 100В - отключим ТЭН (авария)
      fl.Udown = 1;       //поставим флажок низкого сетевого
      fl.razg_on = 0;     //выключим режим разгона
      pdm = 0;            //выключим твердотельное реле
      fl.TRelay = 0;      //выключим контактное реле
    }
    else {
      fl.Udown = 0;
    }
    fl.sum = 0;
    fl.DisplayOut = 1; //Обновление информации на дисплее
  } //======================Обработка данных от АЦП и корректировка выдаваемой мощности============
  //
  //
  if (fl.DisplayOut) { //========================Вывод информации на дисплей=============  
    if (fl.dspMenu) { //===========Если мы в меню, то...
      if (fl.dspRefresh) {  //Обновляем дисплей
        OzOled.clearDisplay();
        OzOled.printString("Viberite ustavku", 0, 0);
        OzOled.printString("P=", 0, 1);
        OzOled.printString("----------------", 0, 3);
        OzOled.printString("Vibor:", 0, 4);
        OzOled.printString("knopka 'P-'", 5, 5);
        OzOled.printString("Primenitb:", 0, 6);
        OzOled.printString("pro4ie knopki", 3, 7);
      }
      static uint8_t PDMold = 0;
      if ((PDMold != PDMset[PDMcount]) || fl.dspRefresh) {
        PDMold = PDMset[PDMcount];
        uint32_t p = Pnom << 1; p *= PDMold; p /= CICLE;  p++; p = p >> 1; // Считаем уставку с округлением
        OzOled.printString("              ", 3, 1);
        OzOled.printNumber((long)p, 3, 1);
      }
      fl.dspRefresh = 0;
      //
    }
    else { //======================А если не в меню, то...
      if (fl.dspRefresh) {  //Обновляем дисплей
        OzOled.clearDisplay();
        OzOled.printString("U     V; pdm    ", 0, 0);
        OzOled.printString("P      ; Pn     ", 0, 3);
        //
        OzOled.printNumber((long)U_real, 2, 0);
        OzOled.printNumber((long)pdm, 13, 0);
        OzOled.printNumber((long)Pnom, 12, 3);
        OzOled.printNumber((long)Pust, 2, 3);
        //
        fl.dspRefresh = 0;
      }
      //
      static uint16_t U_real_old = 0;
      if (U_real_old != U_real) {
        U_real_old = U_real;
        OzOled.printString("    ", 2, 0);
        OzOled.printNumber((long)U_real_old, 2, 0);
      }
      static uint8_t pdm_old = 0;
      if (pdm_old != pdm) {
        pdm_old = pdm;
        OzOled.printString("    ", 13, 0);
        OzOled.printNumber((long)pdm_old, 13, 0);
      }
      static uint16_t Pnom_old = 0;
      if (Pnom_old != Pnom) {
        Pnom_old = Pnom;
        OzOled.printString("    ", 12, 3);
        OzOled.printNumber((long)Pnom_old, 12, 3);
      }
      static uint16_t Pust_old = 0;
      if (Pust_old != Pust) {
        Pust_old = Pust;
        OzOled.printString("    ", 2, 3);
        OzOled.printNumber((long)Pust_old, 2, 3);
      }
      //
      if (fl.Udown || fl.NotZero) {
        OzOled.printString("  Avariya sety  ", 0, 4);
      }
      else {
        if (fl.Ulow) {
          OzOled.printString("Nedost napr.sety", 0, 4);
        }
        else {
          OzOled.printString("----------------", 0, 4);
        }
      }
    //
      if (fl.razg_on) {
        static uint8_t count_1 = 0;
        byte x1 = 3 - (count_1 & 3);
        byte x2 = 15 - x1;
        OzOled.printString("----<Razgon>----", 0, 1);
        OzOled.printChar('<', x1, 1);
        OzOled.printChar('>', x2, 1);
        count_1++;
      }
      else {
        OzOled.printString("----------------", 0, 1);
      }
      //
      if (fl.TRelay) {
        OzOled.printString("Relay ON ", 0, 7);
      }
      else {
        OzOled.printString("Relay OFF", 0, 7);
      }
    }
  //
#ifdef Serial_out
  Serial.print("Ureal="); Serial.print(U_real); Serial.print(" ;PDM="); Serial.println(pdm);
//  Serial.print("Razgon="); Serial.print(fl.razg_on); Serial.print(" ;fl.razg"); Serial.print(fl.razg); Serial.print(" ;fl.TRelay"); Serial.println(fl.TRelay);
//Serial.print("PID_ust="); Serial.print(PID_ust);Serial.print(" ;sc_sum="); Serial.print(sc_sum);Serial.print(" ;U_sum="); Serial.println(U_sum);
#endif
//
      fl.DisplayOut = 0;
 }//========================Вывод информации на дисплей=============  
}
