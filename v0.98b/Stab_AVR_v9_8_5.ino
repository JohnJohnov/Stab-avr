//Регулятор ТЭНа полуволнами с программным детектором нуля
//--https://github.com/JohnJohnov/Stab-avr
//--https://alcodistillers.ru/forum/viewtopic.php?id=1549
//--JohnJohnov-----------------------
//--использован код OldBean----------
//--v0.2-------------------
//--добавлен дисплей
//--v0.3-------------------
//--ПИД-подстройка частоты сети по переходу через ноль
//--опрос кнопок
//--режим разгона
//--v0.4-------------------
//--выборки набираются за целое количество периодов
//--v0.5-------------------
//--оптимизация
//--v0.6-------------------
//--организована корректная обработка отсутствия сети
//--v0.7-------------------
//--исправлена ошибка выставления мощности менее 200Вт
//--битовые переменные упакованы в структуры
//--убрано ненужное мерцание символов на дисплее
//--добавлена возможность вернуть установленную мощность после экстренного отключения (идея d.styler)
//--v0.8-------------------
//--менюшка при возвращении уст.мощности после экстр.откл.
//--v0.81------------------
//--сделано выравнивание значений по правому краю
//--перекомпонован дежурный экран
//--выводится установленная мощность в Вт и процентах
//--напряжение сети выводится с одним знаком после запятой
//--v0.9-------------------
//--оптимизация кода
//--переход на более другую библиотеку дисплея
//--русский шрифт
//--номинальная мощность устанавливается/записывается/выбирается в начальном меню
//--уставки, выбираемые в меню после экстр.откл., могут быть записаны в EEPROM
//--v0.95------------------
//--исправлены ошибки
//--значение задержки для защиты от дребезга вынесено в дефайны
//--v0.96------------------
//--исправлены ошибки, оптимизирован код
//--расширены границы диапазона сетевой частоты для поддержки канадского коллеги
//--добавлена поддержка универсального протокола общения с управляющей программой
//--v0.97------------------
//--добавлена поддержка протокола общения с Samovar (начало посылки кириллицей)
//--добавлено моргание светодиода в отладочных целях
//--добавлен таймаут менюшек
//--добавлена поддержка протокола общения с РМВ-К
//--логотип
//--оптимизация кода
//--v0.98------------------
//--добавлен альтернативный интерфейс с большими символами для опытных пользователей
//--изменена работа с EEPROM
//--повышена точность регулировки (до 0,2%)
//--оптимизация кода
//--v0.98.4----------------
//--оптимизация кода
//--уменьшение размера кода для поддержки ATmega168
//--добавлено отключение разгона внешним сигналом
//--добавлено аварийное отключение нагрузки внешним сигналом
//--работа с портами организована через регистры без использования ардуиновских функций
//----v0.98.5----------------
//--оптимизация кода
//--добавлен выбор полярности входных сигналов аварии и отключения разгона
//--устранена ошибка, приводящая к неверной работе при измеренном сетевом напряжении выше 256В
//--добавлен выбор доп.параметра по запросу контроллера (протокол AD)
//--
//-------------------------
#include <Wire.h>
#include <EEPROM.h>
#include <avr/eeprom.h>
#include <ASOLED.h>
//
#define VERSION "v0.98.5"   // Версия скетча
#define VERSION_LEN 7     // Длина версии скетча в символах для правильного вывода на дисплей
//
//================================================================
//===========Настраиваемые параметры==============================
//================================================================
//#define Debug           // Раскомментить для дебажения
//#define PS_Debug        // Раскомментить для дебажения
//#define LED_debug       // Раскомментить для вывода на светодиод сигнала управления твердотельным реле
//#define OFFLINE_debug	  // Раскомментить для отладки в отсутствие сетевого напряжения
//
#define LOGO              // Раскомментить для отображения логотипа
//
#define INTERFACE_ALT     // Раскомментить для включения альтернативного интерфейса с большими символами
//
#define High_level_triac  // Раскомментить, если твердотельное реле ТЭНа управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
#define High_level_relay  // Раскомментить, если контактное реле ТЭНа управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
//
//#define High_level_razgonoff	// Раскомментить, если отключение разгона управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
//#define High_level_staboff	// Раскомментить, если аварийное отключение стаба управляется ВЫСОКИМ уровнем, иначе НИЗКИМ
//
//#define NOT_LM358       	// Раскомментить, если в датчике напряжения стоит НЕ LM358, а rail-to-rail операционник
//
//#define DisplayReset      	// Раскомментить, если используется вывод сброса дисплея
//
#define DEBOUNCE 26       	// Значение для обеспечения защиты от дребезга кнопок DEBOUNCE=1 + <задержка в ms>/10
//
//#define U_LINE_FREQ_60	// Раскомментить, если частота сети 60Гц, иначе - 50Гц
//
#define U_LINE 230   		// Номинальное значение действующего напряжения в сети, для которого указана номинальная мощность ТЭНа
#define U_MIN 100   		// Значение напряжения в сети, ниже которого сеть считается аварийной
//
#define MENU_TIMEOUT 60  	// Таймаут выхода из меню в секундах (не более 255)
//
//=====Настройки коммуникации по последовательному порту============
#define USE_USART           // Раскомментить для инициализации общения стаба с внешним контроллером (по умолчанию используется универсальный протокол)
//#define USE_RMVK        	// Раскомментить для выбора протокола Samovar и/или РМВ-К
//
//==================================================================
//===========НЕнастраиваемые параметры==============================
//==================================================================
//
#ifndef USE_USART
  #undef USE_RMVK
#else
  #ifndef USE_RMVK
    #define USE_ADprotocol  // По умолчанию используется универсальный протокол
  #endif // USE_RMVK
#endif // USE_USART
//
//
#ifdef U_LINE_FREQ_60
  #define LINE_FREQ 129 // Определяет начальную частоту для фазовой автоподстройки частоты сети (60,1Гц)
  #define PSUM_MAX 60   // Количество периодов для набора отсчетов АЦП (60 - это за 1 сек, это порядка 5000 отсчетов)
  #define P_TIME_MAX 120  // Количество полупериодов сети в секунду для отсчета времени
#else // U_LINE_FREQ_60
  #define LINE_FREQ 155   // Определяет начальную частоту для фазовой автоподстройки частоты сети (50,08Гц)
  #define PSUM_MAX 50     // Количество периодов для набора отсчетов АЦП (50 - это за 1 сек, это порядка 5000 отсчетов)
  #define P_TIME_MAX 100  // Количество полупериодов сети в секунду для отсчета времени
#endif //U_LINE_FREQ_60
//
//=============вход АЦП================
#define pin_VACin 0       //  Пин входа измеряемого напряжения (A0)
//
//===========входные выводы============
#define pin_PC_InPullUp(pin) {DDRC &=~(1 << ((pin) - 14)); PORTC |=(1 << ((pin) - 14));} // Инициализация входа с подтяжкой к VCC
//
#define pin_RAZGON_OFF 17 // Пин входа отключения разгона (A3 - это D17/PC3, при изменении подредактировать следующие два макроса)
#define pin_RAZGON_OFF_INIT pin_PC_InPullUp(pin_RAZGON_OFF) // Определяем вывод отключения разгона, как вход и подтягиваем его внутренним резюком к VCC
//
#define pin_STAB_OFF 16   //  Пин входа отключения стабилизатора (A2 - это D16/PC2, при изменении подредактировать следующие два макроса)
#define pin_STAB_OFF_INIT pin_PC_InPullUp(pin_STAB_OFF) // Определяем вывод останова стаба, как вход и подтягиваем его внутренним резюком к VCC
//
//==============Hardware-управление режимами=============
#define pin_PC_STATE(pin) {PINC >> ((pin) - 14);}	// Запрос состояния вывода со сдвигом
//
#ifdef High_level_razgonoff
 #define pin_RAZGON_OFF_STATE (pin_PC_STATE(pin_RAZGON_OFF) & 1)	// Читаем состояние пина и переводим в булев формат (активный - высокий)
#else
 #define pin_RAZGON_OFF_STATE (~(pin_PC_STATE(pin_RAZGON_OFF)) & 1)	// Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
#endif // High_level_razgonoff
//
#ifdef High_level_staboff
 #define pin_STAB_OFF_STATE (pin_PC_STATE(pin_STAB_OFF) & 1)	// Читаем состояние пина и переводим в булев формат (активный - высокий)
#else
 #define pin_STAB_OFF_STATE (~(pin_PC_STATE(pin_STAB_OFF)) & 1)	// Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
#endif // High_level_staboff
//
//===========вЫходные выводы===========
#define pin_PD_Out(pin) {DDRD |=(1 << (pin));}   // Инициализация выхода
#define pin_PD_HIGH(pin) (PORTD |=(1 << (pin))) // Установка выхода
#define pin_PD_LOW(pin) (PORTD &=~(1 << (pin))) // Сброс выхода
#define pin_PD_INV(pin) {PORTD ^=(1 << (pin));}  // Инверсия выхода
#define pin_PD_In(pin) {DDRD &=~(1 << (pin)); PORTD &=~(1 << (pin));}	// Инициализация входа
//
#define pin_OLEDres 2     //  Пин сброса OLED индикатора.
#define pin_OLEDres_INIT pin_PD_Out(pin_OLEDres)
#define pin_OLEDres_HIGH pin_PD_HIGH(pin_OLEDres)
#define pin_OLEDres_LOW pin_PD_LOW(pin_OLEDres)
//
#define pin_TOut 4        //  Пин выхода управления ТЭНом (на твердотельное реле)
#define pin_TOut_INIT pin_PD_Out(pin_TOut)
#define pin_TOut_HIGH pin_PD_HIGH(pin_TOut)
#define pin_TOut_LOW pin_PD_LOW(pin_TOut)
//
#define pin_TRelay 6      //  Пин выхода управления ТЭНом (на контактное реле в режиме максимальной мощности)
#define pin_TRelay_INIT pin_PD_Out(pin_TRelay)
#define pin_TRelay_HIGH pin_PD_HIGH(pin_TRelay)
#define pin_TRelay_LOW pin_PD_LOW(pin_TRelay)
//
//===========тестовые выводы===========
#define pin_ZeroOut 5     //  Пин выхода импульса ноля (D5 - PD5)
#define pin_ZeroOut_INIT pin_PD_Out(pin_ZeroOut)
#define pin_ZeroOut_INV pin_PD_INV(pin_ZeroOut)
#define pin_ZeroOut_HIGH pin_PD_HIGH(pin_ZeroOut)
#define pin_ZeroOut_LOW pin_PD_LOW(pin_ZeroOut)
//
#define pin_DebugOut 7    //  Пин для отладки (D7 - PD7)
#define pin_DebugOut_INIT pin_PD_Out(pin_DebugOut)
#define pin_DebugOut_INV pin_PD_INV(pin_DebugOut)
#define pin_DebugOut_HIGH pin_PD_HIGH(pin_DebugOut)
#define pin_DebugOut_LOW pin_PD_LOW(pin_DebugOut)
//
#define pin_TestOut 3     //  Пин для отладки (D3 - PD3)
#define pin_TestOut_INIT pin_PD_Out(pin_TestOut)
#define pin_TestOut_INV pin_PD_INV(pin_TestOut)
#define pin_TestOut_HIGH pin_PD_HIGH(pin_TestOut)
#define pin_TestOut_LOW pin_PD_LOW(pin_TestOut)
//
#define pin_TestPS 3     //  Пин для отладки (D3 - PD3)
#define pin_TestPS_Out pin_PD_Out(pin_TestPS)
#define pin_TestPS_HIGH pin_PD_HIGH(pin_TestPS)
#define pin_TestPS_LOW pin_PD_LOW(pin_TestPS)
#define pin_TestPS_Z pin_PD_In(pin_TestPS)
//
//===========выводы подключения кнопок===========
#define pin_PB_STATE(pin) (~(PINB >> ((pin) - 8)) & 1) // Запрос состояния вывода со сдвигом и инверсией результата
#define pin_PB_InPullUp(pin) {DDRB &=~(1 << ((pin) - 8)); PORTB |=(1 << ((pin) - 8));} // Инициализация входа с подтяжкой к VCC
#define pin_PB_OutLOW(pin) {DDRB |=(1 << ((pin) - 8)); PORTB &=~(1 << ((pin) - 8));}   // Инициализация выхода и сброс его
//
#define pin_butt_1 10     // Пин кнопки "Р-". Уменьшение уставки мощности. (D10 - PB2)
#define pin_butt_1_INIT pin_PB_InPullUp(pin_butt_1) // Определяем вывод кнопки, как вход и подтягиваем его внутренним резюком к VCC
#define pin_butt_1_STATE pin_PB_STATE(pin_butt_1)   // Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
//
#define pin_butt_2 9      // Пин кнопки "Р+". Увеличение уставки мощности. (D9 - PB1)
#define pin_butt_2_INIT pin_PB_InPullUp(pin_butt_2) // Определяем вывод кнопки, как вход и подтягиваем его внутренним резюком к VCC
#define pin_butt_2_STATE pin_PB_STATE(pin_butt_2)   // Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
//
#define pin_butt_3 12     // Пин кнопки "Стоп". Экстренное отключение ТЭНа. (D12 - PB4)
#define pin_butt_3_INIT pin_PB_InPullUp(pin_butt_3) // Определяем вывод кнопки, как вход и подтягиваем его внутренним резюком к VCC
#define pin_butt_3_STATE pin_PB_STATE(pin_butt_3)   // Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
//
#define pin_butt_4 11     // Пин кнопки "Разгон". Включение/отключение разгона. (D11 - PB3)
#define pin_butt_4_INIT pin_PB_InPullUp(pin_butt_4) // Определяем вывод кнопки, как вход и подтягиваем его внутренним резюком к VCC
#define pin_butt_4_STATE pin_PB_STATE(pin_butt_4)   // Читаем состояние пина и переводим в булев формат с учетом инверсии (активный - низкий)
//
#define pin_buttGND 8     // Пин временного общего провода для подключения кнопок. (D8 - PB0)
#define pin_buttGND_INIT pin_PB_OutLOW(pin_buttGND) // Определяем вывод, как вЫход и устанавливаем низкий уровень
//
//===============бортовой светодиод==============
#define pin_LED 13  // Бортовой светодиод подключен к выводу D13 (PB5)
#define pin_LED_INIT pin_PB_OutLOW(pin_LED)       // Определяем вывод, как вЫход и устанавливаем низкий уровень
#define TURN_LED_ON  PORTB |=(1 << (pin_LED - 8)) // Включаем светодиод
#define TURN_LED_OFF PORTB &=~(1 << (pin_LED - 8))// Выключаем светодиод
//
//==============управление релюшками=============
#ifdef High_level_triac // управление твердотельным реле высоким уровнем
#define TURN_SSR_ON pin_TOut_HIGH   // Включаем ТЭН
#define TURN_SSR_OFF pin_TOut_LOW   // Выключаем ТЭН
#else // управление твердотельным реле низким уровнем
#define TURN_SSR_ON pin_TOut_LOW    // Включаем ТЭН
#define TURN_SSR_OFF pin_TOut_HIGH  // Выключаем ТЭН
#endif // High_level_triac
//
#ifdef High_level_relay // управление контактным реле высоким уровнем
#define TURN_RELAY_ON pin_TRelay_HIGH   // Включаем ТЭН
#define TURN_RELAY_OFF pin_TRelay_LOW   // Выключаем ТЭН
#else // управление контактным реле низким уровнем
#define TURN_RELAY_ON pin_TRelay_LOW    // Выключаем ТЭН
#define TURN_RELAY_OFF pin_TRelay_HIGH  // Включаем ТЭН
#endif // High_level_relay
//
//=============Управляющие макросы================
#define	TIM0_COMPA_FLAGRESET {TIFR0 |= (1 << OCF0A);} // Сброс флага прерывания по совпадению А таймера 0
//
//========коэффициенты для ПИД-регулировки=======
#define Kp 2      // Коэффициент пропорциональности для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Ki 5      // Интегральный коэффициент для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Kd 1      // Дифференциальный коэффициент для ПИД-подстройки ФЧ сети (степень двойки для регистрового сдвига)
#define Km 6      // Коэффициент для целочисленной математики (степень двойки для регистрового сдвига)
#define PHASE 8   // сдвиг фаз между детекцией ноля и прерыванием таймера (в тиках таймера)
#define T_MAX 180 // ограничение максимальной длительности полупериода в тиках таймера
#define T_MIN 100 // ограничение минимальной длительности полупериода в тиках таймера
//
//================прочие константы===============
#define ZSUM_MAX 5000 // Количество отсчетов АЦП без детекции ноля (5000 отсчетов это порядка 1 сек)
#define U_ZERO 512    // Значение нуля АЦП для двуполярного сигнала с постоянной составляющей на выходе ОУ
#define T_ADC 49    // Определяет интервал между запусками АЦП (200 мкс) f_OCn = f_clk / 2*N*(1 + X), где N - коэффициент деления предделителя, X- содержимое регистра OCRnA
#define CICLE 500   // Количество полупериодов в полном цикле регулирования (200 полупериодов - 2сек, 500 полупериодов - 5сек, больше ставить не надо)
//
#define EMPTY_CELL_VALUE 0xFFFF  // Содержимое пустой ячейки памяти EEPROM
#define SRVDATA_ARR_SIZE 14 // Размер блока памяти для вспомогательных данных в EEPROM (в байтах)
#define Pnom_ARR_SIZE 8   // Макс. размер массива записанных номинальных мощностей ТЭНа
#define PDMset_ARR_SIZE 6     // Размер массива уставок мощности ТЭНа sizeof(PDMset)/sizeof(PDMset[0]) (ставить не меньше 4)
#if Pnom_ARR_SIZE > PDMset_ARR_SIZE
  #define ARRAY_SIZE Pnom_ARR_SIZE  // Размер массива для работы с номиналами в начальном меню и уставками
#else
  #define ARRAY_SIZE PDMset_ARR_SIZE  // Размер массива для работы с номиналами в начальном меню и уставками
#endif // Pnom_ARR_SIZE > PDMset_ARR_SIZE
//
#define ERR_A_to_HEX 255	// Код ошибки функции "A_to_HEX"
#define ERR_HEX_to_A 'X'	// Код ошибки функции "HEX_to_A"
//
//
#define ASOled LD		// Заюзаем уже созданный в библиотеке дисплея объект LD

#ifdef __AVR_ATmega168__  // Выключение логотипа, UARTа и прочих плюшек, чтобы влезало в 168атмегу
  #undef LOGO
  #undef USE_USART
  #undef USE_RMVK
  #undef USE_ADprotocol
#else // __AVR_ATmega168__
  #ifdef LOGO
    #include "logo.c"
  #endif
//
  #ifdef USE_USART
    static uint8_t cnt_uartWDT;     // Счетчик секунд для организации отсчета ожидания окончания посылки по USART
  #endif
//
  #ifdef USE_ADprotocol
	static uint16_t last_Pow_setpoint = 0;	// Последняя уставка для выхода из "стопа" по просьбе внешнего контроллера
  #endif	// USE_ADprotocol
//
#endif // __AVR_ATmega168__
//
#ifdef Debug
	static uint8_t flag_OVF = 0; // Флаг переполнения, устанавливается в функции calc_proportion
#endif
//
enum DSP { // Режимы дисплея
	OP_MODE,
	Pust_MENU,
	Pnom_MENU
	};	// Режимы дисплея
//
static uint16_t Pnom;             // Номинальная мощность ТЭНа (хранится в EEPROM и устанавливается из менюшки)
static uint16_t PDMset[2][ARRAY_SIZE] = {};  // Массив уставок мощности ТЭНа с адресами
static uint16_t (&Pnom_arr)[ARRAY_SIZE] = PDMset[0];  // Массив мощностей ТЭНа как ссылка на нулевую строку массива уставок
//
// В EEPROM хранятся значения номинальных мощностей ТЭНа (каждая занимает 2 байта, количество определяется величиной Pnom_ARR_SIZE)
// и уставки мощности для каждой номинальной в формате pdm (каждая занимает 2 байта),
// уставки пишутся не в конкретные ячейки, а по кругу до заполнения выделенного участка EEPROM.
// Так сделано для экономии ресурса EEPROM
static volatile uint16_t old_addr = 0;    // Адрес в EEPROM, где записана самая старая уставка
static volatile uint16_t new_addr;        // Адрес в EEPROM, куда писать новую уставку
static uint16_t start_addr;               // Начальный адрес области записи уставок в EEPROM
static uint16_t end_addr;                 // Конечный адрес области записи уставок в EEPROM
static volatile uint16_t clear_old_addr;  // Дубль адреса в EEPROM, где записана самая старая уставка, предназначенная для стирания
//
static volatile uint32_t sum;         // Сумматор квадратов отсчетов АЦП
static volatile uint16_t sc = 0;      // Счетчик просуммированных квадратов
static volatile uint16_t sc_sum = 0;  // Счетчик просуммированных квадратов, готовый к обработке
static volatile uint16_t Pust = 0;    // Установленная мощность ТЭНа
static volatile uint16_t pdm = 0;     // Текущий уровень PDM (принимает значения от 0 до CICLE)
static volatile int32_t pdm_err = 0;  // Ошибка дискретизации
static volatile uint16_t PDMust = 0;  // PDM, соответствующий установленной мощности ТЭНа
//
static volatile uint32_t U_sum = 0;   // Среднеквадратичное в сети за секунду, умноженное на 10
static uint16_t U_real = U_LINE;      // Среднеквадратичное за секунду (целая часть)
static uint8_t U_real_dec = 0;        // Среднеквадратичное за секунду (дробная часть)
//
static uint8_t PID_ust = LINE_FREQ;// Данные для установки регистра сравнения таймера2
//
// Организуем флаги и индикаторы в структуру
static volatile struct flags {  // Флаги
  unsigned  dspRefresh : 1;   // Флаг выхода из режима меню / полного обновления экрана
  unsigned  dspTimeout : 1;   // Флаг истечения времени ожидания выхода из меню
  unsigned  dspNewData : 1;   // Флаг обновления данных на экране
  unsigned  uartUnhold : 1;   // Флаг разрешения передачи данных по USART
  unsigned  uartReport : 1;   // Флаг разрешения отправки данных внешнему контроллеру
  unsigned  uartTimeout : 1;  // Флаг истечения времени приема посылки по USART
  unsigned  butt : 1;         // Флаг опроса кнопок
  unsigned  writable : 1;     // Флаг записи уставок в EEPROM
  //
  unsigned  zero : 1;         // Флаг перехода через ноль
  unsigned  NotZero : 1;      // Флаг аварии сети (не детектируются переходы через ноль)
  unsigned  Ulow : 1;         // Флаг невозможности выдать установленный уровень мощности
  unsigned  Udown : 1;        // Флаг аварии сети (действующее напряжение ниже 100В)
  unsigned  Tout : 1;         // Флаг включения ТЭНа (твердотельное реле)
  unsigned  TRelay : 1;       // Флаг включения ТЭНа (контактное реле)
  unsigned  razg : 1;         // Флаг режима "разгон"
  unsigned  razg_on : 1;      // Флаг начала режима "разгон"
  //
  unsigned  sum : 1;          // Флаг готовности насуммированных данных к обработке
  unsigned  razg_off : 1;     // Флаг останова режима "разгон"
  unsigned  stab_off : 1;     // Флаг аварийного останова стабилизатора
  unsigned  PP : 1;           // Флаг полупериода сети на входе АЦП (отрицательная полуволна = 0, положительная = 1)
  unsigned  PP_fir : 1;       // Флаг полупериода после КИХ ФНЧ (отрицательная полуволна = 0, положительная = 1)
  unsigned  PP_tm : 1;        // Флаг полупериода по внутреннему таймеру (отрицательная полуволна = 0, положительная = 1)
} fl = {};  // Инициализируем структуру с нулевыми членами
//
//static volatile uint8_t fl_A;  // Байт флажков A
//static volatile uint8_t fl_B;  // Байт флажков B
//static volatile uint8_t fl_C;  // Байт флажков C
//#define flA_dspRefresh  B00000001
//#define flA_dspTimeout  B00000010
//#define flA_dspNewData  B00000100
//#define flA_uartUnhold  B00001000
//#define flA_uartReport  B00010000
//#define flA_uartTimeout B00100000
//#define flA_writable    B01000000
//#define flA_butt        B10000000
//
static uint8_t cnt_Pnom_count;  // Количество предустановок мощности
static uint8_t cnt_Pnom_number; // Номер активной предустановки мощности
static uint8_t cnt_PDMcount;    // Счетчик для перебора уставок мощности ТЭНа
//
static uint8_t cnt_menuWDT;     // Счетчик секунд для организации отсчета ожидания выхода из меню
static uint8_t cnt_dspMenu;     // Индикатор режима меню
//
byte X_position (const byte x, const uint16_t arg = 0, const byte pix = 6); // Функция возвращает начальную позицию по Х для десятичного числа, в зависимости от количества знаков в нём.
byte X_centred (const byte len);    // Функция возвращает начальную позицию по Х для текста длинной len знаков, для размещения оного по центру дисплея.
byte A_to_HEX (const char a);       // Функция переводит символ ASCII в шестнадцатиричную цифру
char HEX_to_A (const byte x);       // Функция переводит шестнадцатиричную цифру в символ ASCII
uint16_t calc_proportion(const uint16_t multiplier1, const uint16_t multiplier2 = Pnom, const uint32_t divider = CICLE);
//
//
//==============================================================================
//============================ПРОЦЕДУРЫ И ФУНКЦИИ===============================
//==============================================================================
//
//=======Функции для работы с флагами================
//== fl_A(BC) - байт с флажками
//== flag - нулевой байт с единицей на месте нужного флажка
//void inline set_flag_A(const uint8_t flag) {
//  fl_A |= flag;
//}
////
//void inline clr_flag_A(const uint8_t flag) {
//  fl_A &= ~flag;
//}
////
//boolean check_flag_A(const uint8_t flag) {
//  return (fl_A && flag)? true : false;
//}
//#define SET_FLAG(fl_byte, fl_mask) fl_byte |= fl_mask //
//#define CLR_FLAG(fl_byte, fl_mask) fl_byte &= ~fl_mask //
//#define CHECK_FLAG(fl_byte, fl_mask) ((fl_byte && fl_mask)? true : false)//

// Функция возвращает начальную позицию по Х для десятичного числа, в зависимости от количества знаков в нём.
byte X_position (const byte x, const uint16_t arg, const byte pix) { // arg-выводимое число; х-позиция для arg, если бы оно было однозначно; pix - ширина шрифта в пикселях
//  byte pix = 6; // Ширина шрифта в пикселях
  byte position = 0;
  if (arg < 10) position = pix * x;
  else if (arg < 100) position = pix * (x-1);
  else if (arg < 1000) position = pix * (x-2);
  else position = pix * (x-3);
  return position;
}
//
// Функция возвращает начальную позицию по Х для текста длинной len знаков, для размещения оного по центру дисплея.
byte X_centred (const byte len) { // len - Количество знакомест в тексте
  byte wdt = 128; // Ширина дисплея в пикселях
  byte pix = 6;   // Ширина шрифта в пикселях
  byte position = 0;
  if (len <= wdt/pix) position = (wdt - (len * pix))/2;
  return position;
}
//
// Функция переводит символ ASCII в шестнадцатиричную цифру, при ошибке возвращает 255
byte A_to_HEX (const char a) { // a - символ 0...F
  byte number = ERR_A_to_HEX;
  if (a >= 48 && a <= 57) number = (byte)(a-48); // Если а - от 0 до 9
  else if (a >= 65 && a <= 70) number = (byte)(a-55); // Если а - от A до F
  else if (a >= 97 && a <= 102) number = (byte)(a-87); // Если а - от a до f
  return number;
//  byte b = (byte)(a-48);
//  if (b < 9) number = b; // Если а - от 0 до 9
//  else if (b -= 7; b < 15) number = b; // Если а - от A до F
//  else if (b -= 32; b < 15) number = b; // Если а - от a до f
//  return number;
}
//
// Функция переводит шестнадцатиричную цифру в символ ASCII, при ошибке возвращает X
char HEX_to_A (const byte x) {  // x - число, кое необходимо перевести в ASCII-код
  char symbol = ERR_HEX_to_A;
  if (x <= 9) symbol = (char)(x + 48);
  else if (x <= 15) symbol = (char)(x + 55);
  return symbol;
}
//
void start_razgon(void) {  //===========Процедура включения режима "Разгон"================
	if (!fl.razg_on) {	// Если разгон вЫключен...
	  fl.razg_on = 1;	// включим режим разгона
	  fl.razg  = 1;		// включим твердотельное реле
	}
}
//
void stop_razgon(void) {  //===========Процедура останова режима "Разгон"================
	if (fl.razg_on) {	// Если разгон включен...
	  fl.razg_on = 0;	// выключим режим разгона
	  fl.TRelay  = 0;	// выключим контактное реле
	}
}
//===========Подпрограммка подсчета Pust================
void inline set_Pust(void) {
  Pust = calc_proportion(PDMust);
} //===========Подпрограммка подсчета Pust(конец)================
//
//=========Функция пропорционального пересчета параметра================
//== Возвращает значение параметра с округлением,
//== пересчитанное из пропорции по формуле
//== (multiplier1 * multiplier2 / divider)
//==
//== Если результат не умещается в выходную размерность,
//== возвращается максимально возможное (для размерности) значение
//==
//== multiplier1 - первый множитель
//== multiplier2 - второй множитель (по умолчанию Pnom)
//== divider - делитель (по умолчанию CICLE)
//
uint16_t calc_proportion(const uint16_t multiplier1, const uint16_t multiplier2, const uint32_t divider) {
  uint32_t p;
  p = (uint32_t)multiplier1 * 2;
  p *= (uint32_t)multiplier2;
  p /= divider;
  p++;
  p /= 2;
  if (p > 0xFFFF) {
	  p = 0xFFFF;
#ifdef Debug
	flag_OVF = 1;
#endif
  }
  return (uint16_t)p;
} //=========Функция пропорционального пересчета параметра(конец)================
//
void pp_Delay(const uint16_t pp) {  //===Пауза, измеряется в полупериодах=====
  uint16_t  PPcount = 0;  // счетчик полупериодов
  boolean PP_tm_last = 0;
  while (PPcount < pp) {
    if (PP_tm_last != fl.PP_tm) {
      PPcount++;
      PP_tm_last = fl.PP_tm;
    }
  }
}
//
#ifdef USE_USART//++++++++++++++++USART initialization++++++++++++++++++++++++++++
//Если задействовано управление регулятором ТЭНа через UART, инициализируем оный
//
void USART_start(void) {
  Serial.begin(9600, SERIAL_8N1); // Инициализируем USART
  #ifdef Debug
  Serial.println("Started");
  #endif
}
//
uint16_t get_Power(void) {  // Функция возвращает значение текущей мощности с учетом режима "Разгон"
  uint32_t tmp_u = 0;
  if (fl.stab_off || fl.Udown || fl.NotZero) {  // Если авария или сеть в дауне - передаем ноль
	tmp_u = 0;
  }
  else if (fl.razg_on || fl.Ulow) { // В разгоне и при недостаточном сетевом передаем расчетную текущую мощность
    tmp_u = U_sum * U_sum;
    tmp_u /= 100;
    tmp_u *= Pnom;
    tmp_u /= U_LINE;
    tmp_u /= U_LINE;	// делим дважды, поскольку делить надо на квадрат номинала сети.
  }
  else {                            // В рабочем режиме - передаем уставку
    tmp_u = Pust;
  }
  if (tmp_u > 0xFFFF) tmp_u = 0xFFFF; // Останемся в размерности
  return (uint16_t)tmp_u;
}
//
void set_newPDM(uint16_t power) { // Функция установки текущей мощности по запросу
//  if (power >= Pnom) PDMust = CICLE; // Останемся в пределах диапазона
//  else PDMust = calc_proportion(power, CICLE, Pnom);
  if (power < Pnom) PDMust = calc_proportion(power, CICLE, Pnom);
  else PDMust = CICLE; // Останемся в пределах диапазона
  //
  set_Pust();    // Пересчитаем Pust
  fl.dspNewData = 1;  // Обновление информации на дисплее
}
//
#endif // USE_USART
//
#ifdef USE_ADprotocol //++++++++++++++++USART++++++++++++++++++++++++++++
//Байт "состав данных" b00010111 (основной параметр - мощность в нагрузке, доп. параметр - напряжение сети) в HEX-формате 0x17
static char USART_InfoData[14] = {'T','0','0','0','0','0','0','0','0','0','0','0','0',0x0D};  // Массив готовых данных для передачи внешнему контроллеру
static char USART_SetData[6] = {};  // Массив управляющих символов от внешнего контроллера
//
// Код основного параметра в протоколе АД
#define BASEPARM_U 0b01	// напряжение на нагрузке
#define BASEPARM_I 0b10	// ток в нагрузке
#define BASEPARM_P 0b11	// мощность в нагрузке
//
// Код дополнительного параметра в протоколе АД
#define AUXPARM_U 0b000001	// напряжение на нагрузке (если основной - напряжение, то напряжение уставки)
#define AUXPARM_I 0b000010	// ток в нагрузке (если основной - ток, то ток уставки)
#define AUXPARM_P 0b000011	// мощность в нагрузке (если основной - мощность, то мощность уставки)
#define AUXPARM_Us 0b000101	// напряжение сети
#define AUXPARM_Rn 0b000110	// сопротивление нагрузки
#define AUXPARM_Pn 0b000111	// номинальная мощность нагрузки
//
static uint8_t aux_param_Number = 0;	// Номер пересылаемого дополнительного параметра + старший бит работает флагом необходимости обновления значения параметра
//
void USART_parser(void) { // Парсим управляющую последовательность по универсальному протоколу
  //
#define MODE_INSTRUCTION_LENGTH 2 // Длина запроса на изменение режима работы
#define COMM_INSTRUCTION_LENGTH 3 // Длина запроса на исполнение команды
#define RATE_INSTRUCTION_LENGTH 5 // Длина запроса на изменение уставки
enum MODE {	// Коды режимов работы стаба
	NORM = '0',
	RAZGON = '1',
	STOP = '2'
		}; // Коды режимов работы стаба
//
  static uint8_t index = 0;
  static uint8_t data_length;
//
  while (Serial.available() > 0) {
	char curr_usart_byte = Serial.read(); // Вычитываем очередной байт;
    if (fl.stab_off) {
//      Serial.read(); // Вычитываем очередной байт, чтобы не засирать буфер
	  continue;
    }
    else if ( !index || fl.uartTimeout ) {   // Начало
//      USART_SetData[0] = Serial.read(); // Вычитываем очередной байт
      fl.uartTimeout = 0;               // Сбросим флаг таймаута ожидания окончания посылки
      cnt_uartWDT = 0;                  // Сбросим таймер ожидания окончания посылки
      curr_usart_byte &= 0b11011111;	// Переведем символ в ВЕРХНИЙ регистр
      switch ( curr_usart_byte ) {		// Ждём первый символ...
        case 'M': {                     // ...запроса на изменение режима работы
          data_length = MODE_INSTRUCTION_LENGTH;
          index=1;
          break;
        }
        case 'N': {                     // ...запроса на исполнение команды
          data_length = COMM_INSTRUCTION_LENGTH;
          index=1;
          break;
        }
        case 'P': {                     // ...запроса на изменение уставки
          data_length = RATE_INSTRUCTION_LENGTH;
          index=1;
          break;
        }
        default: {
          break;
        }
      }
    }
    else {
//      USART_SetData[index] = Serial.read(); // Вычитываем очередной байт
      USART_SetData[index] = curr_usart_byte;
      if ( curr_usart_byte == 0x0D ) { // Ждем последнего символа посылки <CR>
        if ( index == data_length ) {
          switch (data_length) {
            case MODE_INSTRUCTION_LENGTH: { // Парсим запрос на смену режима
              switch ( USART_SetData[1] ) {
                case MODE::NORM: { // Переход в рабочий режим
                  stop_razgon();
                  if ( PDMust == 0 ) {
                	  PDMust = last_Pow_setpoint;
                	  set_Pust();
                  }
                  break;
                }
                case MODE::RAZGON: { // Переход в режим разгона
                  if ((!fl.NotZero) & (!fl.Udown) & (!fl.razg_off)) {  // Если электросеть в дауне или разгон запрещен - не разгонишься
                	  start_razgon();
                  }
                  break;
                }
                case MODE::STOP: { // Отключение нагрузки
                	stop_razgon();
                  if (PDMust) { // Если уставка ненулевая...
                    remember_last_power_setpoint();// Запомним последнюю уставку
                    PDMust = 0;
                    Pust = 0;
                  }
                  break;
                }
                default: {
                  break;
                }
              }
              break;
            }
            case COMM_INSTRUCTION_LENGTH: { // Парсим запрос на смену доп.параметра TODO! Оформить функцией перевод в число
              byte b;
              uint8_t tmp_p = 0;
              for (byte x=1; x <= 2; x++ ) {
                tmp_p *= 16;
                b = A_to_HEX (USART_SetData[x]);
                if (b == ERR_A_to_HEX) {
                break;
                }
                tmp_p += b;
              }
              if (b != ERR_A_to_HEX) {
                aux_param_Number = tmp_p;   // Установим новый код доп.параметра
              }
              break;
            }
            case RATE_INSTRUCTION_LENGTH: { // Парсим запрос на смену уставки TODO! Оформить функцией перевод в число
              byte b;
              uint16_t tmp_p = 0;
              for (byte x=1; x <= 4; x++ ) {
                tmp_p *= 16;
                b = A_to_HEX (USART_SetData[x]);
                if (b == ERR_A_to_HEX) {
                  break;
                }
                tmp_p += b;
              }
              if (b != ERR_A_to_HEX) {
                set_newPDM (tmp_p);   // Установим новую уставку мощности;
              }
              break;
            }
            default:
              break;
          }
        index = 0;
        fl.dspNewData = 1;  //Обновление информации на дисплее
        }
        else index = 0;
      }
      else if ( index++ == data_length ) {
        index = 0;
      }
    }
  }
}
//
void USART_report(void) { //=====Отчет внешнему контроллеру по универсальному протоколу=====
  uint8_t curr_byte;
  uint16_t curr_value;
  //
  // Закодируем "Режим + ошибки"
  if (fl.stab_off) {
	curr_byte = 3;  // b000000(11) - аварийное отключение нагрузки (удаленное включение невозможно)
  }
  else if (fl.Udown || fl.NotZero) {
	curr_byte = 6;  // b000001(10) - отсутствие сетевого напряжения, нагрузка отключена
  }
  else if (fl.razg_on) {
	curr_byte = 1;  // b(000000)(01) - разгон
  }
  else if (PDMust == 0) {
	curr_byte = 2;  // b000000(10) - нагрузка отключена
  }
  else if (fl.Ulow) {
	curr_byte = 8;   // b000010(00) - напряжения сети недостаточно для достижения уставки
  }
  else curr_byte = 0; // b000000(00) - режим рабочий, ошибок нет
  //
  USART_InfoData[3] = HEX_to_A ( curr_byte / 16 );  // Старший разряд байта "Режим + ошибки"
  USART_InfoData[4] = HEX_to_A ( curr_byte % 16 );  // Младший разряд байта "Режим + ошибки"
  //
  // Закодируем основной параметр - мощность на выходе
  curr_value = get_Power();  // Получим текущую мощщу
  USART_InfoData[8] = HEX_to_A ( curr_value % 16 );  // 0 разряд основного параметра
  curr_value /= 16;
  USART_InfoData[7] = HEX_to_A ( curr_value % 16 );  // 1 разряд основного параметра
  curr_value /= 16;
  USART_InfoData[6] = HEX_to_A ( curr_value % 16 );  // 2 разряд основного параметра
  USART_InfoData[5] = HEX_to_A ( curr_value / 16 );  // 3 разряд основного параметра
  //
  // Закодируем "Состав данных" и определим доп.параметр
  if ( ~aux_param_Number & 0b10000000) { // Старший бит aux_param_Number - флаг ненужности перезаписи значения
	  switch ( aux_param_Number ) {
		  case AUXPARM_Pn: {	// Дополнительный - номинал мощности нагрузки
			  curr_byte = AUXPARM_Pn << 2;
			  curr_value = Pnom;
			  aux_param_Number |= 0b10000000;
		  break;
		  }
		  case AUXPARM_P: {	// Дополнительный - ненулевая уставка мощности
			  curr_byte = AUXPARM_P << 2;
			  uint16_t tmp_curr_value = PDMust;
			  if (!PDMust) tmp_curr_value = last_Pow_setpoint;
			  curr_value = calc_proportion(tmp_curr_value);
		  break;
		  }
		  default: {	// Дополнительный по умолчанию - напруга в сети
			  curr_byte = AUXPARM_Us << 2;
			  if (fl.NotZero) curr_value = 0;
			  else curr_value = U_sum;
		  break;
		  }
	  }
	  curr_byte |= BASEPARM_P;	// Основной у нас всегда мощность
	//
	  USART_InfoData[1] = HEX_to_A ( curr_byte / 16 );  // Старший разряд байта "Состав данных"
	  USART_InfoData[2] = HEX_to_A ( curr_byte % 16 );  // Младший разряд байта "Состав данных"
	  // Закодируем доп.параметр
	  USART_InfoData[12] = HEX_to_A ( curr_value % 16 ); // 0 разряд доп.параметра
	  curr_value /= 16;
	  USART_InfoData[11] = HEX_to_A ( curr_value % 16 ); // 1 разряд доп.параметра
	  curr_value /= 16;
	  USART_InfoData[10] = HEX_to_A ( curr_value % 16 ); // 2 разряд доп.параметра
	  USART_InfoData[9]  = HEX_to_A ( curr_value / 16 ); // 3 разряд доп.параметра
  }
  // Отправим
  Serial.write(USART_InfoData, 14);
}
//
#endif  //+++++++++++++++++++++++USART++++++++++++++++++++++++++++
//
#ifdef USE_RMVK //++++++++++++++++RMVK_/_Samovar++++++++++++++++++++++++++++
uint16_t get_Uin(void) {     // Функция возвращает значение текущего напряжения без десятичного знака
  return ((U_real_dec < 5)? U_real : (U_real + 1));
}
//
uint16_t get_Uout(const boolean getReal) {  // Функция возвращает расчетное значение текущего (если getReal=true) или желаемого (если getReal=false) напряжения
  uint16_t Uout;
  if ( fl.Udown || fl.NotZero || (PDMust == 0) ) {    // Если сеть в дауне или стаб в стопе - передаем ноль
	  Uout = 0;
  }
  else if ( getReal && ( fl.razg_on || fl.Ulow ) ) {  // В разгоне и при недостаточном сетевом передаем текущее сетевое, если надо
	  Uout = get_Uin();
  }
  else {    // В рабочем режиме - передаем уставку
	  Uout = calc_proportion(PDMust, U_LINE);
  }
  return Uout;
}
//
void USART_parser(void) { // Парсим управляющую последовательность от RMVK_/_Samovar
//
  static String inoutString;
  static byte index = 0;
//
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();// Вычитываем очередной байт
    if ( !index || fl.uartTimeout ) { // Начало посылки
      if ((inChar == 'A') || (byte(inChar) == 0xD0)) { // Ждём первый символ посылки "A" или первый байт UTF-кириллицы из протокола Samovar'a
        inoutString = inChar;
        index=1;
        fl.uartTimeout = 0; // Сбросим флаг таймаута ожидания окончания посылки
        cnt_uartWDT = 0;    // Сбросим таймер ожидания окончания посылки
      }
    }
    else if ( index++ < 13 ) {  // Пока посылка не длиннее 13 символов, считаем её корректной
      if ( inChar == 0x0D ) {   // Ждем последнего символа посылки <CR>
        index = 0;
        // Парсим строку, поскольку кончилась
        // В протоколе Samovar стандартное начало посылки "АТ" пересылается русскими символами в Юникоде. Баг или фича?
        if (( inoutString == ("AT+VI?")) ||  // Запрос текущего напряжения сети
            ( inoutString == ("АТ+VI?"))) {  // В этой строке "АТ" - русскими символами!
          if (fl.NotZero) { // Если сети нет, то и на выходе пусто
            inoutString = String(0);
          }
          else {
            inoutString = String(get_Uin());
          }
        }
        else if (( inoutString == F("АТ+VO?")) || ( inoutString == F("АТ+VS?"))) {  // Запрос текущей мощности от Samovar. В этой строке "АТ" - русскими символами!
          inoutString = String(get_Power());
        }
        else if ( inoutString == F("AT+VO?") ) {  // Запрос текущего напряжения на выходе от РМВ-К
          inoutString = String(get_Uout(true));
        }
        else if ( inoutString == F("AT+VS?") ) {  // Запрос напряжения уставки на выходе от РМВ-К
          inoutString = String(get_Uout(false));
        }
        else if ( inoutString == F("AT+ON?") ) {  // Запрос состояния выхода от РМВ-К
          if ((PDMust == 0) || (fl.NotZero) || (fl.Udown)) {  // Если на выходе 0
            inoutString = String("OFF");
          }
          else {
            inoutString = String("ON");
          }
        }
        else if (( inoutString == F("AT+SS?")) ||   // Запрос режима от Samovar
                ( inoutString == F("АТ+SS?"))) {    // В этой строке "АТ" - русскими символами!
          if (fl.stab_off || fl.Udown || fl.NotZero) {  // При аварии, сильно пониженном напряжении сети или его отсутствии - передаем ошибку
            inoutString = String(3);
          }
          else if (fl.razg_on) {            // Передаем "Разгон"
            inoutString = String(1);
          }
          else if (PDMust == 0) {           // Передаем "Стоп"
            inoutString = String(2);
          }
          else {                            // Передаем "Рабочий режим"
            inoutString = String(0);
          }
        }
        else if (( inoutString == F("AT+ON=0")) ||  // Запрос на выключение стабилизатора
                ( inoutString == F("АТ+ON=0"))) {   // В этой строке "АТ" - русскими символами!
          if (!fl.stab_off) {  // Если стаб не выключен аварийно...
            PDMust = 0;
            stop_razgon();
            Pust = 0;
            fl.dspNewData = 1;  //Обновление информации на дисплее
            inoutString = "";
          }
        }
        else if (( inoutString == F("AT+ON=1")) ||  // Запрос на включение режима "Разгон"
                ( inoutString == F("АТ+ON=1"))) {   // В этой строке "АТ" - русскими символами!
          if ((!fl.stab_off) && (!fl.NotZero) && (!fl.Udown) && (!fl.razg_off)) {  // Если авария, электросеть в дауне или разгон запрещен - не разгонишься
        	start_razgon();
            fl.dspNewData = 1;  //Обновление информации на дисплее
          }
          inoutString = "";
        }
        else if ( inoutString.substring(0,8) == F("АТ+VS=") ) {  // Запрос на изменение уставки от Samovar. В этой строке "АТ" - русскими символами!
          if (!fl.stab_off) {  // Если стаб не выключен аварийно...
            //выключаем разгон, на всякий случай
            stop_razgon();
            set_newPDM (inoutString.substring(8).toInt());        // Установим новую уставку мощности
            inoutString = "";
          }
        }
        else if ( inoutString.substring(0,6) == F("AT+VS=") ) { // Запрос на изменение уставки от РМВ-К
          if (fl.stab_off || fl.Udown || fl.NotZero) {  // Если авария или сеть в дауне - ничего не меняем, передаем ошибку
            inoutString = String(F("error"));
          }
          else {
            uint16_t tmp_u = inoutString.substring(6).toInt();
            if ( tmp_u < U_LINE ) {
              tmp_u *= CICLE;
              PDMust = tmp_u / U_LINE;
            }
            else PDMust = CICLE;
            //выключаем разгон, на всякий случай
            stop_razgon();
            set_Pust();         // Посчитаем Pust
            fl.dspNewData = 1;  // Обновление информации на дисплее
            inoutString = String(get_Uout(false));
          }
        }
        else {  // Неизвестная или закосяченная команда
          #ifdef Debug
            inoutString = String(F("(o_O unknown!)"));
          #else
            inoutString = "";
          #endif
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
#endif  //+++++++++++++++++++++++RMVK_/_Samovar++++++++++++++++++++++++++++
//
//-------------------------------------------------------------------------
//===========Подпрограмма запоминания последней уставки====================
//== Проверяет последнюю уставку на совпадение с уже записанными в массив уставок
//== и запоминает, если надо
//
void remember_last_power_setpoint(void) {  // Запомним последнюю уставку
#ifdef USE_ADprotocol
  last_Pow_setpoint = PDMust;  //Запоминаем текущую мощность ТЭНа
#endif	// USE_ADprotocol
  boolean isnew = 1;
  for (uint8_t x = 0; x < PDMset_ARR_SIZE; x++) {  // Проверим новое значение на совпадение с уже записанными
    if (PDMust == PDMset[0][x]) {
      isnew = 0; break;
    }
  }
  if (isnew) { // Если новое значение действительно новое, то...
    PDMset[0][PDMset_ARR_SIZE - 1] = PDMust;  //Запоминаем текущую мощность ТЭНа
    PDMset[1][PDMset_ARR_SIZE - 1] = 0;       //Адрес зануляем на всякий случай
    cnt_PDMcount = PDMset_ARR_SIZE - 1;       //Ставим счетчик на запомненную уставку
  }
}
//
//===========Подпрограмма запоминания последней уставки(конец)=============
//-------------------------------------------------------------------------
//===========Подпрограмма обмена двух ячеек массива========================
//== arr - массив
//== row - индекс строки обмениваемых ячеек
//== col1 - индех столбца первой обмениваемой ячейки
//== col2 - индех столбца второй обмениваемой ячейки
//
// change_arr_cell(arr, row, col1, col2)
//
void change_arr_cell(uint16_t arr[2][ARRAY_SIZE], const uint8_t row, const uint8_t col1, const uint8_t col2) {
            uint16_t k = arr[row][col1];
            arr[row][col1] = arr[row][col2];// Обмениваемся
            arr[row][col2] = k;
}
//===========Подпрограмма обмена двух ячеек массива(конец)=================
//-------------------------------------------------------------------------
//===========Подпрограмма чтения начальных данных из EEPROM================
void read_Pnoms_from_EEPROM(void) {
//
  uint8_t idx;
  uint16_t value;
  for (idx = 0; idx < Pnom_ARR_SIZE; idx++) {
    EEPROM.get((idx * 2),value);
    if ((value < 10000) && value) Pnom_arr[idx] = value;  // Если значение корректно, пишем его в массив
    else break;                                           // если нет - уходим.
  }
  cnt_Pnom_count = idx;
  Pnom = Pnom_arr[0]; // По умолчанию установим номинальную мощность из первой ячейки
}
//===========Подпрограмма чтения начальных данных из EEPROM(конец)=========
//-------------------------------------------------------------------------
//===========Функция чтения непустых ячеек из EEPROM=======================
//== возвращает количество прочитанных ячеек
//== val_arr - массив значений с адресами
//== addr_arr - массив адресов
//== count - счетчик значений
//== start_addr - начальный адрес
//== end_addr - конечный адрес
//== threshold - пороговое значение счетчика для выхода из цикла
//
// tmpcount += get_noempty_cells(PDMtmp, tmpcount, start_cell, end_cell, (PDMset_ARR_SIZE-2));
//
uint8_t get_noempty_cells(uint16_t val_arr[2][ARRAY_SIZE], const uint8_t count, const uint16_t start_cell, const uint16_t end_cell, const uint8_t threshold) {
  uint16_t addr;
  uint16_t value;
  uint8_t idx = count;
    for (addr = start_cell; addr <= end_cell; addr += 2) { // Почитаем из области для записи уставок в EEPROM, начиная с конца
      EEPROM.get(addr,value);
      if ((value <= CICLE) && value) {      // Если считанное значение корректное, то
        val_arr[0][idx] = value;         // запишем его в массив
        val_arr[1][idx] = addr;          // и туда же запишем адрес значения в памяти EEPROM
        if (++idx > threshold) break; // Закончим, заполнив массив
      }
    }
  return idx;
}
//
//===========Функция чтения непустых ячеек из EEPROM(конец)=============
//
void read_PDMs_from_EEPROM(void) { //===========Подпрограмма чтения уставок из EEPROM для выбранной Pnom================
//
  uint16_t PDMtmp[2][ARRAY_SIZE] = {};
  uint8_t tmpcount = 0;
  uint16_t addr;
  uint16_t value;
  ////===Заполним массив уставок какими-то значениями
  //
  value = CICLE / PDMset_ARR_SIZE;
  for (uint8_t idx=0; idx < PDMset_ARR_SIZE - 1; idx++) {
    PDMset[0][idx] = value * (idx + 1);
  }
  PDMset[0][PDMset_ARR_SIZE - 1] = CICLE;
  //
  ////===Определим границы области для записи уставок
  //
  addr = (EEPROM.length() - SRVDATA_ARR_SIZE - 2 * Pnom_ARR_SIZE)/(Pnom_ARR_SIZE);  // Размер области для записи уставок в EEPROM
  if (addr & 1) addr--; // Округлим до четного вниз, чтобы влезало целое количество двухбайтовых слов
  start_addr = Pnom_ARR_SIZE * 2 + cnt_Pnom_number * addr;  // Начальный адрес области
  end_addr = start_addr + addr - 2;             // Конечный адрес области
  //
  //---Дальше пытаемся читать запомненные значения, они должны быть расположены подряд---
  //
  uint16_t read_region_end_addr = end_addr; // Адрес ячейки, на которой чтение можно окончить
  EEPROM.get(end_addr,value);               // Прочтем последнюю ячейку области
  if ((value <= CICLE) && value) {          // Если считанное значение корректное, то читаем из конца области
    read_region_end_addr = start_addr + 2 * ((PDMset_ARR_SIZE - 2) - 1);  // На 3 меньше, потому что одно значение уже есть, как минимум, другое в память не пишется, ну и размер на единицу больше наибольшего индекса, Так-то!
    tmpcount = get_noempty_cells(PDMtmp, 0, (end_addr - 2 * (PDMset_ARR_SIZE-2)), end_addr, (PDMset_ARR_SIZE-2));
  }
  if (tmpcount < PDMset_ARR_SIZE - 1) {         // Проверим массив уставок на заполненность и если не полон, то читаем дальше
    tmpcount += get_noempty_cells(PDMtmp, tmpcount, start_addr, read_region_end_addr, (PDMset_ARR_SIZE-2));
  }
  //
  //---Прочитали---
  //
  if (tmpcount) { // Если записанные уставки есть, то...
    tmpcount--;   // Декрементируем счетчик, чтобы индексы начинались с нуля
    new_addr = PDMtmp[1][tmpcount] + 2;                   // Адрес для записи новой уставки...
    //new_addr += 2;
    if (new_addr > end_addr) new_addr = start_addr; // на единицу больше адреса последней считанной, но не больше границы области
    if (tmpcount == PDMset_ARR_SIZE-2) {  // Если массив полон
      old_addr = PDMtmp[1][0]; // то адрес самой старой уставки в нулевой ячейке
    }
    { // Уберем дублирующиеся значения
      uint8_t PDMdiff[PDMset_ARR_SIZE] = {0};   // Заведем временный массив совпадений
      for (uint8_t i = 0; i < PDMset_ARR_SIZE; i++) {   // Пробежимся по обоим массивам
        for (uint8_t j = 0; j <= tmpcount; j++) {
          if (PDMset[0][i] == PDMtmp[0][j]) PDMdiff[i] = 1; // И заполним массив совпадений
        }
      }
      for (uint8_t i = 0; i < PDMset_ARR_SIZE - 1; i++) { // Сортируем
        for (uint8_t j = i + 1; j < PDMset_ARR_SIZE; j++) {
          if (PDMdiff[j]) {             // Если в данной позиции есть совпадение
            change_arr_cell(PDMset, 0, i, j);
            PDMdiff[j] = PDMdiff[i];
            break;
          }
        }
      }
    }
    for (uint8_t i = 0; i <= tmpcount; i++) { // Допишем в рабочий массив считанное из EEPROM
      PDMset[0][i] = PDMtmp[0][i];
      PDMset[1][i] = PDMtmp[1][i];
    }
  }
  else {          // Записанных уставок нет
    new_addr = start_addr;  // Адрес для записи новой уставки равен начальному
  }
}//===========Подпрограмма чтения уставок из EEPROM для выбранной Pnom================
//
void ADC_init(void) { //===============Инициализация АЦП===================
  ADMUX = 0;
  ADMUX |= ( 1 << REFS0); // Задаем ИОН равный напряжению питания
  ADMUX |= 0;             // Выбираем пин A0 для преобразования
  ADCSRA |= (1 << ADPS2 ) | (1 << ADPS1) | (1 << ADPS0); // предделитель на 128
  ADCSRA |= (1 << ADIE);  // Разрешаем прерывания по завершении преобразования
  ADCSRA |= (1 << ADEN);  // Включаем АЦП
  //
  ADCSRA |= (1 << ADATE); // Включаем автозапуск преобразования по событию
  ADCSRB = 0;
  ADCSRB |= (0 << ADTS2 ) | (1 << ADTS1) | (1 << ADTS0);	// В качестве события выбираем "Timer/Counter0 Compare Match A"

}//================================Инициализация АЦП===================
//
void Timers_init(void) { //===============Инициализация таймеров===================
  //---Инициализация таймера 0 для тактирования АЦП -------------
  TCCR0A = 0;
  TCCR0B = 0;
  TCCR0A |= (1 << WGM01); // Счетчик работает в режиме CTC (сброс по совпадению)
  TCCR0B |= (1 << CS01) | (1 << CS00); // Предделитель на 64 (на счетчик - 250 кГц)
  OCR0A = T_ADC; // Определяет период запуска АЦП
  // TIMSK0 |= (1 << OCIE0A); // Разрешаем прерывания по совпадению с OCR0A
  // Инициализация таймера 2 для формирования импульса нуля Zero
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A |= (1 << WGM21); // Счетчик работает в режиме CTC (сброс по совпадению)
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Предделитель на 1024 (сч. - 15.625 кГц/64мкс)
  OCR2A = LINE_FREQ; // Прерывание с удвоенной частотой сети
  TIMSK2 |= (1 << OCIE2A); // Разрешаем прерывания по совпадению с OCR2A
}//=================================Инициализация таймеров===================
//
void Pins_init(void) { //======Инициализация входов/выходов контроллера=========
#ifdef DisplayReset
  pin_OLEDres_INIT;
  pin_OLEDres_LOW;  // Сбрасываем дисплей (!!! НЕ ЗАБЫТЬ ПЕРЕКЛЮЧИТЬ НА ВЫСОКИЙ !!!) TODO!
#endif
  pin_RAZGON_OFF_INIT;
  pin_STAB_OFF_INIT;
  //
  pin_TOut_INIT;
  pin_TRelay_INIT;
  TURN_RELAY_OFF; // Выключаем ТЭН (контактное реле)
  TURN_SSR_OFF;   // Выключаем ТЭН (твердотельное реле)
  //
  pin_buttGND_INIT;
  //
  pin_butt_1_INIT;
  pin_butt_2_INIT;
  pin_butt_3_INIT;
  pin_butt_4_INIT;
  //
#ifdef Debug
  pin_ZeroOut_INIT;
  pin_ZeroOut_LOW;
  pin_DebugOut_INIT;
  pin_DebugOut_LOW;
  pin_TestOut_INIT;
  pin_TestOut_LOW;
#endif
//
#ifdef PS_Debug
  pin_TestPS_Out;
  pin_TestPS_LOW;
#endif
//
#ifdef LED_debug
  pin_LED_INIT;
#endif
//
}//========================Инициализация входов/выходов контроллера=========
//
void Razgon_(void) { //===========Подпрограмма обработки режима разгона================
  ////===Обеспечивает шунтирование контактов контактного реле
  ////===симистором твердотельного
  ////===в момент включения/выключения режима "Разгон"
  ////===
  #define RELAY_SHUNTING_TIME 50      // количество полупериодов, в течение которых шунтируются контакты реле
  static uint8_t cnt_P_relay=0;       // Счетчик полупериодов шунтирования контактного реле
    if (fl.razg_on &&                 // Если включен разгон..
        !fl.TRelay &&                 // ..и НЕ включено контактное реле
        (++cnt_P_relay > RELAY_SHUNTING_TIME)) { // ..и все это длится уже более 500мс,
      fl.TRelay = 1; cnt_P_relay = 0; // то включим контактное реле и обнулим счетчик
    }
    if (fl.razg &&                    // Если включен максимум для твердотельного реле..
        !fl.razg_on &&                // ..и выключен разгон
        (++cnt_P_relay > RELAY_SHUNTING_TIME)) { // ..и все это длится уже более 500мс,
      fl.razg = 0; cnt_P_relay = 0;   // то выключим реле и обнулим счетчик
    }
}//===========Подпрограмма обработки режима разгона================
//
void PDM_(void) { //===========Подпрограмма управления твердотельным реле ТЭНа================
  if (fl.razg) {
    pdm = CICLE; // В режиме разгона твердотельное всегда открыто
  }
  static int8_t ps = 0;     // Текущее значение постоянной составляющей
  int32_t lev = pdm + pdm_err;    // Текущий уровень с учетом ошибки дискретизации, сделанной на предыдущем полупериоде.
  //Текущее значение постоянной составляющей
  if (fl.Tout) {
    if (fl.PP_tm) ps--;
    else ps++;
  }
//
#ifdef PS_Debug
  switch (ps) {
	  case -1: {
		  pin_TestPS_Out; //ОТЛАДКА
		  pin_TestPS_LOW;
		  break;
	  }
	  case 1: {
		  pin_TestPS_Out; //ОТЛАДКА
		  pin_TestPS_HIGH;
		  break;
	  }
	  case 0:
	  default: {
		  pin_TestPS_Z; //ОТЛАДКА
		  break;
	  }
  }
#endif
//
  if ((lev >= CICLE/2)
		&& ((ps == 0) || (fl.PP_tm && (ps < 0))
		|| (!fl.PP_tm && (ps > 0))))
  	  { // Ставим флаг включения ТЭНа с учетом значения постоянной составляющей
    fl.Tout = 1; pdm_err = lev - CICLE;         // и считаем ошибку для следующего полупериода
  }
  else {
    fl.Tout = 0; pdm_err = lev;                 // Снимаем флаг включения ТЭНа и считаем ошибку
  }
}//========================Подпрограмма управления твердотельным реле ТЭНа================
//
void Buttons_(void) { //==============Опрос кнопок=====================
//
  enum BT { // Коды кнопок
	P_MINUS = 1,
	P_PLUS = 2,
	STOP = 4,
	RAZGON = 8
  }; // Коды кнопок
//
  static struct buttons {
    unsigned butt_1 : 1;    // текущее состояние кнопки (0 - не нажата)
    unsigned butt_2 : 1;    // текущее состояние кнопки
    unsigned butt_3 : 1;    // текущее состояние кнопки
    unsigned butt_4 : 1;    // текущее состояние кнопки
    unsigned no_select : 1; // вспомогательный флажок для начального меню
    unsigned writePnom : 1; // вспомогательный флажок записи нового Pnom в EEPROM
    unsigned clear_old : 1; // вспомогательный флажок стирания старой уставки из EEPROM
  } bt ={}; // Инициализируем структуру с нулевыми членами
  //
  static uint8_t butt = 0;       // код текущей нажатой кнопки
  static uint8_t last_butt = 0;  // код предыдущей нажатой кнопки
  static uint8_t butt_count = 0;        // счетчик для устранения дребезга
  static uint8_t butt_force_count = 0;  // счетчик для форсирования инкремента/декремента
//
  if (bt.clear_old) { //=====Стираем старую уставку, если нужно
    eeprom_update_word((uint16_t*)clear_old_addr,EMPTY_CELL_VALUE);    // Стираем самую старую уставку
    bt.clear_old = 0;                                     // Снимаем флажок стирания
  }
  //
  bt.butt_1 = pin_butt_1_STATE;
  bt.butt_2 = pin_butt_2_STATE;
  bt.butt_3 = pin_butt_3_STATE;
  bt.butt_4 = pin_butt_4_STATE;
//
  uint8_t button_sum = bt.butt_1 + bt.butt_2 + bt.butt_3 + bt.butt_4;
//
  if ( (button_sum == 0) && butt_force_count ) butt_force_count--; // уменьшаем счетчик форсирования инкремента/декремента
  if ( button_sum == fl.butt ) { // Или нажата одна кнопка или ни одной
    butt = bt.butt_1 + (bt.butt_2 << 1) + (bt.butt_3 << 2) + (bt.butt_4 << 3);
    if ( butt == last_butt ) {
      butt_count++;
    }
    else {
      butt_count = 1;
      last_butt = butt;
    }
  }
  else if (--butt_count < 1) {
    butt_count = 1;
  }

  //
  if ( (butt_count == DEBOUNCE) || fl.dspTimeout ) {  // Есть нажатая кнопка или достаточная пауза после нажатия или таймаут выхода из меню
    if (!fl.stab_off) { // Если нет аварийного останова...
      switch (cnt_dspMenu) { //=====Проверяем режимы меню
        case DSP::Pnom_MENU:  {  //=============Если мы в начальном меню выбора номинальной мощности, то...
          if (fl.dspTimeout) {                  // Если кнопки слишком долго не нажимались...
            if (Pnom_arr[0] != 0xffff) {        // и есть записанное значение, уходим
              cnt_Pnom_number = 0;              //
              Pnom = Pnom_arr[0]; 				// По умолчанию установим номинальную мощность из нулевой ячейки
              fl.writable = 1;                  // Уставки пишутся в EERPOM
              read_PDMs_from_EEPROM();			// Читаем уставки
              fl.uartUnhold = 1;                // Разрешим обращение к USART
              cnt_dspMenu = DSP::OP_MODE;		// Выйдем из менюшки
              fl.dspRefresh = 1;                // Ставим флаг обновления экрана
            }
            fl.dspTimeout = 0;                  // Снимаем флаг таймаута выхода из меню
            break;
          }
          switch (butt) {
            case BT::P_MINUS: { //-----Кнопкой "P-" перебираем записанные значения или уменьшаем значение Pnom
              if (bt.no_select) { //Если не выбираем, а вводим значение,...
                if (butt_force_count > 20) {      // Если очень долго держим...
                  if (Pnom > 100) Pnom -= 100;    // Убавляем по соточке, пока есть куда
                  else butt_force_count = 10;     // Если некуда убавлять - снижаем форсаж
                }
                else if (butt_force_count > 10) { // Если долго держим...
                  if (Pnom > 10) Pnom -= 10;      // Убавляем по десяточке, пока есть куда
                  else butt_force_count = 0;      // Если некуда убавлять - снижаем форсаж
                }
                else {
                  if (--Pnom == 0) Pnom=1;        // Убавляем по чуть-чуть
                }
              }
              else {  //Если выбираем из записанных в EEPROM...
                if (++cnt_PDMcount > cnt_Pnom_count) cnt_PDMcount=0; // Перебираем значения уставок мощности ТЭНа
                Pnom = Pnom_arr[cnt_PDMcount];
              }
              butt_force_count++;
              break;                      //Закончили
            }
            case BT::P_PLUS: { //-----Кнопкой "P+" увеличиваем значение Pnom
              if (butt_force_count > 20) {
                if ((Pnom += 100) > 9999) Pnom=9999;// Если очень долго держим, прибавляем по соточке
              }
              else if (butt_force_count > 10) {
                if ((Pnom += 10) > 9999) Pnom=9999; // Если долго держим, прибавляем по десяточке
              }
              else {
                if (++Pnom > 9999) Pnom=9999;       // Прибавляем по чуть-чуть
              }
              bt.no_select = 1;	// Ставим флажок не выбора, а ввода нового значения
              butt_force_count++;
              break;                      //Закончили
            }
            case BT::STOP: { //-----Кнопкой "Стоп" пишем значение в память и выходим из менюшки
              bt.writePnom = 1; // Ставим флаг записи нового значения Pnom в EEPROM
              fl.writable = 1;  // Ставим флаг записи уставок в EEPROM
              // break;			// Дальнейшие действия совпадают с обработкой кнопки "Разгон", потому break не ставим
            }
            case BT::RAZGON: { //-----Кнопкой "Разгон" выходим из менюшки
              if (Pnom < 10000) { // Если значение реальное...
                cnt_Pnom_number = cnt_PDMcount;               // Запомним порядковый номер выбранного Pnom
                if (bt.no_select) {             // Если значение НЕ выбрано из записанных в EEPROM, а введено...
                  for (int8_t x = cnt_Pnom_count; x >= 0; x--) { // Проверим новое значение на совпадение с уже записанными
                    if (Pnom == Pnom_arr[x]) {                // Если такое значение уже есть в EEPROM...
                      cnt_Pnom_number = x;                    // Запомним порядковый номер совпавшего Pnom
                      bt.writePnom = 0;           	  // Снимем флаг записи нового значения Pnom в EEPROM
                      fl.writable = 1;                        // Ставим флаг записи уставок в EEPROM
                      break;
                    }
                  }
                }
                else {                            	// Если значение выбрано из записанных в EEPROM...
                  bt.writePnom = 0;					// Снимем флаг записи нового значения Pnom в EEPROM
                  fl.writable = 1;             		// Ставим флаг записи уставок в EEPROM
                }
                //
                cnt_PDMcount=0;                  	//Сбрасываем счетчик
                //
                if (fl.writable) {                	// Если уставки пишутся в EERPOM, то
                  read_PDMs_from_EEPROM();			// читаем ранее записанное
                }
                if (bt.writePnom) {   				// Запишем новое значение Pnom, если необходимо
                  eeprom_update_word((uint16_t*)(cnt_Pnom_number * 2),Pnom);
                  bt.writePnom = 0;           		// и сбросим флаг записи нового значения Pnom
                }
                cnt_dspMenu = DSP::OP_MODE;			// Снимаем флаг перехода в меню
                //
                #ifdef USE_USART      //========================================
                  fl.uartUnhold = 1;  // Разрешим обращение к USART
                #endif                //========================================
                fl.dspRefresh = 1;  // Ставим флаг обновления экрана
              }
              fl.butt = 0;        // После нажатия должна быть пауза
              break;              // Закончили
            }
            default:
              fl.butt = 1;        // достаточная пауза между нажатиями
              break;
          }
          break;
        }
        case DSP::Pust_MENU:  {  //=============Если мы в меню выбора уставки, то...
          if (fl.dspTimeout) {  		// Если кнопки слишком долго не нажимались, уходим
            cnt_dspMenu = DSP::OP_MODE;	// Выйдем из менюшки
            fl.dspRefresh = 1;  		// Ставим флаг обновления экрана
            fl.dspTimeout = 0;  		// Снимаем флаг таймаута выхода из меню
            break;
          }
          switch (butt) {
            case BT::P_MINUS: { //=====По кнопке "Р-" перебираем значения
              if (++cnt_PDMcount > PDMset_ARR_SIZE - 1) cnt_PDMcount=0; //Перебираем значения уставок мощности ТЭНа
    //          fl.butt = 0;            //После нажатия должна быть пауза
              break;                    //Закончили
            }
            case BT::P_PLUS: { //=====По кнопке "Р+" перебираем значения
              if (cnt_PDMcount-- == 0) cnt_PDMcount=PDMset_ARR_SIZE - 1;//Перебираем значения уставок мощности ТЭНа
    //          fl.butt = 0;            //После нажатия должна быть пауза
              break;                    //Закончили
            }
            case BT::STOP: { //=====По кнопке "стоп" записываем уставку, если нужно, принимаем и выходим
              PDMust = PDMset[0][cnt_PDMcount];		// Устанавливаем выбранную мощность ТЭНа
              if ( fl.writable && 					// Если уставки запоминаются...
            		  PDMset[0][cnt_PDMcount] && 	// и выбранная - НЕ нулевая...
					  !PDMset[1][cnt_PDMcount] ) {  // и НЕ уже записанная...
                  //
            	  eeprom_update_word((uint16_t*)new_addr,PDMset[0][cnt_PDMcount]); // Пишем новую уставку
                  PDMset[1][cnt_PDMcount] = new_addr; // Заносим в массив адрес свежезаписанной уставки
                  new_addr += 2;
                  if (new_addr > end_addr) new_addr = start_addr; // Инкрементируем адрес для новой уставки и следим, чтобы не выходило за границы области
                  if (cnt_PDMcount == PDMset_ARR_SIZE - 1) {  // Если новое значение - последнее в списке
                    //
                    if (!old_addr) {  // Если в массиве уставок есть незаписанные в EEPROM значения, то сначала стираем их
                      boolean swapped = 1; uint8_t upper_index = PDMset_ARR_SIZE - 1; //=====Пузырьковая сортировка
                      while (swapped) {                                       // Пока есть обмены, сортируем
                        swapped = 0;
                        for (uint8_t i = 1; i < upper_index; i++) {
                          if (PDMset[1][i] < PDMset[1][i - 1]) {
                            change_arr_cell(PDMset, 0, i, i - 1);
                            change_arr_cell(PDMset, 1, i, i - 1);
                            swapped = 1;
                          }
                        }
                        upper_index--;
                      } //=====Закончили сортировку
                      old_addr = PDMset[1][0];  // Обновляем адрес самой старой уставки
                    }
                    //
                    if (old_addr) {           // Если в массиве уставок все значения записаны в EEPROM, то стираем самое старое
                      bt.clear_old = 1;       // Ставим флажок стирания (сотрём в следующий вызов подпрограммы опроса кнопок)
                      clear_old_addr = old_addr; // Плодим сущности без устали!
                    }
                    //
                    uint16_t k = PDMset[0][0];
                    for (uint8_t x = 0; x < PDMset_ARR_SIZE - 1; x++) { // Сдвинем массив
                      PDMset[0][x] = PDMset[0][x + 1];
                      PDMset[1][x] = PDMset[1][x + 1];
                    }
                    //
                    PDMset[0][PDMset_ARR_SIZE - 1] = k; // Запишем во временную ячейку свежеудаленное значение
                    PDMset[1][PDMset_ARR_SIZE - 1] = 0;
                    cnt_PDMcount--;
                    old_addr = PDMset[1][0];    // Обновляем адрес самой старой уставки
                    //
                  }
              }
              cnt_dspMenu = DSP::OP_MODE;	//Снимаем флаг перехода в меню
              fl.dspRefresh = 1;        //Ставим флаг обновления экрана
              fl.butt = 0;              //После нажатия должна быть пауза
              break;                    //Закончили
            }
            case BT::RAZGON: { //=====По кнопке "разгон" принимаем и выходим
              PDMust = PDMset[0][cnt_PDMcount];//Устанавливаем выбранную мощность ТЭНа
              cnt_dspMenu = DSP::OP_MODE;	//Снимаем флаг перехода в меню
              fl.dspRefresh = 1;        //Ставим флаг обновления экрана
              fl.butt = 0;              //После нажатия должна быть пауза
              break;                    //Закончили
            }
            default:
              fl.butt = 1;              // достаточная пауза между нажатиями
              break;
          }
          break;
        }
        default: {  //=============А если не в меню, то...
          switch (butt) {
            case BT::P_MINUS: {
              if (PDMust-- == 0) PDMust = 0; //Уменьшаем установленную мощность до минимума
              break;
            }
            case BT::P_PLUS: {
              if (++PDMust > CICLE) PDMust = CICLE; //Увеличиваем установленную мощность до максимума
              break;
            }
            case BT::STOP: {
              stop_razgon();                	// Остановим разгон
              if (PDMust == 0) {    			// Если мы не в меню и мощность ТЭНа нулевая, то...
                cnt_dspMenu = DSP::Pust_MENU;	// Ставим флаг перехода в меню
                fl.dspRefresh = 1;  			// Ставим флаг обновления экрана
              }
              else {                //Если мы не в меню и мощность ТЭНа НЕнулевая, то...
                remember_last_power_setpoint();	// Запомним последнюю уставку
                PDMust = 0;                   	// Экстренно выключим ТЭН
              }
              fl.butt = 0;          //После нажатия должна быть пауза
              break;
            }
            case BT::RAZGON: {
              fl.razg_on = ((!fl.NotZero) & (!fl.Udown) & (!fl.razg_off) & (!fl.razg_on)); //Триггер режима разгона (гистерезис организован в обработке начала полупериода)
              if (fl.razg_on) {
            	fl.razg = 1;	//Если разгон включили, то твердотельное реле на максимум сразу
              }
              else {
            	fl.TRelay = 0;	//Если разгон выключили, то контактное реле выключаем сразу
              }
              fl.butt = 0;                                //После нажатия должна быть пауза
              break;
            }
            default: {
              fl.butt = 1;  // достаточная пауза между нажатиями
              break;
            }
          }
          break;
        }
      }
    }
    //
    if (butt) {  // Если нажата кнопка,
      cnt_menuWDT = 0;  // сбросим таймер ожидания выхода из меню
      fl.stab_off = 0;  // и сбросим флажок аварийного останова
    }
    butt_count = 1;
    butt = 0;
    set_Pust();         // Пересчитаем Pust
    fl.dspNewData = 1;  //Обновление информации на дисплее
  }
  //
//  if (pin_STAB_OFF_STATE && !fl.stab_off) { // Если есть сигнал аварийного останова
//    if (PDMust) { // Если уставка ненулевая...
//      remember_last_power_setpoint();// Запомним последнюю уставку
//      PDMust = 0;                   // Экстренно выключим ТЭН
//      Pust = 0;                     // Пересчитаем Pust
//    }
//    stop_razgon();    		// Остановим разгон
//    fl.dspNewData = 1;		// Обновление информации на дисплее
//    fl.stab_off = 1;  		// Поставим соответствующий флажок
//  }
//  else {
//    fl.razg_off = pin_RAZGON_OFF_STATE; // Прочитаем состояние вывода отключения разгона
//    if (fl.razg_off) {		// Если разгон и есть внешний сигнал останова разгона...
//      fl.dspNewData = 1;	// Обновление информации на дисплее
//      stop_razgon();  		// остановим разгон
//    }
//  }
	static uint8_t stab_off_delay = 0;
	static uint8_t razgon_off_delay = 0;
	if (!pin_STAB_OFF_STATE) {
		stab_off_delay = 0;
		bool ros = pin_RAZGON_OFF_STATE;	// Прочитаем состояние вывода отключения разгона
		if ((ros != fl.razg_off) && (++razgon_off_delay > 30)) {
			razgon_off_delay = 0;
			fl.razg_off = ros;		// Поставим соответствующий флажок
			fl.dspNewData = 1;		// Обновление информации на дисплее
			stop_razgon();  		// остановим разгон
		}
	}
	else if (!fl.stab_off && (++stab_off_delay > 30)) { // Если есть сигнал аварийного останова дольше 300мс
  		stab_off_delay = 0;
      if (PDMust) { // Если уставка ненулевая...
        remember_last_power_setpoint();// Запомним последнюю уставку
        PDMust = 0;                   // Экстренно выключим ТЭН
        Pust = 0;                     // Пересчитаем Pust
      }
      stop_razgon();    		// Остановим разгон
      fl.dspNewData = 1;		// Обновление информации на дисплее
      fl.stab_off = 1;  		// Поставим соответствующий флажок
    }
  //
} //================================Опрос кнопок=====================
//
ISR(TIMER2_COMPA_vect) { //======Обработчик начала очередного полупериода по таймеру2=========
  //
  #ifdef Debug
    pin_TestOut_INV; //ОТЛАДКА
  #endif
  //
  Razgon_();
  if (pdm) {
    PDM_();
  }
  else {
    fl.Tout = 0;    // Не будем зря теребить подпрограмму, если pdm = 0
    //pdm_err = 0;  // и обнулим ошибку дискретизации (а нужно ли?) TODO!
  }
//
  fl.PP_tm = !fl.PP_tm; // Инвертируем флаг полуволны
  OCR2A = PID_ust;    // Грузим новое значение в регистр сравнения
//
#ifdef LED_debug  // Продублируем сигнал управления твердотелкой на светодиоде, если надо
  fl.Tout ? TURN_LED_ON : TURN_LED_OFF ;
#endif
  fl.Tout ? TURN_SSR_ON : TURN_SSR_OFF ;  // Включаем или выключаем ТЭН (твердотельное реле)
  fl.TRelay ? TURN_RELAY_ON : TURN_RELAY_OFF ;  // Включаем или выключаем ТЭН (контактное реле)
  //
sei(); // разрешим прерывания
  // Считаем время
  static uint8_t cnt_P_time=0;  // Счетчик полупериодов для организации отсчета времени
  if (++cnt_P_time == P_TIME_MAX) {  // Уже секунду суммируем
    cnt_P_time = 0;
    //fl.dspNewData = 1;  // Раз в секунду не грех обновить дисплей, мало ли...
    if ((cnt_dspMenu != DSP::OP_MODE) && (++cnt_menuWDT == MENU_TIMEOUT)) { // Если мы в меню и слишком долго не жмутся кнопки
      fl.dspTimeout = 1;                                        // Установим флаг таймаута
      cnt_menuWDT = 0;                                          // Сбросим таймер ожидания выхода из меню
    }
    //
    #ifdef USE_USART
      if (++cnt_uartWDT == 10) {  // Если прошло уже 10 секунд от начала приема посылки по USART
        fl.uartTimeout = 1;       // Установим флаг таймаута ожидания окончания посылки
        cnt_uartWDT = 0;          // Сбросим таймер ожидания окончания посылки
      }
    #endif
    #ifdef USE_ADprotocol
      fl.uartReport = 1;          // пора слать рапорт
    #endif
    //
  }
//
  Buttons_();   // Опрашиваем кнопки

}//==============================Обработчик начала очередного полупериода по таймеру2=========
//------------------------------------------------------------------------------
//ISR(TIMER0_COMPA_vect) { //======Обработчик запуска преобразования АЦП по таймеру0=========
//  ADCSRA |=  (1 << ADSC); // Запуск преобразования
//}//==============================Обработчик запуска преобразования АЦП по таймеру0=========
//------------------------------------------------------------------------------
ISR(ADC_vect) { //===============Обработчик окончания преобразования АЦП===================
#ifdef Debug
  pin_DebugOut_HIGH;  //ОТЛАДКА
#endif
  static volatile uint8_t TM2_current;
  static volatile int16_t Ufir = 0;           // Буферная переменная для НЧ-фильтрации
  static volatile int16_t Udelta = 0;         // Буферная переменная для НЧ-фильтрации
{
  int16_t U_adc;
  uint8_t TM2_tmp;
  TM2_tmp = TCNT2;  // забрали значение из таймера синхронизации с сетью
  U_adc = ADCL; U_adc += ADCH << 8; // забрали результат преобразования АЦП
//
#ifndef OFFLINE_debug
  U_adc -= U_ZERO;  // Убираем постоянную составляющую из оцифрованного сигнала
#else // OFFLINE_debug
  {
	  static uint8_t adc_step = 0;			// Примерно раз в 200мкс запускается АЦП
	  if ( adc_step++ == 100 ) adc_step=0;	// значит 100 запусков = 1 период сети
	  if ( adc_step < 50 ) U_adc=126;		// это положительный полупериод меандра 218В (218*0.577=126)
	  else U_adc=-126;						// это отрицательный полупериод
  }
#endif // OFFLINE_debug
//
  TIM0_COMPA_FLAGRESET // Сбросили флаг прерывания для обеспечения следующего запуска АЦП
  {//===Суммирование квадратов=======================================
  int32_t mpl_tmp = (int32_t)U_adc * U_adc; // Возводим в квадрат выборку АЦП
  sum +=(uint32_t)mpl_tmp; // и суммируем с предыдущими
  ++sc;     // Счетчик выборок АЦП
  }//===Суммирование квадратов=======================================
  //
  //===детекция перехода через ноль и ПИД-синхронизация=================================
  //
  Udelta += (U_adc - Ufir); //
  U_adc = Udelta / 32;      //КИХ ФНЧ 1-го порядка с коэффициентом 1/32
  ////===
  static uint8_t cnt_P_sum = 0;     // Счетчик полупериодов для суммирования отсчетов АЦП
  static uint16_t cnt_notzero = 0;  // Счетчик выборок АЦП без перехода через ноль
  //
  if ((!fl.zero) &&
    (U_adc >= 0) &&
    (Ufir <= 0) &&
    (U_adc != Ufir)) { //=======переход через ноль детектед=======
#ifdef Debug
    pin_ZeroOut_HIGH;
#endif
    cnt_notzero = 0;  // Обнуляем счетчик выборок АЦП без перехода через ноль
    fl.NotZero = 0;   // Снимаем флажок отсутствия детекции перехода через ноль
    //
    if (++cnt_P_sum == PSUM_MAX) { //===Проверка насуммированных отсчетов============================
      U_sum = sum; fl.sum = 1; sc_sum = sc; // Насуммированное готово к обработке
      sc = 0; sum = 0; cnt_P_sum = 0;     // Сбрасываем счетчик, сумматор и счетчик полупериодов
    } //===Проверка насуммированных отсчетов============================
    TM2_current = TM2_tmp;  // Запомним значение для дальнейшей обработки
    fl.zero = 1;
  }
  //
  else { //=======переход через ноль  NOT детектед=======
#ifdef Debug
    pin_ZeroOut_LOW;
#endif
    fl.zero = 0;
    if (++cnt_notzero == ZSUM_MAX) {  // Насуммировали достаточно
      fl.NotZero = 1; cnt_notzero = 0;
      PID_ust = LINE_FREQ;
      stop_razgon();
      pdm = 0; fl.Tout = 0; //выключим твердотельное реле
      U_real = 0; sc = 0; sum = 0; cnt_P_sum = 0; // Обнулим счетчик, сумматор, счетчик полупериодов и значение напряжения
      fl.dspNewData = 1;
    }
   //
  }
  //
  Ufir = U_adc;
}
  //===детекция перехода через ноль и ПИД-синхронизация=================================
  //
  sei(); // Следующие фрагменты длительны, но не требуют атомарности; разрешим прерывания
  //
  if (fl.zero) {//===ПИД-подстройка частоты внутреннего таймера к частоте сети===
  static uint16_t PID_reg = PID_ust << Km;   // Функция управления ПИД
  static int32_t PID_err_old = 0;            // Разность фаз из предыдущего шага
  static int32_t PID_int = 0;                // Интегральная составляющая из предыдущего шага
  int32_t temp_32 = (TM2_current + PHASE) << Km; // Разность фаз
  if (!fl.PP_tm) {
    temp_32 -= PID_reg + (1 << Km);  // Разность фаз должна быть с соответствующим знаком
  }
  PID_int += (temp_32 >> Ki);                 // Считаем интегральную составляющую
  PID_reg += temp_32 >> Kp;                   // Считаем новую функцию управления
  PID_reg += PID_int;
  PID_reg += ( temp_32 - PID_err_old ) >> Kd;
  PID_err_old = temp_32;
  // Готовим данные для записи в регистр сравнения таймера 2
  if ( PID_reg > (T_MAX << Km)) {
    PID_reg = (T_MAX << Km);    // Ограничим сверху
  }
  else if ( PID_reg < (T_MIN << Km)) {
    PID_reg = (T_MIN << Km);  // Ограничим снизу
  }
    temp_32 = PID_reg >> (Km - 1);// ...и правильно округлим
    temp_32++;                  // используя уже не нужную в этой подпрограмме
    PID_ust = temp_32 / 2;    // переменную temp_32
    //
  }//===ПИД-подстройка частоты внутреннего таймера к частоте сети===
  //
#ifdef Debug
  pin_DebugOut_LOW; //ОТЛАДКА
#endif
}//===============================Обработчик окончания преобразования АЦП===================
//
//
void RefreshMenu (void) { //===============Подпрограмма обновления меню===================
    ASOled.clearDisplay();
    ASOled.printString_6x8(F("Ст Принять и записать"), 0, 6);
    ASOled.printString_6x8(F("Рз Принять без записи"), 0, 7);
    ASOled.printString_6x8(F("Управление:"), X_position (1), 3);
    ASOled.printString_6x8(F("P- Выбор"), 0, 4);
#ifdef INTERFACE_ALT

#else
    ASOled.printString_6x8(F("Выберите"), 0, 0);
#endif
    //
    switch (cnt_dspMenu) { //=====Проверяем режимы меню
        case DSP::Pnom_MENU:  {  //=============Если мы в начальном меню, то...
#ifdef INTERFACE_ALT
          ASOled.printString_6x8(F("В"), X_position (20), 0);
          ASOled.printNumber((long)U_LINE, X_position (16), 0);
          ASOled.printString_6x8(F("Рном=         Вт"), 0, 1);
#else
          ASOled.printString_6x8(F("/введите Рном"), X_position (8), 0);
          ASOled.printString_6x8(F("Рном=      Вт, (   В)"), 0, 1);
          ASOled.printNumber((long)U_LINE, X_position (16), 1);
#endif
          ASOled.printString_6x8(F("==Мощность нагрузки=="), 0, 2);
          ASOled.printString_6x8(F("/уменьшение"), X_position (8), 4);
          ASOled.printString_6x8(F("P+ Увеличение"), 0, 5);
          break;
          }
          //
        case DSP::Pust_MENU:  {  //=============Если мы в меню выбора уставки, то...
#ifdef INTERFACE_ALT
          ASOled.printString_6x8(F("Руст=         Вт"), 0, 1);
#else
          ASOled.printString_6x8(F("уставку"), X_position (9), 0);
          ASOled.printString_6x8(F("Руст=      Вт"), 0, 1);
#endif
          ASOled.printString_6x8(F("=======Уставка======="), 0, 2);
          ASOled.printString_6x8(F("P+ Выбор"), 0, 5);
          if (!fl.writable) {  // Если уставки не пишутся в EEPROM, то...
            ASOled.printString_6x8(F("Ст Принять без записи"), 0, 6);
          }
          break;
          }
          //
        default: {
        }
    }
} //===============Подпрограмма обновления меню===================
//
//==============Подпрограмма печати строки минусов================
//==str - номер строки, куда печатать минуса
//==
void Asoled_printstring_6x8_minus (const uint8_t str) {
  ASOled.printString_6x8(F("---------------------"), 0, str);
}
//===========Подпрограмма печати строки минусов(конец)============
//
void setup(void) {
//
  cnt_dspMenu = DSP::Pnom_MENU; // Сначала - начальное меню
  Pins_init();    // Инициализируем входы/выходы
  ADC_init();     // Инициализируем АЦП
  Timers_init();  // Инициализируем таймеры
  sei();          // Разрешаем глобальные прерывания
  //
//
  pp_Delay(20);   // Подождем 20 полупериодов
//
#ifdef DisplayReset
  pin_OLEDres_HIGH; // Разрешаем работу дисплея
#endif
//
  pp_Delay(10);   // Подождем 10 полупериодов для гарантированного разрешения
  //
  ASOled.init();                   	// Инициализируем OLED дисплей
  //ASOled.SetTurnedOrientation();	// Переворачиваем OLED дисплей
  //
  #ifdef LOGO
  ASOled.clearDisplay();                // Очищаем, иначе некорректно работает для дисплеев на SH1106 (косяк библиотеки)
  ASOled.drawBitmap(logo, 0, 0, 128, 8);// Даём заставочку
  pp_Delay(400);                        // Подождем 400 полупериодов, пережидаем переходные процессы и любуемся заставкой
  #endif
  //
  ASOled.clearDisplay();  // Очищаем, иначе некорректно работает для дисплеев на SH1106 (косяк библиотеки)
  ASOled.printString_6x8(F("Стабилизатор мощности"), X_centred (21), 0);
  ASOled.printString_6x8(F("ТЭНа"), X_centred (4), 1);
  #ifdef INTERFACE_ALT
    ASOled.printString_12x16(F("STAB-AVR"), X_centred (16), 2);
  #else
    ASOled.printString_6x8(F("STAB-AVR"), X_centred (8), 2);
  #endif
  ASOled.printString_6x8(F(VERSION), X_centred (VERSION_LEN), 4);
  ASOled.printString_6x8(F("JohnJohnov"), X_centred (10), 6);
  ASOled.printString_6x8(F("alcodistillers.ru"), X_centred (17), 7);
  //
  read_Pnoms_from_EEPROM();  // Прочитаем из EEPROM записанные номиналы ТЭНов
  //
  pp_Delay(800);        // Подождем 600 полупериодов, пережидаем переходные процессы и любуемся заставкой
  //
  fl.dspRefresh = 1;

#ifdef USE_USART//++++++++++++++++USART initialization++++++++++++++++++++++++++++
//Если задействовано управление регулятором ТЭНа через UART, инициализируем оный
USART_start();
//
#endif // USE_USART
}
//
void loop(void) {
  if (fl.sum) { //==========Обработка данных от АЦП и корректировка выдаваемой мощности============
#ifdef NOT_LM358
    // 0,(5) - Коэффициент нормирования ((380/512)^2, 380В максимальное амплитудное) для Rail-to-Rail операционника
    U_sum /= 9;			// Нормируем сумму квадратов среднеквадратичного...
    U_sum *= 5;			// ...целочисленно
#else
    // 3 - Коэффициент нормирования ((380/220)^2, 380В максимальное амплитудное) для стандартно установленного LM358
    U_sum *= 3;         //Нормированная сумма квадратов среднеквадратичного
#endif
    U_sum /= sc_sum;    //Нормированный квадрат среднеквадратичного
    //=====Корректируем pdm
    //
    {
		uint16_t u_line_q = U_LINE * U_LINE;
		uint16_t tmp = calc_proportion(PDMust, u_line_q, U_sum);
		//
		if (tmp > CICLE || fl.razg) { // Следим, чтобы pdm не превышала CICLE
		  pdm = CICLE;
		  fl.Ulow = !fl.razg; // Или напряжение сети не позволяет выдать установленный уровень мощности, или разгон
		}
		else {
		  fl.Ulow = 0;
		  pdm = tmp;
		}
    }

    // Проверяем величину напряжения
    U_sum *= (uint32_t)400;               // Произведем некоторое математическое колдунство,
    U_sum = sqrt(U_sum);        // чтобы получить один знак после запятой без float
    U_sum++; U_sum /= 2;    // и с правильным округлением.
    U_real_dec = U_sum % 10;    // Среднеквадратичное (дробная часть)
    U_real = U_sum / 10;        // Среднеквадратичное (целая часть)
    //
    // Контролируем значение
    if ( U_real < U_MIN ) { //Действующее напряжение сети ниже U_MIN - отключим ТЭН (авария)
      fl.Udown = 1;       //поставим флажок низкого сетевого
      stop_razgon();
      pdm = 0;            //выключим твердотельное реле
    }
    else {
      fl.Udown = 0;
    }
    fl.sum = 0;
    fl.dspNewData = 1;  //Обновление информации на дисплее
  } //======================Обработка данных от АЦП и корректировка выдаваемой мощности============
  //
  #ifdef USE_ADprotocol
  if (fl.uartReport && fl.uartUnhold) {  //==========Отправка отчета внешнему контроллеру============
    USART_report();
    fl.uartReport = 0;
  } //=========================Отправка отчета внешнему контроллеру============
  #endif
  //
  if (fl.dspNewData) { //========================Вывод информации на дисплей=============
    if (fl.dspRefresh) {
      RefreshMenu(); //Обновляем дисплей, если надо
    }
    //
    switch (cnt_dspMenu) { //=====Проверяем режимы меню
      case DSP::Pnom_MENU:  {  //=============Если мы в начальном меню, то...
        static uint16_t Pnomold = 0;
        if (!Pnom || Pnom > 9999) {
          Pnomold = Pnom;
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("****"), X_position (3,0,12), 0);
#else
          ASOled.printString_6x8(F("****"), X_position (6), 1);
#endif
//
        }
        else if ((Pnomold != Pnom) || fl.dspRefresh) {
          Pnomold = Pnom;
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("    "), X_position (3,0,12), 0);
          ASOled.printNumber((long)Pnom, X_position (6,Pnom,12), 0);
#else
          ASOled.printString_6x8(F("    "), X_position (6), 1);
          ASOled.printNumber((long)Pnom, X_position (9,Pnom), 1);
#endif
          //
        }
        fl.dspRefresh = 0;
        break;
      }
      case DSP::Pust_MENU:  {  //=============Если мы в меню выбора уставки, то...
        static uint16_t PDMold = 0;
        if ((PDMold != PDMset[0][cnt_PDMcount]) || fl.dspRefresh) {
          PDMold = PDMset[0][cnt_PDMcount];
          uint16_t p = calc_proportion(PDMold); // Считаем уставку с округлением
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("    "), X_position (3,0,12), 0);
          ASOled.printNumber((long)p, X_position (6,p,12), 0);
#else
          ASOled.printString_6x8(F("    "), X_position (6), 1);
          ASOled.printNumber((long)p, X_position (9,p), 1);
#endif
          if (PDMset[1][cnt_PDMcount]) {  // Если значение записано в EEPROM
            ASOled.printString_6x8(F("R"), X_position (20), 1);     // поставим значок
          }
          else {
          ASOled.printString_6x8(F(" "), X_position (20), 1);   // а если не записано - уберем
          }
        }
        fl.dspRefresh = 0;
        break;
      }
      default: {  //=============А если не в меню, то...
#ifdef INTERFACE_ALT
    #define str_Ureal_big 0
      #define str_Ureal 1
    #define str_ust_big 3
    #define str_ust 4
        #define str_Ustat 2
    #define str_Razgon 5
        #define str_Pnom 6
        #define str_Relay 7
#else
    #define str_Ureal 0
        #define str_Ustat 1
        #define str_ust 3
        #define str_Pnom 6
        #define str_Razgon 4
        #define str_Relay 7
#endif
        //
        if (fl.dspRefresh) {  //Обновляем дисплей
          ASOled.clearDisplay();
#ifdef INTERFACE_ALT
          ASOled.printString_6x8(F("Вт       ,  %"), X_position (8), str_ust);
#else
          ASOled.printString_6x8(F("Руст      Вт;    ,  %"), 0, str_ust);
#endif
          ASOled.printString_6x8(F("Напр.сети        ,  В"), 0, str_Ureal);
          ASOled.printString_6x8(F("Ном. мощность      Вт"), 0, str_Pnom);
          ASOled.printNumber((long)Pnom, X_position (17,Pnom), str_Pnom);
//          ASOled.printString_6x8(F("Реле "), X_position (0), str_Relay);
        }
        //
        static uint16_t U_real_old = 0;
        if ((U_real_old != U_real) || fl.dspRefresh) {
          U_real_old = U_real;
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("   "), X_position (7,100,12) + 5, str_Ureal_big);
          ASOled.printNumber((long)U_real_old, X_position (7,U_real_old,12) + 5, str_Ureal_big);
#else
          ASOled.printString_6x8(F("    "), X_position (13), str_Ureal);
          ASOled.printNumber((long)U_real_old, X_position (16,U_real_old), str_Ureal);
#endif
        }
        static uint8_t U_real_dec_old = 0;
        if ((U_real_dec_old != U_real_dec) || fl.dspRefresh) {
          U_real_dec_old = U_real_dec;
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F(" "), X_position (9,0,12), str_Ureal_big);
          ASOled.printNumber((long)U_real_dec_old, X_position (9,0,12), str_Ureal_big);
#else
          ASOled.printNumber((long)U_real_dec_old, X_position (18), str_Ureal);
#endif
        }
        static uint16_t Pust_old = 0;
        if ((Pust_old != Pust) || fl.dspRefresh) {
        	Pust_old = Pust;	//
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("    "), 0, str_ust_big);
          ASOled.printNumber((long)Pust_old, X_position (3,Pust_old,12), str_ust_big);
#else
          ASOled.printString_6x8(F("    "), X_position (5), str_ust);
          ASOled.printNumber((long)Pust_old, X_position (8,Pust_old), str_ust);
#endif
//
        }
        //
        static uint16_t PDMust_old = 0;
        if ((PDMust_old != PDMust) || fl.dspRefresh) {
          PDMust_old = PDMust;
          uint32_t x = 1000*(uint32_t)PDMust_old;
          x /= CICLE;
          uint8_t percent = x / 10;    // посчитаем процент
          uint8_t percent_dec = x % 10;// посчитаем десятые процента
#ifdef INTERFACE_ALT
          ASOled.printString_12x16(F("   "), X_position (7,100,12) + 5, str_ust_big);
          ASOled.printNumber((long)(percent), X_position (7,percent,12) + 5, str_ust_big);
          ASOled.printNumber((long)(percent_dec), X_position (9,0,12), str_ust_big);
#else
          ASOled.printString_6x8(F("    "), X_position (13), str_ust);
          ASOled.printNumber((long)(percent), X_position (16,percent), str_ust);
          ASOled.printNumber((long)(percent_dec), X_position (18), str_ust);
#endif
        }
        //
        if (fl.Udown || fl.NotZero) {
          ASOled.printString_6x8(F("-----Авария сети-----"), 0, str_Ustat);
        }
        else if (fl.Ulow) {
          ASOled.printString_6x8(F("--Недост.напр. сети--"), 0, str_Ustat);
        }
        else {
          Asoled_printstring_6x8_minus(str_Ustat);
        }
      //
        if (fl.razg_on) {
          static uint8_t count_1 = 0;
          ASOled.printString_6x8(F("------<Разгон!>------"), 0, str_Razgon);
          ASOled.printString_6x8(F("<"), X_position (5 - count_1), str_Razgon);
          ASOled.printString_6x8(F(">"), X_position (15 + count_1), str_Razgon);
          if (++count_1 > 5) count_1 = 0;
          //
        }
        else {
          Asoled_printstring_6x8_minus(str_Razgon);
        }
        //
        {
          static uint8_t trigger = 1;
          if (trigger && fl.stab_off) {
            ASOled.printString_6x8(F("!!АВАРИЙНЫЙ ОСТАНОВ!!"), 0, str_Relay);
            trigger = 0;
          }
          else {
            Asoled_printstring_6x8_minus(str_Relay);
            trigger = 1;
          }
        }
        //
		#ifdef Debug
			if (flag_OVF) {
				ASOled.printNumber((long)(1), X_position (20), str_Relay);
				flag_OVF = 0;
			}
			else ASOled.printNumber((long)(0), X_position (20), str_Relay);
		#endif
        //
        fl.dspRefresh = 0;
      }
    }
  //
      fl.dspNewData = 0;
 }//========================Вывод информации на дисплей=============
  //
  #ifdef USE_USART
  if (fl.uartUnhold) {
  USART_parser();
  }
  #endif
}
//
