/*
 * main.c
 *
 *  Created on: 10.12.2018
 *      Author: max
 */

// Atmega328P
// 16MHz

#include <avr/io.h>
//#include <util/delay.h>
#include <avr/interrupt.h>

#define LED_A_BIT PC3 //выход информационного светодиода А
#define LED_A_DDR DDRC
#define LED_A_PORT PORTC

#define LED_B_BIT PC2 //выход информационного светодиода В
#define LED_B_DDR DDRC
#define LED_B_PORT PORTC

//сигнальный вход A
#define INPUT_A_BIT PC0
#define INPUT_A_DDR DDRC
#define INPUT_A_PORT PORTC
#define INPUT_A_PIN PINC

//сигнальный вход B
#define INPUT_B_BIT PB0
#define INPUT_B_DDR DDRB
#define INPUT_B_PORT PORTB
#define INPUT_B_PIN PINB

//Порты мотора A
//ШИМ
#define MOTOR_A_PWM_BIT PD5
#define MOTOR_A_PWM_DDR DDRD
#define MOTOR_A_PWM_PORT PORTD
#define MOTOR_A_PWM_PIN PIND

//Направление вращения
#define MOTOR_A_CW_BIT PB1
#define MOTOR_A_CW_DDR DDRB
#define MOTOR_A_CW_PORT PORTB
#define MOTOR_A_CW_PIN PINB

//Вход датчика хола
#define MOTOR_A_FG_BIT PD3
#define MOTOR_A_FG_DDR DDRD
#define MOTOR_A_FG_PORT PORTD
#define MOTOR_A_FG_PIN PIND

#define MOTOR_A_PWM_TIMER_REG OCR0B

//Порты мотора B
//ШИМ
#define MOTOR_B_PWM_BIT PD6
#define MOTOR_B_PWM_DDR DDRD
#define MOTOR_B_PWM_PORT PORTD
#define MOTOR_B_PWM_PIN PIND

//Направление вращения
#define MOTOR_B_CW_BIT PB2
#define MOTOR_B_CW_DDR DDRB
#define MOTOR_B_CW_PORT PORTB
#define MOTOR_B_CW_PIN PINB

//Вход датчика хола
#define MOTOR_B_FG_BIT PD2
#define MOTOR_B_FG_DDR DDRD
#define MOTOR_B_FG_PORT PORTD
#define MOTOR_B_FG_PIN PIND

#define MOTOR_B_PWM_TIMER_REG OCR0A

//значение старшего байта(3-й байт) счетчика сигнала, если до него досчитал,
//т.е. Timer = (NoPWM_TimerCount)(00)(00) то считаем что PWM сигнала нет ~65ms (стандартный период 20ms)
#define NO_SIGNAL_TIMER_COUNT 2

//длительности сигналов при счетчике таймера 16000000/8 = 2000000 тиков
#define SIGNAL_LEN_MINMIN 1600 //0.8 ms
#define SIGNAL_LEN_MIN 2000 //1 ms
#define SIGNAL_LEN_MED 3000 //1.5ms
#define SIGNAL_LEN_MAX 4000 //2 ms
#define SIGNAL_LEN_MAXMAX 4400 //2.2 ms
#define SIGNAL_LEN_DIFF 100 // ms

#define PWM_DEAD_ZONE 20 //мертвая зона ШИМ

//количество суммирований сигнала, степень двойки (0(1), 1(2), 2(4), 3(8), 4(16), 5(32)), для дальнейшего усреднения
#define SUMM_LEN_AMOUNT 2

typedef struct {
	uint8_t init; //состояние инициализации, т.е. должен стартовать со среднего сигнала
	uint8_t initCount; //счетчик инициализации
	uint8_t timerHiByte; //старший байт таймера
	uint16_t startSignal; //время начала сигнала в тиках таймера
	uint32_t lenSignal; //длина сигнала в тиках таймера
	uint32_t summLenSignal; //переменная для суммирования длинн сигнала
	uint8_t summCount; //счетчик суммирования
	uint8_t noSignalCount; //счетчик отсутствия сигнала
	volatile uint8_t* pinSignal; //пин входного сигнала
	uint8_t bitSignalMask; //маска бит входного сигнала
	volatile uint8_t* portLed; //порт светодиода
	uint8_t bitLedMask; //маска бит светодиода
	volatile uint8_t* timerPWMReg;		//канал таймера
	volatile uint8_t* portCw; //порт выхода направления вращения
	volatile uint8_t* pinCw; //пин выхода направления вращения
	uint8_t bitCwMask; //маска бит выхода направления вращения
	volatile uint8_t* pinFg; //пин входа датчика холла
	uint8_t bitFgMask; //маска бит входа датчика холла
	int32_t odomCount; //счетчик одометрии
} MotorData;

void SetSpeedMotor(MotorData *data);
void TimerProcess(MotorData *data);
void SignalStateChange(MotorData *data);
void OdometrProcess(MotorData *data);
void StopMotor(MotorData *data);

MotorData motorA;
MotorData motorB;

ISR(TIMER1_OVF_vect)
{
	TimerProcess(&motorA);
	TimerProcess(&motorB);
}

//прерывание по изменению состояния управляющего входа B
ISR(PCINT0_vect)
{
	SignalStateChange(&motorB);
}

//прерывание по изменению состояния управляющего входа A
ISR(PCINT1_vect)
{
	SignalStateChange(&motorA);
}

//прерывание по изменению состояния входа от датчика хола A
ISR(INT0_vect)
{
	OdometrProcess(&motorA);
}

//прерывание по изменению состояния входа от датчика хола B
ISR(INT1_vect)
{
	OdometrProcess(&motorB);
}


void TimerProcess(MotorData *data)
{
	data->timerHiByte++;
	if (data->timerHiByte == NO_SIGNAL_TIMER_COUNT)
	{
		data->timerHiByte = 0;
		data->noSignalCount++;
		if (data->noSignalCount == 8) //~500 ms
		{
			*data->portLed ^= data->bitLedMask; //инвертируем светодиод
			data->noSignalCount = 0;

			StopMotor(data); //останавливаем мотор
		}
	}
}

void SignalStateChange(MotorData *data)
{
	//смотрим состояние входа
	if (*data->pinSignal & data->bitSignalMask) //если Лог1, начало сигнала
	{
		data->timerHiByte = 0; //обнулил старший байт таймера

		cli();
		data->startSignal = TCNT1; //запомнили счетчик таймера
		sei();
	}
	else //Лог0, конец сигнала
	{
		cli();
		uint32_t endSignal = ((uint32_t)data->timerHiByte << 16) + TCNT1; //окончание сигнала
		sei();
		uint32_t lenSignal = endSignal - data->startSignal; //длина сигнала = время окончания сигнала - время начала сигнала

		//проверяем длину сигнала на допустимые значения
		if ((lenSignal > SIGNAL_LEN_MINMIN) && (lenSignal < SIGNAL_LEN_MAXMAX))
		{

			data->summLenSignal += lenSignal;
			data->summCount++;

			if (data->summCount == (1 << SUMM_LEN_AMOUNT))
			{
				data->lenSignal = (data->summLenSignal >> SUMM_LEN_AMOUNT); //усредняем
				data->summLenSignal = 0;
				data->summCount = 0;

				if (data->init)
				{
					*data->portLed ^= data->bitLedMask; //инвертируем светодиод; //инвертируем светодиод
					SetSpeedMotor(data); //задаем скорость мотору
				}
				else
				{
					//если сигнал в середине диапазона
					if ((lenSignal > (SIGNAL_LEN_MED - SIGNAL_LEN_DIFF)) && (lenSignal < (SIGNAL_LEN_MED + SIGNAL_LEN_DIFF)))
					{
						data->initCount++;
						if (data->initCount == 12) //примерно секунда
						{
							data->init = 1;
							data->initCount = 0;
						}
						*data->portLed |= data->bitLedMask; //зажигаем светодиод

					}
					else
					{
						data->initCount = 0;
						*data->portLed &= ~data->bitLedMask; //гасим светодиод
					}
				}
			}
		}
		else //если сигнал не проходит по параметрам
		{
			*data->portLed &= ~data->bitLedMask; //гасим светодиод
			//StopMotor(data);

		}
	}
}

void SetSpeedMotor(MotorData *data)
{
	uint16_t len;
	//нормализуем значение
	if (data->lenSignal > SIGNAL_LEN_MED)
	{
		(*data->portCw) |= data->bitCwMask; //задаем напраление вращения
		len = data->lenSignal - SIGNAL_LEN_MED;
		if (len > (SIGNAL_LEN_MED - SIGNAL_LEN_MIN))
			len = SIGNAL_LEN_MED - SIGNAL_LEN_MIN;
	}
	else
	{
		(*data->portCw) &= ~data->bitCwMask; //задаем напраление вращения
		len = SIGNAL_LEN_MED - data->lenSignal;
		if (len > (SIGNAL_LEN_MED - SIGNAL_LEN_MIN))
			len = SIGNAL_LEN_MED - SIGNAL_LEN_MIN;
	}

	// длина сигнала 2000тик, т.е. от среднего значения +-1000
	// в шим регистр надо запихать 0..255
	// 1000/4 = 250, поеэтому добавляю 1 на каждые 50 ед.
	uint8_t pwm = (len >> 2) + (len >> 2)/50;
	if (pwm < PWM_DEAD_ZONE)
		pwm = 0;

	*data->timerPWMReg = pwm; //задаем значение ШИМ в регистр таймера

}

void StopMotor(MotorData *data)
{
	*data->timerPWMReg = 0; //задаем значение ШИМ в регистр таймера
}


void OdometrProcess(MotorData *data)
{
	if (*data->portCw & data->bitCwMask) //в зависимости от направления вращения
		data->odomCount++; //увеличиваем счетчик одометра
	else
		data->odomCount--; //уменьшаем счетчик одометра
}

void ResetMotorData(MotorData *data)
{
	data->init = 0;
	data->initCount = 0;
	data->timerHiByte = 0; //старший байт таймера
	data->startSignal = 0; //время начала сигнала в тиках таймера
	data->lenSignal = 0; //длина сигнала в тиках таймера
	data->summLenSignal = 0; //переменная для суммирования длинн сигнала
	data->summCount = 0; //счетчик суммирования
	data->noSignalCount = 0; //счетчик отсутствия сигнала
	data->odomCount = 0; //счетчик одометрии
}

int main(void)
{
	//инициализируем таймер 0 для генерации ШИМ управления оборотами двигателя
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //подключаем ноги к выходам, режим FastPWM
	TCCR0B = (3 << CS00); //делитель 64

	MOTOR_A_PWM_TIMER_REG = 0; //ШИМ = 0
	MOTOR_B_PWM_TIMER_REG = 0;

	//инициализируем таймер 1 для счетчика времени при захвате шим
	TCCR1A = 0;
	TCCR1B = (2 << CS10); //16000000/8 = 2000000 тиков таймера в сек
	TCCR1C = 0;
	TIMSK1 = (1 << TOIE1); //прерывания по переполнению

	//подключение входов и разрешение прерываний для захвата ШИМ
	PCMSK0 = (1 << PCINT0); //Вход Signal_B
	PCMSK1 = (1 << PCINT8); //Вход Signal_A
	PCICR = (1 << PCIE1) | (1 << PCIE0); //разрешаем прерывания по изменению состояния входа

	//подключение входов и разрешение прерываний для входов датчика холла
	EICRA = (3 << ISC10) | (3 << ISC00); //ловим восходящий фронт
	EIMSK = (1 << INT1) | (1 << INT0); //разрешаем прерывания для входов INT1, INT0

	//конфигурируем порты IO
	LED_A_DDR |= (1 << LED_A_BIT);
	LED_B_DDR |= (1 << LED_B_BIT);

	MOTOR_A_PWM_DDR |= (1 << MOTOR_A_PWM_BIT);
	MOTOR_A_CW_DDR |= (1 << MOTOR_A_CW_BIT);

	MOTOR_B_PWM_DDR |= (1 << MOTOR_B_PWM_BIT);
	MOTOR_B_CW_DDR |= (1 << MOTOR_B_CW_BIT);

	//Заполняем структуры
	ResetMotorData(&motorA);
	motorA.pinSignal = &INPUT_A_PIN;
	motorA.bitSignalMask = (1 << INPUT_A_BIT);
	motorA.portLed = &LED_A_PORT;
	motorA.bitLedMask = (1 << LED_A_BIT);
	motorA.portCw = &MOTOR_A_CW_PORT;
	motorA.pinCw = &MOTOR_A_CW_PIN;
	motorA.bitCwMask = (1 << MOTOR_A_CW_BIT);
	motorA.timerPWMReg = &MOTOR_A_PWM_TIMER_REG;
	motorA.pinFg = &MOTOR_A_FG_PIN;
	motorA.bitFgMask = (1 << MOTOR_A_FG_BIT);

	ResetMotorData(&motorB);
	motorB.pinSignal = &INPUT_B_PIN;
	motorB.bitSignalMask = (1 << INPUT_B_BIT);
	motorB.portLed = &LED_B_PORT;
	motorB.bitLedMask = (1 << LED_B_BIT);
	motorB.portCw = &MOTOR_B_CW_PORT;
	motorB.pinCw = &MOTOR_B_CW_PIN;
	motorB.bitCwMask = (1 << MOTOR_B_CW_BIT);
	motorB.timerPWMReg = &MOTOR_B_PWM_TIMER_REG;
	motorB.pinFg = &MOTOR_B_FG_PIN;
	motorB.bitFgMask = (1 << MOTOR_B_FG_BIT);

	sei(); //разрешаем прерывания

	while(1);

	return 0;
}
