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

#define LED_A_BIT PC3 //����� ��������������� ���������� �
#define LED_A_DDR DDRC
#define LED_A_PORT PORTC

#define LED_B_BIT PC2 //����� ��������������� ���������� �
#define LED_B_DDR DDRC
#define LED_B_PORT PORTC

//���������� ���� A
#define INPUT_A_BIT PC0
#define INPUT_A_DDR DDRC
#define INPUT_A_PORT PORTC
#define INPUT_A_PIN PINC

//���������� ���� B
#define INPUT_B_BIT PB0
#define INPUT_B_DDR DDRB
#define INPUT_B_PORT PORTB
#define INPUT_B_PIN PINB

//����� ������ A
//���
#define MOTOR_A_PWM_BIT PD5
#define MOTOR_A_PWM_DDR DDRD
#define MOTOR_A_PWM_PORT PORTD
#define MOTOR_A_PWM_PIN PIND

//����������� ��������
#define MOTOR_A_CW_BIT PB1
#define MOTOR_A_CW_DDR DDRB
#define MOTOR_A_CW_PORT PORTB
#define MOTOR_A_CW_PIN PINB

//���� ������� ����
#define MOTOR_A_FG_BIT PD3
#define MOTOR_A_FG_DDR DDRD
#define MOTOR_A_FG_PORT PORTD
#define MOTOR_A_FG_PIN PIND

#define MOTOR_A_PWM_TIMER_REG OCR0B

//����� ������ B
//���
#define MOTOR_B_PWM_BIT PD6
#define MOTOR_B_PWM_DDR DDRD
#define MOTOR_B_PWM_PORT PORTD
#define MOTOR_B_PWM_PIN PIND

//����������� ��������
#define MOTOR_B_CW_BIT PB2
#define MOTOR_B_CW_DDR DDRB
#define MOTOR_B_CW_PORT PORTB
#define MOTOR_B_CW_PIN PINB

//���� ������� ����
#define MOTOR_B_FG_BIT PD2
#define MOTOR_B_FG_DDR DDRD
#define MOTOR_B_FG_PORT PORTD
#define MOTOR_B_FG_PIN PIND

#define MOTOR_B_PWM_TIMER_REG OCR0A

//�������� �������� �����(3-� ����) �������� �������, ���� �� ���� ��������,
//�.�. Timer = (NoPWM_TimerCount)(00)(00) �� ������� ��� PWM ������� ��� ~65ms (����������� ������ 20ms)
#define NO_SIGNAL_TIMER_COUNT 2

//������������ �������� ��� �������� ������� 16000000/8 = 2000000 �����
#define SIGNAL_LEN_MINMIN 1600 //0.8 ms
#define SIGNAL_LEN_MIN 2000 //1 ms
#define SIGNAL_LEN_MED 3000 //1.5ms
#define SIGNAL_LEN_MAX 4000 //2 ms
#define SIGNAL_LEN_MAXMAX 4400 //2.2 ms
#define SIGNAL_LEN_DIFF 100 // ms

#define PWM_DEAD_ZONE 20 //������� ���� ���

//���������� ������������ �������, ������� ������ (0(1), 1(2), 2(4), 3(8), 4(16), 5(32)), ��� ����������� ����������
#define SUMM_LEN_AMOUNT 2

typedef struct {
	uint8_t init; //��������� �������������, �.�. ������ ���������� �� �������� �������
	uint8_t initCount; //������� �������������
	uint8_t timerHiByte; //������� ���� �������
	uint16_t startSignal; //����� ������ ������� � ����� �������
	uint32_t lenSignal; //����� ������� � ����� �������
	uint32_t summLenSignal; //���������� ��� ������������ ����� �������
	uint8_t summCount; //������� ������������
	uint8_t noSignalCount; //������� ���������� �������
	volatile uint8_t* pinSignal; //��� �������� �������
	uint8_t bitSignalMask; //����� ��� �������� �������
	volatile uint8_t* portLed; //���� ����������
	uint8_t bitLedMask; //����� ��� ����������
	volatile uint8_t* timerPWMReg;		//����� �������
	volatile uint8_t* portCw; //���� ������ ����������� ��������
	volatile uint8_t* pinCw; //��� ������ ����������� ��������
	uint8_t bitCwMask; //����� ��� ������ ����������� ��������
	volatile uint8_t* pinFg; //��� ����� ������� �����
	uint8_t bitFgMask; //����� ��� ����� ������� �����
	int32_t odomCount; //������� ���������
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

//���������� �� ��������� ��������� ������������ ����� B
ISR(PCINT0_vect)
{
	SignalStateChange(&motorB);
}

//���������� �� ��������� ��������� ������������ ����� A
ISR(PCINT1_vect)
{
	SignalStateChange(&motorA);
}

//���������� �� ��������� ��������� ����� �� ������� ���� A
ISR(INT0_vect)
{
	OdometrProcess(&motorA);
}

//���������� �� ��������� ��������� ����� �� ������� ���� B
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
			*data->portLed ^= data->bitLedMask; //����������� ���������
			data->noSignalCount = 0;

			StopMotor(data); //������������� �����
		}
	}
}

void SignalStateChange(MotorData *data)
{
	//������� ��������� �����
	if (*data->pinSignal & data->bitSignalMask) //���� ���1, ������ �������
	{
		data->timerHiByte = 0; //������� ������� ���� �������

		cli();
		data->startSignal = TCNT1; //��������� ������� �������
		sei();
	}
	else //���0, ����� �������
	{
		cli();
		uint32_t endSignal = ((uint32_t)data->timerHiByte << 16) + TCNT1; //��������� �������
		sei();
		uint32_t lenSignal = endSignal - data->startSignal; //����� ������� = ����� ��������� ������� - ����� ������ �������

		//��������� ����� ������� �� ���������� ��������
		if ((lenSignal > SIGNAL_LEN_MINMIN) && (lenSignal < SIGNAL_LEN_MAXMAX))
		{

			data->summLenSignal += lenSignal;
			data->summCount++;

			if (data->summCount == (1 << SUMM_LEN_AMOUNT))
			{
				data->lenSignal = (data->summLenSignal >> SUMM_LEN_AMOUNT); //���������
				data->summLenSignal = 0;
				data->summCount = 0;

				if (data->init)
				{
					*data->portLed ^= data->bitLedMask; //����������� ���������; //����������� ���������
					SetSpeedMotor(data); //������ �������� ������
				}
				else
				{
					//���� ������ � �������� ���������
					if ((lenSignal > (SIGNAL_LEN_MED - SIGNAL_LEN_DIFF)) && (lenSignal < (SIGNAL_LEN_MED + SIGNAL_LEN_DIFF)))
					{
						data->initCount++;
						if (data->initCount == 12) //�������� �������
						{
							data->init = 1;
							data->initCount = 0;
						}
						*data->portLed |= data->bitLedMask; //�������� ���������

					}
					else
					{
						data->initCount = 0;
						*data->portLed &= ~data->bitLedMask; //����� ���������
					}
				}
			}
		}
		else //���� ������ �� �������� �� ����������
		{
			*data->portLed &= ~data->bitLedMask; //����� ���������
			//StopMotor(data);

		}
	}
}

void SetSpeedMotor(MotorData *data)
{
	uint16_t len;
	//����������� ��������
	if (data->lenSignal > SIGNAL_LEN_MED)
	{
		(*data->portCw) |= data->bitCwMask; //������ ���������� ��������
		len = data->lenSignal - SIGNAL_LEN_MED;
		if (len > (SIGNAL_LEN_MED - SIGNAL_LEN_MIN))
			len = SIGNAL_LEN_MED - SIGNAL_LEN_MIN;
	}
	else
	{
		(*data->portCw) &= ~data->bitCwMask; //������ ���������� ��������
		len = SIGNAL_LEN_MED - data->lenSignal;
		if (len > (SIGNAL_LEN_MED - SIGNAL_LEN_MIN))
			len = SIGNAL_LEN_MED - SIGNAL_LEN_MIN;
	}

	// ����� ������� 2000���, �.�. �� �������� �������� +-1000
	// � ��� ������� ���� �������� 0..255
	// 1000/4 = 250, �������� �������� 1 �� ������ 50 ��.
	uint8_t pwm = (len >> 2) + (len >> 2)/50;
	if (pwm < PWM_DEAD_ZONE)
		pwm = 0;

	*data->timerPWMReg = pwm; //������ �������� ��� � ������� �������

}

void StopMotor(MotorData *data)
{
	*data->timerPWMReg = 0; //������ �������� ��� � ������� �������
}


void OdometrProcess(MotorData *data)
{
	if (*data->portCw & data->bitCwMask) //� ����������� �� ����������� ��������
		data->odomCount++; //����������� ������� ��������
	else
		data->odomCount--; //��������� ������� ��������
}

void ResetMotorData(MotorData *data)
{
	data->init = 0;
	data->initCount = 0;
	data->timerHiByte = 0; //������� ���� �������
	data->startSignal = 0; //����� ������ ������� � ����� �������
	data->lenSignal = 0; //����� ������� � ����� �������
	data->summLenSignal = 0; //���������� ��� ������������ ����� �������
	data->summCount = 0; //������� ������������
	data->noSignalCount = 0; //������� ���������� �������
	data->odomCount = 0; //������� ���������
}

int main(void)
{
	//�������������� ������ 0 ��� ��������� ��� ���������� ��������� ���������
	TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00); //���������� ���� � �������, ����� FastPWM
	TCCR0B = (3 << CS00); //�������� 64

	MOTOR_A_PWM_TIMER_REG = 0; //��� = 0
	MOTOR_B_PWM_TIMER_REG = 0;

	//�������������� ������ 1 ��� �������� ������� ��� ������� ���
	TCCR1A = 0;
	TCCR1B = (2 << CS10); //16000000/8 = 2000000 ����� ������� � ���
	TCCR1C = 0;
	TIMSK1 = (1 << TOIE1); //���������� �� ������������

	//����������� ������ � ���������� ���������� ��� ������� ���
	PCMSK0 = (1 << PCINT0); //���� Signal_B
	PCMSK1 = (1 << PCINT8); //���� Signal_A
	PCICR = (1 << PCIE1) | (1 << PCIE0); //��������� ���������� �� ��������� ��������� �����

	//����������� ������ � ���������� ���������� ��� ������ ������� �����
	EICRA = (3 << ISC10) | (3 << ISC00); //����� ���������� �����
	EIMSK = (1 << INT1) | (1 << INT0); //��������� ���������� ��� ������ INT1, INT0

	//������������� ����� IO
	LED_A_DDR |= (1 << LED_A_BIT);
	LED_B_DDR |= (1 << LED_B_BIT);

	MOTOR_A_PWM_DDR |= (1 << MOTOR_A_PWM_BIT);
	MOTOR_A_CW_DDR |= (1 << MOTOR_A_CW_BIT);

	MOTOR_B_PWM_DDR |= (1 << MOTOR_B_PWM_BIT);
	MOTOR_B_CW_DDR |= (1 << MOTOR_B_CW_BIT);

	//��������� ���������
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

	sei(); //��������� ����������

	while(1);

	return 0;
}
