#include "motor.h"

motor::motor(int pwm = 0, int in1 = 0,int in2 = 0)
{
	PWMPin = pwm;
	in11 = in1;
	in22 = in2;
	maxHiz = 255;
	pinMode(PWMPin, OUTPUT);
	pinMode(in11, OUTPUT);
	pinMode(in22, OUTPUT);
	digitalWrite(in11, LOW);
	digitalWrite(in22, LOW);
	analogWrite(PWMPin, 0);
	motorHizi = 0;
}

void motor::dur()
{
	digitalWrite(in11, LOW);
	digitalWrite(in22, LOW);
	analogWrite(PWMPin, 0);
}

void motor::hizAyarla(int hiz)
{
	if (hiz > maxHiz) hiz = maxHiz;

	analogWrite(PWMPin, hiz);
}

void motor::duzAyarla()
{
	digitalWrite(in11, HIGH);
	digitalWrite(in22, LOW);
}

void motor::geriAyarla()
{
	digitalWrite(in11, LOW);
	digitalWrite(in22, HIGH);
}

void motor::ileri()
{
	analogWrite(PWMPin, maxHiz);
}

void motor::maxHizAyarla(int hiz){ maxHiz = hiz; }

motor::~motor(){}

