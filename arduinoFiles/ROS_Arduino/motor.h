
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class motor
{
	int motorHizi, maxHiz;
	int PWMPin, in11, in22;
public:
	void hizAyarla(int);
	void geriAyarla();
	void duzAyarla();
	void dur();
	void maxHizAyarla(int);
	void ileri();
	motor(int,int,int);
	~motor();
};

#endif
