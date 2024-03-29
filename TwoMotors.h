/*
  Autor: Hugo Max M. Teixeira
  Data: 05/2021

  Para o controle de dois motores com encoder;
  Arquivo .h (de cabeçalho)

*/

#ifndef TwoMotors_h
#define TwoMotors_h

#include <DC_motor_controller.h>

class TwoMotors{
public:

	DC_motor_controller* m1 =new DC_motor_controller();
	DC_motor_controller* m2 =new DC_motor_controller();

	TwoMotors(DC_motor_controller* motor1, DC_motor_controller* motor2);
	TwoMotors(DC_motor_controller &motor1, DC_motor_controller &motor2);
	void together(float vel);
	void together(float vel, float rot);
	void together(float vel1, float rot1, float vel2, float rot2);
	void setGyreDegreesRatio(float rot, float ang);
	void turnDegree(float vel, float deg);
	void stop(unsigned int time=100);
	void stop_vel(unsigned int vel1, unsigned int vel2);
	void reset();
	void run(int pwm);
	void run(int pwm1, int pwm2);
	void ifNegativeAllNegative(float &val_1, float &val_2);

private:
	float rot_per_degree;
	unsigned int stop_time[2];
	bool can_stop_vel = false;
	void resetMotors();
};

#endif
