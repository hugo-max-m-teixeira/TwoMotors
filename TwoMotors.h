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
	void together(float vel);
	void together(float vel, float rot);
	void together(float vel1, float rot1, float vel2, float rot2);
	void setGyreDegreesRatio(float rot, float ang);
	void turnDegree(float vel, float deg);

private:
	float rot_per_degree;
};

#endif
