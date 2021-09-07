/*
  Autor: Hugo Max M. Teixeira
  Data: 05/2021

  Para o controle de dois motores com encoder;
  Arquivo .cpp

*/

#include <TwoMotors.h>	// Inclusão do arquivo .h da biblioteca

TwoMotors::TwoMotors(DC_motor_controller* motor1, DC_motor_controller* motor2){	//Construtor que recebe dois objetos do tipo DC_motor_controller (que serão sincronizados)
	this->m1=motor1; this->m2=motor2;
}

void TwoMotors::setGyreDegreesRatio(float rot, float ang){
	this-> rot_per_degree = rot/ang;
}

void TwoMotors::together(float vel){
	m1->walk(vel);
	m2->walk(vel);
}


void TwoMotors::together(float vel, float rot){ // Para a movimentação dos dois motores em uma mesma velocidade e com o mesmo valor de rotações
	if(rot != 0){
		m1->resetForGyrate();
		m2->resetForGyrate();
		while(m1->canRun() || m2->canRun()){
			m1->gyrate(vel, rot);
			m2->gyrate(vel, rot);
		}
		m1->run(0);
		m2->run(0);
	} else {
		m1->walk(vel);
		m2->walk(vel);
	}
}

void TwoMotors::together(float vel1, float rot1, float vel2, float rot2){	// Para a movimentação dos motores com velocidades e rotaçẽos diferentes
	m1->resetForGyrate();
	m2->resetForGyrate();
	while(m1->canRun() || m2->canRun()){
		m1->gyrate(vel1, rot1);
		m2->gyrate(vel2, rot2);
	}
	m1->run(0);
	m2->run(0);
}

void TwoMotors::turnDegree(float vel, float degrees){ // For turn angles (in degrees)
	m1->resetForGyrate();
	m2->resetForGyrate();
	float rot = degrees*rot_per_degree;
	if(degrees > 0){
		together(vel, rot, -vel, -rot);
	} else if(degrees < 0){
		together(-vel, -rot, vel, rot);
	}
}
