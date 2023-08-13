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

TwoMotors::TwoMotors(DC_motor_controller &motor1, DC_motor_controller &motor2){	//Construtor que recebe dois objetos do tipo DC_motor_controller (que serão sincronizados)
	this->m1=&motor1; this->m2=&motor2;
}

void TwoMotors::setGyreDegreesRatio(float rot, float ang){
	this-> rot_per_degree = rot/ang;
}

void TwoMotors::together(float vel){
	m1->walk(vel);
	m2->walk(vel);
}

void TwoMotors::run(int pwm){
	m1->run(pwm);
	m2->run(pwm);
}

void TwoMotors::run(int pwm1, int pwm2){
	m1->run(pwm1);
	m2->run(pwm2);
}

void TwoMotors::together(float vel, float rot){ // Para a movimentação dos dois motores em uma mesma velocidade e com o mesmo valor de rotações
	if(rot != 0){
		resetMotors();
		while(m1->canRun() || m2->canRun()){
			if(m1->canRun())	m1->gyrate(vel, rot);
			if(m2->canRun())	m2->gyrate(vel, rot);
		}
		resetMotors();

		unsigned int lastTime = millis();
		while ((millis() - lastTime) < 100){
			m1->stop_both();
			m2->stop_both();
		}
		stop(200);
		resetMotors();
	} else {
		m1->walk(vel);
		m2->walk(vel);
	}
}

void TwoMotors::together(float vel1, float rot1, float vel2, float rot2){	// Para a movimentação dos motores com velocidades e rotaçẽos diferentes
	resetMotors();
	while(m1->canRun() || m2->canRun()){				// While can run...
		m1->gyrate(vel1, rot1);										// compute RPM, PID and run...
		m2->gyrate(vel2, rot2);
	}
	stop_vel(vel1, vel2);
}

void TwoMotors::turnDegree(float vel, float degrees){ // For turn angles (in degrees)
	resetMotors();
	float rot = degrees*rot_per_degree; 				// Necessary rotations
	together(vel, rot, -vel, -rot);						// Compute RPM, PID and run
}

void TwoMotors::stop(unsigned int time/*= 100*/){
	if(time <= m1->getRefreshTime()){								// If "t" time is too small...
		resetMotors();
		m1->run(0); m2->run(0);									// Just turn off the motors
	} else {
		resetMotors();
		m1->can_stop = true;
		m2->can_stop = true;
		while(m1->can_stop || m2->can_stop){
			if(can_stop_vel){
				if(m1->can_stop)	m1->stop_both(stop_time[0]);
				if(m2->can_stop)	m2->stop_both(stop_time[1]);
			} else {
				if(m1->can_stop)	m1->stop_both(time);
				if(m2->can_stop)	m2->stop_both(time);
			}		
		}
		if(can_stop_vel) can_stop_vel = false;
		m1->reset();
		m2->reset(); 										// Disable the motors
	}
}

void TwoMotors::stop_vel(unsigned int vel1, unsigned int vel2){
	stop_time[0] = m1->anti_inertia_time(vel1);
	stop_time[1] = m2->anti_inertia_time(vel2);
	can_stop_vel = true;
	stop();
}

void TwoMotors::resetMotors(){
	m1->resetForGyrate(); m2->resetForGyrate();	// Reseet the motors
}

void TwoMotors::reset(){
	resetMotors(); 				// Another name for the function resetMotors()
}

void TwoMotors::ifNegativeAllNegative(float &val_1, float &val_2){
	if ((val_1 < 0) || (val_2 < 0)){ // Garante que os dois valores sejam negativos no caso de um dos valores ser negativo.
  		val_1 = -abs(val_1);
  		val_2 = -abs(val_2);
  	}
}
