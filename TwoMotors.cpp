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
			m1->gyrate(vel, rot);
			m2->gyrate(vel, rot);
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
	stop();
}

void TwoMotors::turnDegree(float vel, float degrees){ // For turn angles (in degrees)
	resetMotors();
	float rot = degrees*rot_per_degree; 				// Necessary rotations
	together(vel, rot, -vel, -rot);						// Compute RPM, PID and run
}

void TwoMotors::stop(unsigned int t=100){
	if(t <= m1->getRefreshTime()){								// If "t" time is too small...
		resetMotors();
		m1->run(0); m2->run(0);									// Just turn off the motors
	} else {
		unsigned int lastT_local = millis();					// Set current time
		resetMotors();
		while((millis()-lastT_local) < t){						// During "t" time
			m1->stop_both(); m2->stop_both(); 					// Stop both motors
		}
		m1->run(0); m2->run(0); 										// Disable the motors
	}
}

void TwoMotors::resetMotors(){
	m1->resetForGyrate(); m2->resetForGyrate();	// Reseet the motors
}

void TwoMotors::reset(){
	resetMotors(); 				// Another name for the function resetMotors()
}
