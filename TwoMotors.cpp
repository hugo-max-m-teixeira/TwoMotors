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
/*
void TwoMotors::setGyreDegreesRatio(float rot, float ang){
	this-> rot_per_degree = rot/ang;
}
*/
void TwoMotors::run(int pwm){
	run(pwm, pwm);
}

void TwoMotors::run(int pwm1, int pwm2){
	m1->run(pwm1);
	m2->run(pwm2);
}

void TwoMotors::together(float velocity, float rotations/* = 0*/){ // Para a movimentação dos dois motores em uma mesma velocidade e com o mesmo valor de rotações
	if(rotations != 0){
		unsigned long 	startTime;
		bool	m1CanRun = true,
				m2CanRun = true;
		
		bool accelTriangleM1 = m1->isAccelerationTriangle(velocity, rotations, m1->getAcceleration());
		bool accelTriangleM2 = m2->isAccelerationTriangle(velocity, rotations, m2->getAcceleration());
		
		bool	m1CanAccelerate, m2CanAccelerate;
		
		unsigned long m1accelerationTimeInMs = abs(velocity)/m1->getAcceleration() * 1000;
		unsigned long m2accelerationTimeInMs = abs(velocity)/m2->getAcceleration() * 1000;
		
		long m1LastDesiredPulses = m1->rotationsToPulses(m1->getAcceleration()*pow((float)m1accelerationTimeInMs/1000.0,2.0))/120;
		long m2LastDesiredPulses = m2->rotationsToPulses(m2->getAcceleration()*pow((float)m2accelerationTimeInMs/1000.0,2.0))/120;
		
		m1->accelerateProcess(1,1,1, true); // Resets time variable
		m2->accelerateProcess(1,1,1, true); // Resets time variable
		
		startTime = millis();
		//Serial.println("Started accelerating both!");
		do{
			m1CanAccelerate = m1->accelerateProcess(velocity, m1->getAcceleration(), startTime) && !accelTriangleM1;
			m2CanAccelerate = m2->accelerateProcess(velocity, m2->getAcceleration(), startTime) && !accelTriangleM2;	
		}while(m1CanAccelerate || m2CanAccelerate);
		
		startTime = millis();
		
		float m1RemeaningRotations = rotations - m1->pulsesToRotations(m1->pulses[1]);
		float m2RemeaningRotations = rotations - m2->pulsesToRotations(m2->pulses[1]);
		
		if(velocity > 0){
			m1->pulses[1] -= m1LastDesiredPulses; // pulses error from previous accelerate() is considered and charged in pulses[1]
			m2->pulses[1] -= m2LastDesiredPulses;
		} else {
			m1->pulses[1] += m1LastDesiredPulses;
			m2->pulses[1] += m2LastDesiredPulses;
		}
		//Serial.println("Ended accelerating both! Preparing to start gyrate() on both");
		
		//reset();
					
		m1->gyrate(1,1,1, true); // Resets time variable
		m2->gyrate(1,1,1, true); // Resets time variable
		startTime = millis();
		
		do{
			//Serial.println(" 1-1 1-1 -1-M1 data below:");
			if(m1CanRun){
				m1CanRun = m1->gyrate(velocity, m1RemeaningRotations, startTime);
			} else {
				m1->stop();
			}
			//Serial.println("M1 can run: " + String(m1CanRun));
			//Serial.println(" 2-2 2-2 -2-M2 data below:");
			if(m2CanRun){
				m2CanRun = m2->gyrate(velocity, m2RemeaningRotations, startTime);
			} else {
				m2->stop();
			}
			//Serial.println("Can run M1: " + String(m1CanRun) + "\t can rum M2: " + String(m2CanRun));
			//Serial.print("Rotations done since start M1: " + (String)(m1->pulsesToRotations(m1->pulses[1])));
			//Serial.println("\tRotations done since start M2: " + (String)(m2->pulsesToRotations(m2->pulses[1])));
		} while(m1CanRun || m2CanRun);
		//Serial.println("Ended gyrate both!");
		reset();
	} else {
		m1->walk(velocity);
		m2->walk(velocity);
	}
}
/*
void TwoMotors::together(float vel1, float rot1, float vel2, float rot2){	// Para a movimentação dos motores com velocidades e rotaçẽos diferentes
	reset();
	while(m1->canRun() || m2->canRun()){				// While can run...
		m1->gyrate(vel1, rot1);										// compute RPM, PID and run...
		m2->gyrate(vel2, rot2);
	}
	stop_vel(vel1, vel2);
}
*/
/*
void TwoMotors::turnDegree(float vel, float degrees){ // For turn angles (in degrees)
	reset();
	float rot = degrees*rot_per_degree; 				// Necessary rotations
	together(vel, rot, -vel, -rot);						// Compute RPM, PID and run
}
*/
/*
void TwoMotors::stop(unsigned int time/*= 100*//*){
	if(time <= m1->getRefreshTime()){								// If "t" time is too small...
		reset();
		m1->run(0); m2->run(0);									// Just turn off the motors
	} else {
		reset();
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
*/
/*
void TwoMotors::stop_vel(unsigned int vel1, unsigned int vel2){
	stop_time[0] = m1->anti_inertia_time(vel1);
	stop_time[1] = m2->anti_inertia_time(vel2);
	can_stop_vel = true;
	stop();
}*/

void TwoMotors::reset(){
	m1->reset(); m2->reset();	// Reseet the motors
}

void TwoMotors::ifNegativeAllNegative(float &val_1, float &val_2){
	if ((val_1 < 0) || (val_2 < 0)){ // Garante que os dois valores sejam negativos no caso de um dos valores ser negativo.
  		val_1 = -abs(val_1);
  		val_2 = -abs(val_2);
  	}
}
