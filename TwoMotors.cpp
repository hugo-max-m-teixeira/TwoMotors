/*
  Autor: Hugo Max M. Teixeira
  Data: 05/2021

  Para o controle de dois motores com encoder;
  Arquivo .cpp

*/

#include <TwoMotors.h>	// Inclusão do arquivo .h da biblioteca
/*
TwoMotors::TwoMotors(DC_motor_controller* motor1, DC_motor_controller* motor2){	//Construtor que recebe dois objetos do tipo DC_motor_controller (que serão sincronizados)
	this->m1=motor1; this->m2=motor2;
}

TwoMotors::TwoMotors(DC_motor_controller &motor1, DC_motor_controller &motor2){	//Construtor que recebe dois objetos do tipo DC_motor_controller (que serão sincronizados)
	this->m1=&motor1; this->m2=&motor2;
}*/
void TwoMotors::setMotors(DC_motor_controller &motor1, DC_motor_controller &motor2){
	//m1 = &motor1;
	//m2 = &motor2;
	motor[0] = &motor1;
	motor[1] = &motor2;
	
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
	motor[0]->run(pwm1);
	motor[1]->run(pwm2);
}

void TwoMotors::together(float velocity, float rotations/* = 0*/){ // Para a movimentação dos dois motores em uma mesma velocidade e com o mesmo valor de rotações
	if(rotations != 0){
		together(velocity, rotations, velocity, rotations);
	} else {
		for(int i=0; i<2; i++){
			motor[i]->walkAtConstantVelocity(velocity);
		}
	}
}

void TwoMotors::together(float velocity1, float rotations1, float velocity2, float rotations2){	// Para a movimentação dos motores com velocidades e rotaçẽos diferentes
	//if(rotations != 0){
		//DC_motor_controller** motor =new DC_motor_controller*[2];
		//motor[0] = m1;
		//motor[1] = m2;
		
		float velocity[2], rotations[2];
		
		velocity[0] = velocity1;
		velocity[1] = velocity2;
		
		rotations[0] = rotations1;
		rotations[1] = rotations2;
		
		float velocityRatio = velocity[0]/velocity[1];
		float copyOfOriginalAcceleration = motor[1]->getAcceleration();
		
		motor[1]->setAcceleration(copyOfOriginalAcceleration/velocityRatio);

		unsigned long 	startTime;
		/*bool	m1CanRun = true,
				m2CanRun = true;*/
		bool canRun[2] = {true, true};

		int directionCoefficient[2];
		bool accelTriangle[2];
		bool canAccelerate[2];
		unsigned long accelerationTimeInMs[2];
		long lastDesiredPulses[2];
		
		for(int i = 0; i<2; i++){
			if(velocity[i] < 0 || rotations[i] < 0){
				directionCoefficient[i] = -1;
			} else {
				directionCoefficient[i] = 1;
			}
		
			accelTriangle[i] = motor[i]->isAccelerationTriangle(velocity[i], rotations[i], motor[i]->getAcceleration());
			
			accelerationTimeInMs[i] = abs(velocity[i])/motor[i]->getAcceleration() * 1000;
			
			lastDesiredPulses[i] = motor[i]->rotationsToPulses(motor[i]->getAcceleration()*pow((float)accelerationTimeInMs[i]/1000.0,2.0))/120;
			
			motor[i]->accelerateProcess(1,1,1, true); // Resets time variable
		}

		startTime = millis();
		//Serial.println("Started accelerating both!\tAccelTriangle: " + String(accelTriangle[0]) + " - " + String(accelTriangle[1]));

		reset();
		do{
			for(int i=0; i<2; i++){
				canAccelerate[i] = motor[i]->accelerateProcess(velocity[i], motor[i]->getAcceleration(), startTime) && !accelTriangle[i];
			}
		}while(canAccelerate[0] || canAccelerate[1]);

		motor[1]->setAcceleration(copyOfOriginalAcceleration);
		
		Serial.println("Endend accelerating both!");
		
		startTime = millis();
		
		float remeaningRotations[2];
		long remeaningPulsesError[2];
		
		for(int i=0; i<2; i++){
			remeaningRotations[i] = rotations[i] - motor[i]->pulsesToRotations(motor[i]->pulses[1]);
			//Serial.println("motor[" + String(i) + "] - Remeaning rotations: " + String(remeaningRotations[i]));
			remeaningPulsesError[i] = motor[i]->pulses[1] - directionCoefficient[i]*lastDesiredPulses[i];
			//Serial.println("motor[" + String(i) + "] - Remeaning pulses error: " + String(remeaningPulsesError[i]));
			//motor[i]->gyrate(1,1,1, true); // Resets time variable
		}
		
		for(int i=0; i<2; i++){
			motor[i]->gyrate(1,1,1, true); // Resets time variable
			motor[i]->pulses[1] = remeaningPulsesError[i];
		}

		startTime = millis()/* - pseudoElapsedTime*/;
		
		do{
			//Serial.println(" 1-1 1-1 -1-M1 data below:");
			for(int i=0; i<2; i++){
				if(canRun[i]){
					canRun[i] = motor[i]->gyrate(velocity[i], remeaningRotations[i], startTime);
				} else {
					motor[i]->stop();
				}				
			}
			//Serial.println("Can run M1: " + String(m1CanRun) + "\t can rum M2: " + String(m2CanRun));
			//Serial.print("Rotations done since start M1: " + (String)(m1->pulsesToRotations(m1->pulses[1])));
			//Serial.println("\tRotations done since start M2: " + (String)(m2->pulsesToRotations(m2->pulses[1])));
		} while(canRun[0] || canRun[1]);
		//Serial.println("Ended gyrate both!");
		reset();
	//} else {
		//m1->walk(velocity1);
		//m2->walk(velocity2);
	//}
}

/*
void TwoMotors::turnDegree(float vel, float degrees){ // For turn angles (in degrees)
	reset();
	float rot = degrees*rot_per_degree; 				// Necessary rotations
	together(vel, rot, -vel, -rot);						// Compute RPM, PID and run
}
*/

void TwoMotors::stop(unsigned int time/*= 0*/){
	if(time <= motor[0]->getRefreshTime()){								// If "t" time is too small...
		reset();
		for(int i=0; i<2; i++){
			motor[i]->run(0);
		}
	} else {
		
	}	
}

/*
void TwoMotors::stop_vel(unsigned int vel1, unsigned int vel2){
	stop_time[0] = m1->anti_inertia_time(vel1);
	stop_time[1] = m2->anti_inertia_time(vel2);
	can_stop_vel = true;
	stop();
}*/

void TwoMotors::reset(){
	for(int i=0; i<2; i++){
		motor[i]->reset();	// Reset the motors
	}
}

void TwoMotors::ifNegativeAllNegative(float &val_1, float &val_2){
	if ((val_1 < 0) || (val_2 < 0)){ // Garante que os dois valores sejam negativos no caso de um dos valores ser negativo.
  		val_1 = -abs(val_1);
  		val_2 = -abs(val_2);
  	}
}
/*
unsigned int TwoMotors::equivalentTimeInMs(float velocity, float elapsedRotations){
	return elapsedRotations*60000/velocity;
}*/
