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
		/*unsigned long 	startTime;
		bool	m1CanRun = true,
				m2CanRun = true;

		int directionCoefficient;

		if(velocity < 0 || rotations < 0){
			directionCoefficient = -1;
		} else {
			directionCoefficient = 1;
		}
		
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
		reset();
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

		long remeaningPulsesError[2];

		remeaningPulsesError[0] = m1->pulses[1] - directionCoefficient*m1LastDesiredPulses;
		remeaningPulsesError[1] = m2->pulses[1] - directionCoefficient*m2LastDesiredPulses;
					
		m1->gyrate(1,1,1, true); // Resets time variable and pulses[1] (now = 0)
		m2->gyrate(1,1,1, true); // Resets time variable and pulses[1] (now = 0)

		m1->pulses[1] = remeaningPulsesError[0];
		m2->pulses[1] = remeaningPulsesError[1];
					
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
		reset();*/
	} else {
		for(int i=0; i<2; i++){
			motor[i]->walkAtConstantVelocity(0);
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
		
			accelTriangle[i] = motor[0]->isAccelerationTriangle(velocity[i], rotations[i], motor[i]->getAcceleration());
			
			accelerationTimeInMs[i] = abs(velocity[i])/motor[i]->getAcceleration() * 1000;
			
			lastDesiredPulses[i] = motor[i]->rotationsToPulses(motor[i]->getAcceleration()*pow((float)accelerationTimeInMs[i]/1000.0,2.0))/120;
			
			motor[i]->accelerateProcess(1,1,1, true); // Resets time variable
		}

		startTime = millis();
		Serial.println("Started accelerating both!");

		reset();
		do{
			for(int i=0; i<2; i++){
				canAccelerate[i] = motor[i]->accelerateProcess(velocity[i], motor[i]->getAcceleration(), startTime) && !accelTriangle[i];
			}
		}while(canAccelerate[0] || canAccelerate[1]);

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
		motor[i]->run(0);	// Reset the motors
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
