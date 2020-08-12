package robotpilot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.robotics.RegulatedMotor;

public class RobotPilot {
	private EV3LargeRegulatedMotor motor1;
	private EV3LargeRegulatedMotor motor2;
	private float wheelDiameter;
	private float chassisWidth;
	private boolean inverted = false;
	private float rotateSpeed = 25f;
	private float speed;
	
	public RobotPilot(Port leftMotor, Port rightMotor, float wheelDiameter, float chassisWidth){
		this.motor1 = new EV3LargeRegulatedMotor(leftMotor);
		this.motor2 = new EV3LargeRegulatedMotor(rightMotor);
		motor1.setSpeed(motor1.getMaxSpeed());
		motor2.setSpeed(motor2.getMaxSpeed());
		this.wheelDiameter = wheelDiameter;
		this.chassisWidth = chassisWidth;
	}

	public RobotPilot(Port leftMotor, Port rightMotor, float wheelDiameter, float chassisWidth, boolean inverted) {
//		Set inverted to true if your robot is moving backwards when traveling forward and vice versa
		this.motor1 = new EV3LargeRegulatedMotor(leftMotor);
		this.motor2 = new EV3LargeRegulatedMotor(rightMotor);
		motor1.setSpeed(motor1.getMaxSpeed());
		motor2.setSpeed(motor2.getMaxSpeed());
		this.wheelDiameter = wheelDiameter;
		this.chassisWidth = chassisWidth;
		this.inverted = inverted;

	}
	
	public RobotPilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, float wheelDiameter, float chassisWidth){
		this.motor1 = leftMotor;
		this.motor2 = rightMotor;
		motor1.setSpeed(motor1.getMaxSpeed());
		motor2.setSpeed(motor2.getMaxSpeed());
		this.wheelDiameter = wheelDiameter;
		this.chassisWidth = chassisWidth;
	}
	
	public RobotPilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, float wheelDiameter, float chassisWidth, boolean inverted){
//		Set inverted to true if your robot is moving backwards when traveling forward and vice versa
		this.motor1 = leftMotor;
		this.motor2 = rightMotor;
		motor1.setSpeed(motor1.getMaxSpeed());
		motor2.setSpeed(motor2.getMaxSpeed());
		this.wheelDiameter = wheelDiameter;
		this.chassisWidth = chassisWidth;
		this.inverted = inverted;
	}

	public void travel(float distance) {
//		Starts a forward motion
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, true);
		motor2.rotate((int) degreesToTravel);
		
	}
	
	public void travel(float distance, boolean immediateReturn) {
//		Starts a forward motion
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, true);
		if (immediateReturn) {
			motor2.rotate((int) degreesToTravel, true);
		} else {
			motor2.rotate((int) degreesToTravel);
		}
		
	}
	
	public void setRotateSpeed(float rotateSpeed) {
//		Sets the speed the robot uses when rotating
		this.rotateSpeed = rotateSpeed;
	}
	
	public void forward() {
//		Starts forward movement of the robot
		if (inverted) {
			motor1.backward();
			motor2.backward();
		} else {
			motor1.forward();
			motor2.forward();
		}
		
	}
	
	public void backward() {
//		Starts backward movement of the robot
		if (inverted) {
			motor1.forward();
			motor2.forward();
		} else {
			motor1.backward();
			motor2.backward();
		}
		
	}
	
	public void stop() {
//		Stops the robot
		motor1.stop(true);
		motor2.stop();
	}
	
	public void flt() {
//		Floats the motors of the robot
		motor1.flt(true);
		motor2.flt();
	}
	
	public void flt(boolean immediateReturn) {
//		Floats the motors of the robot
		motor1.flt(true);
		motor2.flt(immediateReturn);
	}
	
	public void moveLeftMotor(float distance) {
//		Same as travel(distance) but only moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel);
	}
	
	public void moveLeftMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but only moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void moveRightMotor(float distance) {
//		Same as travel(distance) but only moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel);
	}
	
	public void moveRightMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but only moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void forwardLeftMotor() {
//		Starts forward movement of the left motor
		if (inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void backwardLeftMotor() {
//		Starts backward movement of the left motor
		if (!inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void forwardRightMotor() {
//		Starts forward movement of the right motor
		if (inverted) {
			motor2.backward();
		} else {
			motor2.forward();
		}
			
	}
	
	public void backwardRightMotor() {
//		Starts backward movement of the right motor
		if (!inverted) {
			motor2.backward();
		} else {
			motor2.forward();
		}
			
	}
	
	public void stopLeftMotor() {
//		Stops the left motor
		motor1.stop();
	}
	
	public void stopRightMotor() {
//		Stops the left motor
		motor2.stop();
	}
	
	public void floatLeftMotor() {
//		Floats the left motor
		motor1.flt();
	}
	
	public void floatLeftMotor(boolean immediateReturn) {
//		Floats the left motor
		motor1.flt(immediateReturn);
	}
	
	public void floatRightMotor() {
//		Floats the right motor
		motor2.flt();
	}
	
	public void floatRightMotor(boolean immediateReturn) {
//		Floats the right motor
		motor2.flt(immediateReturn);
	}
	
	public void close() {
//		Closes the connections
		motor1.close();
		motor2.close();
	}
	
	public void setSpeed(float speed) {
//		Sets the speed of the robot
		this.speed = speed;
		motor1.setSpeed(speed);
		motor2.setSpeed(speed);
	}
	
	public void startRotate(boolean right) {
//		Starts a rotating motion
//		If right is true, the robot will rotate right, otherwise he will rotate left
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		if (right) {
			if (inverted) {
				motor1.backward();
				motor2.forward();
			} else {
				motor1.forward();
				motor2.backward();
			}
		} else {
			if (inverted) {
				motor1.forward();
				motor2.backward();
			} else {
				motor1.backward();
				motor2.forward();
			}
		}
		setSpeed(speed);
	}
	
	public float getMaxSpeed() {
//		Returns the maximal speed the robot can achieve
		float mot1speed = motor1.getMaxSpeed();
		float mot2speed = motor2.getMaxSpeed();
		return Math.min(mot1speed, mot2speed);
		
	}
	
	public void setAcceleration(int acceleration) {
//		Sets the Linear acceleration of the robot
		motor1.setAcceleration(acceleration);
		motor2.setAcceleration(acceleration);
	}
	
//  When using an EV3GyroSensor, it`s recommended to use a method like this, where pilot is RobotPilot and gyroSensor is EV3GyroSensor: 
//	
//	static void rotate(float degrees) {
//		pilot.stop();
//		gyroSensor.reset();
//		if (degrees < 0) {
//			pilot.startRotate(false);
//			while (getGyroData() > degrees) {}
//			pilot.stop();
//		} else if( degrees > 0) {
//			pilot.startRotate(true);
//			while (getGyroData() < degrees) {}
//			pilot.stop();
//		}
//  }
//	
//	instead of using the rotate() method
	
	public void rotate(float angle) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
		float friction = 1;
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction));
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction));
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
		float friction = 1;
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), immediateReturn);
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), immediateReturn);
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public EV3LargeRegulatedMotor getLeftMotor() {
//		Returns the left motor as EV3LargeRegulatedMotor
		return motor1;
	}
	
	public EV3LargeRegulatedMotor getRightMotor() {
//		Returns the right motor as EV3LargeRegulatedMotor
		return motor2;
	}
	
	public boolean isMoving() {
//		Returns true if the robot is moving
		return motor1.isMoving() || motor2.isMoving();
	}
	
	public void waitComplete() {
//		Waits until the motors stop moving/rotating
		while (isMoving()) {}
	}
	
	public void rotateLeftMotorTo(int angle) {
//		Rotates the left motor to a specific angle
		motor1.rotateTo(angle);
	}
	
	public void rotateLeftMotorTo(int angle, boolean immediateReturn) {
//		Rotates the left motor to a specific angle
		motor1.rotateTo(angle, immediateReturn);
	}
	
	public void rotateRightMotorTo(int angle) {
//		Rotates the right motor to a specific angle
		motor2.rotateTo(angle);
	}
	
	public void rotateRightMotorTo(int angle, boolean immediateReturn) {
//		Rotates the right motor to a specific angle
		motor2.rotateTo(angle, immediateReturn);
	}
	
	public float getSpeed() {
//		Returns the speed of the RobotPilot
		return speed;
	}
	
	public float getRotateSpeed() {
//		Returns the rotate speed of the RobotPilot
		return rotateSpeed;
	}
	
	public boolean isInverted() {
//		Returns whether the RobotPilot is inverted or not
		return inverted;
	}
	
	public void setInverted(boolean inverted) {
//		Sets the inverted field
		this.inverted = inverted;
	}
	
	public void synchronizeWith(RegulatedMotor[] syncList) {
//		Specify a set of motors that should be kept in synchronization with these two
		motor1.synchronizeWith(syncList);
		motor2.synchronizeWith(syncList);
	}
	
	public void synchronizeLeftMotorWith(RegulatedMotor[] syncList) {
//		Specify a set of motors that should be kept in synchronization with this one
		motor1.synchronizeWith(syncList);
	}
	
	public void synchronizeRightMotorWith(RegulatedMotor[] syncList) {
//		Specify a set of motors that should be kept in synchronization with this one
		motor2.synchronizeWith(syncList);
	}
	
	public void startSynchronization() {
//		Begin a set of synchronized motor operations
		motor1.startSynchronization();
		motor2.startSynchronization();
	}
	
	public void startLeftMotorSynchronization() {
//		Begin a set of synchronized motor operations
		motor1.startSynchronization();
	}
	
	public void startRightMotorSynchronization() {
//		Begin a set of synchronized motor operations
		motor2.startSynchronization();
	}
	
	
	public void endSynchronization() {
//		Complete a set of synchronized motor operations
		motor1.endSynchronization();
		motor2.endSynchronization();
	}
	
	public void endLeftMotorSynchronization() {
//		Complete a set of synchronized motor operations
		motor1.endSynchronization();
	}
	
	public void endRightMotorSynchronization() {
//		Complete a set of synchronized motor operations
		motor2.endSynchronization();
	}
	
	public boolean isLeftMotorMoving() {
//		Returns true if the left motor is moving
		return motor1.isMoving();
	}
	
	public boolean isRightMotorMoving() {
//		Returns true if the right motor is moving
		return motor2.isMoving();
	}
}