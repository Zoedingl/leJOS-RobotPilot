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
	private int acceleration = 0;
	public final int QUICK_ACCELERATION = 9999;
	
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
//		Makes a forward motion in units which where used when setting the wheelDiameter and the chassisWidth
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		setSpeed(speed);
		
		synchronizeMotors();
		motor1.rotate((int) degreesToTravel, true);
		motor2.rotate((int) degreesToTravel, true);
		motor1.endSynchronization();
		waitComplete();
		
	}
	
	public void travel(float distance, boolean immediateReturn) {
//		Makes a forward motion in units which where used when setting the wheelDiameter and the chassisWidth
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		setSpeed(speed);
		synchronizeMotors();
		motor1.rotate((int) degreesToTravel, true);
		motor2.rotate((int) degreesToTravel, true);
		motor1.endSynchronization();
		if (!immediateReturn) {
			waitComplete();
		}
		
	}
	
	public void travelDegrees(int degrees) {motor1.setSpeed(0);
	setSpeed(speed);
	    synchronizeMotors();
		motor1.rotate(degrees, true);
		motor2.rotate(degrees, true);
		motor1.endSynchronization();
		waitComplete();
	}
	
	public void travelDegrees(int degrees, boolean immediateReturn) {
		synchronizeMotors();
		setSpeed(speed);
		motor1.rotate(degrees, true);
		motor2.rotate(degrees, true);
		motor1.endSynchronization();
		if (!immediateReturn) {
			waitComplete();
		}
	}
	
	public void setRotateSpeed(float rotateSpeed) {
//		Sets the speed the robot uses when rotating
		if (this.rotateSpeed != rotateSpeed) {
			this.rotateSpeed = rotateSpeed;
		}
	}
	
	public void forward() {
//		Starts forward movement of the robot
		setSpeed(speed);
		synchronizeMotors();
		if (inverted) {
			motor1.backward();
			motor2.backward();
		} else {
			motor1.forward();
			motor2.forward();
		}
		motor1.endSynchronization();
		
	}
	
	public void backward() {
//		Starts backward movement of the robot
		setSpeed(speed);
		synchronizeMotors();
		motor1.setSpeed(0);
		motor2.setSpeed(0);
		if (inverted) {
			motor1.forward();
			motor2.forward();
		} else {
			motor1.backward();
			motor2.backward();
		}
		motor1.endSynchronization();
		
	}
	
	public void stop() {
//		Stops the robot
		synchronizeMotors();
		motor1.stop(true);
		motor2.stop(true);
		motor1.endSynchronization();
		while (isMoving()) {}
	}
	
	public void stop(boolean immediateReturn) {
//		Stops the robot
		synchronizeMotors();
		motor1.stop(true);
		motor2.stop(true);
		motor1.endSynchronization();
		if (!immediateReturn) {
			while (isMoving()) {}
		}
	}
	
	public void flt() {
//		Floats the motors of the robot
		synchronizeMotors();
		motor1.flt(true);
		motor2.flt(true);
		motor1.endSynchronization();
		while (isMoving()) {}
	}
	
	public void flt(boolean immediateReturn) {
//		Floats the motors of the robot
		synchronizeMotors();
		motor1.flt(true);
		motor2.flt(true);
		motor1.endSynchronization();
		if (!immediateReturn) {
			while (isMoving()) {}
		}
		
	}
	
	public void moveLeftMotor(float distance) {
//		Same as travel(distance) but only moves one motor
		setSpeed(speed);
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel);
	}
	
	public void moveLeftMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but only moves one motor
		setSpeed(speed);
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void moveRightMotor(float distance) {
//		Same as travel(distance) but only moves one motor
		setSpeed(speed);
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel);
	}
	
	public void moveRightMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but only moves one motor
		setSpeed(speed);
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void forwardLeftMotor() {
//		Starts forward movement of the left motor
		setSpeed(speed);
		if (inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void backwardLeftMotor() {
//		Starts backward movement of the left motor
		setSpeed(speed);
		if (!inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void forwardRightMotor() {
//		Starts forward movement of the right motor
		setSpeed(speed);
		if (inverted) {
			motor2.backward();
		} else {
			motor2.forward();
		}
			
	}
	
	public void backwardRightMotor() {
//		Starts backward movement of the right motor
		setSpeed(speed);
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
	
	public void stopLeftMotor(boolean immediateReturn) {
//		Stops the left motor
		motor1.stop(immediateReturn);
	}
	
	public void stopRightMotor() {
//		Stops the right motor
		motor2.stop();
	}
	
	public void stopRightMotor(boolean immediateReturn) {
//		Stops the right motor
		motor2.stop(immediateReturn);
	}
	
	public void fltLeftMotor() {
//		Floats the left motor
		motor1.flt();
	}
	
	public void fltLeftMotor(boolean immediateReturn) {
//		Floats the left motor
		motor1.flt(immediateReturn);
	}
	
	public void fltRightMotor() {
//		Floats the right motor
		motor2.flt();
	}
	
	public void fltRightMotor(boolean immediateReturn) {
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
			synchronizeMotors();
			motor1.setSpeed(speed);
			motor2.setSpeed(speed);
			motor1.endSynchronization();
		
	}
	
	public void startRotate(boolean right) {
//		Starts a rotating motion
//		If right is true, the robot will rotate right, otherwise he will rotate left
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		synchronizeMotors();
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
		motor1.endSynchronization();
	}
	
	public float getMaxSpeed() {
//		Returns the maximal speed the robot can achieve
		float mot1speed = motor1.getMaxSpeed();
		float mot2speed = motor2.getMaxSpeed();
		return Math.min(mot1speed, mot2speed);
		
	}
	
	public void setAcceleration(int acceleration) {
//		Sets the Linear acceleration of the robot
		this.acceleration = acceleration;
		synchronizeMotors();
		motor1.setAcceleration(acceleration);
		motor2.setAcceleration(acceleration);
		motor1.endSynchronization();
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
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / (float) ((Math.PI * wheelDiameter) / 360)); 
//		Rotate the motors
		synchronizeMotors();
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), true);
		motor1.endSynchronization();
		waitComplete();
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
//		Set the speed of motors to rotateSpeed
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		synchronizeMotors();
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), true);
		motor1.endSynchronization();
		waitComplete();
		
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
		float friction = 1;
//		Set the speed of motors to rotateSpeed
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		synchronizeMotors();
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), true);
		motor1.endSynchronization();
//		if immediate return is false, wait for the motors to complete rotations
		if (!immediateReturn) {
			waitComplete();
		}
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the value the robot rotates.
//		Set the speed of motors to rotateSpeed
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		double distanceToTravel = circleValue / (360 / angle);
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		synchronizeMotors();
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), true);
		motor1.endSynchronization();
//		if immediate return is false, wait for the motors to complete rotations
		if (!immediateReturn) {
			waitComplete();
		}
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
			motor1.waitComplete();
			motor2.waitComplete();
			if (motor1.isMoving()) {
				motor1.waitComplete();
			}
	}
	
	public void rotateLeftMotorTo(int angle) {
//		Rotates the left motor to a specific angle
		setSpeed(speed);
		motor1.rotateTo(angle);
	}
	
	public void rotateLeftMotorTo(int angle, boolean immediateReturn) {
//		Rotates the left motor to a specific angle
		setSpeed(speed);
		motor1.rotateTo(angle, immediateReturn);
	}
	
	public void rotateRightMotorTo(int angle) {
//		Rotates the right motor to a specific angle
		setSpeed(speed);
		motor2.rotateTo(angle);
	}
	
	public void rotateRightMotorTo(int angle, boolean immediateReturn) {
//		Rotates the right motor to a specific angle
		setSpeed(speed);
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
	
//	The travelArc method doesn't work yet
//	It should not be used, since it only works partly and is totally unusable
	@Deprecated
	public void travelArc(float distance, int ratio) throws IllegalArgumentException {
//		if ratio is -1, the robot will rotate left on the spot, if ration is 1, the robot will rotate right on the spot, and if the ration is something between, the robot will arc forward with the given ratio
//		if ration is 0, the robot will travel forward
//		distance is the the distance the faster motor will travel
//		TODO: make the travelArc method work
		if (ratio > 1 || ratio < -1) {
			throw new IllegalArgumentException("Invalid ratio, ratio must be between -1 and 1");
		} else {
			if (ratio == -1) {
				int degreesToRotate = (int) (distance / ((float) ((Math.PI * wheelDiameter) / 360)));
				motor1.rotate(degreesToRotate, true);
				motor2.rotate(-degreesToRotate);
				waitComplete();
			} else if (ratio == 1) {
				int degreesToRotate = (int) (distance / ((float) ((Math.PI * wheelDiameter) / 360)));
				motor1.rotate(-degreesToRotate, true);
				motor2.rotate(degreesToRotate);
				waitComplete();
			} else if (ratio == 0) {
				travel(distance);
			} else if (ratio > -1 && ratio <= -0.5) {
				
			}
		}
	}
	
	public int getLeftTachoCount() {
		return motor1.getTachoCount();
	}
	
	public int getRightTachoCount() {
		return motor2.getTachoCount();
	}
	
	public int getTachoCount() {
		return (int) (((float) (motor1.getTachoCount()) + (float) (motor2.getTachoCount())) / 2);
	}
	
	public void resetTachoCount() {
		motor1.resetTachoCount();
		motor2.resetTachoCount();
	}
	
	public void resetLeftTachoCount() {
		motor1.resetTachoCount();
	}
	
	public void resetRightTachoCount() {
		motor2.resetTachoCount();
	}
	
	public void quickStop() {
		motor1.setAcceleration(QUICK_ACCELERATION);
		motor2.setAcceleration(QUICK_ACCELERATION);
		stop();
		setAcceleration(acceleration);
	}
	
	public void quickStop(boolean immediateReturn) {
		motor1.setAcceleration(QUICK_ACCELERATION);
		motor2.setAcceleration(QUICK_ACCELERATION);
		stop(immediateReturn);
		setAcceleration(acceleration);
	}
	
	public void quickStopLeftMotor() {
		motor1.setAcceleration(QUICK_ACCELERATION);
		stopLeftMotor();
		setAcceleration(acceleration);
	}
	
	public void quickStopLeftMotor(boolean immediateReturn) {
		motor1.setAcceleration(QUICK_ACCELERATION);
		stopLeftMotor(immediateReturn);
		setAcceleration(acceleration);
	}
	
	public void quickStopRightMotor() {
		motor2.setAcceleration(QUICK_ACCELERATION);
		stopRightMotor();
		setAcceleration(acceleration);
	}
	
	public void quickStopRightMotor(boolean immediateReturn) {
		motor2.setAcceleration(QUICK_ACCELERATION);
		stopRightMotor(immediateReturn);
		setAcceleration(acceleration);
	}
	
	private void synchronizeMotors() {
		motor1.synchronizeWith(new RegulatedMotor[] {motor2});
		motor1.startSynchronization();
	}
				
}