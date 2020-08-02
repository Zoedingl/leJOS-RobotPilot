import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.utility.Delay;

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

	public void travel(float distance) {
//		moves the robot forward
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, true);
		motor2.rotate((int) degreesToTravel);
		
	}
	
	public void travel(float distance, boolean immediateReturn) {
//		moves the robot forward
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
		if (inverted) {
			motor1.backward();
			motor2.backward();
		} else {
			motor1.forward();
			motor2.forward();
		}
		
	}
	
	public void backward() {
		if (inverted) {
			motor1.forward();
			motor2.forward();
		} else {
			motor1.backward();
			motor2.backward();
		}
		
	}
	
	public void stop() {
		motor1.stop(true);
		motor2.stop();
	}
	
	public void flt() {
		motor1.flt(true);
		motor2.flt();
	}
	
	public void flt(boolean immediateReturn) {
		motor1.flt(true);
		motor2.flt(immediateReturn);
	}
	
	public void moveLeftMotor(float distance) {
//		Same as travel(distance) but onlz moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel);
	}
	
	public void moveLeftMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but onlz moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void moveRightMotor(float distance) {
//		Same as travel(distance) but onlz moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel);
	}
	
	public void moveRightMotor(float distance, boolean immediateReturn) {
//		Same as travel(distance) but onlz moves one motor
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void forwardLeftMotor() {
		if (inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void backwardLeftMotor() {
		if (!inverted) {
			motor1.backward();
		} else {
			motor1.forward();
		}
			
	}
	
	public void forwardRightMotor() {
		if (inverted) {
			motor2.backward();
		} else {
			motor2.forward();
		}
			
	}
	
	public void backwardRightMotor() {
		if (!inverted) {
			motor2.backward();
		} else {
			motor2.forward();
		}
			
	}
	
	public void stopLeftMotor() {
		motor1.stop();
	}
	
	public void stopRightMotor() {
		motor2.stop();
	}
	
	public void floatLeftMotor() {
		motor1.flt();
	}
	
	public void floatLeftMotor(boolean immediateReturn) {
		motor1.flt(immediateReturn);
	}
	
	public void floatRightMotor() {
		motor2.flt();
	}
	
	public void floatRightMotor(boolean immediateReturn) {
		motor2.flt(immediateReturn);
	}
	
	public void close() {
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
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		Delay.msDelay(25);
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
//		returns the maximal speed the robot can achieve
		float mot1speed = motor1.getMaxSpeed();
		float mot2speed = motor2.getMaxSpeed();
		return Math.min(mot1speed, mot2speed);
		
	}
	
	public void setAcceleration(int acceleration) {
//		Sets the Linear acceleration of the robot
		motor1.setAcceleration(acceleration);
		motor2.setAcceleration(acceleration);
	}
	
	public void rotate(float angle) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the friction.
		float friction = 1;
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		Delay.msDelay(25);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		float distanceToTravel = (float) (circleValue / (360 / angle));
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction));
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the friction.
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		Delay.msDelay(25);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		float distanceToTravel = (float) (circleValue / (360 / angle));
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction));
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the friction.
		float friction = 1;
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		Delay.msDelay(25);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		float distanceToTravel = (float) (circleValue / (360 / angle));
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), immediateReturn);
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
	public void rotate(float angle, float friction, boolean immediateReturn) {
//		The friction value is 1 by default. You should change it depending on your robot`s friction when rotating. To find out the right value, just try and test until the robot turns the right angle. The higher the value, the higher the friction.
//		Set the speed of motors to rotateSpeed
		motor1.setSpeed(rotateSpeed);
		motor2.setSpeed(rotateSpeed);
		Delay.msDelay(25);
//		Perform calculations
		double circleValue = chassisWidth * Math.PI; 
		float distanceToTravel = (float) (circleValue / (360 / angle));
		int degreesToTravel = (int) (distanceToTravel / ((float) ((Math.PI * wheelDiameter) / 360))); 
//		Rotate the motors
		motor1.rotate((int) (degreesToTravel * friction), true);
		motor2.rotate((int) (-degreesToTravel * friction), immediateReturn);
//		Set the speed of motors back to speed
		setSpeed(speed);
	}
	
}