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

	public RobotPilot(Port leftMotor, Port rightMotor, float wheelDiameter, float chassisWidth, boolean inverted){
		this.motor1 = new EV3LargeRegulatedMotor(leftMotor);
		this.motor2 = new EV3LargeRegulatedMotor(rightMotor);
		motor1.setSpeed(motor1.getMaxSpeed());
		motor2.setSpeed(motor2.getMaxSpeed());
		this.wheelDiameter = wheelDiameter;
		this.chassisWidth = chassisWidth;
		this.inverted = inverted;

	}

	public void travel(float distance) {
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360)); 
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, true);
		motor2.rotate((int) degreesToTravel);
		
	}
	
	public void travel(float distance, boolean immediateReturn) {
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
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel);
	}
	
	public void moveLeftMotor(float distance, boolean immediateReturn) {
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor1.rotate((int) degreesToTravel, immediateReturn);
	}
	
	public void moveRightMotor(float distance) {
		float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
		if (inverted) {
			degreesToTravel = -degreesToTravel;
		}
		motor2.rotate((int) degreesToTravel);
	}
	
	public void moveRightMotor(float distance, boolean immediateReturn) {
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
		this.speed = speed;
		motor1.setSpeed(speed);
		motor2.setSpeed(speed);
	}
	
	public void startRotate(boolean right) {
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
		float mot1speed = motor1.getMaxSpeed();
		float mot2speed = motor2.getMaxSpeed();
		return Math.min(mot1speed, mot2speed);
		
	}
	
	public void setAcceleration(int acceleration) {
		motor1.setAcceleration(acceleration);
		motor2.setAcceleration(acceleration);
	}
}