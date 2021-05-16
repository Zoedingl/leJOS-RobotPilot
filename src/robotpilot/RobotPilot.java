package robotpilot;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3GyroSensor;
import lejos.robotics.RegulatedMotor;

public class RobotPilot {
//  Private fields
    private EV3LargeRegulatedMotor leftMotor;
    private EV3LargeRegulatedMotor rightMotor;
    private float speed = 250;
    private float rotateSpeed = 100;
    private float currentSpeed = 100;
    private SpeedType speedType = SpeedType.SPEED;
    private boolean inverted = false;
    private float wheelDiameter;
    private float chassisWidth;
    private float acceleration = 9999;
    private EV3GyroSensor gyroSensor;
    private float friction = 1;
//  Public fields
    public final int QUICK_ACCELERATION = 9999;
    public enum Side {
        LEFT, RIGHT
    }

    private enum SpeedType {
        SPEED, ROTATE_SPEED, OTHER
    }
//  Constructors
    /**
     * Creates a new instance of the RobotPilot.
     * @param leftMotor The Port of the left EV3LargeRegulatedMotor
     * @param rightMotor The port of the right EV3LargeRegulatedMotor
     * @param wheelDiameter The diameter of the robot's wheels, used to calculate rotations and distances
     * @param chassisWidth The width of the robot's chassis, measured from the center of one wheel to the center of the second wheel, used to calculate rotations
     */
    public RobotPilot(Port leftMotor, Port rightMotor, float wheelDiameter, float chassisWidth) {
        this.leftMotor = new EV3LargeRegulatedMotor(leftMotor);
        this.rightMotor = new EV3LargeRegulatedMotor(rightMotor);
        this.wheelDiameter = wheelDiameter;
        this.chassisWidth = chassisWidth;
        setSpeed(speed);
    }

    /**
     * Creates a new instance of the RobotPilot.
     * @param leftMotor The Port of the left EV3LargeRegulatedMotor
     * @param rightMotor The port of the right EV3LargeRegulatedMotor
     * @param wheelDiameter The diameter of the robot's wheels, used to calculate rotations and distances
     * @param chassisWidth The width of the robot's chassis, measured from the center of one wheel to the center of the second wheel, used to calculate rotations
     * @param inverted Sets whether the motors will be moved inverted, set it to true if the robot moves backward when it should go backward and vice versa
     */
    public RobotPilot(Port leftMotor, Port rightMotor, float wheelDiameter, float chassisWidth, boolean inverted) {
        this.leftMotor = new EV3LargeRegulatedMotor(leftMotor);
        this.rightMotor = new EV3LargeRegulatedMotor(rightMotor);
        this.wheelDiameter = wheelDiameter;
        this.chassisWidth = chassisWidth;
        this.inverted = inverted;
        setSpeed(speed);
    }

    /**
     * Creates a new instance of the RobotPilot.
     * @param leftMotor The left EV3LargeRegulatedMotor
     * @param rightMotor The right EV3LargeRegulatedMotor
     * @param wheelDiameter The diameter of the robot's wheels, used to calculate rotations and distances
     * @param chassisWidth The width of the robot's chassis, measured from the center of one wheel to the center of the second wheel, used to calculate rotations
     */
    public RobotPilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, float wheelDiameter, float chassisWidth) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.wheelDiameter = wheelDiameter;
        this.chassisWidth = chassisWidth;
        setSpeed(speed);
    }

    /**
     * Creates a new instance of the RobotPilot.
     * @param leftMotor The left EV3LargeRegulatedMotor
     * @param rightMotor The right EV3LargeRegulatedMotor
     * @param wheelDiameter The diameter of the robot's wheels, used to calculate rotations and distances
     * @param chassisWidth The width of the robot's chassis, measured from the center of one wheel to the center of the second wheel, used to calculate rotations
     * @param inverted Sets whether the motors will be moved inverted, set it to true if the robot moves backward when it should go backward and vice versa
     */
    public RobotPilot(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, float wheelDiameter, float chassisWidth, boolean inverted) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.wheelDiameter = wheelDiameter;
        this.chassisWidth = chassisWidth;
        this.inverted = inverted;
        setSpeed(speed);
    }

//  Equals
    public boolean equals(RobotPilot robotPilot) { return leftMotor.equals(robotPilot.leftMotor) && rightMotor.equals(robotPilot.rightMotor) && chassisWidth == robotPilot.chassisWidth && wheelDiameter == robotPilot.wheelDiameter; }
//  Getters and setters
    /**
     * Returns the left motor as EV3LargeRegulatedMotor.
     * The motor can be used without conflicting with the RobotPilot.
     * IMPORTANT: Never call close() on a single motor, use close() in the RobotPilot instead.
     * @return The left motor as EV3LargeRegulatedMotor
     */
    public RegulatedMotor getLeftMotor() { return leftMotor; }

    /**
     * Sets the left motor.
     * Can be used to change the left motor of the pilot without using a constructor.
     * @param leftMotor The motor
     */
    public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
        this.leftMotor = leftMotor;
    }

    /**
     * Sets the left motor.
     * Can be used to change the left motor of the pilot without using a constructor.
     * @param port The port
     */
    public void setLeftMotor(Port port) {
        this.leftMotor = new EV3LargeRegulatedMotor(port);
    }

    /**
     * Returns the right motor as EV3LargeRegulatedMotor.
     * The motor can be used without conflicting with the RobotPilot.
     * IMPORTANT: Never call close() on the single motor, use close() in the RobotPilot instead.
     * @return The right motor as EV3LargeRegulatedMoto
     */
    public RegulatedMotor getRightMotor() { return rightMotor; }

    /**
     * Sets the right motor.
     * Can be used to change the right motor of the pilot without using a constructor.
     * @param rightMotor The motor
     */
    public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
        this.rightMotor = rightMotor;
    }

    /**
     * Sets the right motor.
     * Can be used to change the right motor of the pilot without using a constructor.
     * @param port The port
     */
    public void setRightMotor(Port port) {
        this.rightMotor = new EV3LargeRegulatedMotor(port);
    }

    /**
     * Returns the currently set wheelDiameter.
     * @return The currently set wheelDiameter
     */
    public float getWheelDiameter() { return wheelDiameter; }

    /**
     * Sets the wheelDiameter.
     * @param wheelDiameter THe wheel diameter that will be used in distance calculations
     */
    public void setWheelDiameter(float wheelDiameter) { this.wheelDiameter = wheelDiameter; }

    /**
     * Returns the currently set chassisWidth.
     * @return The currently set chassisWidth
     */
    public float getChassisWidth() { return chassisWidth; }

    /**
     * Sets the chassisWidth.
     * @param chassisWidth The chassis width that will be used in rotation calculations
     */
    public void setChassisWidth(float chassisWidth) { this.chassisWidth = chassisWidth; }

    /**
     * Returns the current inverted value.
     * @return True only if the RobotPilot is currently inverted
     */
    public boolean isInverted() { return inverted; }

    /**
     * Sets the current inverted value
     * @param inverted The new inverted value
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;

    }

    /**
     * Returns true only, if at least one of these motors is stalled (his movement is blocked by an external force.)
     * @return A boolean value determining if the robot is stalled
     */
    public boolean isStalled() { return leftMotor.isStalled() || rightMotor.isStalled();}

    /**
     * Returns the current friction value used when calculating rotation angles
     * @return The current friction value used when calculating rotation angles
     */
    public float getFriction() { return friction; }

    /**
     * Sets the friction value used when calculating rotation angles
     * The default value is one, if the robot rotates less degrees than it should,
     * increase the value a bit and vice versa.
     * @param friction The new friction value that will be used
     */
    public void setFriction(float friction) { this.friction = friction; }

    /**
     * Resets the friction field to it's default value
     */
    public void resetFriction() { friction = 1; }

    /**
     * Sets the EV3GyroSensor used in RotateWithGyro()
     * @param sensor The EV3GyroSensor
     */
    public void setGyroSensor(EV3GyroSensor sensor) { this.gyroSensor = sensor; }

    /**
     * Sets the EV3GyroSensor used in RotateWithGyro()
     * @deprecated Use setGyroSensor(EV3GyroSensor) instead
     * @param port The port to initialize the sensor
     */
    @Deprecated
    public void setGyroSensor(Port port) { this.gyroSensor = new EV3GyroSensor(port); }

    public EV3GyroSensor getGyroSensor() { return gyroSensor; }


//  Getters and setters with direct impact on robot performance

    /**
     * Returns the robot's current set speed, NOT the speed the robot is currently moving with.
     * @return The robot's currently set speed
     */
    public float getSpeed() { return speed; }

    /**
     * Sets the robot's current speed, has immediate impact on robot's performance.
     * @param speed Speed
     */
    public void setSpeed(float speed) {
        this.speed = speed;
        if (speedType == SpeedType.SPEED) {
            setCurrentSpeed(speed);
        }
    }

    /**
     * Sets the speed the robot will be using while rotating, has immediate impact on robot's performance.
     * @param rotateSpeed The new rotateSpeed
     */
    public void setRotateSpeed(float rotateSpeed) {
        this.rotateSpeed = rotateSpeed;
        if (speedType == SpeedType.ROTATE_SPEED) {
            setCurrentSpeed(rotateSpeed);
        }
    }

    /**
     * Returns the current speed used for rotations.
     * @return The current rotateSpeed
     */
    public float getRotateSpeed() {
        return rotateSpeed;
    }


    /**
     * Returns robot's current acceleration.
     * @return Robot's current acceleration
     */
    public float getAcceleration() { return acceleration; }

    /**
     * Sets the robot's current acceleration, has direct impact on robot's performance.
     * @param acceleration Acceleration
     */
    public void setAcceleration(int acceleration) {
        this.acceleration = acceleration;
        startSynchronization();
        leftMotor.setAcceleration(acceleration);
        rightMotor.setAcceleration(acceleration);
        endSynchronization();
    }

    /**
     * Returns the maximal speed the robot can achieve
     * @return Maximal speed the robot can achieve
     */
    public float getMaxSpeed() { return Math.min(leftMotor.getMaxSpeed(), rightMotor.getMaxSpeed()); }

    /**
     * Only returns true if the robot currently is moving.
     * @return A boolean determining if the robot is currently moving.
     */
    public boolean isMoving() { return leftMotor.isMoving() || rightMotor.isMoving();}

    /**
     * Returns the current tacho count of the robot
     * @return The current tacho count of the robot.
     */
    public float getTachoCount() { return (leftMotor.getTachoCount() + rightMotor.getTachoCount()) / 2f;}

    /**
     * Resets the tacho count
     */
    public void resetTachoCount() {
        leftMotor.resetTachoCount();
        rightMotor.resetTachoCount();
    }

//  Movement methods

    /**
     * Suspends all robot's movement until another method is called to start another movement
     */
    public void suspendRegulation() {
        startSynchronization();
        leftMotor.suspendRegulation();
        rightMotor.suspendRegulation();
        endSynchronization();
    }

    /**
     * Rotates both motors the specified amount of degrees
     * @param degrees Amount of degrees each motor should rotate
     */
    public void travelDegrees(int degrees) { travelDegrees(degrees, false); }

    /**
     * Rotates both motors the specified amount of degrees
     * @param degrees Amount of degrees each motor should rotate
     * @param immediateReturn Determines whether the method should return immediately
     */
    public void travelDegrees(int degrees, boolean immediateReturn) {
        if (currentSpeed != speed) {
            setCurrentSpeed(speed);
        }
        startSynchronization();
        if (inverted) { degrees = -degrees; }
        leftMotor.rotate(degrees, immediateReturn);
        rightMotor.rotate(degrees, immediateReturn);
        endSynchronization();
        if (!immediateReturn) { waitComplete(); }
    }

    /**
     * Moves the robot in units that were used to specify the value of wheelDiameter
     * @param distance Distance in units that were used to set the value of wheelDiameter
     */
    public void travel(float distance) {
        travel(distance, false);
    }

    /**
     * Moves the robot in units that were used to specify the value of wheelDiameter
     * @param distance Distance in units that were used to set the value of wheelDiameter
     * @param immediateReturn Determines whether the method should return immediately
     */
    public void travel(float distance, boolean immediateReturn) {
        float degreesToTravel = distance / ((float) ((Math.PI * wheelDiameter) / 360));
        travelDegrees(Math.round(degreesToTravel), immediateReturn);
    }

    public void forward() {
        if (currentSpeed != speed) {
            setCurrentSpeed(speed);
        }
        startSynchronization();
        if (inverted) {
            leftMotor.backward();
            rightMotor.backward();
        } else {
            leftMotor.forward();
            rightMotor.forward();
        }
        endSynchronization();

    }

    public void backward() {
        if (currentSpeed != speed) {
            setCurrentSpeed(speed);
        }
        startSynchronization();
        if (inverted) {
            leftMotor.forward();
            rightMotor.forward();
        } else {
            leftMotor.backward();
            rightMotor.backward();
        }
        endSynchronization();

    }

    /**
     * Stops all robot's movement, if acceleration is set, the robot will slow down and eventually stop.
     * If you want the robot to stop immediately, use quickStop(boolean immediateReturn) instead.
     */
    public void stop() {
        stop(false);
    }

    /**
     * Stops all robot's movement, if acceleration is set, the robot will slow down and eventually stop.
     * If you want the robot to stop immediately, use quickStop(boolean immediateReturn) instead.
     * @param immediateReturn Determines whether the method should return immediately
     */
    public void stop(boolean immediateReturn) {
        startSynchronization();
        leftMotor.stop(immediateReturn);
        rightMotor.stop(immediateReturn);
        endSynchronization();
        if (!immediateReturn) { waitComplete(); }
    }

    /**
     * Immediately stops any robot's movement
     */
    public void quickStop() {
        quickStop(false);
    }

    /**
     * Immediately stops any robot's movement
     * @param immediateReturn Determine whether the method should instantly return
     */
    public void quickStop(boolean immediateReturn) {
        startSynchronization();
        leftMotor.setAcceleration(QUICK_ACCELERATION);
        rightMotor.setAcceleration(QUICK_ACCELERATION);
        endSynchronization();
        stop(immediateReturn);
        startSynchronization();
        leftMotor.setAcceleration(Math.round(acceleration));
        rightMotor.setAcceleration(Math.round(acceleration));
        endSynchronization();
        if (!immediateReturn) { waitComplete(); }
    }

    /**
     * Stops all motor movements and leaves the motors without any force, so they can be moved around
     */
    public void flt() {
        flt(false);
    }
    /**
     * Stops all motor movements and leaves the motors without any force, so they can be moved around
     * @param immediateReturn Determines whether the method should instantly return
     */
    public void flt(boolean immediateReturn) {
        startSynchronization();
        leftMotor.flt(immediateReturn);
        rightMotor.flt(immediateReturn);
        endSynchronization();
        if (!immediateReturn) { waitComplete(); }
    }

    /**
     * Starts a rotating movement of the robot.
     * Use RobotPilot.Side.RIGHT or RobotPilot.Side.LEFT as parameter
     * @param side The direction of rotation either RobotPilot.Side.LEFT or RobotPilot.Side.RIGHT
     */
    public void startRotate(Side side) {
        if (currentSpeed != rotateSpeed) {
            setCurrentSpeed(rotateSpeed);
        }
        if (side == Side.LEFT) {
            startSynchronization();
            if (inverted) {
                leftMotor.forward();
                rightMotor.backward();
            } else {
                leftMotor.backward();
                rightMotor.forward();
            }
            endSynchronization();
        } else if(side == Side.RIGHT) {
            startSynchronization();
            if (inverted) {
                leftMotor.backward();
                rightMotor.forward();
            } else {
                leftMotor.forward();
                rightMotor.backward();
            }
            endSynchronization();
        } else {
            throw (new IllegalArgumentException("Argument must be Either RobotPilot.Side.LEFt or RobotPilot.Side.RIGHT"));
        }


    }

    /**
     * Rotates the robot on the spot the amount of specified degrees.
     * The field friction is used to get accurate rotation values.
     * @param degrees The amount of degrees the robot should rotate. (A value of 90 with the correct friction value will result in a 90 degrees rotation to the right side.)
     */
    public void rotate(float degrees) {
        rotate(degrees, false);
    }

    /**
     * Rotates the robot on the spot the amount of specified degrees.
     * The field friction is used to get accurate rotation values.
     * @param degrees The amount of degrees the robot should rotate. (A value of 90 with the correct friction value will result in a 90 degrees rotation to the right side.)
     * @param immediateReturn Determines whether the method should immediately return
     */
    public void rotate(float degrees, boolean immediateReturn) {
        if (currentSpeed != rotateSpeed) {
            setCurrentSpeed(rotateSpeed);
        }
        double circleValue = chassisWidth * Math.PI;
        double distanceToTravel = circleValue / (360 / degrees);
        int degreesToTravel = (int) (distanceToTravel / (float) ((Math.PI * wheelDiameter) / 360));
        degreesToTravel *= friction;
        if (inverted) { degreesToTravel = -degreesToTravel; }
        startSynchronization();
        leftMotor.rotate(degreesToTravel, immediateReturn);
        rightMotor.rotate(-degreesToTravel, immediateReturn);
        endSynchronization();
        if (!immediateReturn) { waitComplete(); }
    }


//  Utility methods

    public void waitComplete() {
        while (isMoving()) {
            leftMotor.waitComplete();
            rightMotor.waitComplete();
        }
    }

//  Close

    /**
     * Closes all connections, making it possible to initialize a new EV3LargeRegulatedMotor on the ports used by the RobotPilot
     */
    public void close() {
        try {
            leftMotor.close();
            rightMotor.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

//  Private usage methods

    private void startSynchronization() {
        leftMotor.synchronizeWith(new RegulatedMotor[] {rightMotor});
        leftMotor.startSynchronization();
    }

    private void endSynchronization() { leftMotor.endSynchronization(); }

    private void setCurrentSpeed(float speed) {
        currentSpeed = speed;
        if (speed == this.speed) {
            speedType = SpeedType.SPEED;
        } else if (speed == this.rotateSpeed) {
            speedType = SpeedType.ROTATE_SPEED;
        } else {
            speedType = SpeedType.OTHER;
        }
        startSynchronization();
        leftMotor.setSpeed(Math.round(speed));
        rightMotor.setSpeed(Math.round(speed));
        endSynchronization();
    }

}