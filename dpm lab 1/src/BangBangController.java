/* Written and modified by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.*;

public class BangBangController implements UltrasonicController {
	private final int bandCenter, bandwith;
	private final int motorLow, motorHigh;
	private final int motorStraight = 250;
	private final NXTRegulatedMotor leftMotor = Motor.B, rightMotor = Motor.C,
			headMotor = Motor.A;
	private int distance;
	private int currentLeftSpeed;

	public static final int WALLDIST = 22; /* code change here, WALLDIST increased */
	public static final int DEADBAND = 3;

	static UltrasonicSensor mySensor = new UltrasonicSensor(SensorPort.S1);

	public BangBangController(int bandCenter, int bandwith, int motorLow,
			int motorHigh) {
		// Default Constructor

		this.bandCenter = bandCenter;
		this.bandwith = bandwith;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		//headMotor.stop();
		//leftMotor.setSpeed(motorStraight);
		//rightMotor.setSpeed(motorStraight);
		//leftMotor.forward();
		//rightMotor.forward();
		currentLeftSpeed = 0;
	}


	/* Method for a sharp right turn. */
	public void turnRight(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		leftMotor.forward();
		rightMotor.backward();
	}
	
	/* Method to move forward */
	public void moveForward(int straightSpeed) {
		leftMotor.setSpeed(straightSpeed);
		rightMotor.setSpeed(straightSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}
	
	/* Method turn left. (Takes a wide left turn) */
	public void turnLeft(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed); 
		rightMotor.setSpeed(rightSpeed); 
		leftMotor.forward();
		rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in
		// (BANG-BANG style)
		
		int error = 0;

		error = distance - WALLDIST;
		
		/* Remove comment from the line below in order to accurately set the angle of the sensor.
		 * Set the sensor to face directly forward before running the statement below.*/
		//headMotor.rotateTo(-45);

		/* If the error is within the tolerance continue to move straight. */
		if (Math.abs(error) <= DEADBAND) 
		{
			moveForward(motorStraight);
		}
		/* If the error is negative then we are too close to the wall, adjust
		 * such that we move away from the wall. */
		else if (error < 0) 
		{ 
			// Turn towards the Right
			turnRight(125, 125);
		}
		/* The third and final case. The error is positive and thus we are too
		 * far away from the wall.
		 * Correct this by moving towards the wall.*/
		else 
		{ 
			// Stop
			leftMotor.stop();
			rightMotor.stop();
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
