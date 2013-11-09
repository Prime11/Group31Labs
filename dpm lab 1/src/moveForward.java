//Created this class to initially test the robot after building it.

import lejos.nxt.*;

public class moveForward implements UltrasonicController {

	private final int motorStraight = 200;
	private final NXTRegulatedMotor leftMotor = Motor.B, rightMotor = Motor.C,
			headMotor = Motor.A;
	private int distance;
	private int currentLeftSpeed;

	public static final int WALLDIST = 20;
	public static final int DEADBAND = 2;

	static UltrasonicSensor mySensor = new UltrasonicSensor(SensorPort.S1);

	@Override
	public void processUSData(int distance) {

		boolean traveling = true;
		leftMotor.setSpeed(motorStraight);
		rightMotor.setSpeed(motorStraight);
		leftMotor.forward();
		rightMotor.forward();

		LCD.clear(); // Sets up the screan for distance readings.
		LCD.drawString("Bang-Bang", 0, 0, false);
		LCD.drawString("Distance:", 0, 32, false);

		while (traveling) {
			mySensor.ping();
			distance = mySensor.getDistance();
			LCD.drawInt(distance, 4, 11, 4);

		}

	}

	@Override
	public int readUSDistance() {
		// TODO Auto-generated method stub
		return 0;
	}

}