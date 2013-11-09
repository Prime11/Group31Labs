import lejos.nxt.LCD;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class USLocalizer {
	public enum LocalizationType {
		FALLING_EDGE, RISING_EDGE
	};

	public static int ROTATION_SPEED = 90;
	public static int FORWARD_SPEED = 250;
	public static int WALL_THRESHOLD = 50;

	private Odometer odo;

	private TwoWheeledRobot robot;
	private UltrasonicSensor us;
	private LocalizationType locType;
	private Navigation hodor;

	public USLocalizer(Odometer odo, UltrasonicSensor us,
			LocalizationType locType) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.us = us;
		this.locType = locType;
		this.hodor = new Navigation(this.odo);

		// switch off the ultrasonic sensor
		// us.off();
	}

	public void doLocalization() {
		double[] pos = new double[3];
		int filterCount = 0;
		int filterMax = 6;
		double initAngle = 0;
		double angleA = 0, angleB = 0;
		double gammaA = 0;
		double deltaAngle = 0;
		robot.setSpeeds(FORWARD_SPEED, ROTATION_SPEED);
		odo.getPosition(pos);
		if (locType == LocalizationType.FALLING_EDGE) {

			// rotate the robot until it sees no wall
			robot.rotateClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					LCD.drawString("First Loop Over", 0, 5);
					odo.setPosition(new double[] { 0.0, 0.0, 0.0 },
							new boolean[] { true, true, true });
					odo.getPosition(pos);
					initAngle = pos[2];
					LCD.drawInt((int) initAngle, 0, 6);
					break;
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle
			robot.rotateClockwise();
			filterCount = 0;
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					LCD.drawString("Second Loop Over", 0, 5);
					odo.getPosition(pos);
					filterCount = 0;
					angleA = pos[2];// -angleA;
					LCD.drawInt((int) angleA, 4, 6);
					break;
				}
			}

			// switch direction and wait until it sees no wall

			robot.rotateCounterClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					gammaA = angleA - pos[2];
					break;
				}
			}
			// keep rotating until the robot sees a wall, then latch the angle
			robot.rotateCounterClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					filterCount = 0;
					odo.getPosition(pos);
					angleB = angleA + 360 - pos[2];
					LCD.drawInt((int) angleB, 8, 6);
					break;
				}
			}

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			odo.getPosition(pos);
			if (angleA < angleB) {
				deltaAngle = angleB - 90 - gammaA;// 45a
				LCD.drawInt((int) deltaAngle, 12, 6);
				hodor.turnTo(Math.toRadians(deltaAngle));
			} else {
				deltaAngle = angleB - 90 - gammaA;
				;// 45
				LCD.drawInt((int) deltaAngle, 14, 6);
				hodor.turnTo(Math.toRadians(deltaAngle));
			}

			// update the odometer position (example to follow:)
			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {
					true, true, true });
		} else {
			/*
			 * The robot should turn until it sees the wall, then look for the
			 * "rising edges:" the points where it no longer sees the wall. This
			 * is very similar to the FALLING_EDGE routine, but the robot will
			 * face toward the wall for most of it.
			 */

			// rotate the robot until it sees a wall
			robot.rotateCounterClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					LCD.drawString("First Loop Over", 0, 5);
					odo.setPosition(new double[] { 0.0, 0.0, 0.0 },
							new boolean[] { true, true, true });
					odo.getPosition(pos);
					initAngle = pos[2];
					LCD.drawInt((int) initAngle, 0, 6);
					break;
				}
			}
			// keep rotating until the robot sees no wall, then latch the angle
			robot.rotateCounterClockwise();
			filterCount = 0;
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					LCD.drawString("Second Loop Over", 0, 5);
					odo.getPosition(pos);
					filterCount = 0;
					angleA = pos[2];// -angleA;
					LCD.drawInt((int) angleA, 4, 6);
					break;
				}
			}

			// switch direction and wait until it sees a wall

			robot.rotateClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() <= 29 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					odo.getPosition(pos);
					filterCount = 0;
					gammaA = angleA - pos[2];
					break;
				}
			}
			// keep rotating until the robot sees no wall, then latch the angle
			robot.rotateClockwise();
			while (getFilteredData() > 0) {
				if (getFilteredData() > 30 && filterCount < filterMax) {
					filterCount++;
				} else if (filterCount == filterMax) {
					Sound.beep();
					filterCount = 0;
					odo.getPosition(pos);
					angleB = angleA - 45 - pos[2];
					LCD.drawInt((int) angleB, 8, 6);
					break;
				}
			}

			// angleA is clockwise from angleB, so assume the average of the
			// angles to the right of angleB is 45 degrees past 'north'
			odo.getPosition(pos);
			deltaAngle = angleB - 90 - gammaA;// 45a
			LCD.drawInt((int) deltaAngle, 12, 6);
			hodor.turnTo(Math.toRadians(deltaAngle));

			// update the odometer position (example to follow:)
			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {
					true, true, true });
		}
	}

	private int getFilteredData() {
		int distance;

		// do a ping
		us.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(15);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance = us.getDistance();

		if (distance > WALL_THRESHOLD) {
			distance = WALL_THRESHOLD;
		}
		LCD.drawInt(distance, 0, 7);

		return distance;
	}

}
