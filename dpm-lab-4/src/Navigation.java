import lejos.nxt.Motor;
import lejos.nxt.NXTRegulatedMotor;
import lejos.nxt.LCD;

public class Navigation {
	// put your navigation code here

	private TwoWheeledRobot robot;

	private Odometer odo;
	NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	private boolean isNav = false;
	private double currentX = 0, currentY = 0, currentTheta = 0;
	private double deltaX = 0, deltaY = 0, deltaTheta = 0, heading;
	private static final double PI = Math.PI;
	private double wheelRadii, width;
	private double[] pos = new double[3];

	// Navigation constructor
	public Navigation(Odometer odometer) {
		this.odo = odometer;
		this.robot = odo.getTwoWheeledRobot();
		wheelRadii = 2.125; // wheel radius
		width = 14.9; // width between wheels
	}


	// writes to the LCD Screen for convenience...

	// Travel to method which determines the distance that robot needs to travel
	public void travelTo(double x, double y) {
		// Sets Navigation to true
		isNav = true;
		// set the motor speeds
		robot.setForwardSpeed(250);
		// clear and write to the LCD
		odo.getPosition(pos);
		// Calculate the heading of the robot and turn towards it
		currentX = pos[0];
		currentY = pos[1];
		// currentX = this.oriOdo.getX();
		// currentY = this.oriOdo.getY();
		deltaX = x - currentX;
		deltaY = y - currentY;
		heading = Math.atan2(deltaX, deltaY);
		turnTo(heading);

		this.odo.getPosition(pos);
		// update screen
		// write(x, y, pos[0], pos[1], pos[2], heading);

		// Until the robot is at its destination within 1 cm, it will move
		// forward in the heading's
		// direction
		while (Math.abs(x - pos[0]) >= 10 || Math.abs(y - pos[1]) >= 10) {
			// while (Math.abs(x - this.oriOdo.getX()) >= 1
			// || Math.abs(y - this.oriOdo.getY()) >= 1) {
			// write(x, y, pos[0], pos[1], pos[2], heading);
			/* MIGHT NOT ACTUALLY DO ANYTHING */
			if (Math.abs(Math.toDegrees(heading) - pos[2]) > 10) {
				// if (Math.abs(heading - this.oriOdo.getTheta()) > 1) {
				LCD.clear(5);
				LCD.drawInt((int) x, 0, 4);
				turnTo(Math.toRadians(heading));
			}
			robot.stahp();
			robot.setForwardSpeed(250);
			robot.goForward();
		}

		// When it exits the loop, STOP
		robot.stahp();

		// 1 second cat-nap
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected
			// that the odometer will be interrupted by another thread
		}

		// sets navigation to false when it gets to destination
		isNav = false;

		// UPDATE LCD
		// this.odo.getPosition(pos);
	}

	public void turnTo(double theta) {
		isNav = true;

		// Finds current heading according to odometer
		currentTheta = pos[2];
		// currentTheta = this.oriOdo.getTheta();
		deltaTheta = theta - currentTheta;

		// computes acute angle (Shortest turn)
		if (deltaTheta < -PI) {
			deltaTheta += 2 * PI;
		} else if (deltaTheta > PI) {
			deltaTheta -= 2 * PI;
		}

		// Performs rotation
		deltaTheta = Math.toDegrees(deltaTheta);
		leftMotor.rotate(convertAngle(wheelRadii, width, deltaTheta), true);
		rightMotor.rotate(-convertAngle(wheelRadii, width, deltaTheta), false);
	}

	boolean isNavigating() {
		return isNav;
	}

	// Method "borrowed" from SquareDriver.java
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	// Method "borrowed" from SquareDriver.java
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}
