/* NavigationWithUS.java
 * This class is used to allow the robot to navigate to specified coordinates using the odometer.
 * 
 * Written by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */
import lejos.nxt.*;

public class NavigationWithUS extends Thread {

	private Odometer odo;
	private UltrasonicSensor ultrasonic;
	NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	NXTRegulatedMotor headMotor = Motor.C;
	private boolean isNav = false;
	private double currentX = 0, currentY = 0, currentTheta = 0;
	private double deltaX = 0, deltaY = 0, deltaTheta = 0, heading;
	private static final double PI = Math.PI;
	private double wheelRadii, width;

	// Variables for bangBangContrller() and its methods.
	private double distance;
	private final int FILTER_MAX = 50;
	public static final int WALLDIST = 22;
	public static final int DEADBAND = 3;
	private static final int bandCenter = 20;

	// constructor
	public NavigationWithUS(Odometer odometer, double wheelRadii, double width,
			UltrasonicSensor ultrasonic) {
		this.odo = odometer;
		this.wheelRadii = wheelRadii;
		this.width = width;
		this.ultrasonic = ultrasonic;

	}

	// writes to the LCD Screen for convenience...
	public void write(double destinationX, double destinationY,
			double currentX, double currentY, double Theta, double heading) {
		LCD.drawString("< Xf = " + Double.toString(destinationX) + " >", 0, 0);
		LCD.drawString("< Yf = " + Double.toString(destinationY) + " >", 0, 1);
		LCD.drawString("< Xc = " + Double.toString(currentX) + " >", 0, 2);
		LCD.drawString("< Yc = " + Double.toString(currentY) + " >", 0, 3);
		LCD.drawString("< Theta = " + Double.toString(Theta) + " >", 0, 4);
		LCD.drawString("< Heading = " + Double.toString(heading) + " >", 0, 5);
	}

	public void travelTo(double x, double y) {
		LCD.clear(7);
		LCD.drawString("In travelTo", 0, 7);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected
			// that
			// the odometer will be interrupted by another thread
		}

		// Sets Navigation to true
		isNav = true;

		// set the motor speeds
		leftMotor.setSpeed(300);
		rightMotor.setSpeed(300);

		// clear and write to the LCD
		LCD.clear();
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(), 0);

		// Calculate the heading of the robot and turn towards it
		currentX = this.odo.getX();
		currentY = this.odo.getY();
		deltaX = x - currentX;
		deltaY = y - currentY;
		heading = Math.atan2(deltaX, deltaY);
		turnTo(heading);

		// update screen
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
				heading);

		boolean exit = false;
		// will make sure robot exits wall following mode after FILTER_MAX
		// consecutive readings greater than bandCenter
		int filter = FILTER_MAX;

		// The while loop below is used to travel to the desired location
		while (Math.abs(x - this.odo.getX()) >= 2
				|| Math.abs(y - this.odo.getY()) >= 2) {

			// prints out the specific measurement from the US
			LCD.drawString("< D = " + ultrasonic.getDistance() + " >", 0, 6);
			write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
					heading);

			/* MIGHT NOT ACTUALLY DO ANYTHING */
			if (Math.abs(heading - this.odo.getTheta()) > 1) {
				/*
				 * currentX = this.odo.getX(); currentY = this.odo.getY();
				 * deltaX = x - currentX; deltaY = y - currentY; heading =
				 * Math.atan2(deltaX, deltaY);
				 */
				turnTo(heading);
			}

			leftMotor.forward();
			rightMotor.forward();

			// Enter "wall-following" mode and stay in until we have FILTER_MAX
			// readings above bandCenter
			while (filter < FILTER_MAX || ultrasonic.getDistance() < bandCenter) {
				if (!exit) {
					filter = 0;
				}
				// COMMENCE THE BANG-BANG
				bangBangController();
				exit = true;

				// increment filter if sensor reading is above bandCenter
				if (ultrasonic.getDistance() >= bandCenter) {
					filter++;
				}
				// if no wall is detected, restart navigation
				if (ultrasonic.getDistance() > 75 && filter > 30) {
					leftMotor.stop();
					rightMotor.stop();
					LCD.clear(7);
					LCD.drawString("IN THE LOOP", 0, 7);
					currentX = this.odo.getX();
					currentY = this.odo.getY();
					deltaX = x - currentX;
					deltaY = y - currentY;
					heading = Math.atan2(deltaX, deltaY);
					turnTo(heading);
					leftMotor.setSpeed(250);
					rightMotor.setSpeed(250);
					filter = FILTER_MAX;
					// break out of the wall-following loop
					break;
				}
				

			}
			LCD.clear(7);
			LCD.drawString("In travelTo END", 0, 7);
			
			leftMotor.forward();
			rightMotor.forward();

			// reset the filter
			exit = false;
			filter = FILTER_MAX;

		}

		// stop motors
		leftMotor.stop();
		rightMotor.stop();

		// sleep the thread for 1 second
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected
			// that
			// the odometer will be interrupted by another thread
		}
		isNav = false;
		// update LCD values
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
				heading);

	}

	// Rotates the robot by a specific angle
	public void turnTo(double theta) {
		isNav = true;

		LCD.clear(7);
		LCD.drawString("In turnTo", 0, 7);

		// Finds current heading according to odometer
		currentTheta = this.odo.getTheta();
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

	// Returns whether or not the robot is navigating
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

	/* Methods below taken and modified from BangBangController.java from lab1 */

	// method for wall following
	public void bangBangController() {
		LCD.clear(7);
		LCD.drawString("In bangBang", 0, 7);

		distance = ultrasonic.getDistance();
		LCD.drawString("< D = " + ultrasonic.getDistance() + " >", 0, 6);

		double error = distance - bandCenter;

		/* If the error is within the tolerance continue to move straight. */
		if (Math.abs(error) <= DEADBAND) {
			moveForward(300);
		}
		/*
		 * If the error is negative then we are too close to the wall, adjust
		 * such that we move away from the wall.
		 */
		else if (error < 0) {
			// Turn towards the Right
			turnRight(175, 175);
		}
		/*
		 * The third and final case. The error is positive and thus we are too
		 * far away from the wall. Correct this by moving towards the wall.
		 */
		else {
			// Turn towards the left
			turnLeft(180, 220);
		}
	}

	/* Method for a sharp right turn. */
	public void turnRight(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		leftMotor.forward();
		rightMotor.backward();
	}

	/* Method turn left. (Takes a wide left turn) */
	public void turnLeft(int leftSpeed, int rightSpeed) {
		leftMotor.setSpeed(leftSpeed);
		rightMotor.setSpeed(rightSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}

	/* Method to move forward */
	public void moveForward(int straightSpeed) {
		leftMotor.setSpeed(straightSpeed);
		rightMotor.setSpeed(straightSpeed);
		leftMotor.forward();
		rightMotor.forward();
	}

}
