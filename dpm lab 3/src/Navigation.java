/* Navigation.java
 * This class is used to allow the robot to navigate to specified coordinates using the odometer.
 * 
 * Written by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */

import lejos.nxt.*;

public class Navigation extends Thread {

	private Odometer odo;
	NXTRegulatedMotor leftMotor = Motor.A, rightMotor = Motor.B;
	private boolean isNav = false;
	private double currentX = 0, currentY = 0, currentTheta = 0;
	private double deltaX = 0, deltaY = 0, deltaTheta = 0, heading;
	private static final double PI = Math.PI;
	private double wheelRadii, width;

	// Navigation constructor
	public Navigation(Odometer odometer, double wheelRadii, double width) {
		this.odo = odometer;
		this.wheelRadii = wheelRadii;
		this.width = width;
	}

	//writes to the LCD Screen for convenience...
	public void write(double destinationX, double destinationY,
			double currentX, double currentY, double Theta, double heading) {
		LCD.drawString("< Xf = " + Double.toString(destinationX) + " >", 0, 0);
		LCD.drawString("< Yf = " + Double.toString(destinationY) + " >", 0, 1);
		LCD.drawString("< Xc = " + Double.toString(currentX) + " >", 0, 2);
		LCD.drawString("< Yc = " + Double.toString(currentY) + " >", 0, 3);
		LCD.drawString("< Theta = " + Double.toString(Theta) + " >", 0, 4);
		LCD.drawString("< Heading = " + Double.toString(heading) + " >", 0, 5);
	}

	// Travel to method which determines the distance that robot needs to travel
	public void travelTo(double x, double y) {
		//Sets Navigation to true
		isNav = true;
		
		// set the motor speeds
		leftMotor.setSpeed(250);
		rightMotor.setSpeed(250);
		
		// clear and write to the LCD
		LCD.clear();
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(), 0);

		//Calculate the heading of the robot and turn towards it
		currentX = this.odo.getX();
		currentY = this.odo.getY();
		deltaX = x - currentX;
		deltaY = y - currentY;
		heading = Math.atan2(deltaX, deltaY);
		turnTo(heading);
		
		//update screen
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
				heading);
		
		//Until the robot is at its destination within 1 cm, it will move forward in the heading's 
		//direction
		while (Math.abs(x - this.odo.getX()) >= 1
				|| Math.abs(y - this.odo.getY()) >= 1) {
			write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
					heading);
			/*MIGHT NOT ACTUALLY DO ANYTHING*/
			if (Math.abs(heading - this.odo.getTheta()) > 1) {
				turnTo(heading);
			}
			leftMotor.forward();
			rightMotor.forward();
		}

		// When it exits the loop, STOP
		leftMotor.stop();
		rightMotor.stop();
		
		//1 second cat-nap
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// there is nothing to be done here because it is not expected
			// that the odometer will be interrupted by another thread
		}
		
		//sets navigation to false when it gets to destination
		isNav = false;
		
		//UPDATE LCD
		write(x, y, this.odo.getX(), this.odo.getY(), this.odo.getTheta(),
				heading);
	}

	public void turnTo(double theta) {
		isNav = true;
		
		//Finds current heading according to odometer
		currentTheta = this.odo.getTheta();
		deltaTheta = theta - currentTheta;
		
		//computes acute angle (Shortest turn)
		if (deltaTheta < -PI) {
			deltaTheta += 2 * PI;
		} else if (deltaTheta > PI) {
			deltaTheta -= 2 * PI;
		}
		
		//Performs rotation
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
