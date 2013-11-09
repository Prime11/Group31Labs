/*
 * Lab5.java
 */

import java.util.Stack;

import lejos.nxt.*;

public class Lab5b {

	public static void main(String[] args) {
		int buttonChoice; // me, maybe use this later.
		NXTRegulatedMotor headMotor = Motor.C;

		boolean foundSomething;

		LCD.clear();
		LCD.drawString("Herrow! ", 0, 0);
		LCD.drawString("Robot Activated!", 0, 1);

		Button.waitForAnyPress();
		// create the objects for the main method.
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2);
		ColorSensor ls = new ColorSensor(SensorPort.S1);
		UltraDisplay displayDist = new UltraDisplay(us);

		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		LCDInfo lcd = new LCDInfo(odo);
		lcd.timedOut();
		// perform the ultrasonic localization
		USLocalizer usl = new USLocalizer(odo, us,
				USLocalizer.LocalizationType.RISING_EDGE);

		LCD.clear();
		// remove for now
		usl.doLocalization();
		// initialize Object Detection
		// used to identify the object
		UltrasonicSensor us1 = new UltrasonicSensor(SensorPort.S2);
		ObjectDetection odetect = new ObjectDetection(ls, us1);
		// od.Identify();
		LCD.clear();

		Navigation nav = new Navigation(odo, odetect, displayDist);
		Sweep sweeper = new Sweep(125, odetect, nav, odo);

		// Next, move the robot to the (30.48, 30.48) point in order to scan the
		// field
		// Note: This point is a safe point to go to because of the threshold
		// applied to the USlocalizer filter which
		// was at a distance of 50 and the point we are traveling to is less
		// than 50.

		// Assume the 9 by 9 grid where the robot localizes cotains nothing

		// initialize the ultrasonic sensor thread
		displayDist.start();

		// coordinates to go through:

		/*
		 * double[][] array = { { -15.24, 76.2 }, { 15.24, 76.2 }, { 45.72, 76.2
		 * }, { 76.2, 76.2 }, { 76.2, 106.68 }, { 45.72, 106.68 }, { 15.24,
		 * 106.68 }, { -15.24, 106.68 }, { -15.24, 137.16 }, { 15.24, 137.16 },
		 * { 45.72, 137.16 }, { 76.2, 137.16 }, { 76.2, 167.64 }, { 45.72,
		 * 167.64 }, { 15.24, 167.64 }, { -15.24, 167.64 }, { -15.24, 198.12 },
		 * { 15.24, 198.12 }, { 45.72, 198.12 }, { 76.2, 198.12 } };
		 */

		double[][] array = { { -15.24, 198.12 }, { 15.24, 198.12 },
				{ 15.24, 76.2 }, { 45.72, 76.2 }, { 45.72, 198.12 },
				{ 76.2, 198.12 }, { 76.2, 76.2 } };
		// double [][] array = {{76.2, 198.12}};

		Stack<Point> st = new Stack<Point>();
		Stack<Point> safe = new Stack<Point>();
		Point blueBlock;
		int i = array.length - 1;
		while (i >= 0) {
			st.push(new Point(array[i][0], array[i][1], false));
			i--;
		}
		// travel to the first point
		foundSomething = nav.travelTo(-15.24, 0, 14);
		Point a;
		while (!st.empty()) {
			int temp = 2;
			a = st.pop();
			foundSomething = nav.travelTo(a.x, a.y, 14);
			// identify the object that was found.
			if (foundSomething == true) {
				temp = odetect.identify();
			}
			if (temp == 2) {
				safe.push(new Point(a.x, a.y, true));
			} else if (temp == 1) {
				blueBlock = new Point(odo.getX(), odo.getY(), true);
				headMotor.rotateTo(-240);
				// nav.avoidBlock(a.x, a.y, odo.getX(), odo.getY(),
				// odo.getAng());
				nav.travelToGoal(76.2, 198.12, 14);
				Sound.beep();
				Sound.beep();
				Sound.beep();
				break;
			} else if (temp == 0) {
				nav.avoidBlock(a.x, a.y, odo.getX(), odo.getY(), odo.getAng());
			}
		}
		// wait for any button press
		Button.waitForAnyPress();

		// any button press to exit.
		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);
	}

}

class Point {
	double x;
	double y;
	boolean visited;

	public Point(double x, double y, boolean visited) {
		this.x = x;
		this.y = y;
		this.visited = visited;
	}
	/*
	 * public int getX() { return this.x; }
	 * 
	 * public int getY() { return this.y; }
	 * 
	 * public boolean getVisited() { return this.visited; }
	 */

}