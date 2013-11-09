/*
 * Lab3.java
 * (Modified from Lab2.java)
 */

/* Written and modified by:
 * Hadi Sayar, Student ID: 260531679 
 * Antonio D'Aversa, Student ID: 260234498
 */

import lejos.nxt.*;

public class Lab3 {
	static final SensorPort usPort = SensorPort.S2;
	
	public static void main(String[] args) {
		int buttonChoice;
		final double wheelRadius = 2.125;
		final double robotDiameter = 14.9; // 14.975
		
		// some objects that need to be instantiated
		Odometer odometer = new Odometer();
		UltrasonicSensor usSensor = new UltrasonicSensor(usPort);
		Navigation gps = new Navigation(odometer, wheelRadius, robotDiameter);
		NavigationWithUS garmin = new NavigationWithUS(odometer, wheelRadius,
				robotDiameter, usSensor);

		do {
			// clear the display
			LCD.clear();
			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("   Nav | Nav    ", 0, 1);
			LCD.drawString("       | with US", 0, 2);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		// start the odometer, the odometry display and (possibly) the
		// odometry correction

		//44180
		//42138
		
		//Left button choice initiates the odometer and lets it navigate to the desired hardcoded
		//coordinates
		if (buttonChoice == Button.ID_LEFT) {
			odometer.start();
			gps.travelTo(60, 30);
			gps.travelTo(30, 30);
			gps.travelTo(30, 60);
			gps.travelTo(60, 0);
		} 
		//Right button choice initiates the odometer and lets it navigate to the desired hardcoded
		//coordinates while avoiding obstances.
		else {
			odometer.start();
			garmin.travelTo(0, 60);
			garmin.travelTo(60, 0);

		}
		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}