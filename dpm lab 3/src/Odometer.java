/*
 * Odometer.java
 * (Re-used from the previous odometry lab)
 * 
 * Written and modified by: 
 * Hadi Sayar ID: 260531679
 * Antonio D'Aversa ID: 260234498
 * 
 */

import lejos.nxt.Motor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;

	/* Retrieve the tachometer counts for each motor. */
	private double rightTacho, leftTacho;
	private double previousRightTacho = 0, previousLeftTacho=0;

	/* Declare the known variables and then do some math with them */
	final private double omega = 14.9; //14.6/* width between wheels */ //14.975
	final private double wheelRadius = 2.125; /* radius of either wheel */

	/*
	 * The Delta values that need to be calculated by the odometer (need to add
	 * more)
	 */
	double deltaC;
	double deltaTheta;

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer() {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
	}

	
	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			
			//Compute new tacho counts in Radians
			rightTacho = Motor.B.getTachoCount()*Math.PI/180;
			leftTacho = Motor.A.getTachoCount()*Math.PI/180;
			
			//Compute deltaC and deltaTheta
			deltaC = (((leftTacho-previousLeftTacho) + (rightTacho-previousRightTacho)) * wheelRadius) / 2;

			deltaTheta = (((leftTacho-previousLeftTacho) - (rightTacho-previousRightTacho)) * wheelRadius) / omega;
			
			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				/* Working code DO NOT TOUCH*/
				
				//Convert to Radians to properly compute.
				//theta = Math.toRadians(theta);

				//Updates x and y on Odometer with new values.
				theta += deltaTheta;
				y += deltaC * Math.cos((theta + deltaTheta / 2));
				x += deltaC * Math.sin((theta + deltaTheta / 2));				
				
				//Converts Theta to degrees for odometer display
				//theta = Math.toDegrees(theta);

			}
			// Sets Tacho Counts for next loop 
			previousLeftTacho = leftTacho;
			previousRightTacho = rightTacho;

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}