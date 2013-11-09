//Object Detection only works for objects that are in a specific range. In our case its calibrated for when the object is a at distance of 6 detected by the US.
//O

import lejos.nxt.ColorSensor;
import lejos.nxt.LCD;
import lejos.nxt.SensorPort;
import lejos.nxt.Sound;
import lejos.nxt.UltrasonicSensor;

public class ObjectDetection {
	private ColorSensor cs;
	private UltrasonicSensor us;
	private double redValue, blueValue;
	private double wallThreshold = 60;
	private int objectCount;
	private boolean sameObject;
	private double x1, x2;

	// constructor
	public ObjectDetection(ColorSensor cs, UltrasonicSensor us) {
		this.cs = cs;
		this.us = us;
		this.redValue = redValue;
		this.blueValue = blueValue;
		this.objectCount = 0;
		this.x1 = 0;
		this.x2 = 0;
		this.sameObject = false;
	}

	private int getFilteredData() {
		int distance;
		int count = 0;
		double previousDistance = 0;

		// do a ping
		us.ping();

		// wait for the ping to complete
		try {
			Thread.sleep(50);
		} catch (InterruptedException e) {
		}

		// there will be a delay here
		distance = us.getDistance();

		if (distance > wallThreshold) {
			distance = (int) wallThreshold;
			count++;
		}
		LCD.clear(7);
		LCD.drawInt(distance, 0, 7);
		if (distance == (int) wallThreshold && count > 1) {
			count = 0;
			return distance;
		} else if (distance == (int) wallThreshold && count == 1) {
			return (int) previousDistance;
		} else {
			previousDistance = distance;
			return distance;
		}
	}

	public void detect(double currentPosition) {
		this.wallThreshold = 91.2 - 12 - currentPosition;
		if (getFilteredData() < (this.wallThreshold - 10)) {
			// Sound.beep();
			x1 = getFilteredData();
			// LCD.clear(5);
			LCD.drawInt(getFilteredData(), 5, 5);
			if ((x1 - x2) < 0 && Math.abs(x1 - x2) < 5) {
				Sound.beep();
				if (sameObject == false) {
					this.sameObject = true;
					objectCount++;
					Sound.beep();
					LCD.clear(5);
					LCD.drawInt(objectCount, 0, 5);
				}
			} else {
				this.sameObject = false;
			}
			x2 = x1;
		}
	}

	// run method (required for Thread)
	public int identify() {
		LCD.clear();
		int returnValue = 2;

		// set the floodlight to red
		cs.setFloodlight(0);
		// read the normalized light value with the red floodlight
		redValue = cs.getNormalizedLightValue();

		// LCD.drawString(String.valueOf(colorValue), 3, 2);

		// set the floodlight to red
		cs.setFloodlight(2);
		// read the normalized light value with the red floodlight
		blueValue = cs.getNormalizedLightValue();

		// Valurs found through experimental data comparing the ratio of red
		// light to blue light, which was more accurate
		// than simply computing the difference between the two light values.

		// NOTE: we will have to return something when we call this function
		// later in the code.
		if ((redValue / blueValue) > 1.09) {
			LCD.drawString("OBJECT DETECTED!", 0, 5);
			LCD.drawString("NOT BLOCK!!!", 0, 6);
			returnValue = 0;
		}
		if ((redValue / blueValue) <= 1.09 && (redValue / blueValue) > 1.01) {
			LCD.drawString("OBJECT DETECTED!", 0, 5);
			LCD.drawString("BLOCK!!!", 0, 6);
			returnValue = 1;
			
		}
		if ((redValue / blueValue) <= 1.01) {
			LCD.drawString("NO OBJECT FOUND!", 0, 5);
			returnValue = 2;
		}
		return returnValue;
		
	}
	

}
