/*
 * OdometryDisplay.java
 */
import lejos.nxt.LCD;
import lejos.nxt.ColorSensor;
import lejos.nxt.SensorPort;

public class ColorDisplay extends Thread {
	private static final long DISPLAY_PERIOD = 250;
	private ColorSensor cs = new ColorSensor(SensorPort.S1);
	private int colorValue;

	// constructor
	public ColorDisplay(ColorSensor cs) {
		this.cs = cs;
		this.colorValue = colorValue;
	}

	// run method (required for Thread)
	public void run() {
		long displayStart, displayEnd;
	
		// clear the display once
		//LCD.clearDisplay();

		while (true) {
			displayStart = System.currentTimeMillis();
			cs.setFloodlight(2);
			// clear the lines for displaying odometry information
			LCD.clear();
			LCD.drawString("Light Sensor:   ", 0, 1);
			colorValue = cs.getNormalizedLightValue();
			LCD.drawString(String.valueOf(colorValue), 3, 2);

			
			
			// throttle the OdometryDisplay
			displayEnd = System.currentTimeMillis();
			if (displayEnd - displayStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (displayEnd - displayStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that OdometryDisplay will be interrupted
					// by another thread
				}
			}
		}
	}

	public int getColorV() {
		return this.colorValue;
	}
	
	//EXPERIMENT FAILED
	public int identifyBlock(ColorDisplay color){
		int count = 0;
		int samples = 15;
		int averageValue = 0;
		while(samples > count){
			averageValue = averageValue + color.getColorV();
			count++;
		}
		
		//divide by sample to get the average
		averageValue = averageValue/samples;
		
		
		
		return averageValue;
	}

}
