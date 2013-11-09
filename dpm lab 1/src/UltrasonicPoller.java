import lejos.nxt.UltrasonicSensor;

public class UltrasonicPoller extends Thread {
	private UltrasonicSensor us;
	private UltrasonicController cont;

	public UltrasonicPoller(UltrasonicSensor us, UltrasonicController cont) {
		this.us = us;
		this.cont = cont;
	}
	int distance = 0;
	int filter = 0;
	int filterBound = 17;
	public void run() {
		
		while (true) {
			// process collected data
			distance = us.getDistance();
			
			//incorrect value, do not set the distance but increment the filter value
			if (distance == 255 && filter < filterBound) {
				filter++;
			}
			// The value is correct, set the distance to 255.
			else if (distance == 255 && filter == filterBound) {
				cont.processUSData(distance);
				filter = 0;
			}
			// The distance value went below 255. Use the distance value and reset the filter
			else {
				cont.processUSData(distance);
				filter = 0;
			}
			try {
				Thread.sleep(10);
			} catch (Exception e) {
			}
		}
	}

}
