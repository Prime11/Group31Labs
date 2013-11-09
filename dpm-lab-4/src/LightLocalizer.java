import lejos.nxt.ColorSensor;
import lejos.nxt.LightSensor;

public class LightLocalizer {
	private Odometer odo;
	private CopyOfOdometer cOdo;
	private TwoWheeledRobot robot;
	private ColorSensor ls;
	private Navigation hodor;
	private CopyOfNavigation codor;
	
	public LightLocalizer(Odometer odo, ColorSensor ls) {
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		this.hodor = new Navigation(odo);
		// turn on the light
		ls.setFloodlight(true);
	}
	
	public LightLocalizer(CopyOfOdometer cOdo, ColorSensor ls) {
		this.cOdo = cOdo;
		//this.robot = odo.getTwoWheeledRobot();
		this.ls = ls;
		this.codor = new CopyOfNavigation(cOdo, 2.125, 14.5);
		// turn on the light
		ls.setFloodlight(true);
	}
	
	
	public void doLocalization() {
		// drive to location listed in tutorial
		//hodor.turnTo(Math.toRadians(45));
		//robot.goForwardDist();
		codor.travelTo(1, 1);
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
	}

}
