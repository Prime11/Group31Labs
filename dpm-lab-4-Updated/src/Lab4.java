import lejos.nxt.*;

public class Lab4 {

	public static void main(String[] args) {
		Button.waitForAnyPress();
		// setup the odometer, display, and ultrasonic and light sensors
		TwoWheeledRobot patBot = new TwoWheeledRobot(Motor.A, Motor.B);
		Odometer odo = new Odometer(patBot, true);
		LCDInfo lcd = new LCDInfo(odo);
		UltrasonicSensor us = new UltrasonicSensor(SensorPort.S2);
		ColorSensor ls = new ColorSensor(SensorPort.S1);
		// perform the ultrasonic localization
		USLocalizer usl = new USLocalizer(odo, us, USLocalizer.LocalizationType.FALLING_EDGE);
		LightLocalizer lsl = new LightLocalizer(odo, ls);
		
		//remove for now
		usl.doLocalization();
		
		// perform the light sensor localization
		//lsl.doLocalization();		
		
		Button.waitForAnyPress();
		
	}

}
