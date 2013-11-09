import lejos.nxt.LCD;
import lejos.nxt.UltrasonicSensor;

public class Sweep {

	private int wallThreshold;
	private int speed;
	private TwoWheeledRobot robot;
	private ObjectDetection detector;
	private Navigation nav;
	private Odometer odo;
	private double[] pos;

	public Sweep(int speed, ObjectDetection detector, Navigation nav,
			Odometer odo) {
		this.speed = speed;
		this.odo = odo;
		this.robot = odo.getTwoWheeledRobot();
		this.nav = nav;
		// this.nav = new Navigation(this.odo);
		this.detector = detector;
		pos = new double[3];
	}

	public void scanRightTwoSeventy() {
		nav.turnTo(Math.toRadians(270));
		robot.setRotationSpeed(speed);
		// robot.setSpeeds(300, 175);
		robot.rotateClockwise();
		this.odo.getPosition(pos);
		// LCD.clear(6);
		// LCD.drawInt((int) pos[2], 0, 6);
		while (pos[2] < 180 || pos[2] > 260) {
			this.odo.getPosition(pos);
			detector.detect(pos[0]);
		}
		robot.stop(0);
		robot.setRotationSpeed(speed);
		robot.rotateCounterClockwise();
		this.odo.getPosition(pos);
		while (pos[2] > 5 && pos[2] < 355) {
			this.odo.getPosition(pos);
		}
		robot.stop(0);
	}

	public void scanLeftTwoSeventy() {
		nav.turnTo(Math.toRadians(90));
		robot.stop(0);
		robot.setRotationSpeed(speed);
		robot.rotateCounterClockwise();
		this.odo.getPosition(pos);
		LCD.drawInt((int) pos[2], 0, 6);
		while (pos[2] >= 180 || pos[2] <= 110) {
			this.odo.getPosition(pos);
			LCD.drawString("FirstLoop", 0, 7);
			LCD.drawInt((int) pos[2], 0, 6);
			detector.detect( pos[0]);
		}
		robot.stop(0);
		robot.setRotationSpeed(speed);
		robot.rotateClockwise();
		this.odo.getPosition(pos);
		while (pos[2] > 5 && pos[2] < 355) {
			this.odo.getPosition(pos);
			LCD.drawString("SecondLoop", 0, 7);
		}
		robot.stop(0);
	}

	public void scanOneEighty() {
		nav.turnTo(Math.toRadians(270));
		robot.setRotationSpeed(speed);
		// robot.setSpeeds(300, 175);
		robot.rotateClockwise();
		this.odo.getPosition(pos);
		// LCD.clear(6);
		// LCD.drawInt((int) pos[2], 0, 6);
		while (pos[2] > 270 || pos[2] < 90) {
			this.odo.getPosition(pos);
			detector.detect(pos[0]);
		}
		robot.stop(0);
		robot.setRotationSpeed(speed);
		robot.rotateCounterClockwise();
		this.odo.getPosition(pos);
		while (pos[2] > 5 && pos[2] < 355) {
			this.odo.getPosition(pos);
		}
		robot.stop(0);
	}

	public void scanThreeSixty() {
		nav.turnTo(Math.toRadians(20));
		robot.setRotationSpeed(speed);
		// robot.setSpeeds(300, 175);
		robot.rotateClockwise();
		this.odo.getPosition(pos);
		while (pos[2] > 5 && pos[2] < 355) {
			this.odo.getPosition(pos);
			detector.detect(pos[0]);
		}
		robot.stop(0);
	}
}
