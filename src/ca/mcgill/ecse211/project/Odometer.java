package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;

public class Odometer extends Thread{
	private double leftMotorTachoCount;
	private double rightMotorTachoCount;
	private double ratio = Global.WHEEL_RADIUS/Global.TRACK;
	private static final int SLEEP_TIME = 150;
	
	public Odometer() {
		
	}
	public void run() {
		Global.angle=0;
		while(Global.odometerSwitch) {
			leftMotorTachoCount = Global.leftMotor.getTachoCount();
			rightMotorTachoCount = Global.rightMotor.getTachoCount();
			Global.leftMotor.resetTachoCount();
			Global.rightMotor.resetTachoCount();
			Global.angle += Math.toDegrees(ratio*Math.toRadians(rightMotorTachoCount-leftMotorTachoCount));
			if (Global.angle<0) {
				Global.angle+=360;
			}
			if (Global.angle>360) {
				Global.angle-=360;
			}
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
			}
		}
	}
}
