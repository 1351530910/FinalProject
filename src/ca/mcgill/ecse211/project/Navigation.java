package ca.mcgill.ecse211.project;

import java.util.Timer;

import ca.mcgill.ecse211.project.main.Global;
import lejos.hardware.Button;

public class Navigation extends Thread {

	public Navigation() {

	}

	public void run() {
		try {
			Button.waitForAnyPress();
			System.exit(0);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}	

	public void travelTo(int x, int y) throws Exception {
		//this is working only if we start from the starting point
		// start requiring threads
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);
		
		//because our color sensor is behind the robot
		
		
		// move across x
		Global.thirdLine = "travel x";
		if (x != Global.X) {// verify if moving in x is needed

			//moving forward
			if (x > Global.X) {
				move(-30,false);//wall correction
				
				move(Global.KEEP_MOVING, true);//keep moving forward
				
				while (Global.X <= x) {//count the blacklines traveled and stop when the destination is reached
					Global.forthLine = ""+Global.X;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.X++;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
					
				}
				move(-Global.ROBOT_LENGTH, false);//position the robot to the center
			} 
			//moving backward
			else {
				move(30,false);//wall correction
				
				move(-Global.KEEP_MOVING, true);//keep moving backward
				
				while (Global.X >= x) {//count the blacklines traveled and stop when the destination is reached
					Global.forthLine = ""+Global.X;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.X--;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
				
				move(-Global.ROBOT_LENGTH, false);//position the robot to the center
			}
		}
		
		//turn to the correct direction using the black lines
		turn(-Global.KEEP_MOVING, true);
		Thread.sleep(Global.THREAD_SLEEP_TIME);
		Global.BlackLineDetected = false;
		while (!Global.BlackLineDetected) {

		}
		turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
		
		//update display
		Global.thirdLine = "travel y;";
		Global.forthLine = ""+Global.Y;
		
		
		// move across y
		if (y != Global.Y) {//if moving in y is needed
			while(Global.leftMotor.isMoving()) {
				
			}
			if (y > Global.Y) {
				move(-30,false);//wall correction
				
				move(Global.KEEP_MOVING, true);//keep moving forward
				
				while (Global.Y <= y) {//count the blacklines traveled and stop when the destination is reached
					Global.forthLine = ""+Global.Y;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.Y++;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
			} 
			else {
				move(30,false);//wall correction
				
				move(-Global.KEEP_MOVING, true);//keep moving backward
				
				while (Global.Y >= y) {//count the blacklines traveled and stop when the destination is reached
					Global.forthLine = ""+Global.Y;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.Y--;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
			}
		}
		
		
		move(-Global.ROBOT_LENGTH, false);//reposition the robot the the center
		
		// turn and rescan the angle
		turn(90, false);
		Global.leftColorSensorSwitch = false;
		
		//wall correction
		move(-30,false);
		move(30,false);
	}

	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	public void move(double distance, boolean immediatereturn) throws Exception {

		Global.leftMotor.setSpeed(Global.MOVING_SPEED);
		Global.rightMotor.setSpeed(Global.MOVING_SPEED);

		Global.leftMotor.rotate(convertDistance(Global.WHEEL_RADIUS, distance), true);
		Global.rightMotor.rotate(convertDistance(Global.WHEEL_RADIUS, distance), immediatereturn);

		Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
	}

	public void turn(double angle, boolean immediatereturn) throws Exception {
		// clockwise positive
		Global.turning = true;
		Global.leftMotor.setSpeed(Global.ROTATING_SPEED);
		Global.rightMotor.setSpeed(Global.ROTATING_SPEED);
		if (angle > 0) {
			Global.leftMotor.rotate(convertAngle(Global.WHEEL_RADIUS, Global.TRACK, angle), true);
			Global.rightMotor.rotate(-convertAngle(Global.WHEEL_RADIUS, Global.TRACK, angle), immediatereturn);
		} else {
			angle *= -1;
			Global.leftMotor.rotate(-convertAngle(Global.WHEEL_RADIUS, Global.TRACK, angle), true);
			Global.rightMotor.rotate(convertAngle(Global.WHEEL_RADIUS, Global.TRACK, angle), immediatereturn);
		}
		Global.turning = false;
		Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
	}
}
