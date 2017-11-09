package ca.mcgill.ecse211.project;

import com.sun.org.apache.bcel.internal.generic.GOTO;
import com.sun.xml.internal.ws.org.objectweb.asm.Label;

import ca.mcgill.ecse211.project.main.Global;
import lejos.hardware.Button;

/**
 * The main thread of the project. It controls the activation of other threads
 * when it needs them by using the flag defined in {@link main.Global} like
 * {@link main.Global.usSwitch}
 * 
 * @author Chen He
 * @author Vincent d'Orsonnens
 * @version 1.0
 */
public class Navigation extends Thread {

	public Navigation() {
	}

	/**
	 * Running this thread makes the robot achieve a series of task necessary to
	 * complete its goal.
	 */
	public void run() {
		try {
			// initial positioning
			fallingEdge();
			lightPosition();
			setStartingCorner();

			// Travel to transit location
			travelToTransit(true);	
			
			// Cross transit
			if (Global.teamColor == Global.TeamColor.GREEN)
				travelZipline();
			else
				travelWater();
			
			Global.usSwitch = true;
			Global.odometerSwitch = true;
			
			fallingEdge();
			lightPosition();
			
			Button.waitForAnyPress();
			System.exit(0);
			
			// find the flag
			findFlag();

			// go back to transit point
			travelToTransit(false);

			// Go back to starting zone
			if (Global.teamColor == Global.TeamColor.GREEN)
				travelWater();
			else
				travelZipline();

			// Done
			Button.waitForAnyPress();
			System.exit(0);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	
	/**
	 * Travels to the right transit point (zipline or water) depending on which zone
	 * the robot is currently in. This method make sure that we do not have to avoid
	 * any obstacle.
	 * 
	 * @param zone
	 *            True means we are in our own zone, False means we are in the
	 *            opponent's
	 */
	public void travelToTransit(boolean zone) {
		try {
			// first transit
			if (zone) {
				// go to zoneZipline0
				if (Global.teamColor == Global.TeamColor.GREEN) {
					travelTo(Global.zoneZiplineO[0], Global.zoneZiplineO[1]);
					// reset angle
					if (Global.startingCorner == 1) 
						Global.angle = 90;
					else // corner 0
						Global.angle = 0;
				}
				// go to shallowHLL
				else {
					travelTo(Global.shallowHLLx, Global.shallowHLLy);
				}
			}

			// second transit
			else {
				if (Global.teamColor == Global.TeamColor.GREEN) {
					travelTo(Global.shallowHLLx, Global.shallowHLLy);
				} else {
					travelTo(Global.oppZiplineO[0], Global.oppZiplineO[1]);
				}
			}
		} catch (Exception e) {
		}
	}
	
	
	/**
	 * Travels the zipline. The robot has to be at the position specified by
	 * {@link Global.zoneZipline0} or {@link Global.oppZipline0} before calling this
	 * method. The robot then aligns itself with the zipline and traverse it.
	 * 
	 * @throws Exception
	 */
	public void travelZipline() throws Exception {
		
		Global.ziplineMotor.setSpeed(Global.MOVING_SPEED);
		Global.ziplineMotor.backward();
		
		double dx = Math.abs(Global.zoneZipline[0] - Global.zoneZiplineO[0]);
		double dy = Math.abs(Global.zoneZipline[1] - Global.zoneZiplineO[1]);
		double angleWithY = Math.atan(dx/dy);
		angleWithY = angleWithY * 180 / Math.PI;
		
		if (Global.zoneZipline[0] < Global.zoneZiplineO[0])
			angleWithY *= -1;
		
		if (angleWithY!=0) { // different X
			turn(angleWithY, false);
		}
		
		move(Global.ZIPLINE_LENGTH, true);
		Thread.sleep(9000);
		Global.leftMotor.stop();
		Global.rightMotor.stop();
		Global.leftMotor.flt();
		Global.rightMotor.flt();
		
		Thread.sleep(Global.ZIPLINE_TIME);
		Global.ziplineMotor.stop();
		
	}

	/**
	 * Traverse the shallow water. The robot always traverse water from the red zone
	 * @throws Exception 
	 * 
	 *
	 */
	public void travelWater() throws Exception {
		travelTo(Global.shallowVLLx, Global.shallowVLLy-1);
		move(Global.ROBOT_LENGTH, false);
		
		turn(-90, false);
		
		travelY(Global.shallowHLLy);
		move(Global.ROBOT_LENGTH, false);
		
		turn(90, false);
		move(-Global.SQUARE_LENGTH, false);
		
		travelX(Global.shallowHLLx-1);		
		
	}

	/**
	 * Makes the robot search the opponent's zone for the flag. It uses the
	 * ultrasonic sensor and both color sensors. When it finds the flag, it beeps 3
	 * times and the method returns.
	 * @throws Exception 
	 */
	public void findFlag() throws Exception {
		Global.usSwitch = true;
		Global.frontColorSensorSwitch =true;
		travelTo(Global.searchLL[0],Global.searchLL[1]);
		
		boolean findflag = true;
		int count = 0;
		
		//turn and scan for flag
		while (findflag) {
			count++;
			turn(-5, false);
			if (Global.ObstacleDistance<=40) {
				Global.rightMotor.stop();
				Global.leftMotor.stop();
				move(Global.SQUARE_LENGTH*1.5, false);
				if (Global.frontColorID==Global.flagColor) {
					Button.waitForAnyPress();
					move(-Global.SQUARE_LENGTH*1.5, false);
				}
				else {
					move(-Global.SQUARE_LENGTH*1.5, false);
				}
			}
		}
		
		//return to initial position
		while(count-->0) {
			turn(5, false);
		}
	}
	
	/**
	 * Make the robot travel to a given X and Y position. It first moves along the X
	 * axis, then along the Y axis. It also performs wall correction to assure the
	 * robot is moving in a straight line. It uses the left color sensor to detect
	 * lines as it's moving.
	 * @throws Exception 
	 * @param x
	 *            The target x coordinate
	 * @param y
	 *            The target y coordinate
	 */
	public void travelTo(int x, int y) throws Exception {
		// activate required threads
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);

		if (Global.angle == 90 || Global.angle == 270) {
			travelY(y);
			
			// turn to the correct direction using the black lines
			turn(-Global.KEEP_MOVING, true);
			Thread.sleep(Global.THREAD_SLEEP_TIME);
			Global.BlackLineDetected = false;
			while (!Global.BlackLineDetected) {}
			turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
			
			travelX(x);
			turn(90, false);
		} else {
			travelX(x);
			
			// turn to the correct direction using the black lines
			turn(-Global.KEEP_MOVING, true);
			Thread.sleep(Global.THREAD_SLEEP_TIME);
			Global.BlackLineDetected = false;
			while (!Global.BlackLineDetected) {}
			turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
			
			travelY(y);
		}
		
		Global.leftColorSensorSwitch = false;
		
		Global.firstLine = "X: " + Global.X;
		Global.secondLine = "Y: " + Global.Y;
		Global.thirdLine = "";
		Global.forthLine = "";
	}
	
	/**
	 * Moves the robot along the X-axis only.
	 * Assumes that {@link Global.X} is set the 
	 * the right value before the execution. It travels 
	 * by counting the lines.
	 * @param x 	The x coordinate of the destination
	 * @throws Exception
	 */
	public void travelX(int x) throws Exception {
		// move across x
		Global.thirdLine = "travel x";
		if (x != Global.X) {// verify if moving in x is needed

			// moving forward
			if (x > Global.X) {
				// move(-30,false);//wall correction

				move(Global.KEEP_MOVING, true);// keep moving forward

				while (Global.X < x) {// count the blacklines traveled and stop when the destination is reached
					Global.forthLine = "" + Global.X;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.X++;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}

				}
				move(-Global.ROBOT_LENGTH, false);// position the robot to the center
			}
			// moving backward
			else {
				// move(30,false);//wall correction

				move(Global.KEEP_MOVING, true);// keep moving backward

				while (Global.X > x) {// count the blacklines traveled and stop when the destination is reached
					Global.forthLine = "" + Global.X;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.X--;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}

				move(-Global.ROBOT_LENGTH, false);// position the robot to the center
			}
		}
	}

	/**
	 * Moves the robot along the Y-axis only.
	 * Assumes that {@link Global.Y} is set the 
	 * the right value before the execution. It travels 
	 * by counting the lines.
	 * @param y 	The y coordinate of the destination
	 * @throws Exception
	 */
	public void travelY(int y) throws Exception {
		// update display
		Global.thirdLine = "travel y;";
		Global.forthLine = "" + Global.Y;

		// move across y
		if (y != Global.Y) {// if moving in y is needed
			while (Global.leftMotor.isMoving()) {

			}
			if (y > Global.Y) {
				// move(-30,false);//wall correction

				move(Global.KEEP_MOVING, true);// keep moving forward

				while (Global.Y < y) {// count the blacklines traveled and stop when the destination is reached
					Global.forthLine = "" + Global.Y;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.Y++;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
			} else {
				// move(30,false);//wall correction

				move(Global.KEEP_MOVING, true);// keep moving backward

				while (Global.Y > y) {// count the blacklines traveled and stop when the destination is reached
					Global.forthLine = "" + Global.Y;
					if (Global.BlackLineDetected) {
						Global.BlackLineDetected = false;
						Global.Y--;
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
			}
		}
		move(-Global.ROBOT_LENGTH, false);// reposition the robot the the center
	}
	
	
	public void afterZiplineLocalization() throws Exception {	
		// start left color sensor
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);
		
		//Gets off the zipline
		if(Global.zoneZiplineO[0] != Global.oppZiplineO[0]) {
			move(Global.SQUARE_DIAGONAL, false);
		}
		else {
			move(Global.SQUARE_LENGTH, false);
		}
		
		//Turns away from the red zone's back wall
		if(Global.zoneZipline[0] > Global.oppZipline[0]) {
			turn(135, false);
		}
		else if(Global.zoneZipline[0] < Global.oppZipline[0]) {
			turn(-135, false);
		}
		else {
			turn(180, false);
		}
		
		//Wall correction
		move(-45, false);
		move(Global.KEEP_MOVING, true);
		while (!Global.BlackLineDetected) {}
		move(-Global.ROBOT_LENGTH, false);
		
		Global.X = Global.oppZiplineO[0];
		Global.Y = 7;
		Global.angle = 270;
		
		
		if(Global.zoneZipline[0] > Global.oppZipline[0] && Global.oppZiplineO[0] != 1) {
			turn(-90, false);
			travelTo(1, 7);
			Global.angle = 180;
		}
		else if(Global.zoneZipline[0] < Global.oppZipline[0] && Global.oppZiplineO[0] != 7) {
			turn(90, false);
			travelTo(7,7);
			Global.angle = 0;
		}
		else {
			if(Global.zoneZipline[0] >=4) {
				turn(-90, false);
				Global.angle = 0;
			}
			else {
				turn(90, false);
				Global.angle = 180;
			}
			move(125, false);
		}
		move(-Global.KEEP_MOVING, true);
		while (!Global.BlackLineDetected) {}
		move(-Global.ROBOT_LENGTH, false);
		afterZiplineLocalization();
	}
	

	/**
	 * Converts an angle in degrees that the robot has to turn to an angle in
	 * degrees that a wheel has to rotate.
	 * 
	 * @param radius
	 *            The radius of the wheel in cm
	 * @param width
	 *            The width of the robot in cm
	 * @param angle
	 *            The angle in degrees we want the robot to turn
	 * @return The angle in degrees the wheels has to rotate
	 */
	private int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * Converts a distance in cm to an angle in degrees that the wheel with a
	 * certain radius needs to rotate to move the robot the specified distance.
	 * 
	 * @param radius
	 *            The radius of the wheel in cm
	 * @param distance
	 *            The distance we want the robot to move in cm
	 * @return The angle in degrees the wheel has to rotate
	 */
	private int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * Make the robot travel forward at a fixed speed specified by
	 * {@link Global.MOVING_SPEED}. It will travel the distance passed in
	 * centimeter. The method can wait until the distance is traveled before
	 * returning, or it can return immediately, depending on the immediatereturn
	 * flag.
	 * 
	 * @param distance
	 *            the distance in cm the robot has to travel
	 * @param immediatereturn
	 *            Specify if the method should wait for the wheels to be done before
	 *            returning
	 */
	public void move(double distance, boolean immediatereturn) throws Exception {

		Global.leftMotor.setSpeed(Global.MOVING_SPEED);
		Global.rightMotor.setSpeed(Global.MOVING_SPEED);

		Global.leftMotor.rotate(convertDistance(Global.WHEEL_RADIUS, distance), true);
		Global.rightMotor.rotate(convertDistance(Global.WHEEL_RADIUS, distance), immediatereturn);

		Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
	}

	/**
	 * Rotate the wheels in order to turn the robot in a clockwise fashion. If the
	 * angle parameter is negative, the robot will turn counter-clockwise. This
	 * method can return before the wheels are finished rotating if the
	 * immediatereturn parameter is true. Else it will wait until the wheels are
	 * done rotating.
	 * 
	 * @param angle
	 *            The angle specifying how much the to turn in degrees
	 * @param immediatereturn
	 *            Specify if the method should wait for the wheels to be done before
	 *            returning
	 */
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

	/**
	 * Performs the falling edge localization routine. It turns on the appropriate
	 * {@link UltrasonicSensor} thread at the start and uses the data from the
	 * sensor to localize. It turns the sensor thread off at the end.
	 * 
	 * @throws Exception
	 */
	public void fallingEdge() throws Exception {
		Global.rightMotor.setAcceleration(3000);
		Global.leftMotor.setAcceleration(3000);

		// start the corresponding sensor thread
		Global.usSensorThread.start();
		Global.usSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);
		Global.odometerThread.start();
		Global.odometerSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);

		int Angle = 0;

		// make sure there is no wall in front
		while (Global.ObstacleDistance < Global.USThreshhold) {
			turn(90, false);
		}

		// make the robot face a wall
		turn(Global.KEEP_MOVING, true);
		while (Global.ObstacleDistance > Global.USThreshhold) {
		}
		turn(Global.STOP_MOVING, false);

		// set this angle as starting angle
		for (int i = 0; i < 5; i++) {
			Global.angle = 0;
		}

		// redo same thing for other side
		turn(-90, false);
		turn(-Global.KEEP_MOVING, true);
		while (Global.ObstacleDistance > Global.USThreshhold) {
		}
		turn(Global.STOP_MOVING, false);

		// read angle and make it positive
		Angle = (int) Global.angle;

		// divide by 2 and add 45
		if (Angle > 360) {// small correction to make sure it make no big cercles
			Angle -= 360;
		}
		Angle = Angle >> 1;
		Angle += 45;

		turn(Angle, false);
		Global.angle = 0;
		Global.usSwitch = false;
		Global.odometerSwitch = false;
		Global.rightMotor.setAcceleration(Global.ACCELERATION);
		Global.leftMotor.setAcceleration(Global.ACCELERATION);
	}

	/**
	 * Performs a simple light positioning routine to position the robot at the
	 * intersection of two black lines, facing in the positive X direction (angle
	 * 0). It uses the left color sensor.
	 * 
	 * @throws Exception
	 */
	public void lightPosition() throws Exception {
		// start the corresponding sensor thread
		Global.leftColorSensorSwitch = true;
		Global.secondLine = "light positionning";
		Thread.sleep(Global.THREAD_SLEEP_TIME); // wait color sensor to get its values

		// reset X
		// move until sensor sees black line
		move(Global.KEEP_MOVING, true);
		Global.BlackLineDetected = false;
		while (!Global.BlackLineDetected) {
		}

		// move back to black line
		move(-Global.ROBOT_LENGTH, false);
		Thread.sleep(250);

		// reset angle
		// turn until color sensor sees a black line then turn to 90 degree
		turn(-Global.KEEP_MOVING, true);
		while (!Global.BlackLineDetected) {
		}
		turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);

		// reset Y
		// move until sensor sees black line
		move(Global.KEEP_MOVING, true);
		Global.BlackLineDetected = false;
		while (!Global.BlackLineDetected) {
		}

		// move back to black line
		move(-Global.ROBOT_LENGTH, false);
		Thread.sleep(250);

		turn(90, false);
		
		// wall correction
		move(-30, false);
		move(Global.KEEP_MOVING, true);
		while(!Global.BlackLineDetected) {}
		move(-Global.ROBOT_LENGTH, false);
		
		// turn off color sensor
		Global.leftColorSensorSwitch = false;

		// wait color sensor is turned off
		Thread.sleep(200);
		
		// reset coordinates
		Global.angle = 0;
		Global.secondLine = "";
	}
	
	/**
	 * Set the X, Y and angle coordinates
	 * of the robot depending on the corner
	 * it starts in. To be used after the initial
	 * localization.
	 */
	public void setStartingCorner() {
		switch (Global.startingCorner) {
		case 0:
			Global.X = 0;
			Global.Y = 0;
			Global.angle = 0;
			break;
		case 1:
			Global.X = 8;
			Global.Y = 0;
			Global.angle = 90;
			break;
		case 2:
			Global.X = 12;
			Global.Y = 12;
			Global.angle = 180;
			break;
		case 3:
			Global.X = 0;
			Global.Y = 12;
			Global.angle = 270;
		}
	}
}
