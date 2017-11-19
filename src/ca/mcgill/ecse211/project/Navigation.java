package ca.mcgill.ecse211.project;

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

	private double convertAngleConstant;
	private double convertDistanceConstant;
	
	private double angleZipline;
	
	/**
	 * The constructor compute values that will stay constant during the
	 * navigation.
	 */
	public Navigation() {
		convertDistanceConstant = (180.0 / (Math.PI * Global.WHEEL_RADIUS));
		convertAngleConstant = convertDistanceConstant*Math.PI * Global.TRACK  / 360.0;
		angleZipline = 0.0;
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
			if (Global.teamColor == Global.TeamColor.GREEN) {
				travelZipline();
				afterZiplineLocalization();
			}
			else
				travelWater();
			
			
			// turn to the right direction
			int dx = Global.searchLL[0] - Global.X;
			int dy = Global.searchLL[1] - Global.Y;
			if (dx > 0) {
				Global.X--;
				if (dy > 0) {
					Global.Y--;
					turn(Global.angle, false);
					Global.angle = 0;
				} else {
					Global.Y++;
					turn(Global.angle - 270, false);
					Global.angle = 270;
				}
			} else {
				Global.X++;
				if (dy > 0 ) {
					Global.Y--;
					turn(Global.angle - 90, false);
					Global.angle = 90;
				} else {
					Global.Y++;
					turn (Global.angle - 180, false);
					Global.angle = 180;
				}
			}
			
			
			
			Global.firstLine = "" + Global.X;
			Global.secondLine = "" + Global.Y;
			Global.thirdLine = "" + Global.angle;
			
			// BETA: go to search zone.
			travelTo(Global.searchLL[0], Global.searchLL[1], false);
			
			Button.waitForAnyPress();
			System.exit(0);
			
			/*
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
			*/
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
				if (Global.teamColor == Global.TeamColor.GREEN)
					travelTo(Global.zoneZiplineO[0], Global.zoneZiplineO[1], true);
				// go to shallowHLL
				else
					travelTo(Global.shallowHLLx, Global.shallowHLLy, true);
			}

			// second transit
			else {
				if (Global.teamColor == Global.TeamColor.GREEN)
					travelTo(Global.shallowHLLx, Global.shallowHLLy, true);
				else
					travelTo(Global.oppZiplineO[0], Global.oppZiplineO[1], true);
			}
		} catch (Exception e) {}
	}
	
	
	/**
	 * Travels the zipline. The robot has to be at the position specified by
	 * {@link Global.zoneZipline0} or {@link Global.oppZipline0} before calling this
	 * method. The robot then aligns itself with the zipline and traverse it.
	 * 
	 * @throws Exception
	 */
	public void travelZipline() throws Exception {
		
		double dx = Global.zoneZipline[0] - Global.zoneZiplineO[0];
		double dy = Global.zoneZipline[1] - Global.zoneZiplineO[1];
		
		// calculate the orientation of the zipline
		double angleWithY = -1;
		if (dy > 0.1) {
			angleWithY = Math.atan(Math.abs(dx)/dy) * 180 / Math.PI;
			if (dx > 0.1)
				this.angleZipline = 90.0 - angleWithY;
			else
				this.angleZipline = 90.0 + angleWithY;
		} else if (dy < -0.1) {
			dy *= -1;
			angleWithY = Math.atan(Math.abs(dx)/dy) * 180 / Math.PI;
			if (dx > 0.1)
				this.angleZipline = 270 + angleWithY;
			else
				this.angleZipline = 270 - angleWithY;
		} else {
			if (dx > 0.1)
				this.angleZipline = 0;
			else
				this.angleZipline = 180;
		}
		
		// turn robot to zipline's angle using the smallest angle
		double angleToTurn = Global.angle - this.angleZipline;
		if (angleToTurn < -180)
			angleToTurn = 360 + angleToTurn;
		else if (angleToTurn > 180)
			angleToTurn = 360 - angleToTurn;
		
		Thread.sleep(1000);
		
		turn(angleToTurn, false);
		
		// traverse zipline
		Global.ziplineMotor.setSpeed(Global.MOVING_SPEED);
		Global.ziplineMotor.backward();
		move(60, false);
		Global.leftMotor.stop();
		Global.rightMotor.stop();
		Global.leftMotor.flt();
		Global.rightMotor.flt();
		
		Thread.sleep(Global.ZIPLINE_TIME);
		Global.ziplineMotor.stop();
		turn(-15, false); // correct landing
	}

	/**
	 * Traverse the shallow water. The robot always traverse water from the red zone
	 * @throws Exception 
	 * 
	 *
	 */
	public void travelWater() throws Exception {
		travelTo(Global.shallowVLLx, Global.shallowVLLy-1, false);
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
		travelTo(Global.searchLL[0],Global.searchLL[1], false);
		
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
				Thread.sleep(Global.THREAD_SLEEP_TIME);
				if (Global.flagDetected) {
					findflag = false;
					Button.waitForAnyPress();
					move(-Global.SQUARE_LENGTH*1.5, false);
				}
				else {
					move(-Global.SQUARE_LENGTH*1.5, false);
				}
			}
		}
		
		//return to initial orientation
		while(count-->0) {
			turn(5, false);
		}
	}
	
	/**
	 * Make the robot travel to a given X and Y position. It first moves along the X
	 * axis, then along the Y axis. It can also performs wall correction to assure the
	 * robot is moving in a straight line. It uses the left color sensor to detect
	 * lines as it's moving.
	 * @throws Exception 
	 * @param x
	 *            The target x coordinate
	 * @param y
	 *            The target y coordinate
	 * @param wallCorrection
	 * 			  If true, will perform wall correction after turning
	 */
	public void travelTo(int x, int y, boolean wallCorrection) throws Exception {
		// activate required threads
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);

		if (Global.angle == 90 || Global.angle == 270) {
			
			if (Math.abs(y-Global.Y) > 1)
				if (wallCorrection)
					move(-30, false);
			
			travelY(y);
			
			// turn to the correct direction using the black lines
			turn(-Global.KEEP_MOVING, true);
			Thread.sleep(Global.THREAD_SLEEP_TIME);
			Global.leftBlackLineDetected = false;
			while (!Global.leftBlackLineDetected) {}
			turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
			// update angle
			if (Global.angle == 90)
				Global.angle = 180;
			else
				Global.angle = 0;
			
			if (Math.abs(x-Global.X) > 1)
				if (wallCorrection)
					move(-30, false);
			
			travelX(x);
			
		} else {
			if (Math.abs(x-Global.X) > 1)
				if (wallCorrection)
					move(-30, false);
			travelX(x);
			
			// turn to the correct direction using the black lines
			turn(-Global.KEEP_MOVING, true);
			Thread.sleep(Global.THREAD_SLEEP_TIME);
			Global.leftBlackLineDetected = false;
			while (!Global.leftBlackLineDetected) {}
			turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
			// update angle
			if (Global.angle == 0)
				Global.angle = 90;
			else
				Global.angle = 270;
			
			if (Math.abs(y-Global.Y) > 1)
				if (wallCorrection)
					move(-30, false);
			
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
		Global.moving = true;
		Global.leftBlackLineDetected = false;
		int count = 0;
		
		if (x != Global.X) {// verify if moving in x is needed

			// moving forward
			if (x > Global.X) {
				
				if (x == Global.X + 1) {
					Global.X++;
					return;
				}
				
				move(Global.KEEP_MOVING, true);// keep moving forward

				while (Global.X < x) {// count the blacklines traveled and stop when the destination is reached
					if (Global.crossed()) {
						Global.X++;
						Global.leftBlackLineDetected = false;
						
						if (++count == 2) { // hacky correction
							move(0, false);
							turn(Global.CORR_ANGLE, false);
							move(Global.KEEP_MOVING, true);
							count = 0;
						}
						
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}

				}
				move(-Global.ROBOT_LENGTH, false);// position the robot to the center
			}
			// moving backward
			else if (x < Global.X) {
				
				if (x == Global.X - 1) {
					Global.X--;
					return;
				}
				
				move(Global.KEEP_MOVING, true);// keep moving backward

				while (Global.X > x) {// count the blacklines traveled and stop when the destination is reached
					if (Global.crossed()) {
						Global.X--;
						Global.leftBlackLineDetected = false;
						
						if (++count == 2) { // hacky correction
							move(0, false);
							turn(Global.CORR_ANGLE, false);
							move(Global.KEEP_MOVING, true);
							count = 0;
						}
						
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}

				move(-Global.ROBOT_LENGTH, false);// position the robot to the center
			}
		}
		Global.moving = false;
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
		Global.moving = true;
		Global.leftBlackLineDetected = false;
		int count = 0;
		
		// move across y
		if (y != Global.Y) {// if moving in y is needed
			if (y > Global.Y) {
				
				if (y == Global.Y + 1) {
					Global.Y++;
					return;
				}
				
				move(Global.KEEP_MOVING, true);// keep moving forward

				while (Global.Y < y) {// count the blacklines traveled and stop when the destination is reached
					if (Global.crossed()) {
						Global.leftBlackLineDetected = false;
						Global.Y++;
						
						if (++count == 2) {
							move(0, false);
							turn(Global.CORR_ANGLE, false);
							move(Global.KEEP_MOVING, true);
							count = 0;
						}
						
						
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
				move(-Global.ROBOT_LENGTH, false);// reposition the robot the the center
			} else if (y < Global.Y){
				
				if (y == Global.Y - 1) {
					Global.Y--;
					return;
				}
				
				move(Global.KEEP_MOVING, true);// keep moving backward

				while (Global.Y > y) {// count the blacklines traveled and stop when the destination is reached
					if (Global.crossed()) {
						Global.leftBlackLineDetected = false;
						Global.Y--;
						
						if (++count == 2) {
							move(0, false);
							turn(Global.CORR_ANGLE, false);
							move(Global.KEEP_MOVING, true);
							count = 0;
						}
						
						Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
					}
				}
				move(-Global.ROBOT_LENGTH, false);// reposition the robot the the center
			}
		}
		Global.moving = false;
	}
	
	
	/*
	public double timeAngleCorrection(long left, long right) {
		
		double angle = 0;
		double d;
		double deg;
		
		// time in usec
		long diff = (left-right);
		
		if (diff > 0) {
			deg = diff * (long)Global.MOVING_SPEED / 1000;
			d = (double)Global.WHEEL_RADIUS * deg / (360);
			angle = Math.asin(d/Global.S_TO_S);
		} else {
			deg = -diff * (long)Global.MOVING_SPEED / 1000;
			d = (double)Global.WHEEL_RADIUS * deg / (360);
			angle = Math.asin(d/Global.S_TO_S);
			angle *= -1;
		}
		
		return angle;
	}
	*/
	
	
	/**
	 * Localization routine for when the robot finishes
	 * traversing the zipline. If the zipline is straight, 
	 * simply go the next intersection. Else, move to the middle
	 * of a tile, turn to be perpendicular with a black line and
	 * perform the light positionning {@link Navigation.lightPosition} 
	 * routine.
	 * @throws Exception
	 */
	public void afterZiplineLocalization() throws Exception {
		// start left color sensor
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);
	
		// zipline straight
		if (this.angleZipline % 90 == 0) {
			move(15, false);
			
			// move to intersection
			move(Global.KEEP_MOVING, true);
			Global.leftBlackLineDetected = false;
			while (!Global.leftBlackLineDetected) {}
			move(-Global.ROBOT_LENGTH, false);
			
			// reset angle and position
			Global.angle = this.angleZipline;
			Global.X = Global.oppZiplineO[0];
			Global.Y = Global.oppZiplineO[1];
			
		} else {
			// go to ~middle of tile
			move(45, false);
			
			// closest angle + reset coords (expected coords after lightPos)
			if ((this.angleZipline>=0 && this.angleZipline<=45) || (this.angleZipline>315 && this.angleZipline<=360)) {
				if (this.angleZipline>=0 && this.angleZipline<=45) {
					turn(this.angleZipline, false);
					Global.Y = Global.oppZipline[1] + 1;
				}
				else {
					turn(this.angleZipline - 360.0, false);
					Global.Y = Global.oppZipline[1];
				}
				Global.angle = 0; // now 0, after lightPos 90
				Global.X = Global.oppZipline[0] + 1;
			} else if (this.angleZipline>45 && this.angleZipline<=135) {
				turn(this.angleZipline-90, false);
				Global.angle = 90; // now 90, after lightPos 180
				Global.Y = Global.oppZipline[1] + 1;
				Global.X = Global.oppZipline[0];
				if (this.angleZipline > 90)
					Global.X--;
			} else if (this.angleZipline>135 && this.angleZipline<=225) {
				turn(this.angleZipline-180, false);
				Global.angle = 180; // now 180, after lightPos 270
				Global.X = Global.oppZipline[0] - 1;
				Global.Y = Global.oppZipline[1];
				if (this.angleZipline > 180)
					Global.Y--;
			} else {
				turn(this.angleZipline-270, false);
				Global.angle = 270;  // now 270, after lightPos 0
				Global.X = Global.oppZipline[0];
				Global.Y = Global.oppZipline[1] - 1;
				if (this.angleZipline > 270)
					Global.X++;
			}
			lightPosition();
		}
		
		// turn off color sensor
		Global.leftColorSensorSwitch = false;
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

		Global.leftMotor.rotate((int)(distance*convertDistanceConstant), true);
		Global.rightMotor.rotate((int)(distance*convertDistanceConstant), immediatereturn);

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
			Global.leftMotor.rotate((int)(angle*convertAngleConstant), true);
			Global.rightMotor.rotate((int)(-angle*convertAngleConstant), immediatereturn);
		} else {
			angle *= -1;
			Global.leftMotor.rotate((int)(-angle*convertAngleConstant), true);
			Global.rightMotor.rotate((int)(angle*convertAngleConstant), immediatereturn);
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

		// make sure there is no wall in front
		while (Global.ObstacleDistance < Global.USThreshhold) {
			turn(90, false);
		}

		// make the robot face a wall
		turn(Global.KEEP_MOVING, true);
		while (Global.ObstacleDistance > Global.USThreshhold) {
		}
		turn(Global.STOP_MOVING, false);
		
		// approximately straight
		turn(-60, false);
		
		Global.angle = 0;
		Global.usSwitch = false;
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
		Global.leftBlackLineDetected = false;
		while (!Global.leftBlackLineDetected) {
		}

		// move back to black line
		move(-Global.ROBOT_LENGTH, false);
		Thread.sleep(250);

		// reset angle
		// turn until color sensor sees a black line then turn to 90 degree
		turn(-Global.KEEP_MOVING, true);
		while (!Global.leftBlackLineDetected) {
		}
		turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);

		// reset Y
		// move until sensor sees black line
		move(Global.KEEP_MOVING, true);
		Global.leftBlackLineDetected = false;
		while (!Global.leftBlackLineDetected) {
		}

		// move back to black line
		move(-Global.ROBOT_LENGTH, false);
		Thread.sleep(250);

		turn(90, false);
		
		// turn off color sensor
		Global.leftColorSensorSwitch = false;

		// wait color sensor is turned off
		Thread.sleep(200);
		
		// reset coordinates
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
			Global.X = 8;
			Global.Y = 8;
			Global.angle = 180;
			break;
		case 3:
			Global.X = 0;
			Global.Y = 8;
			Global.angle = 270;
		}
	}
}
