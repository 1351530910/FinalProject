package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;
import lejos.hardware.Button;
import lejos.hardware.Sound;

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
	
	private int waterOrientation;
	private int waterInitAngle;
	private int waterStartX;
	private int waterStartY;
	private int waterEndX;
	private int waterEndY;
	private int waterAngleToTurn;
	private int waterFinalAngle;
	private int beforeWaterX;
	private int beforeWaterY;
	private int waterAngleToMiddle;
	private int waterAngleToWater;
	
	/**
	 * The constructor compute values that will stay constant during the
	 * navigation.
	 */
	public Navigation() {
		convertDistanceConstant = (180.0 / (Math.PI * Global.WHEEL_RADIUS));
		convertAngleConstant = convertDistanceConstant*Math.PI * Global.TRACK  / 360.0;
		angleZipline = 0.0;
		waterOrientation = 0;
		waterInitAngle = 0;
		waterStartX = 0;
		waterStartY = 0;
		waterEndX = 0;
		waterEndY = 0;
		waterAngleToTurn = 0;
		waterFinalAngle = 0;
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
			calculateWaterOrientation();
			
			System.out.println("After initial localization");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// Travel to transit location
			travelToTransit(true);	
			
			System.out.println("\nAfter first travel to transit");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// Cross transit
			if (Global.teamColor == Global.TeamColor.GREEN) {
				travelZipline();
				afterZiplineLocalization(false);
			}
			else
				travelWater();
			
			System.out.println("\nAfter first transit");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
				
			// turn to the right direction for traveling to the search zone
			boolean clockwiseTravel = ziplineOnCCPath(Global.searchLL[0], Global.searchLL[1], false);
			if (clockwiseTravel)
				turnClockwiseTravel(Global.searchLL[0], Global.searchLL[1]);
			else
				turnCounterClockwiseTravel(Global.searchLL[0], Global.searchLL[1]);
			
			System.out.println("\nClockwise travel: " + clockwiseTravel);
			System.out.println("After turning for travel");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// Travel to the search zone and turn to 90 deg.
			travelTo(Global.searchLL[0], Global.searchLL[1], false, clockwiseTravel);
			turn(Global.angle - 90, false);
			Global.angle = 90;
			
			System.out.println("\nAt search zone");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// find the flag
			findFlag();

			System.out.println("\nAfter finding flag");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// go back to transit point
			travelToTransit(false);
			
			System.out.println("\nAfter second travel to transit");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);

			// Go back to starting zone
			if (Global.teamColor == Global.TeamColor.GREEN)
				travelWater();
			else {
				travelZipline();
				afterZiplineLocalization(true);
			}
			
			System.out.println("\nAfter water");
			System.out.println("X: " + Global.X + " Y: " + Global.Y + " Angle: " + Global.angle);
			
			// go to starting corner
			travelToStartingCorner();
			
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
				if (Global.teamColor == Global.TeamColor.GREEN)
					travelTo(Global.zoneZiplineO[0], Global.zoneZiplineO[1], true, false);
				// go to shallowHLL
				else {
					boolean ccZip = ziplineOnCCPath(beforeWaterX, beforeWaterY, true);
					// adjust correction made by setStartingCorner for navigation
					if (ccZip) {
						if (Global.startingCorner == 3) {
							Global.X++;
							Global.Y--;
						} else if (Global.startingCorner == 2){
							Global.X--;
							Global.Y--;
						} else if (Global.startingCorner == 1) {
							Global.X++;
							Global.Y--;
						} else {
							Global.X++;
							Global.Y++;
						}
						turnClockwiseTravel(beforeWaterX, beforeWaterY);
					}
					travelTo(beforeWaterX, beforeWaterY, true, ccZip);
				}
			}

			// second transit
			else {
				if (Global.teamColor == Global.TeamColor.GREEN) {
					boolean ccZip = ziplineOnCCPath(beforeWaterX, beforeWaterY, false);
					if (ccZip)
						turnClockwiseTravel(beforeWaterX, beforeWaterY);
					else
						turnCounterClockwiseTravel(beforeWaterX, beforeWaterY);
					travelTo(beforeWaterX, beforeWaterY, false, ccZip);
				}
				else {
					boolean ccZip = ziplineOnCCPath(Global.oppZiplineO[0], Global.oppZiplineO[1], false);
					if (ccZip)
						turnClockwiseTravel(Global.oppZiplineO[0], Global.oppZiplineO[1]);
					else
						turnCounterClockwiseTravel(Global.oppZiplineO[0], Global.oppZiplineO[1]);
					travelTo(Global.oppZiplineO[0], Global.oppZiplineO[1], false, ccZip);
				}
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
		double dx, dy;
		if (Global.teamColor == Global.TeamColor.GREEN) {
			dx = Global.zoneZipline[0] - Global.zoneZiplineO[0];
			dy = Global.zoneZipline[1] - Global.zoneZiplineO[1];
		} else {
			dx = Global.oppZipline[0] - Global.oppZiplineO[0];
			dy = Global.oppZipline[1] - Global.oppZiplineO[1];
		}
		
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
		
		System.out.println("\nZipline angle: " + this.angleZipline);
		
		// turn robot to zipline's angle using the smallest angle
		double angleToTurn = Global.angle - this.angleZipline;
		if (angleToTurn < -180)
			angleToTurn = 360 + angleToTurn;
		else if (angleToTurn > 180)
			angleToTurn = 360 - angleToTurn;
		
		System.out.println("Turn to zipline: " + angleToTurn);
		
		Thread.sleep(500);
		
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
	 * Traverse the shallow water. The robot always traverse water from the red zone.
	 * Assuming the red zone is above the green, and that we always have to travel
	 * the horizontal part first, may be wrong, hope not, may the gods of luck
	 * be with us.
	 * @throws Exception 
	 * 
	 *
	 */
	public void travelWater() throws Exception {
		Global.leftColorSensorSwitch = true;
		Global.rightColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
		
		System.out.println("\nWATER");
		System.out.println("Init angle: " + waterInitAngle);
		System.out.println("Orientation: " + waterOrientation);
		System.out.println("Start X: " + waterStartX + "  Y: " + waterStartY);
		System.out.println("Angle to turn:" + waterAngleToTurn);
		System.out.println("End X: " + waterEndX + "  Y: " + waterEndY);
		System.out.println("Final orientation: " + waterFinalAngle);
		
		// go to the middle
		turn(Global.angle - waterAngleToMiddle, false);
		move(10, false);
		turn(waterAngleToWater, false);
		//turn(Global.angle - waterInitAngle, false);
		Global.angle = waterInitAngle;
		
		int endAngle;
		
		if (this.waterOrientation == 0) {
			Global.X--;
			travelX(waterEndX);
			move(20, false);
			turn(waterAngleToTurn, false);
			Global.angle = waterFinalAngle;
			if (Global.angle == 270) {
				Global.Y++;
				waterEndY--;
				endAngle = 180;
			}
			else {
				Global.Y--;
				waterEndY++;
				endAngle = 180;
			}
			travelY(waterEndY);
		} 
		else if (this.waterOrientation == 1){
			Global.X++;
			travelX(waterEndX);
			move(20, false);
			turn(waterAngleToTurn, false);
			Global.angle = waterFinalAngle;
			if (Global.angle == 270) {
				Global.Y++;
				waterEndY--;
				endAngle = 0;
			}
			else {
				Global.Y--;
				waterEndY++;
				endAngle = 0;
			}
			travelY(waterEndY);
			
		} else if (waterOrientation == 2) {
			Global.Y--;
			travelY(waterEndY);
			move(20, false);
			turn(waterAngleToTurn, false);
			Global.angle = waterFinalAngle;
			if (Global.angle == 0) {
				Global.X--;
				waterEndX++;
				endAngle = 270;
			}
			else {
				Global.X++;
				waterEndX--;
				endAngle = 270;
			}
			travelX(waterEndX);
			
		} else {
			Global.Y++;
			travelY(waterEndY);
			move(20, false);
			turn(waterAngleToTurn, false);
			Global.angle = waterFinalAngle;
			if (Global.angle == 0) {
				Global.X--;
				waterEndX++;
				endAngle = 90;
			}
			else {
				Global.X++;
				waterEndX--;
				endAngle = 90;
			}
			travelX(waterEndX);
		}
		
		turn(waterAngleToTurn, false);
		Global.leftBlackLineDetected = false;
		move(Global.KEEP_MOVING, true);
		while(!Global.leftBlackLineDetected) {}
		move(0, false);
		move(-Global.ROBOT_LENGTH, false);
		
		Global.angle = endAngle;
		
		Global.leftColorSensorSwitch = true;
		Global.rightColorSensorSwitch = true;
	}

	/**
	 * Makes the robot search the opponent's zone for the flag. It uses the
	 * ultrasonic sensor and both color sensors. When it finds the flag, it beeps 3
	 * times and the method returns.
	 * @throws Exception 
	 */
	public void findFlag() throws Exception {
		
		for (int i=0; i<3; i++) {
			Sound.beep();
		}
		
		/*Global.usSwitch = true;
		Global.frontColorSensorSwitch = true;
		
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
		
		Global.usSwitch = false;
		Global.frontColorSensorSwitch = false;
		*/
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
	public void travelTo(int x, int y, boolean wallCorrection, boolean clockwiseTravel) throws Exception {
		// activate required threads
		Global.leftColorSensorSwitch = true;
		Global.rightColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);

		if (Global.angle == 90 || Global.angle == 270) {
			
			if (Math.abs(y-Global.Y) > 1)
				if (wallCorrection)
					move(-30, false);
			
			travelY(y);
			
			if (!clockwiseTravel) {
				// turn to the correct direction using the black lines
				/*
				turn(-Global.KEEP_MOVING, true);
				Thread.sleep(Global.THREAD_SLEEP_TIME);
				Global.leftBlackLineDetected = false;
				while (!Global.leftBlackLineDetected) {}
				turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
				*/
				turn(-90, false);
				// update angle
				if (Global.angle == 90)
					Global.angle = 180;
				else
					Global.angle = 0;
			} else {
				// turn to the correct direction using the black lines
				turn(90, false);
				// update angle
				if (Global.angle == 90)
					Global.angle = 0;
				else
					Global.angle = 180;
			}
			
			if (Math.abs(x-Global.X) > 1)
				if (wallCorrection)
					move(-30, false);
			
			travelX(x);
			
		} else {
			if (Math.abs(x-Global.X) > 1)
				if (wallCorrection)
					move(-30, false);
			travelX(x);
			
			if (!clockwiseTravel) {
				// turn to the correct direction using the black lines
				/*
				turn(-Global.KEEP_MOVING, true);
				Thread.sleep(Global.THREAD_SLEEP_TIME);
				Global.leftBlackLineDetected = false;
				while (!Global.leftBlackLineDetected) {}
				turn(Global.COLOR_SENSOR_OFFSET_ANGLE, false);
				*/
				turn(-90, false);
				// update angle
				if (Global.angle == 0)
					Global.angle = 90;
				else
					Global.angle = 270;
			} else {
				// turn to the correct direction using the black lines
				turn(90, false);
				// update angle
				if (Global.angle == 0)
					Global.angle = 270;
				else
					Global.angle = 90;
			}
			
			if (Math.abs(y-Global.Y) > 1)
				if (wallCorrection)
					move(-30, false);
			
			travelY(y);
		}
		
		Global.leftColorSensorSwitch = false;
		Global.rightColorSensorSwitch = false;
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
		Global.rightBlackLineDetected = false;
		Global.leftTime = -1;
		Global.rightTime = -1;
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
						Global.rightBlackLineDetected = false;
						Global.leftTime = -1;
						Global.rightTime = -1;
						
						if (++count == 2) { // hacky correction
							move(0, false);
							turn(timeAngleCorrection(Global.savedLeft, Global.savedRight), false);
							//turn(Global.CORR_ANGLE, false);
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
						Global.rightBlackLineDetected = false;
						Global.leftTime = -1;
						Global.rightTime = -1;
						
						if (++count == 2) { // hacky correction
							move(0, false);
							turn(timeAngleCorrection(Global.savedLeft, Global.savedRight), false);
							//turn(Global.CORR_ANGLE, false);
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
						Global.rightBlackLineDetected = false;
						Global.leftTime = -1;
						Global.rightTime = -1;
						Global.Y++;
						
						if (++count == 2) {
							move(0, false);
							turn(timeAngleCorrection(Global.savedLeft, Global.savedRight), false);
							//turn(Global.CORR_ANGLE, false);
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
						Global.rightBlackLineDetected = false;
						Global.leftTime = -1;
						Global.rightTime = -1;
						Global.Y--;
						
						if (++count == 2) {
							move(0, false);
							turn(timeAngleCorrection(Global.savedLeft, Global.savedRight), false);
							//turn(Global.CORR_ANGLE, false);
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
	
	/**
	 * Turn the robot to the right orientation to travel
	 * in a clockwise fashion to the destination
	 * 
	 * @param x 	the x coordinate of the destination
	 * @param y 	the y coordinate of the destination
	 * @throws Exception
	 */
	public void turnClockwiseTravel(int x, int y) throws Exception {
		int dx = x - Global.X;
		int dy = y - Global.Y;
		if (dx >= 0) {
			Global.X--;
			if (dy >= 0) {
				Global.Y--;
				turn(Global.angle - 90, false);
				Global.angle = 90;
			} else {
				Global.Y++;
				turn(Global.angle, false);
				Global.angle = 0;
			}
		} else {
			Global.X++;
			if (dy >= 0) {
				Global.Y--;
				turn(Global.angle - 180, false);
				Global.angle = 180;
			} else {
				Global.Y++;
				turn(Global.angle - 270, false);
				Global.angle = 270;
			}
		}
	}
	
	/**
	 * Turn the robot to the right orientation to travel
	 * in a counter-clockwise fashion to the destination
	 * 
	 * @param x 	the x coordinate of the destination
	 * @param y 	the y coordinate of the destination
	 * @throws Exception
	 */
	public void turnCounterClockwiseTravel(int x, int y) throws Exception {
		// turn to the right direction
		int dx = x - Global.X;
		int dy = y - Global.Y;
		if (dx >= 0) {
			Global.X--;
			if (dy >= 0) {
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
			if (dy >= 0 ) {
				Global.Y--;
				turn(Global.angle - 90, false);
				Global.angle = 90;
			} else {
				Global.Y++;
				turn (Global.angle - 180, false);
				Global.angle = 180;
			}
		}
	}
	
	
	/**
	 * Check if the zipline is on the counter-clockwise path of the robot
	 * @param x 	the x destination
	 * @param y 	the y destionation
	 * @param zone 	true if the robot is in its zone
	 * @return		true if the zipline is on the cc path
	 */
	public boolean ziplineOnCCPath(int x, int y, boolean zone) {
		int zipX, zipY;
		if (zone) {
			zipX = Global.zoneZipline[0];
			zipY = Global.zoneZipline[1];
		} else {
			zipX = Global.oppZipline[0];
			zipY = Global.oppZipline[1];
		}
		
		int dx = x - Global.X;
		int dy = y - Global.Y;
		
		// angle you should be at for cc travel
		int angle;
		if (dx >= 0) {
			if (dy >= 0)
				angle = 0;
			else
				angle = 270;
		} else {
			if (dy >= 0)
				angle = 90;
			else
				angle = 180;
		}
		
		
		if (angle == 0) 
			return (zipX >= Global.X) && (zipX <= x) && (zipY >= Global.Y) && (zipY <= y);
		else if (angle == 90)
			return (zipX >= x) && (zipX <= Global.X) && (zipY >= Global.Y) && (zipY <= y);
		else if (angle == 180)
			return (zipX >= x) && (zipX <= Global.X) && (zipY >= y) && (zipY <= Global.Y);
		else if (angle == 270)
			return (zipX >= Global.X) && (zipX <= x) && (zipY >= y) && (zipY <= Global.Y);
		
		return false;
	}
	
	
	/**
	 * When using two light sensor for navigation, this method
	 * compute the angle the robot should turn to go back to
	 * traveling in a straight line.
	 * 
	 * @param left 	The time when the left sensor saw the black line
	 * @param right	The time when the right sensor saw the black line
	 */
	public double timeAngleCorrection(long left, long right) {
		
		double angle = 0;
		double d;
		double deg;
		
		// time in msec
		long diff = (left-right);
		
		System.out.println("\nDIFF: " + diff + "\n");
		
		if (Math.abs(diff) < 40)
			return 0;
		
		if (diff > 0) {
			angle = Global.CORR_ANGLE_RIGHT;
			//deg = diff * (long)Global.MOVING_SPEED / 1000;
			//d = (double)Global.WHEEL_RADIUS * deg / (360);
			//angle = Math.asin(d/Global.S_TO_S);
		} else {
			angle = -Global.CORR_ANGLE_LEFT;
			//deg = -diff * (long)Global.MOVING_SPEED / 1000;
			//d = (double)Global.WHEEL_RADIUS * deg / (360);
			//angle = Math.asin(d/Global.S_TO_S);
			//angle *= -1;
		}
		
		return angle;
	}
	
	
	
	/**
	 * Localization routine for when the robot finishes
	 * traversing the zipline. If the zipline is straight, 
	 * simply go the next intersection. Else, move to the middle
	 * of a tile, turn to be perpendicular with a black line and
	 * perform the light positionning {@link Navigation.lightPosition} 
	 * routine.
	 * @throws Exception
	 */
	public void afterZiplineLocalization(boolean zone) throws Exception {
		// start left color sensor
		Global.leftColorSensorSwitch = true;
		Thread.sleep(Global.THREAD_SLEEP_TIME);
		
		// zipline straight
		if (this.angleZipline % 90 == 0) {
			move(10, false);
			
			// go to the middle of a tile
			turn(-60, false);
			move(20, false);
			turn(150, false);
			lightPosition();
			
			// reset angle and position
			if (!zone) {
				Global.X = Global.oppZiplineO[0];
				Global.Y = Global.oppZiplineO[1];
			} else {
				Global.X = Global.zoneZiplineO[0];
				Global.Y = Global.zoneZiplineO[1];
			}
			if (this.angleZipline == 0)
				Global.angle = 270;
			else
				Global.angle = this.angleZipline - 90;
			
		} else {
			// go to ~middle of tile
			move(45, false);
			
			int zipX, zipY;
			if (zone) {
				zipX = Global.zoneZipline[0];
				zipY = Global.zoneZipline[1];
			} else {
				zipX = Global.oppZipline[0];
				zipY = Global.oppZipline[1];
			}
			
			// closest angle + reset coords (expected coords after lightPos)
			if ((this.angleZipline>=0 && this.angleZipline<=45) || (this.angleZipline>315 && this.angleZipline<=360)) {
				if (this.angleZipline>=0 && this.angleZipline<=45) {
					turn(this.angleZipline, false);
					Global.Y = zipY + 1;
				}
				else {
					turn(this.angleZipline - 360.0, false);
					Global.Y = zipY;
				}
				Global.angle = 0; // now 0, after lightPos 90
				Global.X = zipX + 1;
			} else if (this.angleZipline>45 && this.angleZipline<=135) {
				turn(this.angleZipline-90, false);
				Global.angle = 90; // now 90, after lightPos 180
				Global.Y = zipY + 1;
				Global.X = zipX;
				if (this.angleZipline > 90)
					Global.X--;
			} else if (this.angleZipline>135 && this.angleZipline<=225) {
				turn(this.angleZipline-180, false);
				Global.angle = 180; // now 180, after lightPos 270
				Global.X = zipX - 1;
				Global.Y = zipY;
				if (this.angleZipline > 180)
					Global.Y--;
			} else {
				turn(this.angleZipline-270, false);
				Global.angle = 270;  // now 270, after lightPos 0
				Global.X = zipX;
				Global.Y = zipY - 1;
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
		if (angle == 270)
			angle = -90;
		else if (angle == -270)
			angle = 90;
		
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
	
	
	/**
	 * Travel back to the starting corner by first hitting a wall, 
	 * then traveling on X to go back to the corner
	 * @throws Exception
	 */
	public void travelToStartingCorner() throws Exception {
		
		Global.leftBlackLineDetected = false;
		Global.leftColorSensorSwitch = true;
		
		int x=0, y=0;
		switch (Global.startingCorner) {
		case 0:
			x = 1;
			y = 1;
			break;
		case 1:
			x = 7;
			y = 1;
			break;
		case 2:
			x = 7;
			y = 7;
			break;
		case 3:
			x = 1;
			y = 7;
			break;
		}
		
		boolean ccPath = ziplineOnCCPath(x, y, true);
		if (ccPath) {
			turnClockwiseTravel(x, y);
		} else {
			turnCounterClockwiseTravel(x, y);
		}
		travelTo(x, y, false, ccPath);
		
		switch (Global.startingCorner) {
		case 0:
			turn(Global.angle - 225, false);
			break;
		case 1:
			turn(Global.angle - 315, false);
			break;
		case 2:
			turn(Global.angle - 45, false);
			break;
		case 3:
			turn(Global.angle - 135, false);
			break;
		}
		
		move(20, false);
		
		/*
		if (Global.teamColor == Global.TeamColor.GREEN) {
			// go to last line, robot should be at 270 already
			turn(Global.angle - 270, false);
			Global.Y++;
			travelY(1);
			
			//wall correction
			move(30, false);
			move(-10, false);
			
			if (Global.startingCorner == 0) {
				turn(90, false);
				Global.angle = 0;
				Global.X++;
			} else {
				turn(-90, false);
				Global.angle = 180;
				Global.X--;
			}
			
			travelX(x);
			
		} else {
			turn(Global.angle - 90, false);
			Global.Y--;
			travelY(y);
			
			// wall correction
			move(30, false);
			move(-10, false);
			
			if (Global.startingCorner == 3) {
				turn(-90, false);
				Global.angle = 180;
				Global.X++;
			} else {
				turn(90, false);
				Global.angle = 0;
				Global.X--;
			}
			
			travelX(x);
		}
		*/
		
		Global.leftBlackLineDetected = false;
		Global.leftColorSensorSwitch = false;
	}
	
	/**
	 * Calculate the orientation of the horizontal part of the
	 * water section.
	 */
	public void calculateWaterOrientation() {
		int[] redLL, redUR;
		if (Global.teamColor == Global.TeamColor.RED) {
			redLL = Global.zoneLL;
			redUR = Global.zoneUR;
		} else {
			redLL = Global.oppLL;
			redUR = Global.oppUR;
		}	
		
		// vertical attached
		if (Global.shallowVLLy == redUR[1]) {
			waterOrientation = 2;
			waterStartY = Global.shallowVLLy;
			beforeWaterY = waterStartY - 1;
			waterInitAngle = 90;
			if (Global.shallowHURx == Global.shallowVURx) {
				waterStartX = Global.shallowVLLx;;
				waterAngleToTurn = -90;
				waterFinalAngle = 180;
				waterEndX = Global.shallowHLLx;
				waterEndY = Global.shallowHLLy;
				waterAngleToMiddle = 0;
				waterAngleToWater = -90;
			} else {
				waterStartX = Global.shallowVURx;
				waterAngleToTurn = 90;
				waterFinalAngle = 0;
				waterEndX = Global.shallowHURx;
				waterEndY = Global.shallowHLLy;
				waterAngleToMiddle = 180;
				waterAngleToWater = 90;
			}
			beforeWaterX = waterStartX;
		} 
		else if (Global.shallowVURy == redLL[1]) {
			waterOrientation = 3;
			waterStartY = Global.shallowVURy;
			beforeWaterY = waterStartY + 1;
			waterInitAngle = 270;
			if (Global.shallowHLLx == Global.shallowVLLx) {
				waterStartX = Global.shallowVURx;
				waterAngleToTurn = -90;
				waterFinalAngle = 0;
				waterEndX = Global.shallowHURx;
				waterEndY = Global.shallowHURy;
				waterAngleToMiddle = 180;
				waterAngleToWater = -90;
			} else {
				waterStartX = Global.shallowVLLx;
				waterAngleToTurn = 90;
				waterFinalAngle = 180;
				waterEndX = Global.shallowHLLx;
				waterEndY = Global.shallowHURy;
				waterAngleToMiddle = 0;
				waterAngleToWater = 90;
			}
			beforeWaterX = waterStartX;
		}
		// horizontal attached
		else if (Global.shallowHURx == redLL[0]) {
			waterOrientation = 1;
			waterStartX = Global.shallowHURx;
			beforeWaterX = waterStartX + 1;
			waterInitAngle = 180;
			
			if (Global.shallowVURy == Global.shallowHURy) {
				waterStartY = Global.shallowHLLy;
				waterAngleToTurn = -90;
				waterEndX = Global.shallowVURx;
				waterEndY = Global.shallowVLLy;
				waterFinalAngle = 270;
				waterAngleToMiddle = 90;
				waterAngleToWater = -90;
			}
			else {
				waterStartY = Global.shallowHURy;
				waterAngleToTurn = 90;
				waterEndX = Global.shallowVURx;
				waterEndY = Global.shallowVURy;
				waterFinalAngle = 90;
				waterAngleToMiddle = 270;
				waterAngleToWater = 90;
			}
			beforeWaterY = waterStartY;
		}
		else if(Global.shallowHLLx == redUR[0]) {
			waterOrientation = 0;
			waterStartX = Global.shallowHLLx;
			beforeWaterX = waterStartX - 1;
			waterInitAngle = 0;
			
			if (Global.shallowVURy == Global.shallowHURy) {
				waterStartY = Global.shallowHLLy;
				waterAngleToTurn = 90;
				waterEndX = Global.shallowVLLx;
				waterEndY = Global.shallowVLLy;
				waterFinalAngle = 270;
				waterAngleToMiddle = 90;
				waterAngleToWater = 90;
			} else {
				waterStartY = Global.shallowHURy;
				waterAngleToTurn = -90;
				waterEndX = Global.shallowVLLx;
				waterEndY = Global.shallowVURy;
				waterFinalAngle = 90;
				waterAngleToMiddle = 270;
				waterAngleToWater = -90;
			}
			beforeWaterY = waterStartY;
		}
	}
}
