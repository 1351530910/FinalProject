package ca.mcgill.ecse211.project;

import java.util.HashMap;
import java.util.Map;

import ca.mcgill.ecse211.WiFiClient.WifiConnection;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;


/**
 * The goal of this software is to make a robot play a
 * capture the flag game against another robot. The robot
 * needs to be able to localize, travel to certain points
 * in a (12X12) grid, cross a zipline and find the opponent's
 * flag. Here are the tasks it needs to perform in under 5 minutes:
 * <ol>
 * <li>Get the parameters from the server</li>
 * <li>Transit to the opponent's zone using the zipline or the shallow water</li>
 * <li>Find the opponent's flag</li>
 * <li>Transit the its own zone using the other method</li>
 * <li>Go back to its starting position</li>
 * </ol>
 * 
 * @author 	Vincent d'Orsonnens
 * @author 	Chen He
 * @version 1.0
*/
public class main {
	

    /**
     * This class holds the abstractions, variables
     * and constants that are shared across different
     * parts of the software.
    */
	public static class Global {
		
		// motors
		public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
		public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
		public static final EV3LargeRegulatedMotor ziplineMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("C"));
		
		// switches
		public static boolean usSwitch = false;
		public static boolean leftColorSensorSwitch = false;
		public static boolean frontColorSensorSwitch = false;
		public static boolean odometerSwitch = false;
		public static boolean turning = false;
		
		// odometer
		public static Odometer odometerThread;
		
		// us sensor
		public static UltrasonicSensor usSensorThread;
		public static Port usPort;
		public static SensorModes usSensor;
		public static SampleProvider usDistance;
		public static float[] usData;
		public static float ObstacleDistance = 0;
		
		// left light sensor
		public static ColorSensor leftColorSensorThread;
		public static Port leftColorSensorPort;
		public static EV3ColorSensor leftColorSensor;
		public static float[] leftColorData;
		public static SampleProvider leftColorProvider;
		public static float leftColor = 0;
		public static float colorThreshhold = 0;
		public static boolean BlackLineDetected = false;
		
		// front light sensor
		public static ColorSensor frontColorSensorThread;
		public static Port frontColorSensorPort;
		public static EV3ColorSensor frontColorSensor;
		public static float[] frontColorData;
		public static SampleProvider frontColorProvider;
		public static int frontColorID = 0;
		public static boolean flagDetected;
		
		// display
		public static String firstLine = "";
		public static String secondLine = "";
		public static String thirdLine = "";
		public static String forthLine = "";
		public static String fifthLine = "";
		
		// constants
		public static final int THREAD_SHORT_SLEEP_TIME = 10;
		public static final int THREAD_SLEEP_TIME = 1500;
		public static final int ACCELERATION = 2000;
		public static final double WHEEL_RADIUS = 2.116;
		public static final double TRACK = 10.0;
		public static final int ROTATING_SPEED = 90;
		public static final int MOVING_SPEED = 125;
		public static double ROBOT_LENGTH = 10.5;
		public static final int COLOR_SENSOR_OFFSET_ANGLE = 31;
		public static final int COLOR_SENSOR_OFFSET_ANGLE_WITH_BLACKBAND = 33;
		public static final double SQUARE_LENGTH = 30.5;
		public static final int KEEP_MOVING = 3000;
		public static final int STOP_MOVING = 0;
		public static final int ZIPLINE_LENGTH = 250;
		public static final int FALLING_EDGE_ANGLE = -65;
		public static final int USThreshhold = 40;
		
		// positioning
		public static int X, Y = 0;
		public static double angle = 0;
		
		// wifi settings
		public static final boolean USE_WIFI = false;
		public static final String SERVER_IP = "192.168.2.3";
		public static final int TEAM_NUMBER = 18;
		public static final boolean WIFI_DEBUG = false;
		
		// game specific data
		public static enum TeamColor {RED, GREEN};
		public static TeamColor teamColor;
		public static int startingCorner;
		public static int flagColor;
		public static int[] zoneLL;
		public static int[] zoneUR;
		public static int[] oppLL;
		public static int[] oppUR;
		public static int[] searchLL;
		public static int[] searchUR;
		public static int[] zoneZipline;
		public static int[] zoneZiplineO;
		public static int[] oppZipline;
		public static int[] oppZiplineO;
		public static int shallowHLLx;
		public static int shallowHLLy;
		public static int shallowHURx;
		public static int shallowHURy;
		public static int shallowVLLx;
		public static int shallowVLLy;
		public static int shallowVURx;
		public static int shallowVURy;
		
	}
	

    /**
     * <p>The main method is responsible for initializing 
     * all the threads needed to complete the task
     * </p>
     * <ul>
     * <li>{@link Display}</li><li>{@link UltrasonicSensor}</li>
     * <li>{@link ColorSensor}</li><li>{@link Navigation}</li>
     * </ul>
     * <p>It also fetches the game data from the server. If an error
     * occurs during the connection with the server, the application
     * exits.</p>
    */
	public static void main(String[] args) {
		// start the display
		Display display = new Display();
		display.start();
		
		// fetch game data from the server
		if (Global.USE_WIFI) {
			Global.firstLine = "Fetching data from server";
			Map gameData = null;
			WifiConnection conn = new WifiConnection(Global.SERVER_IP, Global.TEAM_NUMBER, Global.WIFI_DEBUG);
			try {
				gameData = conn.getData();
			} catch (Exception err) {
				System.out.println("Error " + err.toString());
				Button.waitForAnyPress();
				System.exit(1);
			}
			
			// parse game data
			Global.firstLine = "Parsing game data";
			parseGameData(gameData);
		} else {
		    // Update the values in generateTestData() for your specific test
            parseGameData(generateTestData());
        }
		
		// initialize us sensor
		Global.usPort = LocalEV3.get().getPort("S1");
		try {
			Thread.sleep(Global.THREAD_SHORT_SLEEP_TIME);
		} catch (Exception e) {}
		Global.usSensor = new EV3UltrasonicSensor(Global.usPort);
		Global.usDistance = Global.usSensor.getMode("Distance");
		Global.usData = new float[Global.usDistance.sampleSize()];
		
		
		// initialize light sensors
		Global.leftColorSensorPort = LocalEV3.get().getPort("S2");
		Global.leftColorSensor = new EV3ColorSensor(Global.leftColorSensorPort);
		Global.leftColorProvider = Global.leftColorSensor.getRedMode();
		Global.leftColorData = new float[Global.leftColorProvider.sampleSize() + 1];
		
		Global.frontColorSensorPort = LocalEV3.get().getPort("S3");
		Global.frontColorSensor = new EV3ColorSensor(Global.frontColorSensorPort);
		Global.frontColorProvider = Global.frontColorSensor.getColorIDMode();
		Global.frontColorData = new float[Global.frontColorProvider.sampleSize() + 1];
		
		// initialize threads
		Global.usSensorThread = new UltrasonicSensor();
		Global.leftColorSensorThread  = new ColorSensor(0);
		Global.frontColorSensorThread = new ColorSensor(1);
		Global.odometerThread = new Odometer();
		
		// get a starting value for left color sensor
		Global.secondLine = "Start left sensor ...";
		Button.waitForAnyPress();
		Global.leftColorSensorThread.start();
		try {
			Thread.sleep(Global.THREAD_SLEEP_TIME);
		} catch (Exception e) {}		
		Global.leftColorSensorSwitch = true;
		while(Global.leftColor==0) {}
		Global.colorThreshhold = (float)(Global.leftColor *0.7);
		Global.leftColorSensorSwitch = false;
		
		// start main thread
		Global.firstLine = "";
		Global.secondLine = "";
		Navigation mainthread = new Navigation();
		mainthread.start();
		
	    Button.waitForAnyPress();
        System.exit(0);
	}
	
    /**
     * Populates the fields in {@link main.Global} related
     * to the game data from the given Map
     *  
     * @param 	data 	a Map specifying the game data  
    */
	private static void parseGameData(Map data) {
		int redTeam = intVal(data.get("RedTeam"));
		int redSC = intVal(data.get("RedCorner"));
		int redFlag = intVal(data.get("OR"));
		int redLLx = intVal(data.get("Red_LL_x"));
		int redLLy = intVal(data.get("Red_LL_y"));
		int redURx = intVal(data.get("Red_UR_x"));
		int redURy = intVal(data.get("Red_UR_y"));
		int redSearchLLx = intVal(data.get("SR_LL_x"));
		int redSearchLLy = intVal(data.get("SR_LL_y"));
		int redSearchURx = intVal(data.get("SR_UR_x"));
		int redSearchURy = intVal(data.get("SR_UR_y"));
		int redZipx = intVal(data.get("ZC_R_x"));
		int redZipy = intVal(data.get("ZC_R_y"));
		int redZipDirx = intVal(data.get("ZO_R_x"));
		int redZipDiry = intVal(data.get("ZO_R_y"));
		
		int greenSC = intVal(data.get("GreenCorner"));
		int greenFlag = intVal(data.get("OG"));
		int greenLLx = intVal(data.get("Green_LL_x"));
		int greenLLy = intVal(data.get("Green_LL_y"));
		int greenURx = intVal(data.get("Green_UR_x"));
		int greenURy = intVal(data.get("Green_UR_y"));
		int greenSearchLLx = intVal(data.get("SG_LL_x"));
		int greenSearchLLy = intVal(data.get("SG_LL_y"));
		int greenSearchURx = intVal(data.get("SG_UR_x"));
		int greenSearchURy = intVal(data.get("SG_UR_y"));
		int greenZipx = intVal(data.get("ZC_G_x"));
		int greenZipy = intVal(data.get("ZC_G_y"));
		int greenZipDirx = intVal(data.get("ZO_G_x"));
		int greenZipDiry = intVal(data.get("ZO_G_y"));
		
		Global.shallowHLLx = intVal(data.get("SH_LL_x"));
		Global.shallowHLLy = intVal(data.get("SH_LL_y"));
		Global.shallowHURx = intVal(data.get("SH_UR_x"));
		Global.shallowHURy = intVal(data.get("SH_UR_y"));
		Global.shallowVLLx = intVal(data.get("SV_LL_x"));
		Global.shallowVLLy = intVal(data.get("SV_LL_y"));
		Global.shallowVURx = intVal(data.get("SV_UR_x"));
		Global.shallowVURy = intVal(data.get("SV_UR_y"));
		
		
		if (redTeam == Global.TEAM_NUMBER) {
			Global.teamColor = Global.TeamColor.RED;
			Global.startingCorner = redSC;
			Global.flagColor = greenFlag;
			Global.zoneLL = new int[] {redLLx, redLLy};
			Global.zoneUR = new int[] {redURx, redURy};
			Global.oppLL = new int[] {greenLLx, greenLLy};
			Global.oppUR = new int[] {greenURx, greenURy};
			Global.searchLL = new int[] {greenSearchLLx, greenSearchLLy};
			Global.searchUR = new int[] {greenSearchURx, greenSearchURy};
			Global.zoneZipline = new int[] {redZipx, redZipy};
			Global.zoneZiplineO = new int[] {redZipDirx, redZipDiry};
			Global.oppZipline = new int[] {greenZipx, greenZipy};
			Global.oppZiplineO = new int[] {greenZipDirx, greenZipDiry};
			
		} else {
			Global.teamColor = Global.TeamColor.GREEN;
			Global.startingCorner = greenSC;
			Global.flagColor = redFlag;
			Global.zoneLL = new int[] {greenLLx, greenLLy};
			Global.zoneUR = new int[] {greenURx, greenURy};
			Global.oppLL = new int[] {redLLx, redLLy};
			Global.oppUR = new int[] {redURx, redURy};
			Global.searchLL = new int[] {redSearchLLx, redSearchLLy};
			Global.searchUR = new int[] {redSearchURx, redSearchURy};
			Global.zoneZipline = new int[] {greenZipx, greenZipy};
			Global.zoneZiplineO = new int[] {greenZipDirx, greenZipDiry};
			Global.oppZipline = new int[] {redZipx, redZipy};
			Global.oppZiplineO = new int[] {redZipDirx, redZipDiry};
		}	
	}
	
    /**
     * <p>Transforms an object of type Object to an
     * integer.</p>
     * <p>Best used with the objects inside a Map 
     * received from the server.</p>
     * 
     * @param	obj 	an Object with an integer value
     * @return 			an integer with the same value as obj
     * @see 			main#parseGameData 
    */
	private static int intVal(Object obj) {
		//return ((Long) obj).intValue();
		return (int) obj;
	}
    

    /**
     * Generate a Map with the same key as one
     * we would get from the server. Use this when
     * you don't want to get parameters from the server,
     * when testing for example.
     *
     * @return  A {@link Map} containing the data for a game 
    */
    private static Map<String, Integer> generateTestData() {
        Map<String, Integer> data = new HashMap<String, Integer>();
	    data.put("RedTeam", 1);
		data.put("RedCorner", 3);
		data.put("OR", 1);          // Color of red flag
		data.put("Red_LL_x", 0);
		data.put("Red_LL_y", 7);
		data.put("Red_UR_x", 8);
	    data.put("Red_UR_y", 12);
		data.put("SR_LL_x", 1);
		data.put("SR_LL_y", 9);
		data.put("SR_UR_x", 2);
		data.put("SR_UR_y", 11);
		data.put("ZC_R_x", 4);
		data.put("ZC_R_y", 9);
		data.put("ZO_R_x", 3);
		data.put("ZO_R_y", 10);
		
		data.put("GreenCorner", 1);
		data.put("OG", 2);
		data.put("Green_LL_x", 4);
		data.put("Green_LL_y", 0);
		data.put("Green_UR_x", 12);
		data.put("Green_UR_y", 5);
		data.put("SG_LL_x", 9);
		data.put("SG_LL_y", 1);
		data.put("SG_UR_x", 11);
		data.put("SG_UR_y", 2);
		data.put("ZC_G_x", 9);
		data.put("ZC_G_y", 3);
		data.put("ZO_G_x", 10);
		data.put("ZO_G_y", 2);
		
		data.put("SH_LL_x", 8);
		data.put("SH_LL_y", 9);
		data.put("SH_UR_x", 11);
		data.put("SH_UR_y", 10);
		data.put("SV_LL_x", 10);
		data.put("SV_LL_y", 5);
		data.put("SV_UR_x", 11);
		data.put("SV_UR_y", 10);
         
        return data;
    }
}
