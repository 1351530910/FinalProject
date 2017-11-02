package ca.mcgill.ecse211.project;

import java.util.*;

import ca.mcgill.ecse211.project.main.Global;

/**
 * A thread implementation of an ultrasonic sensor.
 * Running this thread allows to retrieve data from
 * an ultrasonic sensor.
 * 
 * @author Chen He
 * @version 1.0
 */

public class UltrasonicSensor extends Thread{
	
	public static final int COUNT_MAX = 1;
	public static final int MID = 0;
	
	/**
	 * Everything the ultrasonic sensor needs is defined
	 * in {@link main.Global}.
	 */
	public UltrasonicSensor() {
	}
	
	/**
	 * <p> Runs an infinite loop that fetches data from a
	 * physical sensor if the {@link ca.mcgill.ecse211.project.main.Global.usSwitch}
	 * flag is set to true. Else it sleeps. </p>
	 * <p> When fetching data, it stores the result in
	 * {@link ca.mcgill.ecse211.project.main.Global.ObstacleDistance}.
	 */
	public void run() {
		float arr[] = new float[COUNT_MAX];
		while (true) {
			if (Global.usSwitch) {
				for (int i = 0; i < COUNT_MAX; i++) {
					Global.usDistance.fetchSample(Global.usData, 0);
					arr[i] = Global.usData[0];
				}
				Arrays.sort(arr);
				Global.ObstacleDistance = arr[MID] * 100;
			}
			else {
				try {
					Thread.sleep(Global.THREAD_SLEEP_TIME);
				} catch (InterruptedException err) {}
			}
		}

	}
	
	
	
	
	
	
}
