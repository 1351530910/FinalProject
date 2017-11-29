package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;
import lejos.hardware.Sound;

/**
 * Thread implementation of a ColorSensor
 * 
 * @author Vincent d'Orsonnens
 * @version 1.0
 */
public class ColorSensor extends Thread {
	int id;

	/**
	 * The id of the color sensor defines which physical sensor we want to model.
	 * <ul>
	 * <li>0 -> left color sensor in RED mode</li>
	 * <li>1 -> front color sensor in RGB mode</li>
	 * <li>2 -> right color sensor in RED mode</li>
	 * </ul>
	 * 
	 * @param id
	 *            The Id of the physical color sensor
	 */
	public ColorSensor(int id) {
		this.id = id;
	}

	/**
	 * Runs an infinite loop that fetches data from the physical color sensor
	 * specified by the ID if the corresponding flag is set to true. Else the thread
	 * sleeps. It stores the fetched result in {@link Global.leftColor} or
	 * {@link Global.frontColorID} depending on the ID.
	 */
	@Override
	public void run() {

		if (id==0) {
			while (true) {
				if (Global.leftColorSensorSwitch) {					
					
					Global.leftColorProvider.fetchSample(Global.leftColorData, 0);
					Global.leftColor = Global.leftColorData[0];

					if (Global.leftColor < Global.colorThreshhold) {
						Global.leftBlackLineDetected = true;
						Global.leftTime = System.currentTimeMillis();
						try {
							Thread.sleep(Global.THREAD_SLEEP_TIME);
						} catch (InterruptedException e) {
						}
					} else {
						Global.leftBlackLineDetected = false;
						Global.leftTime = -1;
					}
				} else {
					try {
						Thread.sleep(Global.THREAD_SLEEP_TIME * 2);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		}
		else if (id==1) {
			while(true) {
				if (id == 1 && Global.frontColorSensorSwitch) {

					Global.frontColorProvider.fetchSample(Global.frontColorData, 0);
					Global.frontColorID = (int) Global.frontColorData[0];

					if (Global.frontColorID == Global.flagColor) {
						Global.flagDetected = true;
						try {
							Thread.sleep(Global.THREAD_SLEEP_TIME);
						} catch (InterruptedException e) {
						}
					} else {
						Global.flagDetected = false;
					}

				} else {
					try {
						Thread.sleep(Global.THREAD_SLEEP_TIME * 2);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		}
		
		else if (id==2) {
			while (true) {
				if (Global.rightColorSensorSwitch) {

					Global.rightColorProvider.fetchSample(Global.rightColorData, 0);
					Global.rightColor = Global.rightColorData[0];

					if (Global.rightColor < Global.rightColorThreshhold) {
						Global.rightBlackLineDetected = true;
						Global.rightTime = System.currentTimeMillis();
						try {
							Thread.sleep(Global.THREAD_SLEEP_TIME);
						} catch (InterruptedException e) {
						}
					} else {
						Global.rightBlackLineDetected = false;
						Global.rightTime = -1;
					}
				} else {
					try {
						Thread.sleep(Global.THREAD_SLEEP_TIME * 2);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}
		}
		
	}
}
