package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;

public class FrontColorSensor extends Thread {
	public FrontColorSensor() {}
	
	public void run() {
		
		while(true) {
			if (Global.frontColorSensorSwitch) {
				
				Global.frontColorProvider.fetchSample(Global.frontColorData, 0);
				Global.frontColorID = (int) Global.frontColorData[0];
				
				if (Global.frontColorID == Global.flagColor) {
					Global.flagDetected = true;
					try {
						Thread.sleep(Global.THREAD_SLEEP_TIME);
					} catch (InterruptedException e) {}
				} else {
					Global.flagDetected = false;
				}
				
			} else {
				try {
					Thread.sleep(Global.THREAD_SLEEP_TIME*2);
				} catch (InterruptedException err) {}
			}
		}
		
	}
}