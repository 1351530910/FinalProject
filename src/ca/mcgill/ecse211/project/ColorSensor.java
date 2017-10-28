package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;

public class ColorSensor extends Thread {
	public ColorSensor() {}
	
	@Override
	public void run() {
		
		while(true) {
			//save some cpu costs
			if (Global.leftColorSensorSwitch) {
				
				Global.leftColorProvider.fetchSample(Global.leftColorData, 0);
				Global.leftColor = Global.leftColorData[0];
				
				if (Global.leftColor < Global.colorThreshhold) {
					Global.BlackLineDetected = true;
					try {
						Thread.sleep(Global.THREAD_SLEEP_TIME);
					} catch (InterruptedException e) {
					}
				}else {
					Global.BlackLineDetected = false;
				}
			}else {
				try {
					Thread.sleep(Global.THREAD_SLEEP_TIME*2);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

}
