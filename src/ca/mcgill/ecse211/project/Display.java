package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.main.Global;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;

/**
 * A Thread responsible of displaying data
 * on the EV3 screen.
 * 
 * @author 	Chen He
 * @version 1.0
*/
public class Display extends Thread {

	public static int SLEEP_TIME = 500;
	
	public Display() {
	}
	
    /**
     * <p>Can display up to five lines on the screen.
     * It shows the values of the display variables
     * {@link Global.firstLine}, {@link Global.secondLine}
     * and so on. These variables can be set by any thread.</p>
     * <p>The refreshing rate is 2Hz</p>
    */
	public void run() {
		final TextLCD t = LocalEV3.get().getTextLCD();
		
		while (true) {
			
			t.clear();
			t.drawString(Global.firstLine, 0, 0);
			t.drawString(Global.secondLine, 0, 1);
			t.drawString(Global.thirdLine, 0, 2);
			t.drawString(Global.forthLine, 0, 3);
			t.drawString(Global.fifthLine, 0, 4);
			
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch bl
			}
		}
	}

	

}
