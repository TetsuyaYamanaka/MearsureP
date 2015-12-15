import java.util.Timer;

import lejos.utility.Delay;

public class MeasureP {	
	public static void main(String[] args) {
		long maxTime = 8000;
		final long delay = 0;
		final long periodRobot = 10;
		final long periodSensor = 50;
		Timer timerRobo = new Timer(), timerSensor = new Timer();
		long time0 = System.nanoTime();
		SensorMearsure sm = new SensorMearsure(time0,maxTime);
		Robotmove rm = new Robotmove(time0,maxTime,sm);
		
		timerRobo.scheduleAtFixedRate(rm, delay, periodRobot);
		timerSensor.scheduleAtFixedRate(sm, delay, periodSensor);
		Delay.msDelay(maxTime+3000);
		
		timerRobo.cancel();
		timerSensor.cancel();
	}
}
