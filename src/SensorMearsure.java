import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;

public class SensorMearsure extends TimerTask{
	EV3UltrasonicSensor uss;
	float[] dist;
	int size; //�Z���T�v���l�i�[�z��̃T�C�Y
	double[] len;
	long timeMax;
	long time0,time;
	int i,flag,initFlag;
	long delay; //�^�X�N���s�܂ł̒x������(msec)
	
	SensorMearsure(long startTime, long timemax){
		time0 = startTime;
		timeMax = timemax;
		uss = new EV3UltrasonicSensor(SensorPort.S4);
		size = uss.sampleSize();
		dist = new float[size];
		len = new double[2];
		flag = 0; initFlag = 0;
		delay = 3000;
	}
	
	public void run(){
		time = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
		
		if(time>=timeMax){
			uss.close();
			this.cancel();
		}
		
		else if(time>=delay){ //delay(3000msec)�o�߂�����ɖ{�i�I�ȏ��������s������
			if(initFlag == 0){
				uss.enable();
				uss.getDistanceMode().fetchSample(dist,0);
				uss.disable();
				len[1] = dist[0];
				initFlag = 1;
			}
			else if(flag == 0){
				len[0] = len[1];
				uss.enable();
				uss.getDistanceMode().fetchSample(dist,0);
				uss.disable();
				len[1] = dist[0];
				flag = 1;
			}
		}
	}
}
