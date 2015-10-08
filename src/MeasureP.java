import java.io.*;
import java.util.Timer;
import java.util.TimerTask;

import lejos.hardware.motor.*;
import lejos.hardware.lcd.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

public class MeasureP {
	static class zahyou{
		double x,y;
	}
	
	static final double tread = 12.2; //�g���b�h(cm)
	static final double hankei = 1.5; //�ԗ֔��a(cm)
	static final long delay = 0;
	static final long period = 100;
	static final int DATAMAX = 1000;
	
	static EV3UltrasonicSensor uss = new EV3UltrasonicSensor(SensorPort.S1);
	static int cnt = 0;
	static float[] dist = new float[1];
	//static double len,len_p; //��Q���Ƃ̋���
	static int rs_A,rs_B; //�ԗւ̉�]�p���x
	static float real_v; //�ԑ̑��x
	static float a = 0,a_p = 0; //���{�b�g�̈ړ�����
	static float ref_v = 5; //�ڕW�ԑ̑��x(cm/sec)
	static double d_theta,theta,theta_p;
	static double sin_theta,sin_theta_p;
	static double cos_d_theta;
	static double[] len_p = new double[DATAMAX]; //�O�����̏�Q���Ƃ̋���
	static double[] len = new double[DATAMAX]; //�������̏�Q���Ƃ̋���
	static double[] d_a = new double[DATAMAX]; //1�T���v�����O�^�C���̑��s����
	static zahyou[] p_p = new zahyou[DATAMAX], p2_p = new zahyou[DATAMAX]; //�O�����̏�Q�����W(���2��)
	static zahyou[] p = new zahyou[DATAMAX], p2 = new zahyou[DATAMAX]; //�������̏�Q�����W(���2��)
	
	public static void main(String[] args) {
		int i;
		float ref_v_rol = (float)((ref_v*180)/(hankei*Math.PI)); //�ԗւ̉�]�p���x(rad��deg)(����)
		
		TimerTask Task = new TimerTask(){
			public void run(){
				if(cnt != 0){
					len_p[cnt] = len[cnt-1];
				}
				Motor.A.forward(); Motor.B.forward();
				rs_A = Motor.A.getRotationSpeed(); rs_B = Motor.B.getRotationSpeed();
				uss.getDistanceMode().fetchSample(dist,0); //�����̏�Q���Ƃ̋���
				
				len[cnt] = dist[0]*100; //m��cm
				real_v = (float)((hankei*(rs_B + rs_A)/2)*(Math.PI/180));
				a += real_v*(period*0.001);
				d_a[cnt] = a - a_p; //1�T���v�����O�^�C���̈ړ�����
				
				a_p = a;
				cnt++;
			}
		};
		Timer timer = new Timer();
		
		Motor.A.resetTachoCount(); Motor.B.resetTachoCount();
		Motor.A.setAcceleration(100); Motor.B.setAcceleration(100);
		Motor.A.setSpeed(ref_v_rol); Motor.B.setSpeed(ref_v_rol);
		uss.getDistanceMode().fetchSample(dist,0); //����O�̏�Q���Ƃ̋���
		len_p[cnt] = dist[0]*100; //m��cm
		
		timer.scheduleAtFixedRate(Task, delay, period);
		Delay.msDelay(4000);
		timer.cancel();

		Motor.A.stop(); Motor.B.stop();
		
		try{
			FileWriter fw = new FileWriter("dataP_1008.dat");
			BufferedWriter bw = new BufferedWriter(fw);
			PrintWriter pw = new PrintWriter(bw);
			//pw.println("time\tl'\tl\ta\tP'(x,y)\tP(x,y)");
			pw.println("time\tl'\tl\ta\td_theta");
			for(i=0;i<cnt;i++){
				cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - d_a[i]*d_a[i])/(2*len[i]*len_p[i]);
				d_theta = Math.acos(cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
				sin_theta = len_p[i]*Math.sin(d_theta)/d_a[i]; sin_theta_p = len[i]*Math.sin(d_theta)/d_a[i];
				theta = Math.asin(sin_theta); theta_p = Math.asin(sin_theta_p); //�t�O�p�֐��ɂ��ƁA�ƁL�����߂�
				
				pw.println(i*period + "\t" + len_p[i] + "\t" + len[i] + "\t" + d_a[i] + "\t" + d_theta);
				
				/*p_p[i].x = len_p[i]*Math.sin(theta_p); p_p[i].y = len_p[i]*Math.cos(theta_p);
				p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
				p2_p[i].x = p_p[i].x*-1; p2_p[i].y = p_p[i].y*-1;
				p2[i].x = p[i].x*-1; p2[i].y = p[i].y*-1;
				pw.println(i*period + "\t" + len_p[i] + "\t" + len[i] + "\t" + d_a[i] + "\t" + p_p[i].x + ", " + p_p[i].y + "\t" + p[i].x + ", " + p[i].y);*/
							}
			pw.close();
		}catch(IOException e){
			System.out.println(e);
		}
	}
}
