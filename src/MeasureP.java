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
	static double len,len_p; //��Q���Ƃ̋���
	static int rs_A,rs_B; //�ԗւ̉�]�p���x
	static float real_v; //�ԑ̑��x
	static float a = 0,a_p = 0,d_a; //���{�b�g�̈ړ�����
	static float ref_v = 5; //�ڕW�ԑ̑��x(cm/sec)
	static double d_theta,theta,theta_p;
	static double sin_theta,sin_theta_p;
	static double cos_d_theta;
	static double[] kyoril_p = new double[DATAMAX];
	static double[] kyoril = new double[DATAMAX];
	static double[] kyoria = new double[DATAMAX];
	static zahyou[] p_p = new zahyou[DATAMAX], p2_p = new zahyou[DATAMAX]; //�O�����̏�Q�����W(���2��)
	static zahyou[] p = new zahyou[DATAMAX], p2 = new zahyou[DATAMAX]; //�������̏�Q�����W(���2��)
	
	public static void main(String[] args) {
		int i;
		float ref_v_rol = (float)((ref_v*180)/(hankei*Math.PI)); //�ԗւ̉�]�p���x(rad��deg)(����)
		
		TimerTask Task = new TimerTask(){
			public void run(){
				Motor.A.forward(); Motor.B.forward();
				rs_A = Motor.A.getRotationSpeed(); rs_B = Motor.B.getRotationSpeed();
				uss.getDistanceMode().fetchSample(dist,0); //�����̏�Q���Ƃ̋���
				
				len = dist[0]*100; //m��cm
				real_v = (float)((hankei*(rs_B + rs_A)/2)*(Math.PI/180));
				a += real_v*(period*0.001);
				d_a = a - a_p; //1�T���v�����O�^�C���̈ړ�����
				cos_d_theta = (len*len + len_p*len_p - d_a*d_a)/(2*len*len_p);
				d_theta = Math.acos(cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
				sin_theta = len_p*Math.sin(d_theta)/d_a; sin_theta_p = len*Math.sin(d_theta)/d_a;
				theta = Math.asin(sin_theta); theta_p = Math.asin(sin_theta_p); //�t�O�p�֐��ɂ��ƁA�ƁL�����߂�
				
				p_p[cnt].x = len_p*Math.sin(theta_p); p_p[cnt].y = len_p*Math.cos(theta_p);
				p[cnt].x = len*Math.sin(theta); p[cnt].y = len*Math.cos(theta);
				p2_p[cnt].x = p_p[cnt].x*-1; p2_p[cnt].y = p_p[cnt].y*-1;
				p2[cnt].x = p[cnt].x*-1; p2[cnt].y = p[cnt].y*-1;
				kyoril_p[cnt] = len_p; kyoril[cnt] = len; kyoria[cnt] = d_a;
				
				a_p = a; len_p = len;
				cnt++;
			}
		};
		Timer timer = new Timer();
		
		Motor.A.resetTachoCount(); Motor.B.resetTachoCount();
		Motor.A.setAcceleration(100); Motor.B.setAcceleration(100);
		Motor.A.setSpeed(ref_v_rol); Motor.B.setSpeed(ref_v_rol);
		uss.getDistanceMode().fetchSample(dist,0); //����O�̏�Q���Ƃ̋���
		len_p = dist[0]*100; //m��cm
		
		timer.scheduleAtFixedRate(Task, delay, period);
		Delay.msDelay(4000);
		timer.cancel();

		Motor.A.stop(); Motor.B.stop();
		
		try{
			FileWriter fw = new FileWriter("dataP_1008.dat");
			BufferedWriter bw = new BufferedWriter(fw);
			PrintWriter pw = new PrintWriter(bw);
			pw.println("time\tl'\tl\ta\tP'(x,y)\tP(x,y)");
			for(i=0;i<cnt;i++){
				pw.println(i*period + "\t" + kyoril_p[i] + "\t" + kyoril[i] + "\t" + kyoria[i] + "\t" + p_p[cnt].x + ", " + p_p[cnt].y + "\t" + p[cnt].x + ", " + p[cnt].y);
			}
			pw.close();
		}catch(IOException e){
			System.out.println(e);
		}
	}
}
