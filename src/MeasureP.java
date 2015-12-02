import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;
import java.awt.geom.*;

import lejos.hardware.motor.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

class RobotParameter{
	final double dtor = Math.PI/180; //deg��rad�̕ϊ��q
	final int datamax = 1000;
	final double tread = 1.22; //�g���b�h(m)
	final double hankei = 0.15; //�ԗ֔��a(m)
	
	int i;
	double ref_v; //�ԑ̑��x(m/s)
	double ref_omega; //�ԑ̉�]�p�x(rad)
	double[] sisei = new double[datamax]; //�ԑ̎p���p
	double[] d_omega = new double[datamax]; //�T���v�����O�^�C���̎ԑ̉�]�p�x
	double[] a = new double[datamax]; //�T���v�����O�^�C���̑��s����(m)
	Point2D.Double[] location = new Point2D.Double[datamax]; //���{�b�g�̈ʒu
	
	RobotParameter(){
		ref_v = 0.04;
		ref_omega = 0*dtor;
		for(i=0;i<datamax;i++){
			location[i] = new Point2D.Double();
		}
	}
}

public class MeasureP {	
	static final float tread = (float)12.2; //�g���b�h(cm)
	static final float hankei = (float)1.5; //�ԗ֔��a(cm)
	static final long delay = 0;
	static final long period = 10;
	static final int DATAMAX = 1000;
	static final float mstos = (float)0.001; //msec��sec�̕ϊ��q
	
	static EV3UltrasonicSensor uss = new EV3UltrasonicSensor(SensorPort.S1);
	static int cnt = 0;
	static float[] dist = new float[1];
	static int[] rs_A = new int[DATAMAX],rs_B = new int[DATAMAX]; //�ԗւ̉�]�p���x
	static double[] real_v = new double[DATAMAX]; //�ԑ̑��x
	static double real_omega; //�ԑ̊p�x
	static double a = 0,a_p = 0; //���{�b�g�̈ړ�����
	static float ref_v = 4; //�ڕW�ԑ̑��x(cm/sec)
	static float ref_omega = (float)(0*Math.PI/180); //�ڕW�ԑ̊p�x
	
	static long[] timestamp = new long[DATAMAX]; //�^�C���X�^���v
	static long[] dt = new long[DATAMAX]; //1�T���v�����O�^�C��
	static float[] len_p = new float[DATAMAX]; //�O�����̏�Q���Ƃ̋���
	static float[] len = new float[DATAMAX]; //�������̏�Q���Ƃ̋���
	static double[] d_a = new double[DATAMAX]; //1�T���v�����O�^�C���̑��s����
		
	static long time0;
	static long time;
	
	public static void main(String[] args) {
		final long timemax = 5000;
		
		int i;
		Date nowTime = new Date(); //���ݎ����̃f�[�^
		SimpleDateFormat sdf = new SimpleDateFormat("MMdd_HHmmss"); //�����f�[�^���t�H�[�}�b�g��
		String fileName = "DataP_" + sdf.format(nowTime) + ".dat"; //�N�����̎������t�@�C�����ɐݒ�
		String fileName2 = "Obstacle_" + sdf.format(nowTime) + ".dat";
		String fileName3 = "Robo_" + sdf.format(nowTime) + ".dat";
		float ref_rs = (float)((ref_v*180)/(hankei*Math.PI)); //�ԗւ̉�]�p���x(rad��deg)(����)
		double d_theta;
		double theta,theta_p,theta2,theta2_p;
		double sin_theta,sin_theta_p;
		double cos_d_theta;
		Point2D.Double[] p_p = new Point2D.Double[DATAMAX], p2_p = new Point2D.Double[DATAMAX]; //�O�����̏�Q�����W(���{�b�g�n�A���2��)
		Point2D.Double[] p = new Point2D.Double[DATAMAX], p2 = new Point2D.Double[DATAMAX]; //�������̏�Q�����W(���{�b�g�n�A���2��)
		double[] sisei = new double[DATAMAX];
		Point2D.Double[] obst = new Point2D.Double[DATAMAX],obst2 = new Point2D.Double[DATAMAX];
		Point2D.Double[] robo = new Point2D.Double[DATAMAX];
		
		TimerTask Task = new TimerTask(){
			
			public void run(){
				time = System.nanoTime() - time0; //�������擾
				timestamp[cnt] = TimeUnit.MILLISECONDS.convert(time, TimeUnit.NANOSECONDS);
				dt[cnt] = timestamp[cnt] - timestamp[cnt-1];
				len_p[cnt] = len[cnt-1];
				rs_A[cnt] = Motor.A.getRotationSpeed(); rs_B[cnt] = Motor.B.getRotationSpeed();
				uss.getDistanceMode().fetchSample(dist,0); //�����̏�Q���Ƃ̋���
				
				len[cnt] = dist[0]*100; //m��cm
				real_v[cnt] = (rs_B[cnt] + rs_A[cnt])*(Math.PI/180)*hankei/2;
				a += real_v[cnt]*dt[cnt]*mstos;
				d_a[cnt] = a - a_p; //1�T���v�����O�^�C���̈ړ�����
				
				a_p = a;
				cnt++;
			}
		};
		Timer timer = new Timer();
		
		time0 = System.nanoTime();
		time = System.nanoTime() - time0;
		timestamp[cnt] = TimeUnit.MILLISECONDS.convert(time, TimeUnit.NANOSECONDS);
		Motor.A.resetTachoCount(); Motor.B.resetTachoCount();
		Motor.A.setAcceleration(1000); Motor.B.setAcceleration(1000);
		Motor.A.setSpeed(ref_rs); Motor.B.setSpeed(ref_rs);
		uss.getDistanceMode().fetchSample(dist,0); //����O�̏�Q���Ƃ̋���
		len[cnt] = dist[0]*100; //m��cm
		rs_A[cnt] = 0; rs_B[cnt] = 0;
		d_a[cnt] = 0;
		cnt++;
		
		
		Motor.A.forward(); Motor.B.forward();
		time0 = System.nanoTime();
		timer.scheduleAtFixedRate(Task, delay, period);
		Delay.msDelay(5000);
		timer.cancel();

		Motor.A.stop(); Motor.B.stop();
		
		for(i=0;i<DATAMAX;i++){ //�I�u�W�F�N�g�z��̏�����
			p_p[i] = new Point2D.Double();
			p2_p[i] = new Point2D.Double();
			p[i] = new Point2D.Double();
			p2[i] = new Point2D.Double();
			robo[i] = new Point2D.Double();
			obst[i] = new Point2D.Double();
			obst2[i] = new Point2D.Double();
		}
		
		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter(fileName)));
			PrintWriter pw2 = new PrintWriter(new BufferedWriter(new FileWriter(fileName2)));
			PrintWriter pw3 = new PrintWriter(new BufferedWriter(new FileWriter(fileName3)));
				
			for(i=0;i<cnt;i++){
				if(i > 0){
					real_omega = (float)((rs_B[i] - rs_A[i])*hankei/tread);
					sisei[i] = (real_omega/timemax)*dt[i]*mstos + sisei[i-1];
					robo[i].setLocation(d_a[i]*Math.cos(sisei[i])+robo[i-1].x,d_a[i]*Math.sin(sisei[i])+robo[i-1].y);
				}
				cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - d_a[i]*d_a[i])/(2*len[i]*len_p[i]);
				if(cos_d_theta < -1 || 1 < cos_d_theta){ //l',l,a���O�p�`�𐬂��Ȃ��ꍇ
					d_theta = -999;
					theta_p = -999; theta2_p = -999;
					theta = -999; theta2 = -999;
					p_p[i].x = -999; p_p[i].y = -999;
					p2_p[i].x = -999; p2_p[i].y = -999;
					p[i].x = -999; p[i].y = -999;
					p2[i].x = -999; p2[i].y = -999;
					obst[i].x = -999; obst[i].y = -999;
					obst2[i].x = -999; obst2[i].y = -999;
				}
				else{
					d_theta = Math.acos(cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
					sin_theta = len_p[i]*Math.sin(d_theta)/d_a[i]; sin_theta_p = len[i]*Math.sin(d_theta)/d_a[i];
					theta = Math.asin(sin_theta); theta_p = Math.asin(sin_theta_p); //�t�O�p�֐��ɂ��ƁA�ƁL�����߂�
					theta2 = -theta; theta2_p = -theta_p;
				
					p_p[i].x = len_p[i]*Math.sin(theta_p); p_p[i].y = len_p[i]*Math.cos(theta_p);
					p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
					p2_p[i].x = len_p[i]*Math.sin(theta2_p); p2_p[i].y = len_p[i]*Math.cos(theta2_p);
					p2[i].x = len[i]*Math.sin(theta2); p2[i].y = len[i]*Math.cos(theta2);
					
					obst[i].x = p[i].x*Math.sin(sisei[i]) + p[i].y*Math.cos(sisei[i]) + robo[i].x;
					obst[i].y = -p[i].x*Math.cos(sisei[i]) + p[i].y*Math.sin(sisei[i]) + robo[i].y;
					obst2[i].x = p2[i].x*Math.sin(sisei[i]) + p2[i].y*Math.cos(sisei[i]) + robo[i].x;
					obst2[i].y = -p2[i].x*Math.cos(sisei[i]) + p2[i].y*Math.sin(sisei[i]) + robo[i].y;
				}
				if(p[i].x != -999){
					pw.println(timestamp[i] + "\t" + p[i].x + "\t" + p[i].y + "\t" + p2[i].x + "\t" + p2[i].y);
					pw2.println(timestamp[i] + "\t" + obst[i].x + "\t" + obst[i].y + "\t" + obst2[i].x + "\t" + obst2[i].y);
					pw3.println(timestamp[i] + "\t" + robo[i].x + "\t" + robo[i].y);
			 }
		}
			pw.close();
			pw2.close();
			pw3.close();
		}catch(IOException e){
			System.out.println(e);
		}
		uss.close();
	}
}
