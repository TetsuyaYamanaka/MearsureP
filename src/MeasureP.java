import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Timer;
import java.util.TimerTask;

import lejos.hardware.motor.*;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.utility.Delay;

public class MeasureP {
	static class zahyou{
		double x,y;
	}
	
	static final double tread = 12.2; //トレッド(cm)
	static final double hankei = 1.5; //車輪半径(cm)
	static final long delay = 0;
	static final long period = 100;
	static final int DATAMAX = 1000;
	
	static EV3UltrasonicSensor uss = new EV3UltrasonicSensor(SensorPort.S1);
	static int cnt = 0;
	static float[] dist = new float[1];
	static int rs_A,rs_B; //車輪の回転角速度
	static float real_v; //車体速度
	static float a = 0,a_p = 0; //ロボットの移動距離
	static float ref_v = 5; //目標車体速度(cm/sec)
	
	static double[] len_p = new double[DATAMAX]; //前時刻の障害物との距離
	static double[] len = new double[DATAMAX]; //現時刻の障害物との距離
	static double[] d_a = new double[DATAMAX]; //1サンプリングタイムの走行距離
	static zahyou[] p_p = new zahyou[DATAMAX], p2_p = new zahyou[DATAMAX]; //前時刻の障害物座標(候補2つ)
	static zahyou[] p = new zahyou[DATAMAX], p2 = new zahyou[DATAMAX]; //現時刻の障害物座標(候補2つ)
	
	public static void main(String[] args) {
		int i;
		Date nowTime = new Date(); //現在時刻のデータ
		SimpleDateFormat sdf = new SimpleDateFormat("MMdd_HHmmss"); //時刻データをフォーマット化
		String fileName = "DataP" + sdf.format(nowTime) + ".dat"; //起動時の時刻をファイル名に設定
		float ref_rs = (float)((ref_v*180)/(hankei*Math.PI)); //車輪の回転角速度(rad→deg)(入力)
		double d_theta;
		double theta,theta_p,theta2,theta2_p;
		double sin_theta,sin_theta_p;
		double cos_d_theta;
		TimerTask Task = new TimerTask(){
			public void run(){
				if(cnt > 0){
					len_p[cnt] = len[cnt-1];
				}
				Motor.A.forward(); Motor.B.forward();
				rs_A = Motor.A.getRotationSpeed(); rs_B = Motor.B.getRotationSpeed();
				uss.getDistanceMode().fetchSample(dist,0); //動作後の障害物との距離
				
				len[cnt] = dist[0]*100; //m→cm
				real_v = (float)((hankei*(rs_B + rs_A)/2)*(Math.PI/180));
				a += real_v*(period*0.001);
				d_a[cnt] = a - a_p; //1サンプリングタイムの移動距離
				
				a_p = a;
				cnt++;
			}
		};
		Timer timer = new Timer();
		
		Motor.A.resetTachoCount(); Motor.B.resetTachoCount();
		Motor.A.setAcceleration(1000); Motor.B.setAcceleration(1000);
		Motor.A.setSpeed(ref_rs); Motor.B.setSpeed(ref_rs);
		uss.getDistanceMode().fetchSample(dist,0); //動作前の障害物との距離
		len_p[cnt] = dist[0]*100; //m→cm
		
		timer.scheduleAtFixedRate(Task, delay, period);
		Delay.msDelay(5000);
		timer.cancel();

		Motor.A.stop(); Motor.B.stop();
		
		for(i=0;i<DATAMAX;i++){ //オブジェクト配列の初期化
			p_p[i] = new zahyou();
			p2_p[i] = new zahyou();
			p[i] = new zahyou();
			p2[i] = new zahyou();
		}
		
		try{
			FileWriter fw = new FileWriter(fileName);
			BufferedWriter bw = new BufferedWriter(fw);
			PrintWriter pw = new PrintWriter(bw);
			
			for(i=0;i<cnt;i++){
				cos_d_theta = (len[i]*len[i] + len_p[i]*len_p[i] - d_a[i]*d_a[i])/(2*len[i]*len_p[i]);
				if(cos_d_theta < -1 || 1 < cos_d_theta){ //l',l'aが三角形を成さない場合
					d_theta = -999;
					theta_p = -999; theta2_p = -999;
					theta = -999; theta2 = -999;
					p_p[i].x = -999; p_p[i].y = -999;
					p2_p[i].x = -999; p2_p[i].y = -999;
					p[i].x = -999; p[i].y = -999;
					p2[i].x = -999; p2[i].y = -999;
				}
				else{
					d_theta = Math.acos(cos_d_theta); //逆三角関数によりΔθを求める
					sin_theta = len_p[i]*Math.sin(d_theta)/d_a[i]; sin_theta_p = len[i]*Math.sin(d_theta)/d_a[i];
					theta = Math.asin(sin_theta); theta_p = Math.asin(sin_theta_p); //逆三角関数によりθ、θ´を求める
					theta2 = -theta; theta2_p = -theta_p;
				
					p_p[i].x = len_p[i]*Math.sin(theta_p); p_p[i].y = len_p[i]*Math.cos(theta_p);
					p[i].x = len[i]*Math.sin(theta); p[i].y = len[i]*Math.cos(theta);
					p2_p[i].x = len_p[i]*Math.sin(theta2_p); p2_p[i].y = len_p[i]*Math.cos(theta2_p);
					p2[i].x = len[i]*Math.sin(theta2); p2[i].y = len[i]*Math.cos(theta2);
				}
				
				 if(p[i].x != -999){
				 	pw.println(i*period + "\t" + p[i].x + "\t" + p[i].y + "\t" + p2[i].x + "\t" + p2[i].y);
				 }
			}
			pw.close();
		}catch(IOException e){
			System.out.println(e);
		}
	}
}
