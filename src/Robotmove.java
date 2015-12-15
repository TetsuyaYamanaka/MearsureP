import java.awt.geom.Point2D;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import lejos.hardware.motor.Motor;

class RobotParameter{ //ロボットのパラメータ
	double dtor; //deg→radの変換子
	float tread; //トレッド(m)
	float hankei; //車輪半径(m)

	double refV; //目標車体速度(m/s)
	double refOmega; //目標車体回転角度(rad)
	double refDeltaOmega;
	double[] sisei; //車体姿勢角
	double deltaOmega; //サンプリングタイムの車体回転角度
	double runDis; //サンプリングタイムの走行距離(m)
	double sumRunDis; //センサ計測までの走行距離(m)
	Point2D.Double location; //ロボットの位置
	float inputL,inputR; //モータへの入力
	int rsL,rsR; //両輪の回転角速度
	double v,omega; //車体速度、車体回転角度
	
	RobotParameter(){
		dtor = Math.PI/180;
		tread = 0.122f;
		hankei = 0.15f;
		sisei = new double[2];
		location = new Point2D.Double();
		sumRunDis = 0;
	}
}

class CalcObst{ //障害物座標計算に関するパラメータ
	Point2D.Double obser,obser2; //ロボットから見た観測点
	Point2D.Double obserPrime,obserPrime2; //前時刻の観測点(ロボット系)
	Point2D.Double obserThild;
	Point2D.Double obserW,obserW2; //ワールド座標系での観測点
	double deltaTheta;
	double theta,theta2;
	double thetaPrime;
	double thetaThild;
	double sin_thetaThild;
	double sin_theta;
	double cos_d_theta;
	
	CalcObst(){
		obser = new Point2D.Double(); obser2 = new Point2D.Double();
		obserPrime = new Point2D.Double(); obserPrime2 = new Point2D.Double();
		obserThild = new Point2D.Double();
		obserW = new Point2D.Double(); obserW2 = new Point2D.Double();
	}
}

public class Robotmove extends TimerTask{
	double rtod; //rad->deg
	Gridmap gm; //グリッドマップインスタンス
	SensorMearsure sm; //センサインスタンスへの参照
	long timeMax; //msec
	double mstos; //msec->sec
	long time0;
	long[] time;
	long deltaTime;
	long delay;
	RobotParameter robo; //ロボットパラメータ構造体
	CalcObst co;
	
	Robotmove(long startTime, long timemax, SensorMearsure sensor){
		time = new long[2];
		time0 = startTime;
		timeMax = timemax;
		time[0] = time0;
		sm = sensor;
		rtod = 180/Math.PI;
		mstos = 0.001;
		delay = 3000; //タスク実行までの遅延時間(msec)
		robo = new RobotParameter();
		co = new CalcObst();
		gm = new Gridmap();
		
		Motor.A.setAcceleration(1000); Motor.B.setAcceleration(1000);
	}
	
	public void run(){
		time[1] = TimeUnit.MILLISECONDS.convert(System.nanoTime()-time0, TimeUnit.NANOSECONDS);
		
		if(time[1]>=timeMax){
			Motor.A.stop(); Motor.B.stop();
			gm.PrintGrid();
			this.cancel();
		}
		
		else if(time[1]>=delay){
			deltaTime = time[1] - time[0];
			
			runRobot();
			odometry();
			if(sm.flag==1){
				observe();
				if(robo.sumRunDis != 0 && robo.deltaOmega == 0){ //直進
					gm.GaussianUpdate(co.obserW);
					gm.GaussianUpdate(co.obserW2);
				}
				else if(robo.sumRunDis != 0 && robo.deltaOmega != 0){ //旋回
					if(robo.deltaOmega > 0){ //旋回角が正
						gm.GaussianUpdate(co.obserW2);
					}
					else{ //旋回角が負
						gm.GaussianUpdate(co.obserW);
					}
				}
				robo.sumRunDis = 0;
				sm.flag = 0;
			}
			gm.UpdateGrid(robo.location,1);
			
		}
		time[0] = time[1];
	}
	
	public void runRobot(){ //目標車体速度と姿勢角から左右モータの入力値を計算し走行
		getReference(time[1]);
		robo.inputL = (float)(((robo.refV-(robo.tread*robo.refDeltaOmega)/2)/robo.hankei)*rtod); //rad->deg
		robo.inputR = (float)(((robo.refV+(robo.tread*robo.refDeltaOmega)/2)/robo.hankei)*rtod); //rad->deg
		Motor.A.setSpeed(robo.inputL); Motor.B.setSpeed(robo.inputR);
		Motor.A.forward(); Motor.B.forward();
		
		robo.rsL = Motor.A.getRotationSpeed();
		robo.rsR = Motor.B.getRotationSpeed();
		robo.v = (robo.rsR+robo.rsL)*robo.hankei/2;
		robo.deltaOmega = (robo.rsR-robo.rsL)*robo.hankei/robo.tread;
	}
	
	public void odometry(){ //オドメトリ	による位置推定
		robo.runDis = robo.v*(deltaTime*mstos);
		robo.sumRunDis += robo.runDis;
		robo.sisei[1] =  robo.deltaOmega + robo.sisei[0];
		robo.location.setLocation(robo.runDis*Math.cos(robo.sisei[1]) + robo.location.x,robo.runDis*Math.sin(robo.sisei[1]) + robo.location.y); //ロボットの現在位置を推定
		robo.deltaOmega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void observe(){ //距離データから障害物位置の推定
		co.cos_d_theta = (sm.len[1]*sm.len[1] + sm.len[0]*sm.len[0] - robo.sumRunDis*robo.sumRunDis)/(2*sm.len[1]*sm.len[0]); //余弦定理
		co.deltaTheta = Math.acos(co.cos_d_theta); //逆三角関数によりΔθを求める
		co.sin_thetaThild = sm.len[1]*Math.sin(co.deltaTheta)/robo.sumRunDis;
		co.thetaThild = Math.asin(co.sin_thetaThild);
		co.obserThild.setLocation(sm.len[0]*Math.sin(co.thetaThild), sm.len[0]*Math.cos(co.thetaThild));
		co.obserPrime.setLocation(Math.cos(co.deltaTheta)*co.obserThild.x-Math.sin(co.deltaTheta)*co.obserThild.y, Math.sin(co.deltaTheta)*co.obserThild.x+Math.cos(co.deltaTheta)*co.obserThild.y);
		co.thetaPrime = Math.atan(co.obserPrime.x/co.obserPrime.y);
		co.theta = co.thetaPrime + robo.deltaOmega + co.deltaTheta;
		co.obser.setLocation(sm.len[1]*Math.sin(co.theta), sm.len[1]*Math.cos(co.theta));
		co.obser2.setLocation(-sm.len[1]*Math.sin(co.theta), sm.len[1]*Math.cos(co.theta));
		co.obserPrime.setLocation(co.obser); co.obserPrime2.setLocation(co.obser2);
		
		if(robo.sumRunDis != 0 && robo.deltaOmega == 0){ //直進
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]) + co.obser.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW.y = co.obser.x*-Math.cos(robo.sisei[1]) + co.obser.y*Math.sin(robo.sisei[1]) + robo.location.y;
			co.obserW2.x = co.obser2.x*Math.sin(robo.sisei[1]) + co.obser2.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW2.y = co.obser2.x*-Math.cos(robo.sisei[1]) + co.obser2.y*Math.sin(robo.sisei[1]) + robo.location.y;
		}
		else if(robo.sumRunDis != 0 && robo.deltaOmega != 0){ //旋回
			if((robo.sisei[1]>0 && robo.deltaOmega>0) || (robo.sisei[1]<0 && robo.deltaOmega<0) || (robo.sisei[1]<0 && robo.deltaOmega>0 && robo.sisei[1]>Math.abs(robo.deltaOmega))){
				co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]+robo.deltaOmega) + co.obser.y*Math.cos(robo.sisei[1]+robo.deltaOmega) + robo.location.x;
				co.obserW.y = co.obser.x*-Math.cos(robo.sisei[1]+robo.deltaOmega) + co.obser.y*Math.sin(robo.sisei[1]+robo.deltaOmega) + robo.location.y;
			}
		}
	}
	
	public void getReference(long time){
		robo.refV = 0.2;
		robo.refOmega = 0*robo.dtor;
		robo.refDeltaOmega = robo.refOmega/(timeMax*mstos);
	}
}
