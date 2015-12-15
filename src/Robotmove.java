import java.awt.geom.Point2D;
import java.util.TimerTask;
import java.util.concurrent.TimeUnit;

import lejos.hardware.motor.Motor;

class RobotParameter{ //���{�b�g�̃p�����[�^
	double dtor; //deg��rad�̕ϊ��q
	float tread; //�g���b�h(m)
	float hankei; //�ԗ֔��a(m)

	double refV; //�ڕW�ԑ̑��x(m/s)
	double refOmega; //�ڕW�ԑ̉�]�p�x(rad)
	double refDeltaOmega;
	double[] sisei; //�ԑ̎p���p
	double deltaOmega; //�T���v�����O�^�C���̎ԑ̉�]�p�x
	double runDis; //�T���v�����O�^�C���̑��s����(m)
	double sumRunDis; //�Z���T�v���܂ł̑��s����(m)
	Point2D.Double location; //���{�b�g�̈ʒu
	float inputL,inputR; //���[�^�ւ̓���
	int rsL,rsR; //���ւ̉�]�p���x
	double v,omega; //�ԑ̑��x�A�ԑ̉�]�p�x
	
	RobotParameter(){
		dtor = Math.PI/180;
		tread = 0.122f;
		hankei = 0.15f;
		sisei = new double[2];
		location = new Point2D.Double();
		sumRunDis = 0;
	}
}

class CalcObst{ //��Q�����W�v�Z�Ɋւ���p�����[�^
	Point2D.Double obser,obser2; //���{�b�g���猩���ϑ��_
	Point2D.Double obserPrime,obserPrime2; //�O�����̊ϑ��_(���{�b�g�n)
	Point2D.Double obserThild;
	Point2D.Double obserW,obserW2; //���[���h���W�n�ł̊ϑ��_
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
	Gridmap gm; //�O���b�h�}�b�v�C���X�^���X
	SensorMearsure sm; //�Z���T�C���X�^���X�ւ̎Q��
	long timeMax; //msec
	double mstos; //msec->sec
	long time0;
	long[] time;
	long deltaTime;
	long delay;
	RobotParameter robo; //���{�b�g�p�����[�^�\����
	CalcObst co;
	
	Robotmove(long startTime, long timemax, SensorMearsure sensor){
		time = new long[2];
		time0 = startTime;
		timeMax = timemax;
		time[0] = time0;
		sm = sensor;
		rtod = 180/Math.PI;
		mstos = 0.001;
		delay = 3000; //�^�X�N���s�܂ł̒x������(msec)
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
				if(robo.sumRunDis != 0 && robo.deltaOmega == 0){ //���i
					gm.GaussianUpdate(co.obserW);
					gm.GaussianUpdate(co.obserW2);
				}
				else if(robo.sumRunDis != 0 && robo.deltaOmega != 0){ //����
					if(robo.deltaOmega > 0){ //����p����
						gm.GaussianUpdate(co.obserW2);
					}
					else{ //����p����
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
	
	public void runRobot(){ //�ڕW�ԑ̑��x�Ǝp���p���獶�E���[�^�̓��͒l���v�Z�����s
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
	
	public void odometry(){ //�I�h���g��	�ɂ��ʒu����
		robo.runDis = robo.v*(deltaTime*mstos);
		robo.sumRunDis += robo.runDis;
		robo.sisei[1] =  robo.deltaOmega + robo.sisei[0];
		robo.location.setLocation(robo.runDis*Math.cos(robo.sisei[1]) + robo.location.x,robo.runDis*Math.sin(robo.sisei[1]) + robo.location.y); //���{�b�g�̌��݈ʒu�𐄒�
		robo.deltaOmega = robo.sisei[1] - robo.sisei[0];
		robo.sisei[0] = robo.sisei[1];
	}
	
	public void observe(){ //�����f�[�^�����Q���ʒu�̐���
		co.cos_d_theta = (sm.len[1]*sm.len[1] + sm.len[0]*sm.len[0] - robo.sumRunDis*robo.sumRunDis)/(2*sm.len[1]*sm.len[0]); //�]���藝
		co.deltaTheta = Math.acos(co.cos_d_theta); //�t�O�p�֐��ɂ�胢�Ƃ����߂�
		co.sin_thetaThild = sm.len[1]*Math.sin(co.deltaTheta)/robo.sumRunDis;
		co.thetaThild = Math.asin(co.sin_thetaThild);
		co.obserThild.setLocation(sm.len[0]*Math.sin(co.thetaThild), sm.len[0]*Math.cos(co.thetaThild));
		co.obserPrime.setLocation(Math.cos(co.deltaTheta)*co.obserThild.x-Math.sin(co.deltaTheta)*co.obserThild.y, Math.sin(co.deltaTheta)*co.obserThild.x+Math.cos(co.deltaTheta)*co.obserThild.y);
		co.thetaPrime = Math.atan(co.obserPrime.x/co.obserPrime.y);
		co.theta = co.thetaPrime + robo.deltaOmega + co.deltaTheta;
		co.obser.setLocation(sm.len[1]*Math.sin(co.theta), sm.len[1]*Math.cos(co.theta));
		co.obser2.setLocation(-sm.len[1]*Math.sin(co.theta), sm.len[1]*Math.cos(co.theta));
		co.obserPrime.setLocation(co.obser); co.obserPrime2.setLocation(co.obser2);
		
		if(robo.sumRunDis != 0 && robo.deltaOmega == 0){ //���i
			co.obserW.x = co.obser.x*Math.sin(robo.sisei[1]) + co.obser.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW.y = co.obser.x*-Math.cos(robo.sisei[1]) + co.obser.y*Math.sin(robo.sisei[1]) + robo.location.y;
			co.obserW2.x = co.obser2.x*Math.sin(robo.sisei[1]) + co.obser2.y*Math.cos(robo.sisei[1]) + robo.location.x;
			co.obserW2.y = co.obser2.x*-Math.cos(robo.sisei[1]) + co.obser2.y*Math.sin(robo.sisei[1]) + robo.location.y;
		}
		else if(robo.sumRunDis != 0 && robo.deltaOmega != 0){ //����
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
