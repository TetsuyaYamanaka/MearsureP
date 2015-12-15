import java.awt.geom.Point2D;
import java.io.PrintWriter;

public class Grid {
	private Point2D.Float zahyou = new Point2D.Float(); //�O���b�h�ʒu
	private int status; //�O���b�h���(0:������, 1:��Ԋm��, 2:��Q���m��, 3:���蒆)
	private double loggit; //�ΐ��I�b�Y�l�i�����l��1�j
	private double exp; //��Q�����ǂ����̊m��(0:��ԁ`1:��Q��) �����l��0.5
	
	Grid(float x,float y){
		zahyou.x = x; zahyou.y = y;
		status = 0;
		exp = 0.5;
		loggit = Math.log(exp/(1-exp));
	}
	
	public Point2D.Float ReturnZahyou(){
		return zahyou;
	}
	
	public int ReturnStatus(){
		return status;
	}
	
	public double ReturnExp(){
		return exp;
	}
	
	public void UpdateGrid(double newLoggit){
		status = 3;
		loggit += newLoggit;
		exp = 1/(1+Math.exp(-loggit));
	}
	
	public void UpdateGrid(int flag){
		switch(flag){
		case 1:
			status = 1;
			exp = 0;
			break;
		case 2:
			status = 2;
			exp = 1;
			break;
		case 3:
			if(exp == 1){
				break;
			}
			else{
				status = 3;
				exp += 0.1;
				if(exp == 1){
					status = 2;
				}
				break;
			}
		}
	}
	
	public void PrintGrid(PrintWriter pw){
		pw.println(zahyou.x + "\t" + zahyou.y + "\t" + exp);
	}
}
