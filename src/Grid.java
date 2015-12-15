import java.awt.geom.Point2D;
import java.io.PrintWriter;

public class Grid {
	private Point2D.Float zahyou = new Point2D.Float(); //グリッド位置
	private int status; //グリッド状態(0:未走査, 1:空間確定, 2:障害物確定, 3:推定中)
	private double loggit; //対数オッズ値（初期値は1）
	private double exp; //障害物かどうかの確率(0:空間～1:障害物) 初期値は0.5
	
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
