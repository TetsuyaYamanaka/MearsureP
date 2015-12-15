import java.awt.geom.Point2D;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Gridmap{ //単位系:m
	Date nowTime = new Date(); //現在時刻のデータ
	//SimpleDateFormat sdf = new SimpleDateFormat("MMdd_HHmmss"); //時刻データをフォーマット化
	float x,y;
	int i,j,xcnt,ycnt;
	int index_x,index_y,index;
	float resolution; //グリッド間隔
	int width; //x方向のグリッド数
	int height; //y方向のグリッド数
	int gridMax = width*height;
	float centerX; //中心のx座標
	float centerY; //中心のy座標
	float startX;
	float startY;
	Grid[] gm; //グリッドマップ
	
	Gridmap(){ //コンストラクタ
		resolution = 0.01f;
		width = 101;
		height = 101;
		gridMax = width*height;
		gm = new Grid[gridMax];
		centerX = 0.5f; centerY = 0;
		startX = 0; startY = -0.5f;
		
		for(ycnt=0;ycnt<height;ycnt++){
			y = ycnt*resolution+startY;
			for(xcnt=0;xcnt<width;xcnt++){
				x = xcnt*resolution+startX;
				index = height*ycnt + xcnt;
				gm[index] = new Grid(x,y); //グリッドマップオブジェクトの初期化
			}
		}
	}
	
	public void UpdateGrid(Point2D.Double world,int flag){ //グリッドの状態を更新するメソッド
		index_x = (int)((world.x - centerX)/resolution - width/2);
		index_y = (int)((world.y - centerY)/resolution - height/2);
		index = (height*index_y + index_x) + (gridMax - 1);
		gm[index].UpdateGrid(flag);
	}
	
	public void GaussianUpdate(Point2D.Double obser){ //ガウス関数による確率の重みづけ
		double sigma=1;
		double distance; //観測点と各グリッドとの距離
		double loggit; //対数オッズ
		
		for(i=0;i<gridMax;i++){
			distance = obser.distance(gm[i].ReturnZahyou());
			if(distance <= 0.05){
				loggit = Math.exp(-Math.pow(distance, 2)/(2*Math.pow(sigma, 2)))/(Math.sqrt(2*Math.PI)*sigma);
				gm[i].UpdateGrid(loggit);
			}
		}
	}
	
	public void PrintGrid(){ //グリッドマップの情報（x,y,障害物の確率）をファイル出力するメソッド
		try{
			PrintWriter pw = new PrintWriter(new BufferedWriter(new FileWriter("Gridmap.dat")));
			for(i=0;i<gridMax;i++){
				gm[i].PrintGrid(pw);
			}
			pw.close();
		} catch(IOException e){
			System.out.println(e);
		}		
	}
}
