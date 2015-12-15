import java.awt.geom.Point2D;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Gridmap{ //�P�ʌn:m
	Date nowTime = new Date(); //���ݎ����̃f�[�^
	//SimpleDateFormat sdf = new SimpleDateFormat("MMdd_HHmmss"); //�����f�[�^���t�H�[�}�b�g��
	float x,y;
	int i,j,xcnt,ycnt;
	int index_x,index_y,index;
	float resolution; //�O���b�h�Ԋu
	int width; //x�����̃O���b�h��
	int height; //y�����̃O���b�h��
	int gridMax = width*height;
	float centerX; //���S��x���W
	float centerY; //���S��y���W
	float startX;
	float startY;
	Grid[] gm; //�O���b�h�}�b�v
	
	Gridmap(){ //�R���X�g���N�^
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
				gm[index] = new Grid(x,y); //�O���b�h�}�b�v�I�u�W�F�N�g�̏�����
			}
		}
	}
	
	public void UpdateGrid(Point2D.Double world,int flag){ //�O���b�h�̏�Ԃ��X�V���郁�\�b�h
		index_x = (int)((world.x - centerX)/resolution - width/2);
		index_y = (int)((world.y - centerY)/resolution - height/2);
		index = (height*index_y + index_x) + (gridMax - 1);
		gm[index].UpdateGrid(flag);
	}
	
	public void GaussianUpdate(Point2D.Double obser){ //�K�E�X�֐��ɂ��m���̏d�݂Â�
		double sigma=1;
		double distance; //�ϑ��_�Ɗe�O���b�h�Ƃ̋���
		double loggit; //�ΐ��I�b�Y
		
		for(i=0;i<gridMax;i++){
			distance = obser.distance(gm[i].ReturnZahyou());
			if(distance <= 0.05){
				loggit = Math.exp(-Math.pow(distance, 2)/(2*Math.pow(sigma, 2)))/(Math.sqrt(2*Math.PI)*sigma);
				gm[i].UpdateGrid(loggit);
			}
		}
	}
	
	public void PrintGrid(){ //�O���b�h�}�b�v�̏��ix,y,��Q���̊m���j���t�@�C���o�͂��郁�\�b�h
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
