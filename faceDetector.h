#ifndef _FACE_DETECTOR_
#define _FACE_DETECTOR_

#include<opencv2/opencv.hpp>
#include<iostream>
#include <mat.h>
#include"fconv.h"
#include<time.h>
#include<algorithm>
#include<opencv2/core/core.hpp>
#define INF 1E20
#define  eps 0.0001
typedef double MatType;
//typedef CV_32FC1 CMatType;
const int  CMatType=CV_64FC1;



using namespace cv;
using namespace std;
const int BOXCACHESIZE =90000; //100000;
const int COM_NUM=13;
//const int PARTS_NUM=39;
const int MAX_PARTS_NUM=68;
const int RESP_NUM=28;
const int CNOV_DATA_Length=146;
//特征 feat
//const int PYRA_FEAT_LEN=28;
const int FEAT_Z_DIM=32;//源代码中的介绍
//滤波器 filters 
const int FILTERS_NUM=146;
const int FILTERS_Z_DIM=32;
//Defs的size定义
const int DEFS_NUM=710;



const int Length_Model_Components[COM_NUM]=//model com中数据的长度
	{39,39,39,//3ge39
	68,68,68,68,68,68,68,//7ge68
	39,39,39//3个39
	};


struct BoxSSort
{
	int index;
	double s;
	bool operator< (const BoxSSort &A)const
	{
		return s<A.s;
	}
};
struct Boxes
{
	
	double s,c;
	double xy;
	Mat xyMat;
	double level;
	
	Boxes()
	{}
	Boxes(const Boxes &A)
	{
		this->s=A.s;
		this->c=A.c;
		this->xy=A.xy;
		this->xyMat=A.xyMat;
		this->level=A.level;
	}
	bool operator< (const Boxes &A)const
	{
		return s<A.s;
	}
};
struct Parts_com_data
{
			MatType defid;
            MatType filterid ;
            MatType parent;
            MatType sizy;
            MatType sizx;
            MatType filterI;
            MatType defI;
            MatType w[4];
            MatType scale;
            MatType starty;
            MatType startx;
            MatType step;
            MatType level;
            MatType score;
            MatType Ix;
            MatType Iy;

			Mat IxMat;
            Mat IyMat;
			Mat socreMat;
			/*int defid;
            int filterid ;
            int parent;
            int sizy;
            int sizx;
            double filterI;
            double defI;
            double w;
            double scale;
            int starty;
            int startx;
            double step;
            double level;
            double score;
            int Ix;
            int Iy;*/
};
struct Parts
{
	int len;//39 or 68
	Parts_com_data  parts_com_data[MAX_PARTS_NUM];
};
struct Components
{
	Parts components_data[COM_NUM];
};
//卷积部分结果
struct Resp
{
	bool emptyOrNot;//1 true mean is empty,0 mean not
	Mat conv_data[CNOV_DATA_Length];
};
struct Feat
{
	Mat feat_data[FEAT_Z_DIM];
};
/*struct Pyra
{
	double padx;
	double pady;
	double scale[PYRA_FEAT_LEN];
	Feat feat[PYRA_FEAT_LEN];
};*/

class MyPyra
{
public:
	double padx;
	double pady;
	int len;//feat and scale  length
	int imx;
	int imy;

	int interval;
	double *scale;//[PYRA_FEAT_LEN]
	Feat *feat;//[PYRA_FEAT_LEN]
	Feat myfeat[100];
	int newMyPyra(int len)
	{
		this->len=len;
		if(len>100)
			return -1;
		scale=(double *)malloc(len*sizeof(double));
		feat=&(myfeat[0]);
		
		return 0;
	}
	~MyPyra()
	{
		free(scale);
		free(feat);
		scale=NULL;
		feat=NULL;
	}

};

struct Filters
{
	int i;
	Mat filter_data[FILTERS_Z_DIM];
};
//用于定义matlab加载的数据
struct Model_Component_Data
{
	int defid;
	int filterid;
	int parent;

};
struct Model_Component
{
	int len;
	Model_Component_Data  model_component_data[MAX_PARTS_NUM];

};
struct Defs
{
	int i;
	int anchor[3];
	double w[4];
};
struct XyPoint
{
	int xy[MAX_PARTS_NUM][2];
};
class Model
{
private:
	Filters filters[FILTERS_NUM];
	//Pyra pyra;
	


	MyPyra mypyra;
	Filters myfilters[FILTERS_NUM];
	Resp resp[RESP_NUM];
	Model_Component model_component[COM_NUM];
	Defs defs[DEFS_NUM];

	int  interval;
	int sbin;
	int maxsize[2];
	double thresh;
	Mat cmat;
public:
	void init();
	int detect(Mat input,char *errorString);
	void paramsLoad();
	int load(string datapath, string filename, string matname,Mat &rs,char *errorString);
	//对于某个目录下的mat文件全部加载它的信息
	int loadComponents(string datapath,char *errorString);
	//加载特征矩阵
	int loadFeat(string datapath,char *errorString);
	int loadFilters(string datapath,string ipath,char *errorString);
	int loadModel_components(string model_componentsData_path,char *errorString);
	int loadDefs(string defsDataPath,char *errorString);

	int fconv(Feat &feat,Filters*filters,int src_start,int src_end,Resp &re,char *errorString);
	void process(void *thread_arg);
	int shiftdt(Mat &child_socre,double w1,double w2,double w3,double w4,double child_startx,double child_starty,double Nx,double Ny,double child_step,Mat &msg,Mat &IxMat,Mat &IyMat,char *errorString);
	int backtrack(int *X,int *Y,int sizeXY,Parts &parts,MyPyra &mypyra,Mat &XY,char *errorString);
	int  nms_face(Boxes *boxes,const int length_boxe,double overlap,Boxes * top,int &topLength,char *errorString);
	void drawBoxes(Mat input,Boxes * top,const int topLength );
	int	featpyramid(Mat input,char *errorString);
	int  modelcomponents(MyPyra &mypyra, Components &mycomponents,char *errorString);

	int xySize(char *errorString);
	

	~Model()
	{
		
	}
};
//convolution
struct thread_data {
  Mat (*A)[FEAT_Z_DIM];
  Mat (*B)[FILTERS_Z_DIM];
  
  Mat mxC;
  int A_dims[3];
  int B_dims[3];
  int C_dims[2];
};
double  inline max(double a,double b)
{
	return a>b? a:b;
}
double  inline min(double a,double b)
{
	return a>b? b:a;
}
int  inline min2(int a,int b)
{
	return a>b? b:a;
}
#endif
