#include"faceDetector.h"
Boxes boxes[BOXCACHESIZE];//存放中间结果
Boxes box3001[30001];//存放最终结果
BoxSSort boxssort[30001];

Components components;
Components mycomponents;


int Model::xySize(char *errorString)
{

	return 0;
}
void Model::init()
{
	for(int i=0;i<RESP_NUM;i++)
		resp[i].emptyOrNot=true;//
}
int Model::load(string datapath, string filename, string matname,Mat &rs,char *errorString)
{
	MATFile *pmatFile = matOpen((datapath + "\\" + filename + ".mat").c_str(),"r");
	if (pmatFile == NULL)
	{
		sprintf(errorString,"MatOpen error %s!!!\n",filename.c_str());
		return -1;
	}



	mxArray *pMxArray = matGetVariable(pmatFile, matname.c_str()); //从文件中获取数据到mxArray结构中
	//if(mxGetClassID(pMxArray) != mxDOUBLE_CLASS)
	//cout<<"no ,it's double"<<endl;
	if (pMxArray == NULL)
	{
		sprintf(errorString,"Error reading existing matrix %s !!!\n",matname);
		return -1;
	}
	MatType *ReadArray = (MatType*) mxGetData(pMxArray);  
	int rows = mxGetM(pMxArray);//行列存储方式不一致，需注意
	int cols= mxGetN(pMxArray);  
	Mat ReadMat(rows,cols,CMatType);  //此处主要是自己工程其他模块需使用float型的
	for (int i=0; i<rows; i++)  
	{
		for (int j=0; j<cols; j++)
		{
			ReadMat.at<MatType>(i,j) = (MatType)ReadArray[i*cols+j];
			//cout<<(float)ReadArray[i*cols+j]<<"   ";
		}
		//cout<<endl;
	}

	mxDestroyArray(pMxArray);
	if (matClose(pmatFile) != 0)
	{
		sprintf(errorString,"Error closing file %s\n",pmatFile);
		return -1;
	}
	//cout<<"Read done!!!"<<endl;
	rs=ReadMat;
	return 0;
}
int Model::loadComponents(string datapath,char *errorString)
{
	string dataPathCom=datapath;
	char fileName[250];
	for(int i=1;i<=COM_NUM;i++)
	{
		int cnt=0;
		for(int j=1;j<=MAX_PARTS_NUM;j++)
		{
			sprintf(fileName,"components_row_%d_col_%d",i,j);
			MATFile *pmatFile = matOpen((dataPathCom + "\\" + fileName + ".mat").c_str(),"r");
			if (pmatFile == NULL)
			{
				if(j>39)
				{
					continue;
				}
				else
				{
					sprintf(errorString,"MatOpen error for file %s!!!\n",fileName);
					return -1;
				}
			}
			cnt++;
			string partParams[]={
				"defid",
				"filterid",
				"parent",
				"sizy",
				"sizx",
				"filterI",
				"defI",
				"w",//7
				"scale",
				"starty",
				"startx",
				"step",
				"level",
				"Ix",
				"Iy",
				"score"
			};
			MatType arrayParams[16]={0};//用于临时存放已知的变量
			for(int k=0;k<16;k++)
			{
				mxArray *pMxArray = matGetVariable(pmatFile, partParams[k].c_str()); //从文件中获取数据到mxArray结构中
				//if(mxGetClassID(pMxArray)== mxDOUBLE_CLASS)
				//	cout<<"yes ,it's double"<<endl;
				if (pMxArray == NULL)
				{
					sprintf(errorString,"Error reading existing matrix %s !!!\n",fileName);
					return -1;
				}

				MatType *ReadArray = (MatType*) mxGetData(pMxArray); 
				if(k==7)//读到w时，由于w是一个矩阵
				{
					//cout<<fileName<<endl;
					int rows = mxGetM(pMxArray);//行列存储方式不一致，需注意
					int cols= mxGetN(pMxArray);  
					//cout<<"rows:"<<rows<<endl;
					//cout<<"cols:"<<cols<<endl;
					for(int a=0;a<4;a++)
					{
						components.components_data[i-1].parts_com_data[j-1].w[a]=0;
					}
					for(int a=0;a<cols;a++)
					{
						components.components_data[i-1].parts_com_data[j-1].w[a]=(MatType)ReadArray[a];
						//cout<<"w["<<a<<"]="<<components.components_data[i-1].parts_com_data[j-1].w[a]<<"  ";
						//getchar();
					}
				}
				else
				{
					arrayParams[k]=(MatType)ReadArray[0];
				}
				mxDestroyArray(pMxArray);
			}
			int ii=i-1;
			int jj=j-1;
			components.components_data[ii].parts_com_data[jj].defid=arrayParams[0];
			components.components_data[ii].parts_com_data[jj].filterid=arrayParams[1];
			components.components_data[ii].parts_com_data[jj].parent=arrayParams[2];
			components.components_data[ii].parts_com_data[jj].sizy=arrayParams[3];
			components.components_data[ii].parts_com_data[jj].sizx=arrayParams[4];
			components.components_data[ii].parts_com_data[jj].filterI=arrayParams[5];
			components.components_data[ii].parts_com_data[jj].defI=arrayParams[6];
			//components.components_data[ii].parts_com_data[jj].w=arrayParams[7];
			components.components_data[ii].parts_com_data[jj].scale=arrayParams[8];
			components.components_data[ii].parts_com_data[jj].starty=arrayParams[9];
			components.components_data[ii].parts_com_data[jj].startx=arrayParams[10];
			components.components_data[ii].parts_com_data[jj].step=arrayParams[11];
			components.components_data[ii].parts_com_data[jj].level=arrayParams[12];
			components.components_data[ii].parts_com_data[jj].Ix=arrayParams[13];
			components.components_data[ii].parts_com_data[jj].Iy=arrayParams[14];
			components.components_data[ii].parts_com_data[jj].score=arrayParams[15];

			if (matClose(pmatFile) != 0)
			{
				sprintf(errorString,"Error closing file %s\n",pmatFile);
				return -1;
			}

		}
		components.components_data[i-1].len=cnt;
		//printf("len of parts %d:%d",i,components.components_data[i-1].len);
	}
	return 0;
}
int Model::loadFeat(string datapath,char *errorString)
{
	char fileName[250];
	string matname="feature";
	for(int i=0;i<mypyra.len;i++)
	{
		for(int j=0;j<FEAT_Z_DIM;j++)
		{
			sprintf(fileName,"pyra_feat_row_%d_z_%d",i+1,j+1);
			int rs=load(datapath, string(fileName),matname ,mypyra.feat[i].feat_data[j],errorString);
			if(rs!=0)
				return rs;
		}
	}

	return 0;
}
int Model::loadFilters(string datapath,string ipath,char *errorString)
{
	char fileName[250];
	string matname="filters_part_data";
	for(int i=0;i<FILTERS_NUM;i++)
	{
		for(int j=0;j<FILTERS_Z_DIM;j++)
		{
			sprintf(fileName,"filters_row_%d_z_%d",i+1,j+1);
			int rs=load(datapath, string(fileName),matname ,filters[i].filter_data[j],errorString);
			if(rs!=0)
				return rs;
		}
	}

	string iMatName="i";
	
	for(int i=0;i<FILTERS_NUM;i++)
	{
		Mat imat;
		sprintf(fileName,"model_filters_%d_w_z_1",i+1);
		int rs=load(ipath, string(fileName),iMatName ,imat,errorString);
			if(rs!=0)
				return rs;
		filters[i].i=int(imat.at<MatType>(0,0)+0.5);
	}

	return 0;
}
int Model::loadModel_components(string model_componentsData_path,char *errorString)
{
	char fileName[250];
	string defidname="defid";
	string filteridname="filterid";
	string parentname="parent";

	for(int i=1;i<=COM_NUM;i++)
	{
		model_component[i-1].len=Length_Model_Components[i-1];
		for(int j=1;j<=Length_Model_Components[i-1];j++)
		{
			sprintf(fileName,"model_components_%d_%d",i,j);
			
			Mat defidMat;
			int  rs=load(model_componentsData_path,string(fileName),defidname,defidMat,errorString);
			if(rs!=0)
				return rs;
			model_component[i-1].model_component_data[j-1].defid=defidMat.at<MatType>(0,0);


			Mat filteridMat;
			rs=load(model_componentsData_path,string(fileName),filteridname,filteridMat,errorString);
			if(rs!=0)
				return rs;
			model_component[i-1].model_component_data[j-1].filterid=filteridMat.at<double>(0,0);


			Mat parentMat;
			rs=load(model_componentsData_path,string(fileName),parentname,parentMat,errorString);
			if(rs!=0)
				return rs;
			model_component[i-1].model_component_data[j-1].parent=parentMat.at<MatType>(0,0);

		}
	}
	return 0;
}
int Model::loadDefs(string defsDataPath,char *errorString)
{
	
	char fileName[250];
	string i_matname="i";
	string anchor_name="anchor";
	string w_name="w";


	for(int i=0;i<DEFS_NUM;i++)
	{
		
			sprintf(fileName,"model_defs_%d",i+1);

			
				Mat imat;
			int rs=load(defsDataPath, string(fileName),i_matname ,imat,errorString);
			if(rs!=0)
				return rs;
			defs[i].i=int(imat.at<double>(0,0)+0.5);
			


			Mat anchormat;
			rs=load(defsDataPath, string(fileName),anchor_name ,anchormat,errorString);
			if(rs!=0)
				return rs;
			defs[i].anchor[0]=int(anchormat.at<double>(0,0)+0.5);
			defs[i].anchor[1]=int(anchormat.at<double>(0,1)+0.5);
			defs[i].anchor[2]=int(anchormat.at<double>(0,2)+0.5);

			
			Mat wmat;
			rs=load(defsDataPath, string(fileName),w_name ,wmat,errorString);

			if(rs!=0)
				return rs;
			if(wmat.cols>1)
			{
				defs[i].w[0]=wmat.at<double>(0,0);
				defs[i].w[1]=wmat.at<double>(0,1);
				defs[i].w[2]=wmat.at<double>(0,2);
				defs[i].w[3]=wmat.at<double>(0,3);
			}
			else
			{
				defs[i].w[0]=wmat.at<double>(0,0);
				defs[i].w[1]=0;
				defs[i].w[2]=0;
				defs[i].w[3]=0;
			}


		
	}

	return 0;

}
void Model::paramsLoad()
{
	//interval sbin maxsize
	char errorString[250];
	string datapathinterval="D:\\Desktop\\face-release1.0-basic\\LoadData\\model";
	string filenameinterval="model_params";
	string matnameinterval="interval";
	Mat intervalMat;
	int rs=load(datapathinterval,filenameinterval,matnameinterval,intervalMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	interval=int(intervalMat.at<MatType>(0,0)+0.5);
	//sbin
	string matnamesbin="sbin";
	Mat sbinMat;
	rs=load(datapathinterval,filenameinterval,matnamesbin,sbinMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	this->sbin=int(sbinMat.at<MatType>(0,0)+0.5);
	//maxsize
	string matnamemaxsize="maxsize";
	Mat maxsizeMat;
	rs=load(datapathinterval,filenameinterval,matnamemaxsize,maxsizeMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	this->maxsize[0]=int(maxsizeMat.at<MatType>(0,0)+0.5);
	this->maxsize[1]=int(maxsizeMat.at<MatType>(0,1)+0.5);


	//interval sbin maxsize
	//thresh
	string datapathThresh="D:\\Desktop\\face-release1.0-basic\\LoadData\\model";
	string filenameThresh="model_params";
	string matnameThresh="thresh";
	Mat threshMat;
	rs=load(datapathThresh,filenameThresh,matnameThresh,threshMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	thresh=threshMat.at<double>(0,0);
	//c params

	string datapathc="D:\\Desktop\\face-release1.0-basic\\LoadData";
	string filenamec="c";
	string matnamec="c";
	rs=load(datapathc,filenamec,matnamec,cmat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	//components
	string ComDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\components";
	rs=loadComponents(ComDataPath,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	//feat
	string FeatDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\pyra\\feat";
	rs=loadFeat(FeatDataPath,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	//load scale part
	string ScaleDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\pyra";
	string scalefileName="pyra_params";
	string matname="scale";
	Mat scaleMat;
	rs=load(ScaleDataPath, scalefileName,matname ,scaleMat,errorString);

	if(rs!=0)
	{
		cout<<errorString;
		return ;
	}
	for(int i=0;i<mypyra.len;i++)
		mypyra.scale[i]=scaleMat.at<MatType>(i,0);
	//load pyra padx and pady
	string padxDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\pyra";
	string padxfileName="pyra_params";
	string padxmatname="padx";
	Mat padxMat;
	rs=load(padxDataPath,padxfileName,padxmatname,padxMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	mypyra.padx=padxMat.at<MatType>(0,0);
	string padymatname="pady";
	rs=load(padxDataPath,padxfileName,padymatname,padxMat,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	mypyra.pady=padxMat.at<MatType>(0,0);


	//filters

	string FiltersDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\filters";

	string ipath="D:\\Desktop\\face-release1.0-basic\\LoadData\\model\\filters";
	rs=loadFilters(FiltersDataPath,ipath,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	//loading model_component
	//model_component[COM_NUM];
	string model_componentsData_path="D:\\Desktop\\face-release1.0-basic\\LoadData\\model\\components";
	rs=loadModel_components(model_componentsData_path,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}
	//load defs
	string defsDataPath="D:\\Desktop\\face-release1.0-basic\\LoadData\\model\\defs";
	rs= loadDefs( defsDataPath,errorString);
	if(rs!=0)
	{
		cout<<errorString;
		return;
	}

	return ;
}
int Model::fconv(Feat &feat,Filters* filters,int src_start,int src_end,Resp &re,char *errorString)
{

	int start=src_start-1;
	int end=src_end-1;
	if(start<0||end<start)
	{
		sprintf(errorString,"fconv start ,end error,start:%d,end:%d!!\n",start,end);
		return -2;
	}
	//do convolution
	int len=end-start+1;
	thread_data td;
	int teA_dims[3];

	teA_dims[0]=feat.feat_data[0].size().height;
	teA_dims[1]=feat.feat_data[0].size().width;
	teA_dims[2]=FEAT_Z_DIM;

	//td.A=&(feat.feat_data);
	for(int i=0;i<len;i++)
	{
		td.B=&((filters[i+start]).filter_data);
		td.A_dims[0]=teA_dims[0];
		td.A_dims[1]=teA_dims[1];
		td.A_dims[2]=teA_dims[2];
		td.A=&(feat.feat_data);

		td.B_dims[0]=td.B[0]->rows;
		td.B_dims[1]=td.B[0]->cols;
		td.B_dims[2]=FILTERS_Z_DIM;

		int height=td.A_dims[0]-td.B_dims[0]+1;
		int width=td.A_dims[1]-td.B_dims[1]+1;
		if (height < 1 || width < 1)
		{
			sprintf(errorString,"A should bigger than B,width:%d,height:%d!!\n",width,height);
			return -2;
		}
		td.C_dims[0] = height;
		td.C_dims[1] = width;

		Mat tempMxC(height,width,CMatType,Scalar(0,0,0));
		td.mxC=tempMxC;



		process((void *)&td);
		re.conv_data[i]=td.mxC;
		//cout<<endl<<resp[level-1].conv_data[f-1]<<endl;
		/*if(i==5)
		{
		cout<<"resp[level-1].conv_data[5]"<<endl;
		cout<<re.conv_data[i].col(0)<<endl;
		}*/


		//printf("printf td.mxC.ptr %lf\n",td.mxC.at<Vec2d>(0,0)[0]);

	}
	cout<<"end conv"<<endl;
	return 0;

}
void Model::process(void *thread_arg)
{
	thread_data *args = (thread_data *)thread_arg;
	Mat *A = *(args->A);
	Mat *B= *(args->B);
	Mat &mxC= args->mxC;
	/*for(int i=0;i<mxC.rows;i++)
	{
	double *data=mxC.ptr<double>(i);
	for(int j=0;j<mxC.cols;j++)
	{
	data[j]=0.0;
	}
	}*/
	//cout<<"一般的初始化mxC"<<mxC<<endl;

	const int *A_dims=args->A_dims;
	//cout<<"A_dims"<<A_dims[0]<<"  "<<A_dims[1]<<"  "<<A_dims[2]<<endl;
	const int *B_dims = args->B_dims;
	//cout<<"B_dims"<<B_dims[0]<<"  "<<B_dims[1]<<"  "<<B_dims[2]<<endl;

	const int *C_dims = args->C_dims;

	//cout<<"C_dims"<<C_dims[0]<<"  "<<C_dims[1]<<endl;


	int num_features = args->A_dims[2];
	for (int f = 0; f < num_features; f++) 
	{
		MatType *dst = mxC.ptr<MatType>(0,0);
		//double *A_src = A + f*A_dims[0]*A_dims[1];  
		MatType *A_src =A[f].ptr<MatType>(0,0);
		//double *B_src = B + f*B_dims[0]*B_dims[1];
		MatType  *B_src=B[f].ptr<MatType>(0,0);

		for (int y = 0; y < C_dims[0]; y++) 
		{
			for (int x = 0; x < C_dims[1]; x++) 
			{
				MatType val = 0;
				for (int xp = 0; xp < B_dims[1]; xp++) 
				{
					MatType *A_off = A_src + (x+xp)*A_dims[0] + y;
					MatType *B_off = B_src + xp*B_dims[0];

					switch(B_dims[0]) 
					{
					case 20: val += A_off[19] * B_off[19];
					case 19: val += A_off[18] * B_off[18];
					case 18: val += A_off[17] * B_off[17];
					case 17: val += A_off[16] * B_off[16];
					case 16: val += A_off[15] * B_off[15];
					case 15: val += A_off[14] * B_off[14];
					case 14: val += A_off[13] * B_off[13];
					case 13: val += A_off[12] * B_off[12];
					case 12: val += A_off[11] * B_off[11];
					case 11: val += A_off[10] * B_off[10];
					case 10: val += A_off[9] * B_off[9];
					case 9: val += A_off[8] * B_off[8];
					case 8: val += A_off[7] * B_off[7];
					case 7: val += A_off[6] * B_off[6];
					case 6: val += A_off[5] * B_off[5];
					case 5: val += A_off[4] * B_off[4];
					case 4: val += A_off[3] * B_off[3];
					case 3: val += A_off[2] * B_off[2];
					case 2: val += A_off[1] * B_off[1];
					case 1: val += A_off[0] * B_off[0];
						break;
					default:	    	      
						for (int yp = 0; yp < B_dims[0]; yp++) 
						{
							val += *(A_off++) * *(B_off++);
						}
					}


				}
				*(dst++) += val;
			}
		}

		/*for (int x = 0; x < C_dims[1]; x++) 
		{
		for (int y = 0; y < C_dims[0]; y++) 
		{
		MatType val = 0;
		for (int xp = 0; xp < B_dims[1]; xp++) 
		{
		MatType *A_off = A_src + (x+xp)*A_dims[0] + y;
		MatType *B_off = B_src + xp*B_dims[0];

		switch(B_dims[0]) 
		{
		case 20: val += A_off[19] * B_off[19];
		case 19: val += A_off[18] * B_off[18];
		case 18: val += A_off[17] * B_off[17];
		case 17: val += A_off[16] * B_off[16];
		case 16: val += A_off[15] * B_off[15];
		case 15: val += A_off[14] * B_off[14];
		case 14: val += A_off[13] * B_off[13];
		case 13: val += A_off[12] * B_off[12];
		case 12: val += A_off[11] * B_off[11];
		case 11: val += A_off[10] * B_off[10];
		case 10: val += A_off[9] * B_off[9];
		case 9: val += A_off[8] * B_off[8];
		case 8: val += A_off[7] * B_off[7];
		case 7: val += A_off[6] * B_off[6];
		case 6: val += A_off[5] * B_off[5];
		case 5: val += A_off[4] * B_off[4];
		case 4: val += A_off[3] * B_off[3];
		case 3: val += A_off[2] * B_off[2];
		case 2: val += A_off[1] * B_off[1];
		case 1: val += A_off[0] * B_off[0];
		break;
		default:	    	      
		for (int yp = 0; yp < B_dims[0]; yp++) 
		{
		val += *(A_off++) * *(B_off++);
		}
		}
		}
		*(dst++) += val;





		}
		}*/
	}
	//cout<<"mxc"<<endl;
	//	cout<<*(mxC.ptr<double>(0,0))<<endl;
	return ;
}
static  int square(int x) { return x*x; }

// dt1d(source,destination_val,destination_ptr,source_step,source_length,
//      a,b,dest_shift,dest_length,dest_step)
static void dt1d(double *src, double *dst, int *ptr, int step, int len, double a, double b, int dshift, int dlen, double dstep) {
	int   *v = new int[len];
	float *z = new float[len+1];
	int k = 0;
	int q = 0;
	v[0] = 0;
	z[0] = -INF;
	z[1] = +INF;

	for (q = 1; q <= len-1; q++) {
		float s = ((src[q*step] - src[v[k]*step]) - b*(q - v[k]) + a*(square(q) - square(v[k]))) / (2*a*(q-v[k]));
		while (s <= z[k]) {
			k--;
			s  = ((src[q*step] - src[v[k]*step]) - b*(q - v[k]) + a*(square(q) - square(v[k]))) / (2*a*(q-v[k]));
		}
		k++;
		v[k]   = q;
		z[k]   = s;
		z[k+1] = +INF;
	}

	k = 0;
	q = dshift;

	for (int i=0; i <= dlen-1; i++) {
		while (z[k+1] < q)
			k++;
		dst[i*step] = a*square(q-v[k]) + b*(q-v[k]) + src[v[k]*step];
		ptr[i*step] = v[k];
		q += dstep;
	}

	delete [] v;
	delete [] z;
}
int  Model::nms_face(Boxes *boxes,const int length_boxe,double overlap,Boxes * top,int &topLength,char *errorString)
{
	int N=length_boxe;
	if(N<=0)
	{
		top=NULL;
		sprintf(errorString,"no faces\n");
		return -11;
	}
	//开始咯
	const int numpart = boxes[0].xyMat.rows;

	if(N>30000)
	{
		sort(boxes,boxes+N);
		/* Boxes b1=boxes[0];
		Boxes b2=boxes[1];
		Boxes b3=boxes[2];
		Boxes bn=boxes[N-1];bn s 最大*/
		const int offset=length_boxe-30000;
		/*for(int k=0;k<=29999;k++)
		{
		boxes[k]=boxes[k+]
		}*/
		boxes=boxes+offset;//重点测试??是否会数组越界
		N=30000;
		cout<<"测试代码 line 562"<<boxes[N-1].level<<endl;;
	}
	double *x1=(double *)malloc(sizeof(double)*N);
	double *y1=(double *)malloc(sizeof(double)*N);
	double *x2=(double *)malloc(sizeof(double)*N);
	double *y2=(double *)malloc(sizeof(double)*N);
	double *area=(double *)malloc(sizeof(double)*N);
	memset(x1,0,sizeof(double)*N);
	memset(y1,0,sizeof(double)*N);
	memset(x2,0,sizeof(double)*N);
	memset(y2,0,sizeof(double)*N);
	for( int nb = 1;nb<=N;nb++)
	{
		if(numpart==1)
		{
			/*
			x1(nb) = boxes(nb).xy(1);
			y1(nb) = boxes(nb).xy(2);
			x2(nb) = boxes(nb).xy(3);
			y2(nb) = boxes(nb).xy(4);
			*/
			x1[nb-1] = boxes[nb-1].xyMat.at<MatType>(0,0);
			y1[nb-1] = boxes[nb-1].xyMat.at<MatType>(0,1);
			x2[nb-1]  = boxes[nb-1].xyMat.at<MatType>(0,2);
			y2[nb-1] =  boxes[nb-1].xyMat.at<MatType>(0,3);
		}
		else
		{
			double x1min=boxes[nb-1].xyMat.at<double>(0,0);
			double y1min=boxes[nb-1].xyMat.at<double>(0,1);
			double x2max=boxes[nb-1].xyMat.at<double>(0,2);
			double y2max=boxes[nb-1].xyMat.at<double>(0,3);


			for(int i=1;i<boxes[nb-1].xyMat.rows;i++)
			{
				if(boxes[nb-1].xyMat.at<double>(i,0)<x1min)
					x1min=boxes[nb-1].xyMat.at<double>(i,0);
				if(boxes[nb-1].xyMat.at<double>(i,1)<y1min)
					y1min=boxes[nb-1].xyMat.at<double>(i,1);
				if(boxes[nb-1].xyMat.at<double>(i,2)>x2max)
					x2max=boxes[nb-1].xyMat.at<double>(i,2);
				if(boxes[nb-1].xyMat.at<double>(i,3)>y2max)
					y2max=boxes[nb-1].xyMat.at<double>(i,3);
			}


			x1[nb-1] =x1min;
			y1[nb-1] =y1min;
			x2[nb-1] =x2max;
			y2[nb-1] =y2max;

		}
		//area(nb) = (x2(nb)-x1(nb)+1) * (y2(nb)-y1(nb)+1);
		area[nb-1] = (x2[nb-1]-x1[nb-1]+1) * (y2[nb-1]-y1[nb-1]+1);
	}

	for(int i=0;i<N;i++)
	{
		boxssort[i].index=i;
		boxssort[i].s=boxes[i].s;

	}
	sort(boxssort,boxssort+N);

	/*BoxSSort s1=boxssort[0];
	BoxSSort s2=boxssort[1];
	BoxSSort s3=boxssort[2];
	BoxSSort sn=boxssort[N-1];*/
	vector<int> I;
	I.clear();
	for(int i=0;i<N;i++)
	{
		I.push_back(boxssort[i].index);
	}
	vector<int> pick;
	pick.clear();

	vector<int> supress;

	double *xx1=(double*)malloc(sizeof(double)*(I.size()));
	double *yy1=(double*)malloc(sizeof(double)*(I.size()));
	double *xx2=(double*)malloc(sizeof(double)*(I.size()));
	double *yy2=(double*)malloc(sizeof(double)*(I.size()));
	double *inter=(double*)malloc(sizeof(double)*(I.size()));


	while(I.size()>0)
	{

		int last=I.size()-1;
		int i=I[last];
		pick.push_back(i);

		supress.clear();
		supress.push_back(last);
		//取消j
		const int sizej=I.size()-1;

		for(int k=0;k<sizej;k++)
		{
			xx1[k]=max(x1[i],x1[k]);
			yy1[k]=max(y1[i],y1[k]);
			xx2[k]=max(x2[i],x2[k]);
			yy2[k]=max(y2[i],y2[k]);
		}

		for(int k=0;k<sizej;k++)
		{
			double w= xx2[k]-xx1[k]+1;
			if(w<0.0)
				w=0.0;

			double h= yy2[k]-yy1[k]+1;
			if(h<0.0)
				h=0.0;
			inter[k]=w*h;//计算面积
		}

		for(int k=sizej-1;k>=0;k--)
		{
			double o1=inter[k]/area[k];
			double o2=inter[k]/area[i];
			if(o1>overlap||o2>overlap)
			{
				supress.push_back(k);
			}
		}
		//清除操作
		for(int k=0;k<supress.size();k++)
		{
			I.erase(I.begin()+supress[k]);
		}

		supress.clear();



	}
	topLength=pick.size();
	for(int j=0;j<topLength;j++)
	{
		box3001[j]=boxes[pick[j]];
	}

	free(xx1);
	free(yy1);
	free(xx2);
	free(yy2);
	free(inter);


	free(x1);
	free(y1);
	free(x2);
	free(y2);
	free(area);
	return 0;
}
int Model::shiftdt(Mat &child_socre,double w1,double w2,double w3,double w4,double child_startx,double child_starty,double Nx,double Ny,double child_step,Mat &msg,Mat &IxMat,Mat &IyMat,char *errorString)
{
	MatType *vals=child_socre.ptr<MatType>(0,0);
	int sizx  =child_socre.cols;
	int sizy  =child_socre.rows;
	double ax = -w1;
	double bx = -w2;
	double ay = -w3;
	double by = -w4;

	int offx  = (int)(child_startx-1);
	int offy  = (int)(child_starty-1);

	int lenx  = (int)(Nx+0.5);
	int leny  = (int)(Ny+0.5);
	double step =child_step;

	Mat mxM=Mat(leny,lenx,CV_64FC1,Scalar(0,0,0));
	Mat mxIy=Mat(leny,lenx,CV_32SC1,Scalar(0,0,0));
	Mat mxIx=Mat(leny,lenx,CV_32SC1,Scalar(0,0,0));

	double   *M = mxM.ptr<double>(0,0);
	int *Iyp= mxIy.ptr<int>(0,0);
	int *Ixp= mxIx.ptr<int>(0,0);
	double   *tmpM =  (double *)malloc(leny*sizx*sizeof(double));
	int *tmpIx = (int *)malloc(leny*sizx*sizeof(int));

	/*for (int x = 0; x < sizx; x++)
	{
	dt1d(vals+x*sizy, tmpM+x*leny, tmpIy+x*leny, 1, sizy, ay, by, offy, leny, step);

	}
	for (int y = 0; y < leny; y++)
	{
	dt1d(tmpM+y, M+y, Ixp+y, leny, sizx, ax, bx, offx, lenx, step);
	}
	*/
	/*for (int x = 0; x < lenx; x++) 
	{
	for (int y = 0; y < leny; y++) 
	{
	int p = x*leny+y;
	Iyp[p] = tmpIy[Ixp[p]*leny+y]+1;
	Ixp[p] = Ixp[p]+1;
	}
	}*/

	for (int y = 0; y < sizy; y++)
	{
		dt1d(vals+y*sizx, tmpM+y*lenx, tmpIx+y*lenx, 1, sizx, ax, bx, offx, lenx, step);
	}

	for (int x = 0; x < lenx; x++)
	{
		dt1d(tmpM+x, M+x, Iyp+x, lenx, sizy, ay, by, offy, leny, step);
	}
	// printf("end dt1d y\n");
	//getchar();
	//cout<<"msg cols1"<<mxM.col(1)<<endl;
	//getchar();
	for (int y = 0; y < leny; y++) 
	{
		for (int x = 0; x < lenx; x++) 
		{
			int p = y*lenx+x;
			Ixp[p] = tmpIx[Iyp[p]*lenx+x]+1;
			Iyp[p] = Iyp[p]+1;

		}
	}

	free(tmpM);
	free(tmpIx);
	msg=mxM;
	IxMat=mxIx;
	IyMat=mxIy;
	return 0;
}
int Model::backtrack(int *X,int *Y,int sizeXY,Parts &parts,MyPyra &mypyra,Mat &XY,char *errorString)
{
	int numparts = parts.len;
	int sizeMatPtr[3];
	sizeMatPtr[0]=numparts;
	sizeMatPtr[1]=2;
	sizeMatPtr[2]=sizeXY;

	Mat ptr(3,sizeMatPtr,CV_64FC1,Scalar(0,0,0));

	int sizeMatBox[3];
	sizeMatBox[0]=numparts;
	sizeMatBox[1]=4;
	sizeMatBox[2]=sizeXY;
	Mat box(3,sizeMatBox,CV_64FC1,Scalar(0,0,0));
	XY=box;
	int k   = 0;
	Parts_com_data p = parts.parts_com_data[k];
	for(int i=0;i<sizeXY;i++)
	{
		ptr.at<double>(k,0,i) = X[i];
		ptr.at<double>(k,1,i) = Y[i];
	}
	double scale = mypyra.scale[int(p.level+0.5)-1];;
	double padx  = mypyra.padx;
	double pady  = mypyra.pady;
	for(int i=0;i<sizeXY;i++)
	{
		//box.at<double>(k,0,i) = (X[i]-1-padx)*scale + 1;
		//box.at<double>(k,1,i) = (Y[i]-1-pady)*scale + 1;
		box.at<double>(k,0,i) = (X[i]-padx)*scale + 1;
		box.at<double>(k,1,i) = (Y[i]-pady)*scale + 1;
		box.at<double>(k,2,i) = box.at<double>(k,0,i) + p.sizx*scale - 1;
		box.at<double>(k,3,i) = box.at<double>(k,1,i) + p.sizy*scale - 1;
	}
	/*cout<<"box col(0)"<<endl;
	for(int i=0;i<sizeXY;i++)
	{
	cout<<box.at<double>(k,2,i)<<endl;
	}
	cout<<"box col(1)"<<endl;
	for(int i=0;i<sizeXY;i++)
	{
	cout<<box.at<double>(k,3,i)<<endl;
	}*/

	for (k =1;k<numparts;k++)
	{
		Parts_com_data p = parts.parts_com_data[k];
		int par =int( p.parent+0.5);
		for(int i=0;i<sizeXY;i++)
		{
			X[i]   = ptr.at<double>(par-1,0,i);
			Y[i]   = ptr.at<double>(par-1,1,i);
		}





		//int inds = (size(p.Ix), y, x);
		//cout<<"p.IxMat cols"<<endl;
		//cout<<p.IxMat.col(0)<<endl;
		for(int i=0;i<sizeXY;i++)
		{
			/*if(k==64&&i==76)
			{
				cout<<"p.IxMat.size("<<p.IxMat.rows<<","<<p.IxMat.cols<<")"<<endl;
			cout<<"xi  "<<X[i]<<endl; 
			cout<<"yi  "<<Y[i]<<endl; 

			}*/
			
			/**/if(Y[i]>=p.IxMat.rows)
			{
				cout<<"Y[i]>=p.IxMat.rows,please check now."<<endl;
				cout<<"Y[i]  "<<Y[i]<<"p.IxMat.rows  "<<p.IxMat.rows<<endl;
				Y[i]=p.IxMat.rows-1;
			}
			if(X[i]>=p.IxMat.cols)
			{
				X[i]=p.IxMat.cols-1;
				cout<<"X[i]"<<X[i]<<"p.IxMat.cols"<<p.IxMat.cols<<endl;
			}

			ptr.at<double>(k,0,i)=p.IxMat.at<int>(Y[i],X[i]);//bug p.IxMat
			ptr.at<double>(k,1,i)=p.IyMat.at<int>(Y[i],X[i]);
		}
		// image coordinates of part k



		scale = mypyra.scale[int(p.level+0.5)-1];
		for(int i=0;i<sizeXY;i++)
		{
			box.at<double>(k,0,i) = (ptr.at<double>(k,0,i)-1-padx)*scale + 1;
			box.at<double>(k,1,i) = (ptr.at<double>(k,1,i)-1-pady)*scale + 1;
			box.at<double>(k,2,i) = box.at<double>(k,0,i) + p.sizx*scale - 1;
			box.at<double>(k,3,i) = box.at<double>(k,1,i) + p.sizy*scale - 1;
		}
		/*cout<<"ptr XY col(0)"<<endl;
		for(int i=0;i<sizeXY;i++)
		{
		cout<<ptr.at<double>(k,0,i)<<endl;
		}

		cout<<"ptr XY col(1)"<<endl;
		for(int i=0;i<sizeXY;i++)
		{
		cout<<ptr.at<double>(k,1,i)<<endl;
		}*/
	}


	return 0;
}
void Model::drawBoxes(Mat input,Boxes * top,const int topLength )
{

	int posemap[13];//= 90:-15:-90;
	for(int k=0;k<13;k++)
		posemap[k]=90-15*k;
	for(int i=0;i<topLength;i++)
	{
		Boxes *b=top+i;
		double partsize = b->xyMat.at<double>(0,2)-b->xyMat.at<double>(0,0)+1;
		double min_b_xy_0=b->xyMat.at<double>(0,0);
		double max_b_xy_2=b->xyMat.at<double>(0,2);
		double min_b_xy_1=b->xyMat.at<double>(0,1);
		cout<<"b cols 1"<<endl;
		cout<<b->xyMat.col(0)<<endl;

		for(int k=1;k<b->xyMat.cols;k++)
		{
			if(min_b_xy_0>(b->xyMat.at<double>(k,0)))
				min_b_xy_0=b->xyMat.at<double>(k,0);
			if(max_b_xy_2<(b->xyMat.at<double>(k,2)))
				max_b_xy_2=b->xyMat.at<double>(k,2);
			if(min_b_xy_1>(b->xyMat.at<double>(k,1)))
				min_b_xy_1=b->xyMat.at<double>(k,1);

		}
		int tx=int((min_b_xy_0+max_b_xy_2)/2.0+0.5);
		int ty=int(min_b_xy_1-partsize/2.0+0.5);
		char tempstr[150];
		sprintf(tempstr,"position = %d\n", posemap[int(b->c+0.5)]);

		string box_text = string(tempstr);
		// putText(frame, box_text, Point(pos_x, pos_y), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);

		putText(input,box_text,Point(tx, ty), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0,255,0), 2.0);
		for(int k=((b->xyMat).rows)-1;k>=0;k--)
		{
			int x1 = int((b->xyMat).at<double>(k,0)+0.5);
			int y1 = int((b->xyMat).at<double>(k,1)+0.5);
			int x2 =int((b->xyMat).at<double>(k,2)+0.5);
			int y2 =int((b->xyMat).at<double>(k,3)+0.5);
			rectangle(input,Point(x1,y1),Point(	x2,y2),Scalar(255,0,0));
			circle(input,Point((x1+x2)/2,(y1+y2)/2),2,Scalar(0,0,255),-1,8);

		}

		// tx = (min(b.xy(:,1)) + max(b.xy(:,3)))/2;
		// ty = min(b.xy(:,2)) - partsize/2;
	}
	namedWindow("srcAndDetect");
	imshow("srcAndDetect",input);
	waitKey(0);
}
static int features(Mat image,int sbin,MyPyra &mypyra,int featMark,char *errorString)
{
	double uu[9] = {1.0000, 
		0.9397, 
		0.7660, 
		0.500, 
		0.1736, 
		-0.1736, 
		-0.5000, 
		-0.7660, 
		-0.9397};
double vv[9] = {0.0000, 
		0.3420, 
		0.6428, 
		0.8660, 
		0.9848, 
		0.9848, 
		0.8660, 
		0.6428, 
		0.3420};

	int dims[3];
	dims[0]=image.rows;
	dims[1]=image.cols;
	dims[2]=3;


	vector<Mat> channels;
	channels.clear();

	split(image,channels);
	if(channels.size()!=3)
	{
		sprintf(errorString,"input iamge is not 3 channnels\n");
		return -1;
	}
	double *im= (double *)malloc(dims[0]*dims[1]*3*sizeof(double));
	int cnt=0;
	for(int k=0;k<channels.size();k++)
	{
		for(int i=0;i<dims[0];i++)
		{
			for(int j=0;j<dims[1];j++)
			{
				im[cnt]=channels[k].at<uchar>(i,j);
				cnt++;
			}

		}
	}
	

	int blocks[2];
	blocks[0] = (int)((double)dims[0]/(double)sbin+0.5);//rows
	blocks[1] = (int)((double)dims[1]/(double)sbin+0.5);

	double *hist = (double *)malloc(blocks[0]*blocks[1]*18* sizeof(double));
	double *norm = (double *)malloc(blocks[0]*blocks[1]* sizeof(double));
	int out[3];
	out[0] = max(blocks[0]-2, 0);
	out[1] = max(blocks[1]-2, 0);
	out[2] = FEAT_Z_DIM;//27+4+1;

	Feat *mxfeat=(mypyra.feat+featMark);
	
	for(int j=0;j<FEAT_Z_DIM;j++)
	{
		//mxfeat->feat_data[j]=Mat(out[0],out[1],CV_64FC1,Scalar(0,0,0));
		mxfeat->feat_data[j].create(out[0],out[1],CV_64FC1);
		//Mat temp=Mat(out[0],out[1],CV_64FC1,Scalar(0,0,0));
		//mxfeat->feat_data[j]=temp;
	}

	int visible[2];
	visible[0] = blocks[0]*sbin;
	visible[1] = blocks[1]*sbin;

	for (int y = 1; y < visible[0]-1; y++) 
	{//144 for
		for (int x = 1; x < visible[1]-1; x++) 
		{//143 for
			// first color channel
			double *s = im + min(x, dims[1]-2) + min(y, dims[0]-2)*dims[1];
			double dy = *(s+dims[1]) - *(s-dims[1]);
			double dx = *(s+1) - *(s-1);
			double v = dx*dx + dy*dy;

			// second color channel
			s += dims[0]*dims[1];
			double dy2 = *(s+dims[1]) - *(s-dims[1]);
			double dx2 = *(s+1) - *(s-1);
			double v2 = dx2*dx2 + dy2*dy2;

			// third color channel
			s += dims[0]*dims[1];
			double dy3 = *(s+dims[1]) - *(s-dims[1]);
			double dx3 = *(s+1) - *(s-1);
			double v3 = dx3*dx3 + dy3*dy3;

			// pick channel with strongest gradient
			if (v2 > v) {
				v = v2;
				dx = dx2;
				dy = dy2;
			} 
			if (v3 > v) {
				v = v3;
				dx = dx3;
				dy = dy3;
			}

			// snap to one of 18 orientations
			double best_dot = 0;
			int best_o = 0;
			for (int o = 0; o < 9; o++) {
				double dot = uu[o]*dx + vv[o]*dy;
				if (dot > best_dot) {
					best_dot = dot;
					best_o = o;
				} else if (-dot > best_dot) {
					best_dot = -dot;
					best_o = o+9;
				}
			}

			// add to 4 histograms around pixel using linear interpolation
			double xp = ((double)x+0.5)/(double)sbin - 0.5;
			double yp = ((double)y+0.5)/(double)sbin - 0.5;
			int ixp = (int)floor(xp);
			int iyp = (int)floor(yp);
			double vx0 = xp-ixp;
			double vy0 = yp-iyp;
			double vx1 = 1.0-vx0;
			double vy1 = 1.0-vy0;
			v = sqrt(v);

			if (ixp >= 0 && iyp >= 0) {
				*(hist + ixp + iyp*blocks[1] + best_o*blocks[0]*blocks[1]) += 
					vx1*vy1*v;
			}

			if (ixp+1 < blocks[1] && iyp >= 0) {
				*(hist + (ixp+1) + iyp*blocks[1] + best_o*blocks[0]*blocks[1]) += 
					vx0*vy1*v;
			}

			if (ixp >= 0 && iyp+1 < blocks[0]) {
				*(hist + ixp + (iyp+1)*blocks[1] + best_o*blocks[0]*blocks[1]) += 
					vx1*vy0*v;
			}

			if (ixp+1 < blocks[1] && iyp+1 < blocks[0]) {
				*(hist + (ixp+1)+ (iyp+1)*blocks[1]  + best_o*blocks[0]*blocks[1]) += 
					vx0*vy0*v;
			}
		}//143 for
	}//144 for
	 // compute energy in each block by summing over orientations
  for (int o = 0; o < 9; o++) 
  {
    double *src1 = hist + o*blocks[0]*blocks[1];
    double *src2 = hist + (o+9)*blocks[0]*blocks[1];
    double *dst = norm;
    double *end = norm + blocks[1]*blocks[0];
    while (dst < end) 
	{
      *(dst++) += (*src1 + *src2) * (*src1 + *src2);
      src1++;
      src2++;
    }
  }
    // compute features
  int cntdst=0;
   for (int y = 0; y < out[0]; y++) 
  {//for 221
    for (int x = 0; x < out[1]; x++)
	{
		cntdst=0;
      //double *dst = feat + x*out[0] + y;  
		double *dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
      double *src, *p, n1, n2, n3, n4;

      p = norm + (x+1) + (y+1)*blocks[1];
      n1 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + (x+1) + y*blocks[1];
      n2 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + x + (y+1)*blocks[1];
      n3 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);
      p = norm + x + y*blocks[1];      
      n4 = 1.0 / sqrt(*p + *(p+1) + *(p+blocks[0]) + *(p+blocks[0]+1) + eps);

      double t1 = 0;
      double t2 = 0;
      double t3 = 0;
      double t4 = 0;

      // contrast-sensitive features
      src = hist + (x+1) + (y+1)*blocks[1];
      for (int o = 0; o < 18; o++) {
	double h1 = min(*src * n1, 0.2);
	double h2 = min(*src * n2, 0.2);
	double h3 = min(*src * n3, 0.2);
	double h4 = min(*src * n4, 0.2);
	*dst = 0.5 * (h1 + h2 + h3 + h4);
	t1 += h1;
	t2 += h2;
	t3 += h3;
	t4 += h4;
	//dst += out[0]*out[1];
	dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
	src += blocks[0]*blocks[1];
      }

      // contrast-insensitive features
      src = hist + (x+1)*blocks[0] + (y+1);
      for (int o = 0; o < 9; o++) {
        double sum = *src + *(src + 9*blocks[0]*blocks[1]);
        double h1 = min(sum * n1, 0.2);
        double h2 = min(sum * n2, 0.2);
        double h3 = min(sum * n3, 0.2);
        double h4 = min(sum * n4, 0.2);
        *dst = 0.5 * (h1 + h2 + h3 + h4);
        //dst += out[0]*out[1];
		dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
        src += blocks[0]*blocks[1];
      }

      // texture features
      *dst = 0.2357 * t1;
      //dst += out[0]*out[1];
	  dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
      *dst = 0.2357 * t2;
      //dst += out[0]*out[1];
	  dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
      *dst = 0.2357 * t3;
      //dst += out[0]*out[1];
	  dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
      *dst = 0.2357 * t4;

      // truncation feature
      //dst += out[0]*out[1];
	  dst=mxfeat->feat_data[cntdst++].ptr<double>(y,x);
      *dst = 0;
    }
  }//for 221

    free(hist);
	free(norm);
	free(im);
	im=NULL;
	return 0;

}
int	Model::featpyramid(Mat input,char *errorString)
{
	int interval=this->interval;
	int sbin=this->sbin;



	int padx = max(this->maxsize[1]-1-1,0);
	int pady = max(this->maxsize[0]-1-1,0);
	if(interval==0)
	{
		sprintf(errorString,"interval zero\n");
		return -1;
	}
	double sc=pow(2,(1.0/interval));
	int imsize[2];
	imsize[0]=input.rows;
	imsize[1]=input.cols;
	int tempimn=min2(imsize[0],imsize[1]);
	double tempdouble =(tempimn*1.0)/(5*sbin*1.0);
	const int max_scale = 1 + floor(log(tempdouble)/log(sc));

	this->mypyra.newMyPyra(max_scale+interval);
	for(int i=0;i<interval;i++)
	{
		double scaleRate=1/pow(sc,i);
		Size imageSize;
		imageSize.height=int(input.rows*scaleRate+0.5);//
		imageSize.width=int(input.cols*scaleRate+0.5);//
		Mat scaled=Mat(imageSize,input.type());
		resize(input,scaled,imageSize);
		int featMark=i;
		int rs=features(scaled,sbin/2,this->mypyra,featMark,errorString);
		if(rs!=0)
			return rs;
		this->mypyra.scale[i]=2*scaleRate;

		featMark=i+interval;
		rs=features(scaled,sbin,this->mypyra,featMark,errorString);


		
		if(rs!=0)
			return rs;
		this->mypyra.scale[i+interval]=scaleRate;
		 // remaining interals

		Size msize=Size(scaled.cols,scaled.rows);
		for(int j=i+interval;j<=max_scale;j+=interval)
		{
			
			msize.height=int((msize.height)/2);
			msize.width=int((msize.width)/2);
			Mat newScaled=Mat(msize,scaled.type());
			resize(scaled,newScaled,msize);

			int featMark=j+interval;
			rs=features(newScaled,sbin,this->mypyra,featMark,errorString);
			if(rs!=0)
				return rs;
			this->mypyra.scale[j+interval]=0.5 * mypyra.scale[j];
		//pyra.feat{j+interval} = features(scaled, sbin);
        //pyra.scale(j+interval) = 0.5 * pyra.scale(j);

		}

	}
	//check
	for(int i=0;i<mypyra.len;i++)
	{
		for(int j=0;j<FEAT_Z_DIM;j++)
		{
			int rows=(mypyra.feat+i)->feat_data[j].rows+pady+1;
			int cols=(mypyra.feat+i)->feat_data[j].cols+padx+1;

			Mat mat=Mat(rows,cols,(mypyra.feat+i)->feat_data[j].type(),Scalar(0,0,0));
			Mat D(mat,Rect(0,0,(mypyra.feat+i)->feat_data[j].cols,(mypyra.feat+i)->feat_data[i].rows));
			addWeighted((mypyra.feat+i)->feat_data[j],1.0,NULL,0.0,0.0,D);

		}
		//ypyra.feat+i)->feat_data[i] = padarray(pyra.feat{i}, [pady+1 padx+1 0], 0);

		//pyra.feat{i}(1:pady+1, :, end) = 1;
		for(int r=0;r<=min(pady,(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].rows-1);r++)
		{
			for(int c=0;c<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].cols;c++)
			{
				(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].at<double>(r,c)=1;
			}
		}

		
		
		//pyra.feat{i}(end-pady:end, :, end) = 1;
		for(int r=(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].rows-pady-1;r>=0&&(r<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].rows);r++)
		{
			for(int c=0;c<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].cols;c++)
			{
				(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].at<double>(r,c)=1;
			}
		}

		//pyra.feat{i}(:, 1:padx+1, end) = 1;
		for(int r=0;r<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].rows;r++)
		{
			for(int c=0;c<padx;c++)
			{
				(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].at<double>(r,c)=1;
			}
		}
		//pyra.feat{i}(:, end-padx:end, end) = 1;
		for(int r=0;r<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].rows;r++)
		{
			for(int c=(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].cols-padx-1;c>=0&&c<(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].cols;c++)
			{
				(mypyra.feat+i)->feat_data[FEAT_Z_DIM-1].at<double>(r,c)=1;
			}
		}


	}//check
	for(int i=0;i<mypyra.len;i++)
	{
		mypyra.scale[i]=(this->sbin)/(mypyra.scale[i]);
	}
	
	mypyra.interval = interval;
	mypyra.imy = imsize[0];
	mypyra.imx = imsize[1];
	mypyra.pady = pady;
	mypyra.padx = padx;

	return 0;
}
int  Model::modelcomponents(MyPyra &mypyra, Components &mycomponents,char *errorString)
{
	//[mycomponents,myfilters,myresp  ,mypyra];
	//COM_NUM
	
	if(COM_NUM!=13)
	{
		sprintf(errorString,"COM_NUM is not 13\n");
		return -1;
	}
	for(int c=0;c<COM_NUM;c++)
	{
		
		for(int k=0;k<this->model_component[c].len;k++)
		{
			//Parts_com_data p=
			Model_Component_Data pp=this->model_component[c].model_component_data[k];
			int x_i=this->filters[pp.filterid-1].i;
			Parts_com_data p;
			p.defid=pp.defid;
			p.filterid=pp.filterid;
			p.parent=pp.parent;
			p.sizy=this->filters[pp.filterid-1].filter_data[0].rows;
			p.sizx=this->filters[pp.filterid-1].filter_data[0].cols;
			p.filterI = x_i;

			Defs x=this->defs[int(p.defid+0.5)-1];
			p.defI=x.i;
			for(int j=0;j<4;j++)
			{
				p.w[j]=x.w[j];
			}
			int par = int(p.parent+0.5)-1;
			if(par>=k)
			{
				sprintf(errorString,"par<k:%d<k\n",par,k);
				return -1;
			}
			int ax  = x.anchor[0];
			int ay  = x.anchor[1];
			int	ds  = x.anchor[2];

			if(par >=0)
				 p.scale = ds + mycomponents.components_data[c].parts_com_data[par].scale;
			else
			{
				if(k != 0)
				{
					sprintf(errorString,"k!=0\n");
					return -1;
				};
			   p.scale = 0;
			}

			double step=pow(2.0,ds*1.0);
			double virtpady = (step-1)*mypyra.pady;
			double virtpadx = (step-1)*mypyra.padx;
            
			p.starty = ay-virtpady;
			p.startx = ax-virtpadx;
			p.step   = step;
			p.level  = 0;
			p.score  = 0;
			p.Ix     = 0;
			p.Iy     = 0;
			
			 mycomponents.components_data[c].parts_com_data[k] = p;



		}//sec for
	}//first for
	return 0;
}
int Model::detect(Mat input,char *errorString)
{

	init();
	memset(boxes,0,sizeof(Boxes)*BOXCACHESIZE);
	paramsLoad();
	int	rs=featpyramid(input,errorString);
	if(rs!=0)
		return rs;
	rs=modelcomponents(this->mypyra, mycomponents,errorString);
	if(rs!=0)
		return rs;

	int cnt=0;

	for(int ii=0;ii<cmat.cols;ii++)
	{
		//int c=int(cmat.at<MatType>(0,ii))-1;
		int c=ii;
		int minlevel=interval+1;
		//length of pyra feat 可能要修改

		//wrong
		int length_pyra_feat=mypyra.len;
		
		for(int rlevel=minlevel;rlevel<=length_pyra_feat;rlevel++)
		{
			Parts parts=components.components_data[c];
			const int numparts=parts.len;
			//计算卷积运行时间
			time_t t_start,t_end;
			t_start=time(NULL);

			for(int k=1;k<=numparts;k++)
			{
				int f=int(parts.parts_com_data[k-1].filterid+0.5);

				double parts_parts_com_data_k_scale=parts.parts_com_data[k-1].scale;
				int level=int(rlevel-parts_parts_com_data_k_scale*interval+0.5);

				if(resp[level-1].emptyOrNot==true)
				{
					//进行卷积操作
					int rs= fconv(mypyra.feat[level-1],filters,1,FILTERS_NUM,resp[level-1],errorString);
					if(rs!=0)
					{
						return rs;
					}
					resp[level-1].emptyOrNot=false;

				}
				//下一步
				parts.parts_com_data[k-1].socreMat=resp[level-1].conv_data[f-1].clone();
				//cout<<"resp["<<level-1<<"].conv_data["<<f-1<<"]"<<endl<<resp[level-1].conv_data[f-1].col(0)<<endl;

				parts.parts_com_data[k-1].level = level;
			}
			t_end=time(NULL);
			//cout<<"time cost:"<<difftime(t_end,t_start)<<"s\n";
			// Walk from leaves to root of tree, passing message to parent
			// Given a 2D array of filter scores 'child', shiftdt() does the following:
			// (1) Apply distance transform
			// (2) Shift by anchor position (child.startxy) of part wrt parent
			// (3) Downsample by child.step


			for(int k=numparts;k>=2;k--)
			{
				Parts_com_data child=parts.parts_com_data[k-1];

				Mat child_scoreMat=parts.parts_com_data[k-1].socreMat.clone();
				child.socreMat=child_scoreMat;
				
				int par   = int(child.parent+0.5)-1;
				int Ny=parts.parts_com_data[par].socreMat.rows;
				int Nx=parts.parts_com_data[par].socreMat.cols;
				Mat msg;
				int rs=shiftdt(child.socreMat,child.w[0],child.w[1],child.w[2],child.w[3],child.startx,child.starty,Nx,Ny,child.step,msg,parts.parts_com_data[k-1].IxMat,parts.parts_com_data[k-1].IyMat,errorString);

				if(rs!=0)
					return rs;
				//cout<<"parts.parts_com_data[par]scoreMat col 1"<<endl<<parts.parts_com_data[par].socreMat.col(0)<<endl;

				addWeighted(parts.parts_com_data[par].socreMat,1,msg,1,0,parts.parts_com_data[par].socreMat);




			}

			int rscore_rows=parts.parts_com_data[0].socreMat.rows;
			int rscore_cols=parts.parts_com_data[0].socreMat.cols;
			Mat rscore(rscore_rows,rscore_cols,parts.parts_com_data[0].socreMat.type());


			addWeighted(parts.parts_com_data[0].socreMat,1.0,NULL,0,parts.parts_com_data[0].w[0],rscore);





			//parts.parts_com_data[0].w[0];
			vector<int> X;
			vector<int> Y;
			X.clear();
			Y.clear();
			for(int i=0;i<rscore_rows;i++)
			{//get x,y
				for(int j=0;j<rscore_cols;j++)
				{
					if(rscore.at<double>(i,j) >(this->thresh))
					{
						X.push_back(j);
						Y.push_back(i);
					}
				}
			}//get x,y
			Mat XY;
			if(X.size()>0)
			{
				int *XX=(int *)malloc(X.size()*sizeof(int));
				int *YY=(int *)malloc(X.size()*sizeof(int));
				for(int i=0;i<X.size();i++)
				{
					XX[i]=X[i];
					YY[i]=Y[i];
				}

				int rs= backtrack(XX,YY,X.size(),parts,mypyra,XY,errorString);
				if(rs!=0)
					return rs;

				free(XX);
				free(YY);

			}
			//Walk back down tree following pointers
			const int lengthX=X.size();
			//cout<<"length X"<<lengthX<<endl;
			for (int ii = 1;ii<=lengthX;ii++)
			{
				int x = X[ii-1];
				int y = Y[ii-1];

				/*if(cnt == BOXCACHESIZE)
				{
				b0 = nms_face(boxes,0.3);
				clear boxes;
				boxes.s  = 0;
				boxes.c  = 0;
				boxes.xy = 0;
				boxes.level = 0;
				boxes(BOXCACHESIZE) = boxes;
				cnt = length(b0);>	FaceRecognization.exe!Model::detect(cv::Mat * input, char * errorString)  Line 810	C++

				boxes(1:cnt) = b0;
				}*/

				cnt = cnt + 1;
				boxes[cnt-1].c = c;
				boxes[cnt-1].s = rscore.at<double>(y,x);
				boxes[cnt-1].level = rlevel;
				Mat temp=Mat(numparts,4,CV_64FC1,Scalar(0,0,0));
				//boxes[cnt-1].=temp;
				temp.copyTo(boxes[cnt-1].xyMat);
				//boxes[cnt-1].xyMat=Mat(numparts,4,CV_64FC1,Scalar(0,0,0));

				for(int r=0;r<numparts;r++)
				{
					for(int c=0;c<4;c++)
					{
						boxes[cnt-1].xyMat.at<double>(r,c)=XY.at<double>(r,c,ii-1);//问题？？
					}
				}
				//cout<<"boxes[cnt-1].xyMat col(0)"<<endl;
				//cout<<boxes[cnt-1].xyMat.col(0)<<endl;


			}

		}
	}
	//box check
	for(int i=0;i<cnt;i++)
	{
		for(int j=0;j<boxes[i].xyMat.rows;j++)
		{
			boxes[i].xyMat.at<double>(j,0)=max(boxes[i].xyMat.at<double>(j,0), 0);
			boxes[i].xyMat.at<double>(j,1)=max(boxes[i].xyMat.at<double>(j,1), 0);
			boxes[i].xyMat.at<double>(j,2)=min(boxes[i].xyMat.at<double>(j,2), input.cols);
			boxes[i].xyMat.at<double>(j,3)=min(boxes[i].xyMat.at<double>(j,3), input.rows);

		}

	}
	cout<<"cnt"<<cnt<<endl;
	int topLen=0;
	rs= nms_face(boxes,cnt,0.3,box3001,topLen,errorString);
	if(rs!=0)
		return rs;
	drawBoxes(input,box3001,topLen);
	return 0;
}