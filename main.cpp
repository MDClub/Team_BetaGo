#include<opencv2/opencv.hpp>
#include<engine.h>
#include<iostream>
#include<matrix.h>
using namespace std;
using namespace cv;


bool init();//初始化引擎等
void release();//释放资源

static Engine *ep=NULL;
int main()
{
	if(init()==false)
		return -1;
	

	VideoCapture capture(".\\model2\\Megamind.avi");//motinas_nikola_dark.avi
	
	while(1)
	{
		Mat frame;
		capture>>frame;
		if(frame.data==NULL)
			break;
		cout<<"channels"<<frame.channels()<<endl;
		
		mxArray* ch1=mxCreateDoubleMatrix(frame.rows,frame.cols, mxREAL);
		mxArray* ch2=mxCreateDoubleMatrix(frame.rows,frame.cols, mxREAL);
		mxArray* ch3=mxCreateDoubleMatrix(frame.rows,frame.cols, mxREAL);

		if(frame.channels()==3)
		{
			vector<Mat> channels;
			split(frame,channels);

			double *ch1dst = (double *)mxGetPr(ch1);
			for(int c=0;c<frame.cols;c++)
			{
				for(int r=0;r<frame.rows;r++)
				{
					 *(ch1dst+c*frame.rows+r)=channels[0].at<uchar>(r,c);
				}
			}

			double *ch2dst = (double *)mxGetPr(ch2);
			for(int c=0;c<frame.cols;c++)
			{
				for(int r=0;r<frame.rows;r++)
				{
					 *(ch2dst+c*frame.rows+r)=channels[1].at<uchar>(r,c);
				}
			}

			double *ch3dst = (double *)mxGetPr(ch3);
			for(int c=0;c<frame.cols;c++)
			{
				for(int r=0;r<frame.rows;r++)
				{
					 *(ch3dst+c*frame.rows+r)=channels[2].at<uchar>(r,c);
				}
			}
			engEvalString(ep,"cd G:\\Users\\xjw\\Desktop\\face-release1.0-basic");
			engPutVariable(ep, "ch1", ch1);
			engPutVariable(ep, "ch2", ch2);
			engPutVariable(ep, "ch3", ch3);

			 engEvalString(ep, "im(:,:,1)=ch1;");
			 engEvalString(ep, "im(:,:,2)=ch2;");
			 engEvalString(ep, "im(:,:,3)=ch3;");

			 engEvalString(ep, "imshow(im);");
			
			 engEvalString(ep, "[outmat,len]=mydetect(im,model);");
			 mxArray *out_gbvs=engGetVariable(ep,"outmat");
			 double *ch=(double *)mxGetPr(out_gbvs);

			 Mat xy=Mat(Size(mxGetM(out_gbvs),mxGetN(out_gbvs)),CV_64FC1);
			 xy.data=(uchar*)ch;

			 mxArray *lenx=engGetVariable(ep,"len");
			 double *lenptr=(double *)mxGetPr(lenx);
			 int len=(*(lenptr)+0.5);
		}
		imshow("读取视频帧",frame);
		waitKey(0);

	}

	waitKey();

	release();
	return 0;
}
bool init()
{
	ep!=NULL;
	engClose(ep);

	if(!(ep=engOpen(NULL)))
		return false;
	else
	{
		engEvalString(ep,"cd D:\\Desktop\\face-release1.0-basic;");
		engEvalString(ep,"init");
		return true;
	}
		
}
void release()
{
	if(ep!=NULL)
		engClose(ep);
}