#include "imageanalysis.h"
#include <QFileDialog>
#include <QMessageBox>

Mat originImage;
Mat grayImage;  
Mat cannyImage;  

#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)

double lowValue = 0;
double averValue = 0;

                                                                         
// 仿照matlab，自适应求高低两个门限                                            
void _AdaptiveFindThreshold(CvMat *dx, CvMat *dy, double *low, double *high)   
{                                                                              
	CvSize size;                                                           
	IplImage *imge=0;                                                      
	int i,j;                                                               
	CvHistogram *hist;                                                     
	int hist_size = 255;                                                   
	float range_0[]={0,256};                                               
	float* ranges[] = { range_0 };                                         
	double PercentOfPixelsNotEdges = 0.7;                                  
	size = cvGetSize(dx);                                                  
	imge = cvCreateImage(size, IPL_DEPTH_32F, 1);                          
	// 计算边缘的强度, 并存于图像中                                        
	float maxv = 0;                                                        
	for(i = 0; i < size.height; i++ )                                      
	{                                                                      
		const short* _dx = (short*)(dx->data.ptr + dx->step*i);        
		const short* _dy = (short*)(dy->data.ptr + dy->step*i);        
		float* _image = (float *)(imge->imageData + imge->widthStep*i);
		for(j = 0; j < size.width; j++)                                
		{                                                              
			_image[j] = (float)(abs(_dx[j]) + abs(_dy[j]));        
			maxv = maxv < _image[j] ? _image[j]: maxv;             

		}                                                              
	}                                                                      
	if(maxv == 0){                                                         
		*high = 0;                                                     
		*low = 0;                                                      
		cvReleaseImage( &imge );                                       
		return;                                                        
	}                                                                      

	// 计算直方图                                                          
	range_0[1] = maxv;                                                     
	hist_size = (int)(hist_size > maxv ? maxv:hist_size);                  
	hist = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);          
	cvCalcHist( &imge, hist, 0, NULL );                                    
	int total = (int)(size.height * size.width * PercentOfPixelsNotEdges); 
	float sum=0;                                                           
	int icount = hist->mat.dim[0].size;                                    

	float *h = (float*)cvPtr1D( hist->bins, 0 );                           
	for(i = 0; i < icount; i++)                                            
	{                                                                      
		sum += h[i];                                                   
		if( sum > total )                                              
			break;                                                 
	}                                                                      
	// 计算高低门限                                                        
	*high = (i+1) * maxv / hist_size ;                                     
	*low = *high * 0.4;                                                    
	cvReleaseImage( &imge );                                               
	cvReleaseHist(&hist);                                                  
}                                                                              

void onChangeThredhold(int hold, void *);

ImageAnalysis::ImageAnalysis(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_showimage, SIGNAL(clicked()), SLOT(OnShowImage()));
	connect(ui.pushButton_exit, SIGNAL(clicked()), SLOT(close()));
}

ImageAnalysis::~ImageAnalysis()
{

}

void ImageAnalysis::OnShowImage()
{
	QString strPath = QFileDialog::getOpenFileName(this, "Choose File", QDir::currentPath() + "\\toBeAnalysis\\");
	if (strPath.isEmpty())
	{
		return;
	}
	originImage = imread(strPath.toLocal8Bit().data());
	if (originImage.empty())
	{
		QMessageBox msgBox;
		msgBox.setText(QStringLiteral("图片无法识别"));
		msgBox.exec();
		return;
	}
	Mat sobelImage;  
	cvtColor(originImage, grayImage, CV_BGR2GRAY);
// 	ImageProcess::Instance()->FourierTrans(originImage);
	imshow("Canny", originImage);



// 	MatND histogram;  
// 	//256个，范围是0，255.  
// 	const int histSize = 256;  
// 	float range[] = {0, 1};  
// 	const float *ranges[] = {range};  
// 	const int channels = 0;  
// 
// 	calcHist(&grayImage, 1, &channels, Mat(), histogram, 1, &histSize, &ranges[0], true, false);  
// 	imshow("Canny", histogram);


	double high = 0;
	ImageProcess::Instance()->AdaptiveFindThreshold(grayImage, &lowValue, &high, &averValue);
	SmoothImage(grayImage, averValue);

	int count = lowValue;
	createTrackbar("thredhold", "Canny", &count, 255, onChangeThredhold);
	onChangeThredhold(count, NULL);
	return;

	//参数为：源图像，结果图像，图像深度，x方向阶数，y方向阶数，核的大小，尺度因子，增加的值
	Sobel(originImage, sobelImage, CV_8U, 1, 0, 3, .3, 128);
	imshow("sobel", sobelImage);
	Mat normalizeImage;  
	normalize(sobelImage, normalizeImage, 255, 0, CV_MINMAX);
// 	imshow("normalize", normalizeImage);
	Mat binaryImage;  
	threshold(normalizeImage,binaryImage, 100, 255, THRESH_BINARY_INV );
	imshow("threshold", binaryImage);
	Mat morphologyImage;
	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10)); 
	morphologyEx(binaryImage, morphologyImage, MORPH_CLOSE, element);
	imshow("morphologyEx", morphologyImage);
	return;
}

void onChangeThredhold(int hold, void *)
{  
	//canny边缘检测
	GaussianBlur(grayImage, cannyImage, Size(5, 5), 0);
	// 	hold = threshold(grayImage, cannyImage, 0, 255, THRESH_OTSU);
	Canny(grayImage, cannyImage, hold, hold * 3);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); 
	morphologyEx(cannyImage, cannyImage, CV_MOP_CLOSE, element);
	ImageProcess::Instance()->ConnectedComponents(cannyImage, 0, 100, 1, Rect(), Point(-1, -1));    //采用多边形拟合处理
	cvtColor(cannyImage, cannyImage, CV_GRAY2BGR);
	cannyImage += originImage;
	imshow("Canny", cannyImage);
}

void ImageAnalysis::SmoothImage(Mat src, int highThredhold)
{
	int nr= src.rows; // number of rows  
	int nc= src.cols * src.channels(); // total number of elements per line
	int step= src.step; // effective width
	uchar *data= src.data;  
	for (int j=0; j<nr; j++) {  
		for (int i=0; i<nc; i++) {
			int pix = *(data+i);
			if ( pix > (highThredhold + 40))
			{
				*(data+i) = highThredhold - 40;
			}
		} // end of row                   
		data+= step;  // next line  
	}
}