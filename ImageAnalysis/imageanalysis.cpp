#include "imageanalysis.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>


#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)


                                                                         
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

ImageAnalysis::ImageAnalysis(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	connect(ui.pushButton_main_showimage, SIGNAL(clicked()), SLOT(OnShowImage()));
	connect(ui.pushButton_main_exit, SIGNAL(clicked()), SLOT(close()));
	connect(ui.dial_main_thredhold, SIGNAL(valueChanged(int)), SLOT(OnValueChanged(int)));
	connect(ui.checkBox_main_usmsharp, SIGNAL(stateChanged(int)), SLOT(OnUsmStateChanged(int)));
	connect(ui.listWidget_main_regions, SIGNAL(itemClicked(QListWidgetItem *)), SLOT(OnRegionItemClicked(QListWidgetItem *)));
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

	// 图像预处理
	PreAnalysisImage();
	return;

	//参数为：源图像，结果图像，图像深度，x方向阶数，y方向阶数，核的大小，尺度因子，增加的值
// 	Sobel(originImage, sobelImage, CV_8U, 1, 0, 3, .3, 128);
// 	imshow("sobel", sobelImage);
// 	Mat normalizeImage;  
// 	normalize(sobelImage, normalizeImage, 255, 0, CV_MINMAX);
// 	Mat binaryImage;  
// 	threshold(normalizeImage,binaryImage, 100, 255, THRESH_BINARY_INV );
// 	imshow("threshold", binaryImage);
// 	Mat morphologyImage;
// 	Mat element = getStructuringElement(MORPH_RECT, Size(10, 10)); 
// 	morphologyEx(binaryImage, morphologyImage, MORPH_CLOSE, element);
// 	imshow("morphologyEx", morphologyImage);
	return;
}

// 图像预处理
void ImageAnalysis::PreAnalysisImage()
{
	Mat sobelImage;  
	cvtColor(originImage, grayImage, CV_BGR2GRAY);
	// 	ImageProcess::Instance()->FourierTrans(originImage);
	// 	imshow("Canny", originImage);

	// 自动找到阈值
	double high = 0;
	ImageProcess::Instance()->AdaptiveFindThreshold(grayImage, &lowValue, &high, &averValue);
	SmoothImage(grayImage, averValue);

	// 进行USM锐化
	if (ui.checkBox_main_usmsharp->isChecked())
	{
		ImageProcess::Instance()->USMSharp(grayImage);
	}

	// 显示图像
	ui.dial_main_thredhold->setValue(lowValue);
}

void ImageAnalysis::ShowImage(Mat image, QLabel *imageLabel)
{
	Mat resizeImage = image;
	int w, h;
	if (image.cols > 1024)
	{
		w = 1024;
		h = 1024 * image.rows / image.cols;
		cv::resize(image, resizeImage, cv::Size(w, h));
	}
	cvtColor(resizeImage, resizeImage, CV_BGR2RGB);
	QImage qImage((const unsigned char *)resizeImage.data, resizeImage.cols, resizeImage.rows, QImage::Format_RGB888);
	imageLabel->setPixmap(QPixmap::fromImage(qImage));
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

// 旋钮移动响应
void ImageAnalysis::OnValueChanged(int value)
{
	ui.spinBox_main_thredhold->setValue(value);
	if (!grayImage.empty())
	{
		Mat cannyImage;
		//canny边缘检测
		GaussianBlur(grayImage, cannyImage, Size(5, 5), 0);
		// 	hold = threshold(grayImage, cannyImage, 0, 255, THRESH_OTSU);
		Canny(grayImage, cannyImage, value, value * 2.5);
		Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); 
		morphologyEx(cannyImage, cannyImage, CV_MOP_CLOSE, element);
		std::vector<CvSeq *> vContours;
		ImageProcess::Instance()->ConnectedComponents(cannyImage, vContours, 0, 100, 1, Rect(), Point(-1, -1));    //采用多边形拟合处理
		int nIndex = 0;
		ui.listWidget_main_regions->clear();
		foreach(CvSeq *contour, vContours)
		{
			nIndex ++;
			double area = cvContourArea( contour );
			QString strItem = "%1, %2";
			strItem = strItem.arg(nIndex).arg(area);
			ui.listWidget_main_regions->addItem(strItem);
		}
		
		cvtColor(cannyImage, cannyImage, CV_GRAY2BGR);
		cannyImage += originImage;
		ShowImage(cannyImage, ui.label_showimage);
	}
}

// USM选择
void ImageAnalysis::OnUsmStateChanged(int state)
{
	if (state == Qt::Checked)
	{
		ImageProcess::Instance()->USMSharp(grayImage);
		ui.dial_main_thredhold->setValue(lowValue);
		OnValueChanged(lowValue);
	}
	else
	{
		PreAnalysisImage();
	}
}

// 当前颗粒列表点击事件
void ImageAnalysis::OnRegionItemClicked(QListWidgetItem *item)
{
	item->setText("22");
}
