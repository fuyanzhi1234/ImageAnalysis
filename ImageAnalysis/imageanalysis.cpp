#include "imageanalysis.h"
#include <QFileDialog>

Mat grayImage;  
Mat cannyImage;  

#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)

double lowValue = 0;

                                                                         
// ����matlab������Ӧ��ߵ���������                                            
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
	// �����Ե��ǿ��, ������ͼ����                                        
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

	// ����ֱ��ͼ                                                          
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
	// ����ߵ�����                                                        
	*high = (i+1) * maxv / hist_size ;                                     
	*low = *high * 0.4;                                                    
	cvReleaseImage( &imge );                                               
	cvReleaseHist(&hist);                                                  
}                                                                              

void AdaptiveFindThreshold(const cv::Mat src, double *low, double *high, int aperture_size=3)
{
	int nr= src.rows; // number of rows  
	int nc= src.cols * src.channels(); // total number of elements per line
	int step= src.step; // effective width
	int nTotal = 0;
	uchar *data= src.data;  
	for (int j=0; j<nr; j++) {  
		for (int i=0; i<nc; i++) {  
			nTotal += *(data+i);  
		} // end of row                   
		data+= step;  // next line  
	}
	int nAva = nTotal / (nr * nc);
	nTotal = 0;
	int num = 0;
	data= src.data;
	for (int j=0; j<nr; j++) {  
		for (int i=0; i<nc; i++) {
			int pix = *(data+i);
			if (abs(pix - nAva) > 60)
			{
				num ++;
				nTotal += pix;
			}
		} // end of row                   
		data+= step;  // next line  
	}
	int nAvaDis = nTotal / num;

	*low = abs(nAva - nAvaDis);
	*high = *low * 3;
}    

// ���
void ConnectedComponents(Mat &mask_process, int poly1_hull0, float perimScale, int number = 0,
	Rect &bounding_box = Rect(), Point &contour_centers = Point(-1, -1))
{
	/*����4�������Ϊ�˼���ԭ�����ӿڣ����ڲ�ʹ�õ���c��񣬵�����ӿ���c++����*/
	IplImage *mask = &mask_process.operator IplImage();
	int *num = &number;
	CvRect *bbs = &bounding_box.operator CvRect();
	CvPoint *centers = &contour_centers.operator CvPoint();
	static CvMemStorage*    mem_storage    = NULL;
	static CvSeq*            contours    = NULL;
	//CLEAN UP RAW MASK
	//���������ã�ƽ��������ȥ��ϸ��,�Ͽ�ȱ��
// 	cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_OPEN, 1 );//������mask���п�������CVCLOSE_ITRΪ�������Ĵ��������Ϊmaskͼ��
	//���������ã�ƽ������������ȱ��
// 	cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_CLOSE, 1 );//������mask���бղ�����CVCLOSE_ITRΪ�ղ����Ĵ��������Ϊmaskͼ��

	//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
	else cvClearMemStorage(mem_storage);
	//CV_RETR_EXTERNAL=0����types_c.h�ж���ģ�CV_CHAIN_APPROX_SIMPLE=2Ҳ���ڸ��ļ��ж����
	CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	//��while�ڲ�ֻ��ԱȽϴ���������߽����滻����
	while( (c = cvFindNextContour( scanner )) != NULL )
	{
		double len = cvContourArea( c );
		double q = 100;(mask->height + mask->width) /perimScale;   //calculate perimeter len threshold
		if( len < q ) //Get rid of blob if it's perimeter is too small
		{
			cvSubstituteContour( scanner, NULL );    //��NULL����ԭ�����Ǹ�����
		}
		else //Smooth it's edges if it's large enough
		{
			CvSeq* c_new;
// 			if(poly1_hull0) //Polygonal approximation of the segmentation
// 				c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, 4,0);
// 			else //Convex Hull of the segmentation
// 				c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
// 			cvSubstituteContour( scanner, c_new ); //�ʼ��������͹�����߶���ʽ��������滻
			numCont++;
		}
	}
	contours = cvEndFindContours( &scanner );    //�����������Ҳ���
	// PAINT THE FOUND REGIONS BACK INTO THE IMAGE
	cvZero( mask );
	IplImage *maskTemp;
	//CALC CENTER OF MASS AND OR BOUNDING RECTANGLES
	if(*num != 0)
	{
		int N = *num, numFilled = 0, i=0;
		CvMoments moments;
		double M00, M01, M10;
		maskTemp = cvCloneImage(mask);
		for(i=0, c=contours; c != NULL; c = c->h_next,i++ )        //h_nextΪ���������е���һ������
		{
			if(i < N) //Only process up to *num of them
			{
				//CV_CVX_WHITE�ڱ��������ǰ�ɫ����˼
				cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
				//Find the center of each contour
				if(centers != &cvPoint(-1, -1))
				{
					cvMoments(maskTemp,&moments,1);    //����maskͼ�����ߴ�3�׵ľ�
					M00 = cvGetSpatialMoment(&moments,0,0); //��ȡx��0�κ�y��0�ξ�
					M10 = cvGetSpatialMoment(&moments,1,0); //��ȡx��1�κ�y��0�ξ�
					M01 = cvGetSpatialMoment(&moments,0,1); //��ȡx��0�κ�y��1�ξ�
					centers[i].x = (int)(M10/M00);    //���þصĽ��������������ĵ�����
					centers[i].y = (int)(M01/M00);
				}
				//Bounding rectangles around blobs
				if(bbs != &CvRect())
				{
					bbs[i] = cvBoundingRect(c); //�������c����Ӿ���
				}
				cvZero(maskTemp);
				numFilled++;
			}
			//Draw filled contours into mask
// 			cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,CV_FILLED,8); //draw to central mask
			cvDrawContours(mask,c,CV_CVX_WHITE,CV_CVX_WHITE,-1,1); //draw to central mask
		} //end looping over contours
		*num = numFilled;
		cvReleaseImage( &maskTemp);
	}
	//ELSE JUST DRAW PROCESSED CONTOURS INTO THE MASK
	else
	{
		for( c=contours; c != NULL; c = c->h_next )
		{
			cvDrawContours(mask,c,CV_CVX_WHITE, CV_CVX_BLACK,-1,CV_FILLED,8);
		}
	}
}

void onchange(int hold, void *)  
{  
	//canny��Ե���
	GaussianBlur(grayImage, cannyImage, Size(5, 5), 0);
// 	hold = threshold(grayImage, cannyImage, 0, 255, THRESH_OTSU);
	Canny(grayImage, cannyImage, hold, hold * 3);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5)); 
	morphologyEx(cannyImage, cannyImage, CV_MOP_CLOSE, element);
	ConnectedComponents(cannyImage, 1, 8.0, 1, Rect(), Point(-1, -1));    //���ö������ϴ���
	cannyImage += grayImage;
	imshow("Canny", cannyImage);
}  
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
	QString strPath = QFileDialog::getOpenFileName(this, "Choose File", QDir::currentPath() + "\\test image\\");
	if (strPath.isEmpty())
	{
		return;
	}
	cv::Mat image = imread(strPath.toStdString());//(100, 100, CV_8UC3, Scalar(0, 0, 255));
	Mat sobelImage;  
	cvtColor(image, grayImage, CV_BGR2GRAY);
// 	FourierTrans(image);
	imshow("Canny", image);

	double high = 0;
	AdaptiveFindThreshold(image, &lowValue, &high);

	int count = lowValue;
	createTrackbar("thredhold", "Canny", &count, 255, onchange);
	onchange(count, NULL);
	return;

	//����Ϊ��Դͼ�񣬽��ͼ��ͼ����ȣ�x���������y����������˵Ĵ�С���߶����ӣ����ӵ�ֵ
	Sobel(image, sobelImage, CV_8U, 1, 0, 3, .3, 128);
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


// 	cvtColor(image, image, CV_BGR2RGB);
// 	QImage *qimage = new QImage(image.data, image.cols, image.rows, QImage::Format_RGB888); 
// 	ui.label_showimage->setPixmap(QPixmap::fromImage(*qimage));
}

void ImageAnalysis::FourierTrans(Mat &image)
{
	cvtColor(image,image,CV_RGB2GRAY);
	image.convertTo(image,CV_32FC1);
	for(int i=0; i<image.rows; i++)        //���Ļ�
	{
		float *p = image.ptr<float>(i);
		for(int j=0; j<image.cols; j++)
		{
			p[j] = p[j] * pow((double)-1, i+j);
		}
	}
	// 	imshow("src",image);

	/////////////////////////////////////���ٸ���Ҷ�任/////////////////////////////////////////////////////
	int oph = getOptimalDFTSize(image.rows);
	int opw = getOptimalDFTSize(image.cols);
	Mat padded;
	copyMakeBorder(image, padded, 0, oph-image.rows, 0, opw-image.cols, BORDER_CONSTANT, Scalar::all(0));

	Mat temp[] = {padded, Mat::zeros(image.size(),CV_32FC1)};
	Mat complexI;
	merge(temp, 2, complexI);

	dft(complexI, complexI);    //����Ҷ�任

	//��ʾƵ��ͼ
	split(complexI, temp);
	Mat aa;
	magnitude(temp[0], temp[1], aa);
	divide(aa, oph*opw, aa);
	imshow("aa",aa);

	///////////////////////////////////////////Ƶ���˲�///////////////////////////////////////////////////////
	//����Ƶ���˲���
	Mat gaussianBlur(image.size(), CV_32FC2);
	Mat gaussianSharpen(image.size(), CV_32FC2);
	float D0 = 2*50*50.;
	for(int i=0; i<oph; i++)
	{
		float *p = gaussianBlur.ptr<float>(i);
		float *q = gaussianSharpen.ptr<float>(i);
		for(int j=0; j<opw; j++)
		{
			float d = pow((double)i-oph/2, 2) + pow((double)j-opw/2, 2);
			p[2*j] = expf(-d / D0);
			p[2*j+1] = expf(-d / D0);

			q[2*j] = 1 - expf(-d / D0);
			q[2*j+1] = 1 - expf(-d / D0);
		}
	}

	//��˹��ͨ�˲��� ��˹��ͨ�˲�
	multiply(complexI, gaussianBlur, gaussianBlur);
	multiply(complexI, gaussianSharpen, gaussianSharpen);

	//����Ҷ���任
	dft(gaussianBlur, gaussianBlur, CV_DXT_INVERSE);
	dft(gaussianSharpen, gaussianSharpen, CV_DXT_INVERSE);

	Mat dstBlur[2], dstSharpen[2];
	split(gaussianBlur, dstBlur);
	split(gaussianSharpen, dstSharpen);

	for(int i=0; i<oph; i++)        //���Ļ�
	{
		float *p = dstBlur[0].ptr<float>(i);
		float *q = dstSharpen[0].ptr<float>(i);
		for(int j=0; j<opw; j++)
		{
			p[j] = p[j] * pow((double)-1, i+j);
			q[j] = q[j] * pow((double)-1, i+j);
		}
	}
	normalize(dstBlur[0], dstBlur[0], 1, 0, CV_MINMAX);
	normalize(dstSharpen[0], dstSharpen[0], 1, 0, CV_MINMAX);
	imshow("dstBlur",dstBlur[0]);
	imshow("dstSharpen",dstSharpen[0]);
	image = dstSharpen[0];
}