#include "ImageProcess.h"

static ImageProcess *inst = NULL;

ImageProcess::ImageProcess(void)
{
}

ImageProcess::~ImageProcess(void)
{
}

ImageProcess * ImageProcess::Instance()
{
	if (inst == NULL)
	{
		inst = new ImageProcess;
	}
	return inst;
}

/* ����Canny�㷨��������ֵ
@param src[in]: Դͼ��
@param low[out]: ����ֵ
@param high[out]: ����ֵ
@param aperture_size[int]:����ֵ�Ŵ��� 
*/ 
void ImageProcess::AdaptiveFindThreshold(const cv::Mat src, double *low, double *high, double *average, int aperture_size)
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
	*average = nTotal / (nr * nc);
	nTotal = 0;
	int num = 0;
	data= src.data;
	for (int j=0; j<nr; j++) {  
		for (int i=0; i<nc; i++) {
			int pix = *(data+i);
			if (abs(pix - *average) > 60)
			{
				num ++;
				nTotal += pix;
			}
		} // end of row                   
		data+= step;  // next line  
	}
	int nAvaDis = nTotal / num;

	*low = abs(*average - nAvaDis);
	*high = *low * aperture_size;
}    

/* ���ͼ���γɱպ���
*/
void ImageProcess::ConnectedComponents(Mat &mask_process, std::vector<CvSeq *> &vContours, int componentType, int area, int number,
	Rect &bounding_box, Point &contour_centers)
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
		double q = 100;//(mask->height + mask->width) /perimScale;   //calculate perimeter len threshold
		if( len < area ) //Get rid of blob if it's perimeter is too small
		{
			cvSubstituteContour( scanner, NULL );    //��NULL����ԭ�����Ǹ�����
		}
		else //Smooth it's edges if it's large enough
		{
			if (componentType == 0)
			{
				// ԭ�������
			}
			else
			{
				CvSeq* c_new = NULL;
				if(componentType == 1) //Polygonal approximation of the segmentation
			 		c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, 4,0);
				else if (componentType == 2)
					//Convex Hull of the segmentation
			 		c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
				cvSubstituteContour( scanner, c_new ); //�ʼ��������͹�����߶���ʽ��������滻
			}

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
			vContours.push_back(c);
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

// ��Ҷ�任����Ƶ��ת��
void ImageProcess::FourierTrans(Mat &image)
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

// ���ֱ��ͼ��Ϣ
void ImageProcess::CalculateHist(Mat &image, HistInfo &histInfo)
{
	MatND histogram;  
	//256������Χ��0��255.  
	const int histSize = 256;  
	float range[] = {0, 255};  
	const float *ranges[] = {range};  
	const int channels = 0;  

	// ����ֱ��ͼ
	calcHist(&image, 1, &channels, Mat(), histogram, 1, &histSize, &ranges[0], true, false);

	double minValue = 0;
	double maxValue = 0;
	int minValue_x = 0;
	int maxValue_x = 0;
	minMaxLoc(histogram, &minValue, &maxValue);
	histInfo.maxValue = maxValue;
	histInfo.minValue = minValue;
	histInfo.hist.create(histSize, histSize, CV_8U);
	for (int i = 0; i < 256; i ++)
	{
		int nPixCount = histogram.at<float>(i);
		histInfo.histArray[i] = nPixCount;
		int nHistValue = histSize * nPixCount / maxValue;
		cv::line(histInfo.hist, cv::Point(i, histSize - nHistValue), cv::Point(i, histSize), Scalar::all(0));
	}
}

/* USM��
@param image[in]: Դͼ��
@param sigma[in]: ��˹ģ���뾶��0.1-100pix��
@param nThreshold[in]: ����ֵ��0-255��
@param amount[int]: ��������1-5��
*/
void ImageProcess::USMSharp(Mat &image, double sigma, int nThreshold, float amount)
{
	Mat imgBlurred;
	GaussianBlur(image, imgBlurred, Size(), sigma, sigma);
	Mat lowContrastMask = abs(image - imgBlurred) < nThreshold;
	image = image * (1+amount)+imgBlurred*(-amount); 
	image.copyTo(image, lowContrastMask);
}
