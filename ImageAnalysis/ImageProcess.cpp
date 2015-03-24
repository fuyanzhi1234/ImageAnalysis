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

/* 计算Canny算法的两个阈值
@param src[in]: 源图像
@param low[out]: 低阈值
@param high[out]: 高阈值
@param aperture_size[int]:高阈值放大倍数 
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

/* 拟合图像形成闭合区
*/
void ImageProcess::ConnectedComponents(Mat &mask_process, int componentType, int area, int number,
	Rect &bounding_box, Point &contour_centers)
{
	/*下面4句代码是为了兼容原函数接口，即内部使用的是c风格，但是其接口是c++风格的*/
	IplImage *mask = &mask_process.operator IplImage();
	int *num = &number;
	CvRect *bbs = &bounding_box.operator CvRect();
	CvPoint *centers = &contour_centers.operator CvPoint();
	static CvMemStorage*    mem_storage    = NULL;
	static CvSeq*            contours    = NULL;
	//CLEAN UP RAW MASK
	//开运算作用：平滑轮廓，去掉细节,断开缺口
	// 	cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_OPEN, 1 );//对输入mask进行开操作，CVCLOSE_ITR为开操作的次数，输出为mask图像
	//闭运算作用：平滑轮廓，连接缺口
	// 	cvMorphologyEx( mask, mask, NULL, NULL, CV_MOP_CLOSE, 1 );//对输入mask进行闭操作，CVCLOSE_ITR为闭操作的次数，输出为mask图像

	//FIND CONTOURS AROUND ONLY BIGGER REGIONS
	if( mem_storage==NULL ) mem_storage = cvCreateMemStorage(0);
	else cvClearMemStorage(mem_storage);
	//CV_RETR_EXTERNAL=0是在types_c.h中定义的，CV_CHAIN_APPROX_SIMPLE=2也是在该文件中定义的
	CvContourScanner scanner = cvStartFindContours(mask,mem_storage,sizeof(CvContour),CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
	CvSeq* c;
	int numCont = 0;
	//该while内部只针对比较大的轮廓曲线进行替换处理
	while( (c = cvFindNextContour( scanner )) != NULL )
	{
		double len = cvContourArea( c );
		double q = 100;//(mask->height + mask->width) /perimScale;   //calculate perimeter len threshold
		if( len < area ) //Get rid of blob if it's perimeter is too small
		{
			cvSubstituteContour( scanner, NULL );    //用NULL代替原来的那个轮廓
		}
		else //Smooth it's edges if it's large enough
		{
			if (componentType == 0)
			{
				// 原曲线拟合
			}
			else
			{
				CvSeq* c_new = NULL;
				if(componentType == 1) //Polygonal approximation of the segmentation
			 		c_new = cvApproxPoly(c,sizeof(CvContour),mem_storage,CV_POLY_APPROX_DP, 4,0);
				else if (componentType == 2)
					//Convex Hull of the segmentation
			 		c_new = cvConvexHull2(c,mem_storage,CV_CLOCKWISE,1);
				cvSubstituteContour( scanner, c_new ); //最开始的轮廓用凸包或者多项式拟合曲线替换
			}

			numCont++;
		}
	}
	contours = cvEndFindContours( &scanner );    //结束轮廓查找操作
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
		for(i=0, c=contours; c != NULL; c = c->h_next,i++ )        //h_next为轮廓序列中的下一个轮廓
		{
			if(i < N) //Only process up to *num of them
			{
				//CV_CVX_WHITE在本程序中是白色的意思
				cvDrawContours(maskTemp,c,CV_CVX_WHITE, CV_CVX_WHITE,-1,CV_FILLED,8);
				//Find the center of each contour
				if(centers != &cvPoint(-1, -1))
				{
					cvMoments(maskTemp,&moments,1);    //计算mask图像的最高达3阶的矩
					M00 = cvGetSpatialMoment(&moments,0,0); //提取x的0次和y的0次矩
					M10 = cvGetSpatialMoment(&moments,1,0); //提取x的1次和y的0次矩
					M01 = cvGetSpatialMoment(&moments,0,1); //提取x的0次和y的1次矩
					centers[i].x = (int)(M10/M00);    //利用矩的结果求出轮廓的中心点坐标
					centers[i].y = (int)(M01/M00);
				}
				//Bounding rectangles around blobs
				if(bbs != &CvRect())
				{
					bbs[i] = cvBoundingRect(c); //算出轮廓c的外接矩形
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

// 傅叶变换，向频域转换
void ImageProcess::FourierTrans(Mat &image)
{
	cvtColor(image,image,CV_RGB2GRAY);
	image.convertTo(image,CV_32FC1);
	for(int i=0; i<image.rows; i++)        //中心化
	{
		float *p = image.ptr<float>(i);
		for(int j=0; j<image.cols; j++)
		{
			p[j] = p[j] * pow((double)-1, i+j);
		}
	}
	// 	imshow("src",image);

	/////////////////////////////////////快速傅里叶变换/////////////////////////////////////////////////////
	int oph = getOptimalDFTSize(image.rows);
	int opw = getOptimalDFTSize(image.cols);
	Mat padded;
	copyMakeBorder(image, padded, 0, oph-image.rows, 0, opw-image.cols, BORDER_CONSTANT, Scalar::all(0));

	Mat temp[] = {padded, Mat::zeros(image.size(),CV_32FC1)};
	Mat complexI;
	merge(temp, 2, complexI);

	dft(complexI, complexI);    //傅里叶变换

	//显示频谱图
	split(complexI, temp);
	Mat aa;
	magnitude(temp[0], temp[1], aa);
	divide(aa, oph*opw, aa);
	imshow("aa",aa);

	///////////////////////////////////////////频域滤波///////////////////////////////////////////////////////
	//生成频域滤波核
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

	//高斯低通滤波， 高斯高通滤波
	multiply(complexI, gaussianBlur, gaussianBlur);
	multiply(complexI, gaussianSharpen, gaussianSharpen);

	//傅里叶反变换
	dft(gaussianBlur, gaussianBlur, CV_DXT_INVERSE);
	dft(gaussianSharpen, gaussianSharpen, CV_DXT_INVERSE);

	Mat dstBlur[2], dstSharpen[2];
	split(gaussianBlur, dstBlur);
	split(gaussianSharpen, dstSharpen);

	for(int i=0; i<oph; i++)        //中心化
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