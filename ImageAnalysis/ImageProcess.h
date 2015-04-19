#ifndef IMAGEPROCESS
#define IMAGEPROCESS
#include <opencv2\opencv.hpp>
#include <opencv\cv.h>
using namespace cv;
#define CV_CVX_WHITE    CV_RGB(0xff,0xff,0xff)
#define CV_CVX_BLACK    CV_RGB(0x00,0x00,0x00)

class ImageProcess
{
public:
	ImageProcess(void);
	~ImageProcess(void);

	struct HistInfo
	{
		int maxValue;
		int minValue;
		int maxValue_x;
		int minValue_x;
		int histArray[256];
		Mat hist;
		HistInfo()
		{
			maxValue = 0;
			minValue = 0;
			maxValue_x = -1;
			minValue_x = -1;
		}
	};

public:
	static ImageProcess * Instance();
	/* 计算Canny算法的两个阈值
	@param src[in]: 源图像
	@param low[out]: 低阈值
	@param high[out]: 高阈值
	@param aperture_size[int]:高阈值放大倍数 
	*/ 
	void AdaptiveFindThreshold(const cv::Mat src, double *low, double *high, double *average, int aperture_size = 3);

	/* 拟合图像形成闭合区
	@param mask_process[in]: 源图像
	@param componentType[in]: 拟合类型 1、多边形；2、曲线；3、原曲线拟合
	@param area[in]: 面积阈值
	@param number[int]: 
	*/
	void ConnectedComponents(Mat &mask_process, std::vector<CvSeq *> &vContours, int componentType, int area, int number = 0,
		Rect &bounding_box = Rect(), Point &contour_centers = Point(-1, -1));

	// 傅叶变换，向频域转换
	void FourierTrans(Mat &image);

	// 获得直方图信息
	void CalculateHist(Mat &image, HistInfo &histInfo);
	
	/* USM锐化
	@param image[in]: 源图像
	@param sigma[in]: 高斯模糊半径（0.1-100pix）
	@param nThreshold[in]: 锐化阈值（0-255）
	@param amount[int]: 锐化数量（1-5）
	*/
	void USMSharp(Mat &image, double sigma = 4.5, int nThreshold = 10, float amount = 3);

private:
};

#endif
