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
	/* ����Canny�㷨��������ֵ
	@param src[in]: Դͼ��
	@param low[out]: ����ֵ
	@param high[out]: ����ֵ
	@param aperture_size[int]:����ֵ�Ŵ��� 
	*/ 
	void AdaptiveFindThreshold(const cv::Mat src, double *low, double *high, double *average, int aperture_size = 3);

	/* ���ͼ���γɱպ���
	@param mask_process[in]: Դͼ��
	@param componentType[in]: ������� 1������Σ�2�����ߣ�3��ԭ�������
	@param area[in]: �����ֵ
	@param number[int]: 
	*/
	void ConnectedComponents(Mat &mask_process, std::vector<CvSeq *> &vContours, int componentType, int area, int number = 0,
		Rect &bounding_box = Rect(), Point &contour_centers = Point(-1, -1));

	// ��Ҷ�任����Ƶ��ת��
	void FourierTrans(Mat &image);

	// ���ֱ��ͼ��Ϣ
	void CalculateHist(Mat &image, HistInfo &histInfo);
	
	/* USM��
	@param image[in]: Դͼ��
	@param sigma[in]: ��˹ģ���뾶��0.1-100pix��
	@param nThreshold[in]: ����ֵ��0-255��
	@param amount[int]: ��������1-5��
	*/
	void USMSharp(Mat &image, double sigma = 4.5, int nThreshold = 10, float amount = 3);

private:
};

#endif
