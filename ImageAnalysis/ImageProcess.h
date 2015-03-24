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
	void ConnectedComponents(Mat &mask_process, int componentType, int area, int number = 0,
		Rect &bounding_box = Rect(), Point &contour_centers = Point(-1, -1));

	// ��Ҷ�任����Ƶ��ת��
	void FourierTrans(Mat &image);

private:
};

#endif
