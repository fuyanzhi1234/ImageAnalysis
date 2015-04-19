#ifndef IMAGEANALYSIS_H
#define IMAGEANALYSIS_H

#include <QtWidgets/QMainWindow>
#include "ui_imageanalysis.h"
#include <opencv2\opencv.hpp>
#include <opencv\cv.h>
#include "ImageProcess.h"

using namespace cv;

class ImageAnalysis : public QWidget
{
	Q_OBJECT

public:
	ImageAnalysis(QWidget *parent = 0);
	~ImageAnalysis();
	void SmoothImage(Mat src, int highThredhold);
	void ShowImage(Mat image, QLabel *imageLabel);

private slots:
	void OnShowImage();
	// 旋钮移动响应
	void OnValueChanged(int value);
	// USM选择
	void OnUsmStateChanged(int state);
	// 图像预处理
	void PreAnalysisImage();
	// 当前颗粒列表点击事件
	void OnRegionItemClicked(QListWidgetItem *item);

private:
	Ui::ImageAnalysisClass ui;
	Mat originImage;
	Mat grayImage;  
	double lowValue;
	double averValue;
};

#endif // IMAGEANALYSIS_H
