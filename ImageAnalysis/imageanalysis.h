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
	// ��ť�ƶ���Ӧ
	void OnValueChanged(int value);
	// USMѡ��
	void OnUsmStateChanged(int state);
	// ͼ��Ԥ����
	void PreAnalysisImage();
	// ��ǰ�����б����¼�
	void OnRegionItemClicked(QListWidgetItem *item);

private:
	Ui::ImageAnalysisClass ui;
	Mat originImage;
	Mat grayImage;  
	double lowValue;
	double averValue;
};

#endif // IMAGEANALYSIS_H
