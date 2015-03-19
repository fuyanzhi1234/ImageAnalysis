#ifndef IMAGEANALYSIS_H
#define IMAGEANALYSIS_H

#include <QtWidgets/QMainWindow>
#include "ui_imageanalysis.h"
#include <opencv2\opencv.hpp>
#include <opencv\cv.h>
using namespace cv;

class ImageAnalysis : public QMainWindow
{
	Q_OBJECT

public:
	ImageAnalysis(QWidget *parent = 0);
	~ImageAnalysis();
	void FourierTrans(Mat &image);

private slots:
	void OnShowImage();

private:
	Ui::ImageAnalysisClass ui;
};

#endif // IMAGEANALYSIS_H
