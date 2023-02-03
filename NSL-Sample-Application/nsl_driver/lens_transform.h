/**************************************************************************************
* Copyright (C) 2019 Espros Photonics Corporation
*
* Open Source Software used:
***************************************************************************************/

#ifndef LENSTRANSFORM_H
#define LENSTRANSFORM_H

struct LensData
{
    double angle;
    double radius;
};

enum LensType { 
	WIDE_FIELD = 110, 
	STANDARD_FIELD = 90, 
	NARROW_FIELD = 50 
};

class LensTransform
{
public:
    LensTransform();
    ~LensTransform(){}
    void initTransformPolynom(double sensorPointSizeMM, int width, int height);
    void initTransformTable(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY);
	void initTransformNarrow(double sensorPointSizeMM, int width, int height, int offsetX, int offsetY); // seobi 수정 해야 함.
    void transformPixel(int srcX, int srcY, double srcZ, double &destX, double &destY, double &destZ,  double sin_angle = 0.0, double cos_angle = 1.0);
    void transformPixelPolynom(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ,  double sin_angle, double cos_angle);
    void transformPixelTable(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ,  double sin_angle, double cos_angle);
    void transformPixelCalib(unsigned int srcX, unsigned int srcY, double srcZ, double &destX, double &destY, double &destZ,  double sin_angle, double cos_angle);
    void initLensDistortionTable(enum LensType nLensType);

    void setLensData(int index, double rp, double angle);
//    void calibrateLensData(int *data);

private:
    int lensTableSize;
    int numCols;
    int numRows;

    double width2;
    double height2;
	LensType lensType;
	double  lensKoef;

    bool isTransformLoaded();
    double interpolate(double x_in, double x0, double y0, double x1, double y1);
    double getAngle(double x, double y, double sensorPointSizeMM);

//    void saveKoefFile(void);
//    void loadKoefFile(void);


};

#endif // LENSTRANSFORM_H
