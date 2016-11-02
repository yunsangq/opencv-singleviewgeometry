#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <fstream>
#define PI 3.14159265358979323846


using namespace cv;
using namespace std;

int x=0, y=0, col=0, row=0;
double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0,
	k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0;

void init() {
	vector<double>paramlist;
	double param = 0.0;
	ifstream infile("param.txt");
	while (infile >> param) {
		paramlist.push_back(param);
	}
	fx = paramlist[0];
	fy = paramlist[1];
	cx = paramlist[2];
	cy = paramlist[3];
	k1 = paramlist[4];
	k2 = paramlist[5];
	p1 = paramlist[6];
	p2 = paramlist[7];
}


void d_geometry(double u, double v) {
	double h = 100.0;
	double tilt = 0.0;
	double CC_ = h;
	double C_P_ = CC_*tan(PI / 2 + tilt - atan(v));
	double CP_ = sqrt(CC_*CC_ + C_P_*C_P_);
	double Cp_ = sqrt(1 + v*v);
	double PP_ = u*CP_ / Cp_;
	double d = sqrt(C_P_*C_P_ + PP_*PP_);
	double seta = -atan2(PP_, C_P_);
	double X = d*cos(seta);
	double Y = d*sin(seta);
	cout << "d_geometry world pos-> X = " << X << ", Y = " << Y
		<< ", d = " << d << ", seta = " << seta << endl;
}

void d_homography(double u, double v) {

}

void un_geometry(double u, double v) {
	double h = 100.0;
	double tilt = 0.0;
	double CC_ = h;
	double C_P_ = CC_*tan(PI / 2 + tilt - atan(v));
	double CP_ = sqrt(CC_*CC_ + C_P_*C_P_);
	double Cp_ = sqrt(1 + v*v);
	double PP_ = u*CP_ / Cp_;
	double d = sqrt(C_P_*C_P_ + PP_*PP_);
	double seta = -atan2(PP_, C_P_);
	double X = d*cos(seta);
	double Y = d*sin(seta);
	cout << "un_geometry world pos-> X = " << X << ", Y = " << Y
		<< ", d = " << d << ", seta = " << seta << endl;
}

void un_homography(double u, double v) {

}

void m_click(int evt, int _x, int _y, int flag, void* param) {
	if (evt == CV_EVENT_LBUTTONDOWN) {
		cout << "X = " << x << ", " << "Y = " << y << endl;
		x = _x;
		y = _y;

		double xn_u = (x - cx) / fx;
		double yn_u = (y - cy) / fy;

		d_geometry(xn_u, yn_u);
		d_homography(xn_u, yn_u);

		double ru2 = xn_u*xn_u + yn_u*yn_u;
		double radial_d = 1 + k1*ru2 + k2*ru2*ru2;

		double xn_d = radial_d*xn_u + 2 * p1*xn_u*yn_u + p2*(ru2 + 2 * xn_u*xn_u);
		double yn_d = radial_d*yn_u + p1*(ru2 + 2 * yn_u*yn_u) + 2 * p2*xn_u*yn_u;

		un_geometry(xn_d, yn_d);
		un_homography(xn_d, yn_d);
	}
}

int main() {
	init();

	Mat img = imread("sample.png");
	col = img.cols;
	row = img.rows;
	namedWindow("img");
	setMouseCallback("img", m_click);
	imshow("img", img);

	while (1) {
		if (waitKey(30) == 27) break;
	}	
	return 0;
}