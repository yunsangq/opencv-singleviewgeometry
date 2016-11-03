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


void homography_from_4pt(const float *x, const float *y, const float *z, const float *w, float h[8])
{
	float t1 = x[0];   float t2 = z[0];  float t4 = y[1];  float t5 = t1 * t2 * t4;
	float t6 = w[1];  float t7 = t1 * t6;  float t8 = t2 * t7;  float t9 = z[1];
	float t10 = t1 * t9;  float t11 = y[0];  float t14 = x[1];  float t15 = w[0];
	float t16 = t14 * t15;  float t18 = t16 * t11;  float t20 = t15 * t11 * t9;
	float t21 = t15 * t4;  float t24 = t15 * t9;  float t25 = t2 * t4;
	float t26 = t6 * t2;  float t27 = t6 * t11;  float t28 = t9 * t11;
	float t30 = 0.1e1 / (-t24 + t21 - t25 + t26 - t27 + t28);
	float t32 = t1 * t15;  float t35 = t14 * t11;  float t41 = t4 * t1;
	float t42 = t6 * t41;  float t43 = t14 * t2;  float t46 = t16 * t9;
	float t48 = t14 * t9 * t11;  float t51 = t4 * t6 * t2;  float t55 = t6 * t14;

	h[0] = -(-t5 + t8 + t10 * t11 - t11 * t7 - t16 * t2 + t18 - t20 + t21 * t2) * t30;
	h[1] = (t5 - t8 - t32 * t4 + t32 * t9 + t18 - t2 * t35 + t27 * t2 - t20) * t30;
	h[2] = t1;
	h[3] = (-t9 * t7 + t42 + t43 * t4 - t16 * t4 + t46 - t48 + t27 * t9 - t51) * t30;
	h[4] = (-t42 + t41 * t9 - t55 * t2 + t46 - t48 + t55 * t11 + t51 - t21 * t9) * t30;
	h[5] = t14;
	h[6] = (-t10 + t41 + t43 - t35 + t24 - t21 - t26 + t27) * t30;
	h[7] = (-t7 + t10 + t16 - t43 + t27 - t28 - t21 + t25) * t30;
}

void homography_from_4corresp(
	const float *p1, const float *p2, const float *p3, const float *p4,
	const float *q1, const float *q2, const float *q3, const float *q4, float H[3][3])
{
	float Hr[3][3], Hl[3][3];

	homography_from_4pt(p1, p2, p3, p4, &Hr[0][0]);
	homography_from_4pt(q1, q2, q3, q4, &Hl[0][0]);

	// the following code computes R = Hl * inverse Hr
	float t2 = Hr[1][1] - Hr[2][1] * Hr[1][2];
	float t4 = Hr[0][0] * Hr[1][1];
	float t5 = Hr[0][0] * Hr[1][2];
	float t7 = Hr[1][0] * Hr[0][1];
	float t8 = Hr[0][2] * Hr[1][0];
	float t10 = Hr[0][1] * Hr[2][0];
	float t12 = Hr[0][2] * Hr[2][0];
	float t15 = 1 / (t4 - t5*Hr[2][1] - t7 + t8*Hr[2][1] + t10*Hr[1][2] - t12*Hr[1][1]);
	float t18 = -Hr[1][0] + Hr[1][2] * Hr[2][0];
	float t23 = -Hr[1][0] * Hr[2][1] + Hr[1][1] * Hr[2][0];
	float t28 = -Hr[0][1] + Hr[0][2] * Hr[2][1];
	float t31 = Hr[0][0] - t12;
	float t35 = Hr[0][0] * Hr[2][1] - t10;
	float t41 = -Hr[0][1] * Hr[1][2] + Hr[0][2] * Hr[1][1];
	float t44 = t5 - t8;
	float t47 = t4 - t7;
	float t48 = t2*t15;
	float t49 = t28*t15;
	float t50 = t41*t15;
	H[0][0] = Hl[0][0] * t48 + Hl[0][1] * (t18*t15) - Hl[0][2] * (t23*t15);
	H[0][1] = Hl[0][0] * t49 + Hl[0][1] * (t31*t15) - Hl[0][2] * (t35*t15);
	H[0][2] = -Hl[0][0] * t50 - Hl[0][1] * (t44*t15) + Hl[0][2] * (t47*t15);
	H[1][0] = Hl[1][0] * t48 + Hl[1][1] * (t18*t15) - Hl[1][2] * (t23*t15);
	H[1][1] = Hl[1][0] * t49 + Hl[1][1] * (t31*t15) - Hl[1][2] * (t35*t15);
	H[1][2] = -Hl[1][0] * t50 - Hl[1][1] * (t44*t15) + Hl[1][2] * (t47*t15);
	H[2][0] = Hl[2][0] * t48 + Hl[2][1] * (t18*t15) - t23*t15;
	H[2][1] = Hl[2][0] * t49 + Hl[2][1] * (t31*t15) - t35*t15;
	H[2][2] = -Hl[2][0] * t50 - Hl[2][1] * (t44*t15) + t47*t15;
}

double h = 100.0;
double tilt = 0.0;

void d_geometry(double u, double v) {
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