#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

float fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0,
k1 = 0.0, k2 = 0.0, p1 = 0.0, p2 = 0.0;

void init() {
	vector<float>paramlist;
	float param = 0.0;
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

float h = 54.0;
float tilt = -45 / 180.0 * 3.1415926;

void geometry(float u, float v) {
	float CC_ = h;
	float C_P_ = CC_*tan(3.1415926 / 2.0 + tilt - atan(v));
	float CP_ = sqrt(CC_*CC_ + C_P_*C_P_);
	float Cp_ = sqrt(1 + v*v);
	float PP_ = u*CP_ / Cp_;
	float d = sqrt(C_P_*C_P_ + PP_*PP_);
	float seta = -atan2(PP_, C_P_);
	float X = d*cos(seta);
	float Y = d*sin(seta);
	cout << "d_geometry world pos-> X = " << X << ", Y = " << Y
		<< ", d = " << d << ", seta = " << seta * 180 / 3.1415926 << endl;
}

void un_geometry(float u, float v) {
	float CC_ = h;
	float C_P_ = CC_*tan(3.1415926 / 2.0 + tilt - atan(v));
	float CP_ = sqrt(CC_*CC_ + C_P_*C_P_);
	float Cp_ = sqrt(1 + v*v);
	float PP_ = u*CP_ / Cp_;
	float d = sqrt(C_P_*C_P_ + PP_*PP_);
	float seta = -atan2(PP_, C_P_);
	float X = d*cos(seta);
	float Y = d*sin(seta);
	cout << "un_geometry world pos-> X = " << X << ", Y = " << Y
		<< ", d = " << d << ", seta = " << seta * 180 / 3.1415926 << endl;
}

void m_click(int evt, int _x, int _y, int flag, void* param) {
	if (evt == CV_EVENT_LBUTTONDOWN) {
		int x = 0, y = 0;
		x = _x;
		y = _y;
		cout << "Pixel X = " << x << ", " << "Pixel Y = " << y << endl;

		float xn_u = (x - cx) / fx;
		float yn_u = (y - cy) / fy;

		geometry(xn_u, yn_u);

		float ru2 = xn_u*xn_u + yn_u*yn_u;
		float radial_d = 1 + k1*ru2 + k2*ru2*ru2;

		float xn_d = radial_d*xn_u + 2 * p1*xn_u*yn_u + p2*(ru2 + 2 * xn_u*xn_u);
		float yn_d = radial_d*yn_u + p1*(ru2 + 2 * yn_u*yn_u) + 2 * p2*xn_u*yn_u;

		un_geometry(xn_d, yn_d);
	}
}

int main() {
	init();

	VideoCapture vc(0);
	if (!vc.isOpened()) return 0;

	Mat input;
	while (1) {
		vc >> input;
		if (input.empty()) break;
		int col = input.cols;
		int row = input.rows;
		setMouseCallback("cam", m_click);

		Point center(col / 2, row / 2);
		Scalar color(0, 0, 255);
		circle(input, center, 5, color, CV_FILLED);



		imshow("cam", input);
		if (waitKey(33) == 27) break;
	}
	destroyAllWindows();
	return 0;
}