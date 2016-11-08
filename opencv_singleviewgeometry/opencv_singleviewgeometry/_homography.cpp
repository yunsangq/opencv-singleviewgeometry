#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
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

vector<Point2f> obj;
vector<Point2f> d_scene;
vector<Point2f> und_scene;
Mat d_H, und_H;
int cnt = 0;

void m_click(int evt, int _x, int _y, int flag, void* param) {
	if (evt == CV_EVENT_LBUTTONDOWN) {
		int x = 0, y = 0;
		x = _x;
		y = _y;
		cout << "Pixel X = " << x << ", " << "Pixel Y = " << y << endl;

		float xn_u = (x - cx) / fx;
		float yn_u = (y - cy) / fy;
		
		float ru2 = xn_u*xn_u + yn_u*yn_u;
		float radial_d = 1 + k1*ru2 + k2*ru2*ru2;

		float xn_d = radial_d*xn_u + 2 * p1*xn_u*yn_u + p2*(ru2 + 2 * xn_u*xn_u);
		float yn_d = radial_d*yn_u + p1*(ru2 + 2 * yn_u*yn_u) + 2 * p2*xn_u*yn_u;

		if (cnt < 3) {
			d_scene.push_back(Point2f(xn_u, yn_u));
			und_scene.push_back(Point2f(xn_d, yn_d));
		}
		else if (cnt == 3) {
			d_scene.push_back(Point2f(xn_u, yn_u));
			und_scene.push_back(Point2f(xn_d, yn_d));
			d_H = findHomography(d_scene, obj);
			und_H = findHomography(und_scene, obj);
		}
		else if (cnt >= 4) {
			float d_a = (xn_u * d_H.at<double>(0, 0) + yn_u * d_H.at<double>(0, 1) + d_H.at<double>(0, 2)) / (xn_u * d_H.at<double>(2, 0) + yn_u * d_H.at<double>(2, 1) + d_H.at<double>(2, 2));
			float d_b = (xn_u * d_H.at<double>(1, 0) + yn_u * d_H.at<double>(1, 1) + d_H.at<double>(1, 2)) / (xn_u * d_H.at<double>(2, 0) + yn_u * d_H.at<double>(2, 1) + d_H.at<double>(2, 2));
			float d_d = sqrt(d_a*d_a + d_b*d_b);
			float d_seta = -atan2(d_b, d_a);

			float und_a = (xn_d * und_H.at<double>(0, 0) + yn_d * und_H.at<double>(0, 1) + und_H.at<double>(0, 2)) / (xn_d * und_H.at<double>(2, 0) + yn_d * und_H.at<double>(2, 1) + und_H.at<double>(2, 2));
			float und_b = (xn_d * und_H.at<double>(1, 0) + yn_d * und_H.at<double>(1, 1) + und_H.at<double>(1, 2)) / (xn_d * und_H.at<double>(2, 0) + yn_d * und_H.at<double>(2, 1) + und_H.at<double>(2, 2));
			float und_d = sqrt(und_a*und_a + und_b*und_b);
			float und_seta = -atan2(und_b, und_a);
			cout << "D_World pos X = " << d_a << ", " << " Y = " << d_b << 
				", d = " << d_d << ", seta = " << d_seta * 180 / 3.1415926 << endl;
			cout << "UND_World pos X = " << und_a << ", " << " Y = " << und_b <<
				", d = " << und_d << ", seta = " << und_seta * 180 / 3.1415926 << endl;
		}
		cnt++;
		
	}
}

int main() {
	init();
	obj.push_back(Point2f(60, 0));
	obj.push_back(Point2f(74, 0));
	obj.push_back(Point2f(74, 7));
	obj.push_back(Point2f(60, 7));

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