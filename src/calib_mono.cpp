#include "opencv2/opencv.hpp"
#include "fstream"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class CalibParameters
{
public:
	vector<vector<cv::Point2f>> points_2d;
	vector<vector<cv::Point3f>> points_3d;	
	vector<cv::Mat> images;
};

VectorXf SetVector(int i, int j, MatrixXf H)
{
	VectorXf v(6);
	H.resize(3,3);
	v[0] = H(0,i)*H(0,j);
	v[1] = H(0,i)*H(1,j) + H(1,i)*H(0,j);
	v[2] = H(2,i)*H(0,j) + H(0,i)*H(2,j);
	v[3] = H(1,i)*H(1,j);
	v[4] = H(2,i)*H(1,j) + H(1,i)*H(2,j);
	v[5] = H(2,i)*H(2,j);
	return v;
}

void Calibration(CalibParameters calib_params)
{
	if(calib_params.images.size()<3)
	{
		std::cerr<<"at least 3 images needed"<<std::endl;
		exit(1);
	}
	Eigen::Matrix<float, 6,6> V;

	for(int image_iter = 0; image_iter < calib_params.images.size(); image_iter++)
	{
		Eigen::Matrix<float, 10, 9> P;
		vector<cv::Point3f> points_3d = calib_params.points_3d[image_iter];
		vector<cv::Point2f> points_2d = calib_params.points_2d[image_iter];

		int point_idx[5] = {0,1,10,9,15};
		for(int i=0;i<5;i++)
		{
			P(i*2+0, 0) = -1*points_3d[point_idx[i]].x;
			P(i*2+0, 1) = -1*points_3d[point_idx[i]].y;
			P(i*2+0, 2) = -1;
			P(i*2+0, 3) = 0;
			P(i*2+0, 4) = 0;
			P(i*2+0, 5) = 0;
			P(i*2+0, 6) = points_2d[point_idx[i]].x*points_3d[point_idx[i]].x;
			P(i*2+0, 7) = points_2d[point_idx[i]].x*points_3d[point_idx[i]].y;
			P(i*2+0, 8) = points_2d[point_idx[i]].x;

			P(i*2+1, 0) = 0;
			P(i*2+1, 1) = 0;
			P(i*2+1, 2) = 0;
			P(i*2+1, 3) = -1*points_3d[point_idx[i]].x;
			P(i*2+1, 4) = -1*points_3d[point_idx[i]].y;
			P(i*2+1, 5) = -1;
			P(i*2+1, 6) = points_2d[point_idx[i]].y*points_3d[point_idx[i]].x;
			P(i*2+1, 7) = points_2d[point_idx[i]].y*points_3d[point_idx[i]].y;
			P(i*2+1, 8) = points_2d[point_idx[i]].y;
		}

		Eigen::JacobiSVD<Eigen::MatrixXf> jsvd(P, ComputeThinU | ComputeThinV); 
		Eigen::MatrixXf H(3,3),temp;
		temp = jsvd.matrixV();

		int idx =0;
		for(int r=0;r<H.rows();r++)
		{
			for(int c=0;c<H.cols();c++)
			{
				H(r,c) = temp(idx++,temp.cols()-1);
			}
		}
		//std::cout << "V : " << temp<<std::endl;
		//std::cout << "H : " << H << std::endl;
		H.resize(3,3);

		Eigen::VectorXf v;
		v=SetVector(0,1,H);
		for(int i=0;i<6;i++)
		{
			V(image_iter*2+0,i) = v[i];
		}
		v=SetVector(0,0,H)-SetVector(1,1,H);
		for(int i=0;i<6;i++)
		{
			V(image_iter*2+1,i) = v[i];
		}
	}	
	std::cout << V << std::endl;
	Eigen::JacobiSVD<Eigen::MatrixXf> jsvd(V, ComputeFullU| ComputeFullV);
	Eigen::MatrixXf B(3,3),temp;
	temp = jsvd.matrixV();
	int idx = 0;
	for(int r=0;r<B.rows();r++)
	{
		for(int c = r; c<B.cols();c++)
		{
			B(r,c) = temp(idx++,temp.cols()-1);
		}
	}

	B(1,0) = B(0,1);
	B(2,0) = B(0,2); B(2,1) = B(1,2);
	std::cout <<"temp : " <<temp<<std::endl;
	std::cout <<"B    : " << B << std::endl; 

	Eigen::LLT<MatrixXf> llt(B);
	Eigen::MatrixXf L = llt.matrixL();
	std::cout<<"L" << L << std::endl;



}

int main(int argc, char* argv[])
{
	const char* video_name = argv[1];
	
	cv::Size board_pattern(10,7); 	// Size(Points Per Row, Points for Column)
	float board_cellsize = 0.025f;
	bool select_images = true;

	// Open Video
	cv::VideoCapture video;
	if(!video.open(video_name))
	{
		std::cerr<<"video input error"<<std::endl;
		return -1;
	}

	CalibParameters calib_params;
	
	while(true)
	{
		cv::Mat img;
		video >> img;
		if(img.empty()) break;

		if(select_images)
		{
			cv::imshow("window",img);
			int key = cv::waitKey(1); 
			if(key == 27) break;	// 'ESC'	: Exit
			else if(key == 32) 		// 'Space' 	: Pause 
			{
				bool status;
				vector<cv::Point2f> points;
				status = cv::findChessboardCorners(img, board_pattern, points, status);
				cv::drawChessboardCorners(img, board_pattern, points, status); // draw all points
				cv::imshow("window", img);

				key = cv::waitKey();
				std::cout<<"found points : "<< points.size() << std::endl;
				if(key == 27) break;
				else if(key == 10)
				{
					calib_params.points_2d.push_back(points);
					calib_params.images.push_back(img);	
				}
			}
		}
	}
	video.release();
	for(int image_iter = 0; image_iter < calib_params.images.size(); image_iter++)
	{
		vector<cv::Point3f> points_3d;
		for(int r = 0;r<board_pattern.height;r++)
		{
			for(int c = 0; c< board_pattern.width; c++)
			{
				points_3d.push_back(cv::Point3f(board_cellsize*c, board_cellsize*r,0));
			}
		}
		calib_params.points_3d.push_back(points_3d);
	}

	Calibration(calib_params);
	



	
	return 0;
}
