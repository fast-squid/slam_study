#include "opencv2/opencv.hpp"
#include "fstream"

int main(int argc, char* argv[])
{
    const char* input = argv[1];
    cv::Size board_pattern(8, 6);
    float board_cellsize = 0.025f;
    bool select_images = true;

    // Open a video
    cv::VideoCapture video;
    if (!video.open(input))
	{
		std::cout<<"no video input"<<std::endl;
		return -1;
	}

    // Select images
    std::vector<cv::Mat> images;
    while (true)
    {
        // Grab an image from the video
        cv::Mat image;
        video >> image;
		image.resize(960,540);
        if (image.empty()) break;

        if (select_images)
        {
            // Show the image and keep it if selected
            cv::imshow("3DV Tutorial: Camera Calibration", image);
            int key = cv::waitKey(1);
            if (key == 27) break;                               // 'ESC' key: Exit
            else if (key == 32)                                 // 'Space' key: Pause
            {
                std::vector<cv::Point2f> pts;
                bool complete = cv::findChessboardCorners(image, board_pattern, pts);
                cv::Mat display = image.clone();
                cv::drawChessboardCorners(display, board_pattern, pts, complete);
                cv::imshow("3DV Tutorial: Camera Calibration", display);
                key = cv::waitKey();
                if (key == 27) break;                           // 'ESC' key: Exit
                else if (key == 10)
				{
					std::cout<<"push image"<<std::endl;
					images.push_back(image);    // 'Enter' key: Select
				}
            }
        }
        else images.push_back(image);
    }
    video.release();
	std::cout<<"1"<<std::endl;

	if (images.empty())
	{
		std::cout<<"no image input"<<std::endl;
		return -1;
	}

	// Find 2D corner points from the given images
	std::vector<std::vector<cv::Point2f>> img_points[2];
	for (size_t i = 0; i < images.size(); i++)
	{
		std::vector<cv::Point2f> pts;
		if (cv::findChessboardCorners(images[i], board_pattern, pts))
			img_points[i%2].push_back(pts);
	}


	// Prepare 3D points of the chess board
	std::vector<std::vector<cv::Point3f>> obj_points(1);
	for (int r = 0; r < board_pattern.height; r++)
		for (int c = 0; c < board_pattern.width; c++)
			obj_points[0].push_back(cv::Point3f(board_cellsize * c, board_cellsize * r, 0));

	obj_points.resize(img_points[0].size(), obj_points[0]); // Copy
	obj_points.resize(img_points[1].size(), obj_points[0]); // Copy

	//std::cout << "3d_points : " << obj_points[0] << std::endl;
	//std::cout << "2d_points : " << img_points[0] << std::endl;
	//std::cout << "2d_points : " << img_points[1] << std::endl;



	// Calibrate the camera
	cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat dist_coeff = cv::Mat::zeros(4, 1, CV_64F);
	std::vector<cv::Mat> rvecs, tvecs;
	//double rms = cv::calibrateCamera(obj_points, img_points, images[0].size(), K, dist_coeff, rvecs, tvecs);



	cv::Mat K1 = cv::Mat::eye(3,3,CV_64F);
	cv::Mat K2 = cv::Mat::eye(3,3,CV_64F);
	cv::Mat R, T;
	cv::Mat E, F;
	cv::Mat dist_coeff1 = cv::Mat::zeros(4, 1, CV_64F);
	cv::Mat dist_coeff2 = cv::Mat::zeros(4, 1, CV_64F);

	std::cout << "obj_points" << obj_points[0] << std::endl;
	std::cout << "img_points" << img_points[0][0] << std::endl;
	std::cout << "img_points" << img_points[1][0] << std::endl;


	cv::stereoCalibrate(obj_points, img_points[0], img_points[1], K1, dist_coeff1, K2, dist_coeff2, images[0].size(), R, T, E, F, cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_SAME_FOCAL_LENGTH, cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5));
	std::cout << "K           :" << std::endl << K1 << std::endl;
	std::cout << "dist        :" << std::endl<< dist_coeff << std::endl;

	std::cout << "Rotation    :"  << std::endl << R << std::endl;
	std::cout << "Translation :"  << std::endl << T << std::endl; 
	std::cout << "Essential   :"  << std::endl << E << std::endl; 
	std::cout << "Fundamental :"  << std::endl << F << std::endl;


	cv::Mat R1, R2, P1, P2;
	cv::stereoRectify(K1,dist_coeff1, K2, dist_coeff2,images[0].size(), R, T, R1, R2, P1, P2,
			cv::noArray(), 0);


	cv::Mat map11, map12, map21, map22;
	cv::initUndistortRectifyMap(K1, dist_coeff1, R1, P1, images[0].size(), CV_32FC1, map11,
			map12);
	cv::initUndistortRectifyMap(K2, dist_coeff2, R2, P2, images[0].size(), CV_32FC1, map21,
			map22);

	cv::Mat pair;
    pair.create(images[0].rows, images[0].cols , CV_8UC3);


    // Setup for finding stereo corrrespondences
	//
	cv::Ptr<cv::StereoSGBM> stereo = cv::StereoSGBM::create(
			-64, 128, 11, 100, 1000, 32, 0, 15, 1000, 16, cv::StereoSGBM::MODE_HH);
	int nframes = (int)obj_points.size();
	std::cout << nframes << std::endl;
	for (int i = 0; i < 1; i++) 
	{
		cv::Mat img1 = images[0];
		cv::Mat img2 = images[1];
		cv::Mat img1r, img2r, disp, vdisp;
		if (img1.empty() || img2.empty())
			continue;
		cv::remap(img1, img1r, map11, map12, cv::INTER_LINEAR);
		cv::remap(img2, img2r, map21, map22, cv::INTER_LINEAR);
		stereo->compute(img1r, img2r, disp);
		cv::normalize(disp, vdisp, 0, 256, cv::NORM_MINMAX,CV_8U);
		cv::namedWindow("disp", cv::WINDOW_NORMAL);
		cv::namedWindow("org1", cv::WINDOW_NORMAL);
		cv::namedWindow("org2", cv::WINDOW_NORMAL);
		cv::namedWindow("rect1",cv::WINDOW_NORMAL);
		cv::namedWindow("rect2",cv::WINDOW_NORMAL);

		cv::resizeWindow("disp",980,540); 
		cv::resizeWindow("org1",980,540);
		cv::resizeWindow("org2",980,540); 
		cv::resizeWindow("rect1",980,540);
		cv::resizeWindow("rect2",980,540);


		cv::imshow("disp",vdisp);
		cv::imshow("org1",img1);
		cv::imshow("org2",img2);
		cv::imshow("rect1",img1r);
		cv::imshow("rect2",img2r);

	}
	cv::waitKey();




	// Report calibration results
	std::ofstream report("camera_calibration.txt");
	if (!report.is_open())
	{
		std::cout<<"report is not opened"<<std::endl;
		return -1;
	}
	for(int i=0;i<3;i++)
	{
		std::cout<<"[";
		for(int j=0;j<3;j++)
		{
			std::cout<< K.at<double>(i,j)<<" ";
		}
		std::cout<<"]"<<std::endl;
	}
    report << "* Distortion coefficient (k1, k2, p1, p2, k3, ...) = " << std::endl << "  " << dist_coeff.t() << std::endl;
    report.close();


    return 0;
}
