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
	std::vector<std::vector<cv::Point2f>> img_points;
	for (size_t i = 0; i < images.size(); i++)
	{
		std::vector<cv::Point2f> pts;
		if (cv::findChessboardCorners(images[i], board_pattern, pts))
			img_points.push_back(pts);
	}


	// Prepare 3D points of the chess board
	std::vector<std::vector<cv::Point3f>> obj_points(1);
	for (int r = 0; r < board_pattern.height; r++)
		for (int c = 0; c < board_pattern.width; c++)
			obj_points[0].push_back(cv::Point3f(board_cellsize * c, board_cellsize * r, 0));

	obj_points.resize(img_points.size(), obj_points[0]); // Copy


	// Calibrate the camera
	cv::Mat K = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat dist_coeff = cv::Mat::zeros(4, 1, CV_64F);
	std::vector<cv::Mat> rvecs, tvecs;
	double rms = cv::calibrateCamera(obj_points, img_points, images[0].size(), K, dist_coeff, rvecs, tvecs);

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
