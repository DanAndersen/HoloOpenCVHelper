#include <opencv2\core\core.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\video.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>

extern "C" {
	__declspec(dllexport) int getInt() {
		return 10;
	}

	__declspec(dllexport) int getSize() {
		cv::Mat m(100, 100, CV_8UC3);
		return m.cols * m.rows;
	}

	class chessPoseController {
	public:
		chessPoseController() {
			rows = 0;
			cols = 0;
			image = NULL;
			got_valid_pose = false;
		}

		void setImage(uchar* imageData) {
			delete image;
			image = new cv::Mat(rows, cols, CV_8UC4, imageData, cols * 4);
			cv::flip(*image, *image, 1);
			cv::cvtColor(*image, gray, CV_BGRA2GRAY);
		}

		void setSize(int row, int col) {
			rows = row;
			cols = col;
		}

		void detect() {
			// do nothing
		}

		bool findExtrinsics(
			int chessX, int chessY, float chessSquareMeters,
			double cam_mtx_fx, double cam_mtx_fy, double cam_mtx_cx, double cam_mtx_cy,
			double dist_k1, double dist_k2, double dist_p1, double dist_p2, double dist_k3) {

			bool success = false;

			//pointBuf.clear();
			//bool found = cv::findChessboardCorners(gray, patternSize, pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

			//std::vector<cv::Point2f> pointBuf;
			//cv::Size pSize(chessX, chessY);
			pSize.width = chessX;
			pSize.height = chessY;
			bool found = cv::findChessboardCorners(gray, pSize, pointBuf, CV_CALIB_CB_FAST_CHECK);

			if (found) {
				cv::cornerSubPix(gray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

				
				cv::Mat cameraMatrix(cv::Size(3, 3), CV_64F, 0.0);
				cameraMatrix.at<double>(0, 0) = cam_mtx_fx;
				cameraMatrix.at<double>(1, 1) = cam_mtx_fy;
				cameraMatrix.at<double>(0, 2) = cam_mtx_cx;
				cameraMatrix.at<double>(1, 2) = cam_mtx_cy;
				cameraMatrix.at<double>(2, 2) = 1.0;

				cv::Mat distCoeffs(cv::Size(1, 5), CV_64F, 0.0);
				distCoeffs.at<double>(0) = dist_k1;
				distCoeffs.at<double>(1) = dist_k2;
				distCoeffs.at<double>(2) = dist_p1;
				distCoeffs.at<double>(3) = dist_p2;
				distCoeffs.at<double>(4) = dist_k3;

				std::vector<cv::Point3f> obj;	// object points (valid for the latest chess dimensions)
				for (int i = 0; i < chessY; i++) {
					for (int j = 0; j < chessX; j++) {
						obj.push_back(cv::Point3f((float)j * chessSquareMeters, (float)i * chessSquareMeters, 0));
					}
				}

				
				cv::Mat rvec, tvec;
				success = cv::solvePnP(obj, pointBuf, cameraMatrix, distCoeffs, rvec, tvec);


				
				if (success) {
					got_valid_pose = true;
					rvec_0 = rvec.at<double>(0);
					rvec_1 = rvec.at<double>(1);
					rvec_2 = rvec.at<double>(2);
					tvec_0 = tvec.at<double>(0);
					tvec_1 = tvec.at<double>(1);
					tvec_2 = tvec.at<double>(2);
				}
				else {
					got_valid_pose = false;
				}
			}
			else {
				got_valid_pose = false;
			}

			return success;
		}

		uchar* getProcessedImage() {
			if (image == NULL) {
				std::cerr << "Error : image not set" << std::endl;
				return NULL;
			}

			//processedImage = cv::Mat(*image);

			if (got_valid_pose) {
				//image->copyTo(processedImage);
				//cv::drawChessboardCorners(processedImage, patternSize, pointBuf, true);
				//cv::cvtColor(*image, processedImage, CV_RGBA2BGRA);

				processedImage = cv::Mat(*image);
				cv::drawChessboardCorners(processedImage, pSize, pointBuf, true);
			}
			else {
				cv::cvtColor(gray, processedImage, CV_GRAY2BGRA);
			}

			//
			
			//cv::bitwise_not(*image, processedImage);

			return processedImage.data;
		}


		~chessPoseController() {
			delete image;
		}

		int getCols() {
			return cols;
		}

		int getRows() {
			return rows;
		}

		double getRvec0() {
			return rvec_0;
		}

		double getRvec1() {
			return rvec_1;
		}

		double getRvec2() {
			return rvec_2;
		}

		double getTvec0() {
			return tvec_0;
		}

		double getTvec1() {
			return tvec_1;
		}

		double getTvec2() {
			return tvec_2;
		}

	private:
		cv::Mat * image;
		cv::Mat gray;
		int rows, cols;
		cv::Mat processedImage;
		bool got_valid_pose;
		double rvec_0;
		double rvec_1;
		double rvec_2;
		double tvec_0;
		double tvec_1;
		double tvec_2;
		std::vector<cv::Point2f> pointBuf;
		cv::Size pSize;
	};

	chessPoseController* ac;
	bool initialized = false;

	__declspec(dllexport) void initChessPoseController() {
		if (!initialized) {
			ac = new chessPoseController();
			initialized = true;
		}
	}

	__declspec(dllexport) void destroyChessPoseController() {
		if (initialized)
			delete ac;
		initialized = false;
	}

	__declspec(dllexport) int isInitialized() {
		if (initialized)
			return 1;
		return 0;
	}

	__declspec(dllexport) void newImage(uchar* imageData) {
		if (initialized) {
			ac->setImage(imageData);
		}
	}

	__declspec(dllexport) void setImageSize(int row, int col) {
		if (initialized)
			ac->setSize(row, col);
	}

	__declspec(dllexport) void detect() {
		if (initialized)
			ac->detect();
	}

	__declspec(dllexport) double GetRvec0() {
		if (initialized) {
			return ac->getRvec0();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) double GetRvec1() {
		if (initialized) {
			return ac->getRvec1();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) double GetRvec2() {
		if (initialized) {
			return ac->getRvec2();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) double GetTvec0() {
		if (initialized) {
			return ac->getTvec0();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) double GetTvec1() {
		if (initialized) {
			return ac->getTvec1();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) double GetTvec2() {
		if (initialized) {
			return ac->getTvec2();
		}
		else {
			return -2;
		}
	}

	__declspec(dllexport) bool findExtrinsics(
		int chessX, int chessY, float chessSquareMeters,
		double cam_mtx_fx, double cam_mtx_fy, double cam_mtx_cx, double cam_mtx_cy,
		double dist_k1, double dist_k2, double dist_p1, double dist_p2, double dist_k3) {
		if (initialized) {
			return ac->findExtrinsics(chessX, chessY, chessSquareMeters, cam_mtx_fx, cam_mtx_fy, cam_mtx_cx, cam_mtx_cy, dist_k1, dist_k2, dist_p1, dist_p2, dist_k3);
		}
		else {
			return false;
		}
	}

	

	__declspec(dllexport) int getRows() {
		if (initialized)
			return ac->getRows();
		else
			return -2;
	}

	__declspec(dllexport) int getCols() {
		if (initialized)
			return ac->getCols();
		else
			return -2;
	}

	__declspec(dllexport) uchar* getProcessedImage() {
		if (initialized)
			return ac->getProcessedImage();
		return NULL;
	}


}

