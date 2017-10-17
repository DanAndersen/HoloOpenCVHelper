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
		}

		void setImage(uchar* imageData) {
			delete image;
			image = new cv::Mat(rows, cols, CV_8UC4, imageData, cols * 4);
			cv::cvtColor(*image, gray, CV_BGRA2GRAY);
		}

		void setSize(int row, int col) {
			rows = row;
			cols = col;
		}

		void detect() {
			// do nothing
		}

		uchar* getProcessedImage() {
			if (image == NULL) {
				std::cerr << "Error : image not set" << std::endl;
				return NULL;
			}

			processedImage = cv::Mat(*image);
			cv::cvtColor(gray, processedImage, CV_GRAY2BGRA);
			
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

	private:
		cv::Mat * image;
		cv::Mat gray;
		int rows, cols;
		cv::Mat processedImage;
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

