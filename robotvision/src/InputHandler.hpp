#ifndef INPUTHANDLER_HPP_
#define INPUTHANDLER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>

#include <iostream>
#include <mutex>

/**
 * Handles video capture and image reading.
 * @author Thomas Maters
 */
class InputHandler
{
public:
    InputHandler()
    {
    }

    InputHandler(const InputHandler& other) :
        cap(other.cap), image(other.image), frame(other.frame)
	{
    }

    virtual ~InputHandler()
    {
        if (cap.isOpened())
        {
            cap.release();
        }
    }

    /**
     * Loads a image from a file location.
     * @param path Path to file.
     * @return True if succesfully loaded.
     * @author Thomas Maters
     */
    bool loadImage(const std::string& path)
    {
        image = cv::imread(path, CV_LOAD_IMAGE_COLOR);
        return !image.empty();
    }

    /**
     * Gets the loaded image.
     * @return Copy to image.
     * @author Thomas Maters
     */
    cv::Mat getImage()
    {
        return image;
    }

    /**
     * Start a video capture device.
     * @param device Port id.
     * @return True if succesfully started the video capture.
     * @author Thomas Maters
     */
    bool startVideoCapture(int device = 0)
    {
        cap = cv::VideoCapture(device);
        return cap.isOpened();
    }

    /**
     * Gets a frame from the video capture.
     * If failed returns the last succesfull read frame.
     * @return Frame from video capture device.
     * @author Thomas Maters
     */
    cv::Mat getVideoCaptureFrame()
    {
    	cv::Mat output;
        try
        {
			videoCaptureMutex.lock();
	        cap.read(frame);
	        output = frame.clone();
//            frame.copyTo(output);
			videoCaptureMutex.unlock();
			cv::waitKey(50);
        }
        catch (std::exception& e)
        {
            std::cerr << __PRETTY_FUNCTION__ << ": " << e.what() << std::endl;
        }
        return output;
    }

    void video_capture()
    {
    	cv::Mat windowFrame;
    	cv::namedWindow("Video_Window");
        while (cap.isOpened())
        {
            videoCaptureMutex.lock();
            cap >> frame;
            windowFrame = frame.clone();
            cv::line(windowFrame, cv::Point(0, frame.rows / 2), cv::Point(frame.cols, frame.rows / 2), cv::Scalar(0,0,255));
            cv::imshow("Video_Window", windowFrame);
            videoCaptureMutex.unlock();
            cv::waitKey(16);
        }
    }

    cv::Mat getWebcamFrame()
    {
        cv::Mat camFrame;

        videoCaptureMutex.lock();
        camFrame = frame.clone();
        videoCaptureMutex.unlock();
        return camFrame;
    }

    /**
     * Gets one line from the console.
     * @return
     * @author Thomas Maters
     */
    std::string getUserInput()
    {
    	inputMutex.lock();
        std::string input;
        std::cin.clear();
        //std::cin.ignore(std::numeric_limits<std::streamsize>::max());
        std::cout << "Enter command: " << std::endl;
        getline(std::cin, input);
        inputMutex.unlock();
        return input;
    }

private:
    cv::VideoCapture cap;  /// Video capture instance.
    cv::Mat image;         /// Image read from disk.
    cv::Mat frame;         /// Last video capture frame.

	std::mutex videoCaptureMutex;  /// Mutex for locking frame fetching operation.
	std::mutex inputMutex;
};


#endif /* INPUTHANDLER_HPP_ */
