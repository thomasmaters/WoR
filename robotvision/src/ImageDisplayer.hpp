#ifndef IMAGEDISPLAYER_HPP_
#define IMAGEDISPLAYER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <typeinfo>

class ImageDisplayer
{
  public:
    ImageDisplayer()
    {
    }

    virtual ~ImageDisplayer()
    {
    }

    static void displayWindow(const cv::Mat& source)
    {
        std::string windowName = "test";
        // cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        imshow(windowName, source);
        cv::waitKey(5);
    }

    static void displayWindow(const cv::Mat& source, const std::string& windowName)
    {
        cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        imshow(windowName, source);
        cv::waitKey(5);
    }

    static void displayWindow(const std::type_info& typeInfo, const cv::Mat& source)
    {
        std::string windowName = std::string(typeInfo.name()) + "_window";
        //   cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
        imshow(windowName, source);
        cv::waitKey(5);
    }

    static void displayWindowMasked(const std::type_info& typeInfo, const cv::Mat& source, const cv::Mat& mask)
    {
        cv::Mat output;
        source.copyTo(output, mask);
        displayWindow(typeInfo, output);
        cv::waitKey(5);
    }
};

#endif /* IMAGEDISPLAYER_HPP_ */
