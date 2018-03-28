#ifndef IMAGEDISPLAYER_HPP_
#define IMAGEDISPLAYER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <ctime>
#include <mutex>
#include <typeinfo>

class ImageDisplayer
{
  public:
    static ImageDisplayer& getInst()
    {
        static ImageDisplayer inst;
        return inst;
    }

    virtual ~ImageDisplayer()
    {
    }

    void updateWindows()
    {
        for (std::map<std::string, std::pair<bool, cv::Mat>>::iterator it = imageStorage.begin();
             it != imageStorage.end(); ++it)
        {
            if (it->second.first)
            {
                it->second.first = false;
                imshow(it->first, it->second.second);
            }
        }
    }

    void displayWindow(const cv::Mat& source, const std::string& windowName)
    {
        checkMatStorage(windowName, source);
    }

    void displayWindow(const std::type_info& typeInfo, const cv::Mat& source)
    {
        std::string windowName = std::string(typeInfo.name()) + "_window";
        checkMatStorage(windowName, source);
    }

    void displayWindowMasked(const std::type_info& typeInfo, const cv::Mat& source, const cv::Mat& mask)
    {
        cv::Mat output;
        source.copyTo(output, mask);
        displayWindow(typeInfo, output);
    }

  private:
    ImageDisplayer() /*: imageStorageMutex()*/
    {
    }

    void checkMatStorage(const std::string& windowName, cv::Mat source)
    {
        //        imageStorageMutex.lock();
        if (imageStorage.find(windowName) == imageStorage.end())
        {
            imageStorage.insert(std::make_pair(windowName, std::make_pair(false, source)));
        }
        else
        {
            imageStorage.at(windowName).first = true;
            imageStorage.at(windowName).second = source;
        }
        //        imageStorageMutex.unlock();
    }

    std::map<std::string, std::pair<bool, cv::Mat>> imageStorage;
    //    std::mutex imageStorageMutex;
};

#endif /* IMAGEDISPLAYER_HPP_ */
