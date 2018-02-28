#ifndef IMAGEFILTER_HPP_
#define IMAGEFILTER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <mutex>

#include "EnumOperators.hpp"
#include "ImageDisplayer.hpp"

/**
 * Class for applieng different filters to images.
 * @author Thomas Maters
 */
class ImageFilter
{
public:
    ImageFilter() : display(ImageDisplayer())
    {
    }

    ImageFilter(const ImageFilter& other) : display(other.display)
    {
    }

    enum class FilterType : unsigned
    {
        BILATERAL         = 1,   /// Applies bilateral blur.
        GAUSSIANBLUR      = 2,   /// Applies gaussian blur.
        MEDIANBLUR        = 4,   /// Applies median blur.
        EQUALIZEHIST      = 8,   /// Applies equalizehist on V channel of HSV image.
        EQUALIZEHIST_RGB  = 16,  /// Applies equalizehist on RGB channels.
        ADAPTIVETHRESHOLD = 32,  /// Applies an adaptive threshold to correct lighting differences in image.
        PYRMEANSHIFT      = 64   /// Applies pyrMeanShift Filtering to equalize colors that are close together.
    };

    /**
     * Applies filters to a image.
     * @param source 8B3C Mat.
     * @param filter Filters to apply. This can be a combined enum.
     * @param showResult Shows the final filter result.
     * @return 8B3C Mat.
     * @author Thomas Maters
     */
    cv::Mat applyFilter(cv::Mat source, ImageFilter::FilterType filter, bool showResult = false);

    virtual ~ImageFilter()
    {
    }

private:
    /**
     * Applies a bilateral filter.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyBilateral(const cv::Mat& source);
    /**
     * Applies a gaussionblur filter.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyGaussianBlur(const cv::Mat& source);
    /**
     * Applies a medianblur filter.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyMedianBlur(const cv::Mat& source);
    /**
     * Applies a hist equalization on the V channel of HSV.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyEqualizeHist(const cv::Mat& source);
    /**
     * Applies a hist equalization on RGB channels.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyEqualizeHistRGB(const cv::Mat& source);
    /**
     * Applies a adaptive threshold filter.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyAdaptiveThreshold(const cv::Mat& source);
    /**
     * Applies a pyreMeanShift filter.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyPyreMeanShift(const cv::Mat& source);
    /**
     * Tries to remove shadow from image.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyRemovingShadows(const cv::Mat& source);
    /**
     * Tries to remove shadow from image.
     * @param source
     * @return 8B3C Mat with filter result.
     * @author Thomas Maters
     */
    cv::Mat applyRemovingShadows2(const cv::Mat& source);

    ImageDisplayer display;  /// Image displayer instance.
    std::mutex filterMutex;
};


#endif /* IMAGEFILTER_HPP_ */
