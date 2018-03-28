#ifndef COLORFILTER_HPP_
#define COLORFILTER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <type_traits>

#include "EnumOperators.hpp"
#include "ImageDisplayer.hpp"
#include "ImageFilter.hpp"

/**
 * Finds colors in images.
 * @author Thomas Maters
 */
class ColorFilter
{
  public:
    ColorFilter() : filter(ImageFilter()), colorFilterMutex()
    {
    }

    enum class Color : unsigned
    {
        BLACK = 1,
        BLUE = 2,
        GREEN = 4,
        RED = 8,
        WHITE = 16,
        YELLOW = 32
    };

    /**
     * Tries to find a specific color.
     * @param source To find the color in.
     * @param color Color to find. This can be a combined enum.
     * @param showResult Shows the results to screen.
     * @return 8B1C mask.
     * @author Thomas Maters
     */
    cv::Mat applyFilter(const cv::Mat& source, const ColorFilter::Color& color, bool showResult = false);

    virtual ~ColorFilter()
    {
    }

  private:
    /**
     * Applies a cv::InRange function on source.
     * @param source
     * @param lowerBound
     * @param upperBound
     * @param result To write the result to.
     * @author Thomas Maters
     */
    static void applyRange(const cv::Mat& source, const cv::Scalar& lowerBound, const cv::Scalar& upperBound,
                           cv::Mat& result);

    ImageFilter filter;  /// Instance for applieng color specific filters.

    std::mutex colorFilterMutex;
};

#endif /* COLORFILTER_HPP_ */
