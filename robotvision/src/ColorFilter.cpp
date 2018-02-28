#include "ColorFilter.hpp"

cv::Mat ColorFilter::applyFilter(const cv::Mat& source, const ColorFilter::Color& color, bool showResult)
{
    cv::Mat result = cv::Mat::zeros(source.rows, source.cols, CV_8UC1);
    cv::Mat temp_source = cv::Mat(source.rows, source.cols, CV_8UC3);

    if ((color & ColorFilter::Color::BLACK) == ColorFilter::Color::BLACK)
    {
        temp_source = filter.applyFilter(source, ImageFilter::FilterType::MEDIANBLUR | ImageFilter::FilterType::PYRMEANSHIFT, showResult);
        applyRange(temp_source, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 15), result);
    }
    if ((color & ColorFilter::Color::BLUE) == ColorFilter::Color::BLUE)
    {
        temp_source = filter.applyFilter(source, ImageFilter::FilterType::GAUSSIANBLUR | ImageFilter::FilterType::PYRMEANSHIFT, showResult);
        applyRange(temp_source, cv::Scalar(87, 90, 103), cv::Scalar(130, 255, 255), result);
    }
    if ((color & ColorFilter::Color::GREEN) == ColorFilter::Color::GREEN)
    {
        temp_source = filter.applyFilter(source, ImageFilter::FilterType::MEDIANBLUR | ImageFilter::FilterType::PYRMEANSHIFT, showResult);
        applyRange(temp_source, cv::Scalar(38, 70, 50), cv::Scalar(88, 255, 255), result);
    }
    if ((color & ColorFilter::Color::RED) == ColorFilter::Color::RED)
    {
        // Invert the colors so we can look for blues instead of 2 red ranges.
        cv::Mat invertedSource = ~source;
        temp_source            = filter.applyFilter(
          invertedSource, ImageFilter::FilterType::GAUSSIANBLUR | ImageFilter::FilterType::EQUALIZEHIST | ImageFilter::FilterType::PYRMEANSHIFT,
		  showResult);
        applyRange(temp_source, cv::Scalar(80, 70, 50), cv::Scalar(100, 255, 255), result);
    }
    if ((color & ColorFilter::Color::WHITE) == ColorFilter::Color::WHITE)
    {
        temp_source = filter.applyFilter(source, ImageFilter::FilterType::GAUSSIANBLUR | ImageFilter::FilterType::PYRMEANSHIFT, showResult);
        applyRange(temp_source, cv::Scalar(0, 0, 160), cv::Scalar(255, 50, 255), result);
    }
    if ((color & ColorFilter::Color::YELLOW) == ColorFilter::Color::YELLOW)
    {
        temp_source = filter.applyFilter(source, ImageFilter::FilterType::GAUSSIANBLUR | ImageFilter::FilterType::PYRMEANSHIFT, showResult);
        applyRange(temp_source, cv::Scalar(15, 80, 133), cv::Scalar(33, 255, 255), result);
    }

    if (showResult)
    {
        display.displayWindow(result.clone(), "ColorFiltersApplied");
    }

    return result;
}

void ColorFilter::applyRange(const cv::Mat& source, const cv::Scalar& lowerBound, const cv::Scalar& upperBound, cv::Mat& result)
{
    cv::Mat image_hsv;    // 3C8B mat
    cv::Mat temp_result;  // 1C8B mat

    cv::cvtColor(source, image_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(image_hsv, lowerBound, upperBound, temp_result);
    cv::bitwise_or(result, temp_result, result);
}
