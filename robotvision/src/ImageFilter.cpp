/*
 * ImageFilter.cpp
 *
 *  Created on: 17 dec. 2017
 *      Author: Thomas
 */

#include "ImageFilter.hpp"

cv::Mat ImageFilter::applyFilter(cv::Mat source, ImageFilter::FilterType filter, bool showResult)
{
    filterMutex.lock();
    // result = applyRemovingShadows(result);
    // display.displayWindow(result, "ImageFilterRemoveShadowsApplied");
    if ((filter & ImageFilter::FilterType::BILATERAL) == ImageFilter::FilterType::BILATERAL)
    {
        source = applyBilateral(source);
    }
    if ((filter & ImageFilter::FilterType::GAUSSIANBLUR) == ImageFilter::FilterType::GAUSSIANBLUR)
    {
        source = applyGaussianBlur(source);
    }
    if ((filter & ImageFilter::FilterType::MEDIANBLUR) == ImageFilter::FilterType::MEDIANBLUR)
    {
        source = applyMedianBlur(source);
    }
    if ((filter & ImageFilter::FilterType::EQUALIZEHIST) == ImageFilter::FilterType::EQUALIZEHIST)
    {
        source = applyEqualizeHist(source);
    }
    if ((filter & ImageFilter::FilterType::EQUALIZEHIST_RGB) == ImageFilter::FilterType::EQUALIZEHIST_RGB)
    {
        source = applyEqualizeHistRGB(source);
    }
    if ((filter & ImageFilter::FilterType::ADAPTIVETHRESHOLD) == ImageFilter::FilterType::ADAPTIVETHRESHOLD)
    {
        source = applyAdaptiveThreshold(source);
    }
    if ((filter & ImageFilter::FilterType::PYRMEANSHIFT) == ImageFilter::FilterType::PYRMEANSHIFT)
    {
        source = applyPyreMeanShift(source);
    }

    if (showResult)
    {
        ImageDisplayer::getInst().displayWindow(source.clone(), "ImageFiltersApplied");
    }

    filterMutex.unlock();

    return source;
}

cv::Mat ImageFilter::applyBilateral(const cv::Mat& source)
{
    cv::Mat result;
    cv::bilateralFilter(source, result, 9, 75, 75);
    return result;
}

cv::Mat ImageFilter::applyGaussianBlur(const cv::Mat& source)
{
    cv::Mat result;
    cv::GaussianBlur(source, result, cv::Size(5, 5), 0);
    return result;
}

cv::Mat ImageFilter::applyMedianBlur(const cv::Mat& source)
{
    cv::Mat result;
    cv::medianBlur(source, result, 5);
    return result;
}

cv::Mat ImageFilter::applyEqualizeHist(const cv::Mat& source)
{
    cv::Mat ycrcb;
    cv::Mat result;

    cv::cvtColor(source, ycrcb, CV_BGR2YUV);

    std::vector<cv::Mat> channels;
    split(ycrcb, channels);
    equalizeHist(channels[0], channels[0]);
    merge(channels, ycrcb);

    cv::cvtColor(ycrcb, result, CV_YUV2BGR);
    return result;
}

cv::Mat ImageFilter::applyEqualizeHistRGB(const cv::Mat& source)
{
    cv::Mat result = source;

    std::vector<cv::Mat> channels;
    split(result, channels);
    equalizeHist(channels[0], channels[0]);
    equalizeHist(channels[1], channels[1]);
    equalizeHist(channels[2], channels[2]);
    merge(channels, result);

    return result;
}

cv::Mat ImageFilter::applyAdaptiveThreshold(const cv::Mat& source)
{
    cv::Mat result;
    cv::adaptiveThreshold(source, result, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);
    return result;
}

cv::Mat ImageFilter::applyPyreMeanShift(const cv::Mat& source)
{
    cv::Mat result;
    cv::pyrMeanShiftFiltering(source, result, 10, 40);
    return result;
}

cv::Mat ImageFilter::applyRemovingShadows(const cv::Mat& source)
{
    cv::Mat result;
    cv::Mat result_norm;

    int dilation_size = 3;

    std::vector<cv::Mat> rgb_planes;
    std::vector<cv::Mat> result_planes;
    std::vector<cv::Mat> result_norm_planes;

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                cv::Point(-1, -1));

    split(source, rgb_planes);

    for (auto& plane : rgb_planes)
    {
        cv::Mat dilated_img;
        cv::Mat bg_img;
        cv::Mat diff_img;
        cv::Mat norm_img;

        dilate(plane, dilated_img, element);
        cv::medianBlur(dilated_img, bg_img, 21);
        cv::absdiff(plane, bg_img, diff_img);
        diff_img = cv::Scalar::all(255) - diff_img;
        cv::normalize(diff_img, norm_img, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        result_planes.push_back(diff_img);
        result_norm_planes.push_back(norm_img);
    }
    cv::merge(result_planes, result);
    cv::merge(result_norm_planes, result_norm);

    ImageDisplayer::getInst().displayWindow(result, "shadow_result");
    ImageDisplayer::getInst().displayWindow(result_norm, "shadow_result_norm");

    return source + result;
}

cv::Mat ImageFilter::applyRemovingShadows2(const cv::Mat& source)
{
    cv::Mat result;
    cv::Mat convertedSource;

    cv::cvtColor(source, convertedSource, CV_BGR2GRAY);

    cv::adaptiveThreshold(convertedSource, result, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_TRUNC, 5, 7);
    ImageDisplayer::getInst().displayWindow(result, "shadow2_result");
    return source;
}
