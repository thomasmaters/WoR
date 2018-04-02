/*
 * ShapeFilter.cpp
 *
 *  Created on: 7 jan. 2018
 *      Author: Thomas
 */

#include "ShapeFilter.hpp"

std::vector<ShapeDetectResult> ShapeFilter::findShape(const cv::Mat& source, const Shape& shapeToFind, bool showResult)
{
    shapeFilterMutex.lock();
    std::vector<ShapeDetectResult> result;
    if ((shapeToFind & ShapeFilter::Shape::RECTANGLE) == ShapeFilter::Shape::RECTANGLE)
    {
        findRectangles(source, result, ShapeFilter::Shape::RECTANGLE, showResult);
    }
    if ((shapeToFind & ShapeFilter::Shape::SQUARE) == ShapeFilter::Shape::SQUARE)
    {
        findRectangles(source, result, ShapeFilter::Shape::SQUARE, showResult);
    }
    if ((shapeToFind & ShapeFilter::Shape::CIRCLE) == ShapeFilter::Shape::CIRCLE)
    {
        findCircles(source, result, showResult);
    }

    shapeFilterMutex.unlock();

    return result;
}

bool ShapeFilter::setRealLifeConversionRate(const cv::Mat& source, float realDistance, bool showResult)
{
    std::vector<ShapeDetectResult> result = std::vector<ShapeDetectResult>();
    findCircles(source, result, showResult);
    if (result.size() != 1 || realDistance <= 0)
    {
        std::cout << "Failed to set conversion" << std::endl;
        return false;
    }

    pixelsToRLFactor = realDistance / (result[0].radiusInPixels * 2);
    std::cout << "Pixel to rl factor: " << pixelsToRLFactor << std::endl;
    return true;
}

ShapeFilter::~ShapeFilter()
{
    // TODO Auto-generated destructor stub
}

float ShapeFilter::pytagoras(const cv::Point& a, const cv::Point& b)
{
    float width = static_cast<float>(std::abs(a.x - b.x));
    float height = static_cast<float>(std::abs(a.y - b.y));
    return std::sqrt(width * width + height * height);
}

void ShapeFilter::applyRLConversion(const cv::Size& screenSize, ShapeDetectResult& shape) const
{
    if (pixelsToRLFactor <= 0)
    {
        return;
    }
    shape.widthInRL = static_cast<float>(shape.widthInPixels) * pixelsToRLFactor;
    shape.heightInRL = static_cast<float>(shape.heightInPixels) * pixelsToRLFactor;
    shape.xPositionRL =
        (static_cast<float>(shape.yPosition) - static_cast<float>(screenSize.height) / 2) * pixelsToRLFactor;
    shape.yPositionRL = (static_cast<float>(shape.xPosition) * pixelsToRLFactor) +
                        60;  // 60 mm ofset for center of gripper to front base.
    shape.radiusInRL = shape.radiusInPixels * pixelsToRLFactor;
    //    shape.toString();
}

void ShapeFilter::findRectangles(const cv::Mat& source, std::vector<ShapeDetectResult>& result,
                                 const ShapeFilter::Shape& filterType, bool showResult)
{
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Point> approx;
    std::vector<cv::Vec4i> hierarchy;
    ShapeDetectResult shapeResult;
    cv::Mat drawing = cv::Mat::zeros(source.size(), CV_8UC1);

    // Find contours.
    cv::findContours(source.clone(), contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // Classify every single found contour.
    for (std::size_t i = 0; i < contours.size(); i++)
    {
        cv::approxPolyDP(cv::Mat(contours[i]), approx, cv::arcLength(cv::Mat(contours[i]), true) * 0.02, true);

        // Is the contourarea not to small.
        if (std::fabs(cv::contourArea(contours[i])) < 100 || !cv::isContourConvex(approx))
        {
            continue;
        }

        if (approx.size() == 3)
        {
            // Triangle
        }
        else if (approx.size() >= 4 && approx.size() <= 6)  // 4 points so its most likely a square.
        {
            // Try to fit a bounding box.
            cv::RotatedRect boundingBox = cv::minAreaRect(approx);
            cv::Point2f corners[4];
            boundingBox.points(corners);

            // Construct result.
            shapeResult.widthInPixels = static_cast<int32_t>(pytagoras(corners[0], corners[1]));
            shapeResult.heightInPixels = static_cast<int32_t>(pytagoras(corners[1], corners[2]));
            shapeResult.xPosition = static_cast<int32_t>(boundingBox.center.x);
            shapeResult.yPosition = static_cast<int32_t>(boundingBox.center.y);
            shapeResult.rotation = boundingBox.angle;
            shapeResult.shapeType = isSquare(shapeResult) ?
                                        ShapeFilter::Shape::SQUARE :
                                        ShapeFilter::Shape::RECTANGLE;  // If it is not a square its a rectangle.

            // Fill in aditional values.
            applyRLConversion(cv::Size(source.cols, source.rows), shapeResult);

            // Are we looking for this type.
            if (filterType != shapeResult.shapeType)
            {
                continue;
            }
            result.push_back(shapeResult);

            if (showResult)
            {
                cv::drawContours(drawing, contours, static_cast<int32_t>(i), cv::Scalar(0, 100, 0), 1, 8, hierarchy, 0,
                                 cv::Point());
                drawSquare(drawing, corners);
            }
        }
    }

    if (showResult)
    {
        cv::Mat drawingClone = drawing.clone();
        for (auto& subResult : result)
        {
            cv::putText(drawingClone, "xPos: " + std::to_string(subResult.xPosition) + " yPos: " +
                                          std::to_string(subResult.yPosition),
                        cv::Point(subResult.xPosition, subResult.yPosition), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6,
                        cv::Scalar(200, 200, 250), 1);
            cv::putText(drawingClone, "xSize: " + std::to_string(subResult.widthInRL) + " ySize: " +
                                          std::to_string(subResult.heightInRL),
                        cv::Point(subResult.xPosition, subResult.yPosition + 25), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6,
                        cv::Scalar(200, 200, 250), 1);
        }
        ImageDisplayer::getInst().displayWindow(drawingClone, "ShapeFilter::findRectangles result");
    }
}

bool ShapeFilter::isSquare(const ShapeDetectResult& shapeData)
{
    if (shapeData.radiusInPixels != 0)
    {
        return false;
    }

    float factor = static_cast<float>(shapeData.widthInPixels) / static_cast<float>(shapeData.heightInPixels);
    return (factor >= 0.9 && factor <= 1.1);
}

void ShapeFilter::drawSquare(cv::Mat& source, const cv::Point2f corners[4])
{
    cv::line(source, corners[0], corners[1], cv::Scalar(255, 255, 255));
    cv::line(source, corners[1], corners[2], cv::Scalar(255, 255, 255));
    cv::line(source, corners[2], corners[3], cv::Scalar(255, 255, 255));
    cv::line(source, corners[3], corners[0], cv::Scalar(255, 255, 255));
}

void ShapeFilter::findCircles(const cv::Mat& source, std::vector<ShapeDetectResult>& result, bool showResult)
{
    std::vector<cv::Vec3f> circles = std::vector<cv::Vec3f>();
    ShapeDetectResult shapeResult;
    cv::Mat drawing = cv::Mat::zeros(source.size(), CV_8UC1);

    // Find circles.
    // Input, output, type, scale, min dis between circles, type param1, type param2, min circle size, max circle size
    cv::HoughCircles(source, circles, CV_HOUGH_GRADIENT, 2, 110, 200, 45, 20, 500);

    for (std::size_t i = 0; i < circles.size(); i++)
    {
        // Construct result.
        shapeResult.shapeType = ShapeFilter::Shape::CIRCLE;
        shapeResult.xPosition = static_cast<int32_t>(circles[i][0]);
        shapeResult.yPosition = static_cast<int32_t>(circles[i][1]);
        shapeResult.radiusInPixels = circles[i][2];

        // Fill in aditional values.
        applyRLConversion(cv::Size(source.cols, source.rows), shapeResult);

        result.push_back(shapeResult);

        if (showResult)
        {
            drawCircle(drawing, circles[i]);
        }
    }

    if (showResult)
    {
        ImageDisplayer::getInst().displayWindow(drawing.clone(), "ShapeFilter::FindCircles result");
    }
}

void ShapeFilter::drawCircle(cv::Mat& source, const cv::Vec3f& circleData)
{
    cv::circle(source, cv::Point(static_cast<int32_t>(circleData[0]), static_cast<int32_t>(circleData[1])),
               static_cast<int32_t>(circleData[2]), cv::Scalar(255, 255, 255));
}
