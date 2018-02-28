#ifndef SHAPEFILTER_HPP_
#define SHAPEFILTER_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include "EnumOperators.hpp"
#include "ImageDisplayer.hpp"

struct ShapeDetectResult;
/**
 * Finds a specific shape from a image.
 * @author Thomas Maters
 */
class ShapeFilter
{
public:
    enum class Shape : unsigned
    {
        SQUARE    = 1,
        RECTANGLE = 2,
        CIRCLE    = 4,
        TRIANGLE  = 8
    };

    ShapeFilter() : display(ImageDisplayer()), pixelsToRLFactor(0)
    {
    }

    /**
     * Tries to find as many shapes as possible.
     * @param source 8B1C Mat.
     * @param shapeToFind Shape as enum to find. This can be a combinated enum.
     * @return
     * @author Thomas Maters
     */
    std::vector<ShapeDetectResult> findShape(const cv::Mat& source, const Shape& shapeToFind, bool showResult = false);

    /**
     * Tries to find a white circle with a know radius to compute the pixel to mm radius.
     * @param source 8B1C Mat.
     * @param realDistance Real life distance in MM.
     * @return True if set.
     * @author Thomas Maters
     */
    bool setRealLifeConversionRate(const cv::Mat& source, float realDistance, bool showResult = false);

    virtual ~ShapeFilter();

private:
    /**
     * Pytagoras thearem
     * @param a
     * @param b
     * @return
     * @author Thomas Maters
     */
    float pytagoras(const cv::Point& a, const cv::Point& b);

    /**
     * Converts shape pixel position and with to real life measurements.
     * @param screenSize Screen size in pixels.
     * @param shape Shape to calculate the real life measurements of.
     * @author Thomas Maters
     */
    void applyRLConversion(const cv::Size& screenSize, ShapeDetectResult& shape);

    /**
     * Finds all rectangles and squares.
     * @param source 8B1C Mat.
     * @param result Reference to output vector.
     * @param filterType Filter on a specific shape. SQUARE or RECTANGLE.
     * @param showResult Output the result to a window.
     * @author Thomas Maters
     */
    void findRectangles(const cv::Mat& source, std::vector<ShapeDetectResult>& result,
                        const ShapeFilter::Shape& filterType = ShapeFilter::Shape::RECTANGLE, bool showResult = false);

    /**
     * Checks if the shape represents a square.
     * @param shapeData Shape to check.
     * @return
     * @author Thomas Maters
     */
    bool isSquare(const ShapeDetectResult& shapeData);

    /**
     * Draws a rectangle on a mat.
     * @param source
     * @param corners Corners of rectangle.
     * @author Thomas Maters
     */
    void drawSquare(cv::Mat& source, const cv::Point2f corners[4]);

    /**
     * Tries to find all the circles shapes in the mat.
     * @param source 8B1C Mat.
     * @param result Reference to output vector.
     * @param showResult Output the result to a window.
     * @author Thomas Maters
     */
    void findCircles(const cv::Mat& source, std::vector<ShapeDetectResult>& result, bool showResult = false);

    /**
     * Draws a circle on a mat.
     * @param source
     * @param circleData Xpos, Ypos, Radius of the circle.
     * @author Thomas Maters
     */
    void drawCircle(cv::Mat& source, const cv::Vec3f& circleData);

    ImageDisplayer display;  /// Visualizer.

    float pixelsToRLFactor;  /// Factor for converting pixels to mm.
};

/**
 * Struct for storing data about the found shape.
 */
struct ShapeDetectResult
{
    ShapeFilter::Shape shapeType = ShapeFilter::Shape::SQUARE;
    float radiusInPixels         = 0;
    float widthInPixels          = 0;
    float heightInPixels         = 0;
    float rotation               = 0;  // degrees
    int32_t xPosition            = 0;
    int32_t yPosition            = 0;
    float widthInRL              = 0;
    float heightInRL             = 0;
    int32_t xPositionRL          = 0;
    int32_t yPositionRL          = 0;

    void toString()
    {
    	std::cout << "x: " << xPosition << " y: " << yPosition << " rot: " << rotation << std::endl;
    	std::cout << "xrl: " << xPositionRL << " yrl: " << yPositionRL << std::endl;
    }
};


#endif /* SHAPEFILTER_HPP_ */
