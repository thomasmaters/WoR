/** 
 * @file:  Detector.hpp
 * @author Peter van Leeuwen
 * @author Mustafa Sabur
 * @date   February 23, 2017, 10:15 AM
 */

#ifndef DETECTOR_HPP
#define DETECTOR_HPP

#include <atomic>

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/video/background_segm.hpp"

#include "ros/ros.h"
#include "robotapplication/arm_position.h"
#include "robotapplication/arm_control_msg.h"
#include "robotapplication/pickup_target.h"

#include "Specification.hpp"
#include "Reader.hpp"

struct Object {
    double x;
    double y;
    double z;
    double width;
    double height;
    double rotation;
};

/**
 * @brief Contains all the main functionality for detection of objects.
 */
class Detector {

    enum State {
        IDLE,
        PROCESS,
        SEARCH,
        SHOW,
    };
public:
    Detector();

    virtual ~Detector();

    /**
     * This function is called when the user enters commands interactively in the terminal to find shapes.
     */
    void interactiveMode();

    /**
     * This function is called when the user uses a batch file to find shapes.
     * @param batchPath The path to the batch file that needs to be loaded.
     */
    void batchMode(const std::string &batchPath);

    /**
     * Open #cam and set camMode
     * @param port Camera system number
     * @return true if cam access cam
     */
    bool openCam(int port = 0);

    /**
     * Filter color ranges
     * @param color The color which range needs to be kept
     * @return Return a new Mat with only one color range other colors are replaced the complimentary of the param color
     */
    cv::Mat colorFilter(const cv::Mat &hsvImage, ColorID color);

    cv::Mat frame; ///< Image matrix to process
    cv::VideoCapture cam; ///< Cam module
private:

    /**
     * Replaces the source pixel with the overlay pixels where it is not transparent
     * @param image The underlying Mat 
     * @param overlay The overlay Mat
     * @return return a new Mat instance created from the inputs.
     */
    cv::Mat setOverlay(const cv::Mat &image, cv::Mat &overlay);

    cv::Mat setBlackLine(const cv::Mat& image, int row);

    /**
     * This will remove shadow in the image.
     * It is used for pre-processing, so that color and shape can be detected.
     * @param aMat			Reference to the matrix, which has the
     *shadow.
     **/
    cv::Mat removeShadowEffects(ColorID aColor);

    /**
     * Filters the right objects in the image
     * @param spec The Specification object, which contains the color and shape.
     * @return Returns the result image, with the detected objects that match the given specifications.
     */
    bool search(Specification &spec);

    Object getObjectData(int halfRows, Specification spec);

    void sendObject(Object obj);


    /**
     * Finds circles in a given Mat.
     * @param src The source Mat where to find the circles.
     * @return Returns true when any circle is detected.
     */
    bool detectCircles(cv::Mat &src);

    /**
     * Finds half of circles in a given Mat.
     * @param contour
     * @param approx
     * @return 
     */
    bool detectHalfCircles(cv::Mat &src);

    /**
     * Detects shapes with corners, such as a rectangle, square and a triangle. Also draws the contours of the detected images on the result.
     * @param source The source matrix that contains the shapes.
     * @param shape The shape to look for.
     * @return Returns true when a shape with the given specification is detected, otherwise false.
     */
    bool detectCorneredShapes(cv::Mat &source, MyShape &shape);

    /**
     * draws all circles from #circles and all shapes from #contours
     */
    void draw();

    /**
     * 
     * @return 
     */
    std::string getDetections();

    /**
     * Determines the center of the given contour
     * @param contour The contour where the center needs to be determined of.
     * @return Returns the coordinate of the center.
     */
    cv::Point2f determineContourCenter(std::vector<cv::Point> &contour);

    /**
     * Reads the input that the user enters in the terminal command line.
     */
    void getUserInput();

    /**
     * Checks if a rectangle is a rectangle and not a square of a triangle.
     * @param contour		The contour to see it.
     * @return True if the rectangle is a rectangle.
     **/
    bool isRectangle(std::vector<cv::Point> contour);

    /**
     * Checks if a contour is a half circle.
     * @param contour		The contour to see it.
     * @return True if the contour is a half circle.
     **/
    bool isHalfCircle(const std::vector<cv::Point> &contour);


    std::atomic<bool> exit; ///< If true, the program will shutdown.
    std::atomic<bool> inProcess; ///< If true, the program is analyzing a given input.
    std::string userInput; ///< Contains the user input that is received from the command line.
    Reader reader; ///< Reader object, that contains all the functionality for reading strings.
    std::vector<cv::Vec3f> circles; ///<
    std::vector<std::vector<cv::Point>> contours; ///<
    cv::Mat result; ///<

    bool camMode; ///<
    clock_t searchTime;
    clock_t batchSearchTime;

    ros::NodeHandle publisherNode;
    ros::Publisher publisher;
    
    double onecm;
};

#endif /* DETECTOR_HPP */

