#include <vector>
#include <thread>
#include <opencv2/imgproc.hpp>
#include <boost/lexical_cast.hpp>

#include "Detector.hpp"
#include "Config.hpp"
#include "Calibration.hpp"

using namespace cv;
using namespace std;

Detector::Detector()
: exit(false), inProcess(false), camMode(false) {
    //    namedWindow("TEST", CV_WINDOW_AUTOSIZE);
    namedWindow("Result", CV_WINDOW_AUTOSIZE);
    publisher = publisherNode.advertise<robotapplication::arm_control_msg>("pickupTarget", 10);
}

Detector::~Detector() {

}

void Detector::interactiveMode() {
    cout << "Voer specificatie in als: [vorm][kleur][breedte][hoogte]. Of 'exit' om het programma af te sluiten.\n"
            "Voorbeeld: rechthoek blauw 6 2\n" << endl;

    Specification spec;
    State state = SHOW;
    Mat display;
    int halfRows;
    thread inputThread(&Detector::getUserInput, this);

    while (!exit) {
        if (inProcess && state != SEARCH) {
            state = PROCESS;
        }

        switch (state) {
            case IDLE:
                break;
            case PROCESS:
                state = SHOW;
                if (spec.strToKnownSpecs(userInput)) {
                    state = SEARCH;
                } else inProcess = false;
                break;
            case SEARCH:
            {
                if (camMode) cam >> frame;
                result = Mat::zeros(frame.rows + BOTTOM_BORDER, frame.cols, CV_8UC4);
                bool found = search(spec);
                string time = "Time: " + to_string(searchTime);
                putText(result, time, Point2d(50, frame.rows + 100), FONT_HERSHEY_SIMPLEX, 1, CV_COLOR_WHITE);

                if (found) {
                    Object obj = getObjectData(halfRows, spec);
                    draw();

                    MyShape whiteCircleShape;
                    whiteCircleShape.id = MyShape::CIRCLE;
                    Specification whiteCircleSpec;
                    whiteCircleSpec.color = ColorID::WHITE;
                    whiteCircleSpec.shape = whiteCircleShape;
                    (ColorID::WHITE, whiteCircleShape);
                    //bool whiteCircleFound = search(whiteCircleSpec);

                    //if (whiteCircleFound) {
                        //whiteCircleSpec.width = 100;
                        //whiteCircleSpec.height = 100;
                        //Object whitecircle = getObjectData(halfRows, whiteCircleSpec);
                        sendObject(obj);
                        //sendObject(whitecircle);
                    //} else {
                       // cerr << "\033[0;31mEr kan geen witte cirkel gevonden worden!\033[0m\n";
                    //}

                } else {
                    cerr << "\033[0;31m" << userInput << " is niet gevonden!\033[0m\n";
                }

                state = SHOW;
                inProcess = false;
            }
                break;
            case SHOW:
                if (camMode) cam >> frame;
                else state = IDLE;

                // state always starts in SHOW;
                halfRows = frame.rows / 2;
                display = setBlackLine(frame, halfRows);
                copyMakeBorder(display, display, 0, BOTTOM_BORDER, 0, 0, BORDER_CONSTANT, CV_COLOR_BLACK);
                imshow("Result", setOverlay(display, result));
                break;
        }
        waitKey(50);
    }
    inputThread.join();
}

Object Detector::getObjectData(int halfRows, Specification spec) {
    Object obj;
    obj.width = spec.width;
    obj.height = spec.height;
    obj.z = 0;
    if (circles.size() > 1 || contours.size() > 1) {
        cerr << "\033[0;31m" << " meerdere objecten van " << userInput << " gevonden!\033[0m\n";
    } else {
        Point2f center;
        double devideVar;
        double objectLongestSidePx;
        if (circles.size() > 0) {
            center = Point2f(round(circles.at(0)[0]), round(circles.at(0)[1]));
            //Vec3f c = circles.at(0);
            //int radius = (int) round(c[2]);
            obj.rotation = 0;
        } else if (contours.size() > 0) {
            vector<Point> contour = contours.at(0);
            center = determineContourCenter(contour);
            RotatedRect rotRect = minAreaRect(contour);
            obj.rotation = rotRect.angle;
            if (rotRect.size.width > rotRect.size.height) {
                objectLongestSidePx = rotRect.size.width;
            } else {
                objectLongestSidePx = rotRect.size.height;
            }
            if (spec.width > spec.height) {
                devideVar = spec.width;
            } else {
                devideVar = spec.height;
            }
        }

        obj.y = halfRows - center.y;
        obj.x = center.x;
        onecm = objectLongestSidePx / devideVar;
        //        cout << "pos in px: " << obj.x << "-------" << obj.y << endl;
        obj.x = obj.x / onecm * 10;
        obj.y = obj.y / onecm * 10;
        cout << "pos in mm: " << obj.x << " ------- " << obj.y << endl;
        //        cout << "width: " << obj.width << endl;
        //        cout << "height: " << obj.height << endl;
        //        cout << "rotation: " << obj.rotation << endl;

        return obj;
    }
}

void Detector::sendObject(Object obj) {
    robotapplication::arm_control_msg objmsg;

    robotapplication::arm_position pos;
    pos.x = obj.x;
    pos.y = obj.y;
    pos.z = obj.z;

    robotapplication::pickup_target target;
    target.width = obj.width;
    target.height = obj.height;

    objmsg.pos = pos;
    objmsg.rot_wrist = obj.rotation;
    objmsg.target = target;

    publisher.publish(objmsg);
    ros::spinOnce();
}

Mat Detector::setOverlay(const Mat &image, Mat &overlay) {
    Mat newMat;
    image.copyTo(newMat);
    cvtColor(newMat, newMat, CV_BGR2BGRA);

    if (newMat.size() != overlay.size()) {
        return newMat;
    }

    for (int y = 0; y < overlay.rows; ++y) {
        for (int x = 0; x < overlay.cols; ++x) {
            Vec4b &pixel = overlay.at<Vec4b>(y, x);
            // if pixel is not fully transparent
            if (pixel[3] != 0) {
                newMat.at<Vec4b>(y, x) = pixel;
            }
        }
    }
    return newMat;
}

Mat Detector::setBlackLine(const Mat& image, int row) {
    Mat newMat;
    image.copyTo(newMat);

    Vec4b pixel = {0, 0, 0, 0};

    for (int cols = 0; cols < image.cols; cols++) {
        newMat.at<Vec4b>(row, cols) = pixel;
    }
    return newMat;
}

Mat Detector::colorFilter(const Mat &hsvImage, ColorID color) {
    Mat grayscaleRange;

    switch (color) {
        case RED:
        {
            Mat lowerRedHueRange, upperRedHueRange;
            inRange(hsvImage, Scalar(MIN_HUE_RED_LOWER, MIN_SAT_RED_LOWER, MIN_VAL_RED_LOWER),
                    Scalar(MAX_HUE_RED_LOWER, MAX_SAT_RED_LOWER, MAX_VAL_RED_LOWER), lowerRedHueRange);
            inRange(hsvImage, Scalar(MIN_HUE_RED_HIGHER, MIN_SAT_RED_HIGHER, MIN_VAL_RED_HIGHER),
                    Scalar(MAX_HUE_RED_HIGHER, MAX_SAT_RED_HIGHER, MAX_VAL_RED_HIGHER), upperRedHueRange);
            addWeighted(lowerRedHueRange, 1.0, upperRedHueRange, 1.0, 0.0, grayscaleRange);
            break;
        }
        case GREEN:
            inRange(hsvImage, Scalar(MIN_HUE_GREEN, MIN_SAT_GREEN, MIN_VAL_GREEN),
                    Scalar(MAX_HUE_GREEN, MAX_SAT_GREEN, MAX_VAL_GREEN), grayscaleRange);
            break;
        case BLUE:
            inRange(hsvImage, Scalar(MIN_HUE_BLUE, MIN_SAT_BLUE, MIN_VAL_BLUE),
                    Scalar(MAX_HUE_BLUE, MAX_SAT_BLUE, MAX_VAL_BLUE), grayscaleRange);
            break;
        case YELLOW:
            inRange(hsvImage, Scalar(MIN_HUE_YELLOW, MIN_SAT_YELLOW, MIN_VAL_YELLOW),
                    Scalar(MAX_HUE_YELLOW, MAX_SAT_YELLOW, MAX_VAL_YELLOW), grayscaleRange);
            break;
        case BLACK:
            inRange(hsvImage, Scalar(MIN_HUE_BLACK, MIN_SAT_BLACK, MIN_VAL_BLACK),
                    Scalar(MAX_HUE_BLACK, MAX_SAT_BLACK, MAX_VAL_BLACK), grayscaleRange);
            break;
        case WHITE:
            inRange(hsvImage, Scalar(MIN_HUE_WHITE, MIN_SAT_WHITE, MIN_VAL_WHITE),
                    Scalar(MAX_HUE_WHITE, MAX_SAT_WHITE, MAX_VAL_WHITE), grayscaleRange);
            break;
        default:
            break;
    }

    return grayscaleRange;
}

Mat Detector::removeShadowEffects(ColorID aColor) {
    Mat HSVScale;
    cvtColor(frame, HSVScale, CV_BGR2HSV);
    ros::NodeHandle publisherNode;
    ros::Publisher publisher = publisherNode.advertise<robotapplication::arm_control_msg>("pickupTarget", 10);
    if (aColor == WHITE || aColor == BLACK)
        return HSVScale;
    for (int i = 0; i < HSVScale.rows; i++) {
        for (int j = 0; j < HSVScale.cols; j++) {
            if (HSVScale.at<Vec3b>(i, j)[1] <= 50 ||
                    HSVScale.at<Vec3b>(i, j)[2] <= 50) {
                HSVScale.at<Vec3b>(i, j)[0] = 0;
                HSVScale.at<Vec3b>(i, j)[1] = 0;
                HSVScale.at<Vec3b>(i, j)[2] = 255;
            }
        }
    }
    return HSVScale;
}

bool Detector::search(Specification &spec) {
    bool found = false;
    clock_t begin = clock();
    Mat colorFiltered = removeShadowEffects(spec.color);
    colorFiltered = colorFilter(colorFiltered, spec.color);

    erode(colorFiltered, colorFiltered, Mat());
    adaptiveThreshold(colorFiltered, colorFiltered, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C,
            CV_THRESH_BINARY, 11, 2);
    medianBlur(colorFiltered, colorFiltered, 5);

    bitwise_not(colorFiltered, colorFiltered);

    //    imshow("TEST", colorFiltered);

    switch (spec.shape.id) {
        case MyShape::UNKNOWN_SHAPE:
            break;
        case MyShape::CIRCLE:
            found = detectCircles(colorFiltered);
            break;
        case MyShape::HALF_CIRCLE:
            found = detectHalfCircles(colorFiltered);
            break;
        default:
            found = detectCorneredShapes(colorFiltered, spec.shape);
            break;
    }
    searchTime = clock() - begin;
    return found;
}

bool Detector::detectCircles(Mat &src) {
    HoughCircles(src, circles, CV_HOUGH_GRADIENT, DP_PARAMETER, 30, 100, 30, 1, 100);
    return circles.size() != 0;
}

bool Detector::detectHalfCircles(Mat &source) {
    bool found = false;

    vector<vector < Point>> cts;
    findContours(source, cts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    vector<Point> approx;

    for (size_t i = 0; i < cts.size(); i++) {
        approxPolyDP(Mat(cts[i]), approx,
                arcLength(Mat(cts[i]), true) * APPROX_POLY_DP_VALUE,
                true);

        if (std::fabs(contourArea(cts[i])) < 1000 || !isContourConvex(approx))
            continue;

        if (approx.size() >= 5 && approx.size() <= 7 && isHalfCircle(cts[i])) {
            contours.push_back(cts[i]);
            found = true;
        }
    }
    return found;
}

bool Detector::detectCorneredShapes(Mat &source, MyShape &shape) {
    bool found = false;

    vector<vector < Point>> cts;
    vector<Point> approx;
    findContours(source, cts, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for (size_t i = 0; i < cts.size(); i++) {
        approxPolyDP(Mat(cts[i]), approx,
                arcLength(Mat(cts[i]), true) * APPROX_POLY_DP_VALUE,
                true);

        // Skip small, non-convex objects and not requested shapes.
        // Negative because objects are counter clockwise and holes are clockwise.
        if (std::fabs(contourArea(cts[i])) > 1000 && isContourConvex(approx) && approx.size() == shape.corners) {

            if (shape.id == MyShape::RECTANGLE || shape.id == MyShape::SQUARE) {
                if (!isRectangle(cts[i])) {
                    continue;
                }

                if (shape.id == MyShape::SQUARE) {
                    Rect2d aRect = boundingRect(approx);
                    if ((aRect.height / aRect.width) > 1.1 || (aRect.height / aRect.width) < 0.90) {
                        continue;
                    }
                }
            }
            contours.push_back(cts[i]);
            found = true;
        }
    }
    return found;
}

void Detector::draw() {

    for (auto c : circles) {
        Point2f center(round(c[0]), round(c[1]));
        int radius = (int) round(c[2]);
        //        double surface = radius * radius * CV_PI;
        circle(result, center, radius, CV_COLOR_GREEN, CONTOUR_LINE_THICKNESS);
        drawMarker(result, center, CV_COLOR_RED);
        //        string surfaceStr = boost::lexical_cast<string>((int) surface) + " px2";
        //        putText(result, surfaceStr, center, FONT_HERSHEY_COMPLEX_SMALL, 1.5, CV_COLOR_RED, 2);

    }
    circles.clear();

    for (auto con : contours) {
        Point2f center = determineContourCenter(con);
        //        string surfaceStr = boost::lexical_cast<string>(contourArea(con)) + " px2";
        drawMarker(result, center, CV_COLOR_RED);
        //        putText(result, surfaceStr, con[0], FONT_HERSHEY_COMPLEX_SMALL, 1.5, CV_COLOR_RED, 2);
    }
    drawContours(result, contours, -1, CV_COLOR_GREEN, CONTOUR_LINE_THICKNESS);
    contours.clear();
}

string Detector::getDetections() {
    stringstream ss;
    size_t i = 0;
    cout << circles.size() << endl;
    cout << contours.size() << endl;

    for (i = 0; i < circles.size(); ++i) {
        Point2f center(round(circles[i][0]), round(circles[i][1]));
        int radius = (int) circles[i][2];

        ss << i + 1 << ":\tcirkel " << endl;
        ss << "\tmiddel x: " << center.x << endl;
        ss << "\tmiddel y: " << center.y << endl;
        ss << "\tradius: " << radius << endl;
        ss << "\toppervlakte: " << (radius * radius * CV_PI) << endl;
        ss << endl;
    }

    for (size_t j = 0; j < contours.size(); ++j) {
        Point2f center = determineContourCenter(contours[j]);
        ss << j + i + 1;

        ss << "\tmiddel x: " << center.x << endl;
        ss << "\tmiddel y: " << center.y << endl;
        ss << "\toppervlakte: " << contourArea(contours[j]) << endl;
        ss << endl;
    }

    ss << "Search time: " << batchSearchTime << endl;

    return ss.str();
}

Point2f Detector::determineContourCenter(vector<Point> &contour) {
    Moments m = moments(contour);
    float cx = float(m.m10 / m.m00);
    float cy = float(m.m01 / m.m00);

    return Point2f(cx, cy);
}

void Detector::getUserInput() {
    while (!exit) {
        if (!inProcess) {
            if (reader.readUserInput(cin, userInput)) { // if not empty
                if (userInput == "q" || userInput == "exit") exit = true;
                else inProcess = true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void Detector::batchMode(const string &batchPath) {
    vector<Specification> specs = reader.readBatchfile(batchPath);
    if (camMode) cam >> frame;
    result = Mat::zeros(frame.rows + 150, frame.cols, CV_8UC4);

    for (size_t i = 0; i < specs.size(); i++) {
        search(specs[i]);
        batchSearchTime += searchTime;
    }

    string time = "Batch Time: " + to_string(batchSearchTime);

    putText(result, time, Point2d(50, frame.rows + 100), FONT_HERSHEY_SIMPLEX, 1, CV_COLOR_WHITE);
    reader.writeBatchOutput(batchPath + ".output", getDetections());
    draw();
}

bool Detector::openCam(int port) {
    cam.open(port);
    if (cam.isOpened()) {
        camMode = true;
    } else {
        camMode = false;
        cout << "Kan camera niet openen." << endl;
    }
    return camMode;
}

bool Detector::isRectangle(vector<Point> contour) {
    RotatedRect rotRect = minAreaRect(contour);
    double contourSurface = contourArea(contour);
    double rotRectSurface = rotRect.size.height * rotRect.size.width;

    return (rotRectSurface / contourSurface < 1.15);
}

bool Detector::isHalfCircle(const std::vector<Point> &contour) {
    double radius;
    RotatedRect roundRect = minAreaRect(contour);

    if (roundRect.size.width < roundRect.size.height)
        radius = roundRect.size.width;
    else
        radius = roundRect.size.height;

    double surface = roundRect.size.width * roundRect.size.height;
    double surfaceCircle = (M_PI * std::pow(radius, 2)) / 2;

    double isHalfCircle = surfaceCircle / surface;

    return abs((isHalfCircle - (M_PI / 4))) <= 0.15;
}
