/** 
 * @file   main.cpp
 * @brief  The main.
 * @author Mustafa Sabur
 * @author Peter van Leeuwen
 * @date   February 20, 2017, 10:00 AM
 */

#include "Reader.hpp"
#include "Detector.hpp"

using namespace cv;
using namespace std;

/// The arguments used by the CommandLineParser
const String keys =
        "{help h ?     |      | print help             }"
//        "{afb          |<none>| pad naar afbeelding    }"
//        "{batch        |<none>| pad naar batch bestand }"
        ;

int main(int argc, char** argv) {
    Detector detector;

    CommandLineParser parser(argc, (const char * const *) argv, keys);
    parser.about("RobotAlgorithm v1.0.0");
    
//    Calibration calibration;
    
    if (parser.has("help")) {
        parser.printMessage();
        return EXIT_SUCCESS;
    }

    if (!parser.check()) {
        parser.printErrors();
        return EXIT_SUCCESS;
    }

//    if (parser.has("afb")) {
//        detector.frame = imread(parser.get<String>("afb"), 1);
//    } else if (!detector.openCam()) {
//        return EXIT_FAILURE;
//    }
//
//    if (parser.has("batch")) {
//        detector.batchMode(parser.get<String>("batch"));
//    }
    
    if (!detector.openCam()) {
        return EXIT_FAILURE;
    }
    detector.interactiveMode();

    return EXIT_SUCCESS;
}