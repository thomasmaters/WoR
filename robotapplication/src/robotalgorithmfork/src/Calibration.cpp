/* 
 * @file:   Calibration.cpp
 * @author: Mustafa Sabur
 * @author: Peter van Leeuwen
 */

#include "Calibration.hpp"

/*RED*/
int MIN_HUE_RED_LOWER = 0;
int MIN_SAT_RED_LOWER = 60;
int MIN_VAL_RED_LOWER = 50;

int MAX_HUE_RED_LOWER = 10;
int MAX_SAT_RED_LOWER = 255;
int MAX_VAL_RED_LOWER = 255;

int MIN_HUE_RED_HIGHER = 160;
int MIN_SAT_RED_HIGHER = 100;
int MIN_VAL_RED_HIGHER = 100;

int MAX_HUE_RED_HIGHER = 180;
int MAX_SAT_RED_HIGHER = 255;
int MAX_VAL_RED_HIGHER = 255;

/*GREEN*/;
int MIN_HUE_GREEN = 40;
int MIN_SAT_GREEN = 60;
int MIN_VAL_GREEN = 60;

int MAX_HUE_GREEN = 90;
int MAX_SAT_GREEN = 255;
int MAX_VAL_GREEN = 255;

/*BLUE*/;
int MIN_HUE_BLUE = 90;
int MIN_SAT_BLUE = 50;
int MIN_VAL_BLUE = 50;

int MAX_HUE_BLUE = 130;
int MAX_SAT_BLUE = 255;
int MAX_VAL_BLUE = 255;

/*YELLOW*/;
int MIN_HUE_YELLOW = 17;
int MIN_SAT_YELLOW = 60;
int MIN_VAL_YELLOW = 60;

int MAX_HUE_YELLOW = 35;
int MAX_SAT_YELLOW = 255;
int MAX_VAL_YELLOW = 255;

/*BLACK*/;
int MIN_HUE_BLACK = 0;
int MIN_SAT_BLACK = 0;
int MIN_VAL_BLACK = 0;

int MAX_HUE_BLACK = 180;
int MAX_SAT_BLACK = 255;
int MAX_VAL_BLACK = 30;

/*WHITE*/;
int MIN_HUE_WHITE = 0;
int MIN_SAT_WHITE = 0;
int MIN_VAL_WHITE = 160;

int MAX_HUE_WHITE = 180;
int MAX_SAT_WHITE = 50;
int MAX_VAL_WHITE = 255;

using namespace cv;

Calibration::Calibration() {
    namedWindow("Calibration Sliders", WINDOW_NORMAL);
//    createTrackbar("RED_MIN_HUE_LOWER", "Calibration Sliders", &MIN_HUE_RED_LOWER, hue_slider_max);
//    createTrackbar("RED_MIN_SAT_LOWER", "Calibration Sliders", &MIN_SAT_RED_LOWER, sat_slider_max);
//    createTrackbar("RED_MIN_VAL_LOWER", "Calibration Sliders", &MIN_VAL_RED_LOWER, val_slider_max);
//    createTrackbar("RED_MAX_HUE_LOWER", "Calibration Sliders", &MAX_HUE_RED_LOWER, hue_slider_max);
//    createTrackbar("RED_MAX_SAT_LOWER", "Calibration Sliders", &MAX_SAT_RED_LOWER, sat_slider_max);
//    createTrackbar("RED_MAX_VAL_LOWER", "Calibration Sliders", &MAX_VAL_RED_LOWER, val_slider_max);
//
//    createTrackbar("RED_MIN_HUE_HIGHER", "Calibration Sliders", &MIN_HUE_RED_HIGHER, hue_slider_max);
//    createTrackbar("RED_MIN_SAT_HIGHER", "Calibration Sliders", &MIN_SAT_RED_HIGHER, sat_slider_max);
//    createTrackbar("RED_MIN_VAL_HIGHER", "Calibration Sliders", &MIN_VAL_RED_HIGHER, val_slider_max);
//    createTrackbar("RED_MAX_HUE_HIGHER", "Calibration Sliders", &MAX_HUE_RED_HIGHER, hue_slider_max);
//    createTrackbar("RED_MAX_SAT_HIGHER", "Calibration Sliders", &MAX_SAT_RED_HIGHER, sat_slider_max);
//    createTrackbar("RED_MAX_VAL_HIGHER", "Calibration Sliders", &MAX_VAL_RED_HIGHER, val_slider_max);
//
//    createTrackbar("RED_MIN_HUE_HIGHER", "Calibration Sliders", &MIN_HUE_RED_HIGHER, hue_slider_max);
//    createTrackbar("RED_MIN_SAT_HIGHER", "Calibration Sliders", &MIN_SAT_RED_HIGHER, sat_slider_max);
//    createTrackbar("RED_MIN_VAL_HIGHER", "Calibration Sliders", &MIN_VAL_RED_HIGHER, val_slider_max);
//    createTrackbar("RED_MAX_HUE_HIGHER", "Calibration Sliders", &MAX_HUE_RED_HIGHER, hue_slider_max);
//    createTrackbar("RED_MAX_SAT_HIGHER", "Calibration Sliders", &MAX_SAT_RED_HIGHER, sat_slider_max);
//    createTrackbar("RED_MAX_VAL_HIGHER", "Calibration Sliders", &MAX_VAL_RED_HIGHER, val_slider_max);

//    createTrackbar("GREEN_MIN_HUE", "Calibration Sliders", &MIN_HUE_GREEN, hue_slider_max);
//    createTrackbar("GREEN_MIN_SAT", "Calibration Sliders", &MIN_SAT_GREEN, sat_slider_max);
//    createTrackbar("GREEN_MIN_VAL", "Calibration Sliders", &MIN_VAL_GREEN, val_slider_max);
//    createTrackbar("GREEN_MAX_HUE", "Calibration Sliders", &MAX_HUE_GREEN, hue_slider_max);
//    createTrackbar("GREEN_MAX_SAT", "Calibration Sliders", &MAX_SAT_GREEN, sat_slider_max);
//    createTrackbar("GREEN_MAX_VAL", "Calibration Sliders", &MAX_VAL_GREEN, val_slider_max);
//
//    createTrackbar("YELLOW_MIN_HUE", "Calibration Sliders", &MIN_HUE_YELLOW, hue_slider_max);
//    createTrackbar("YELLOW_MIN_SAT", "Calibration Sliders", &MIN_SAT_YELLOW, sat_slider_max);
//    createTrackbar("YELLOW_MIN_VAL", "Calibration Sliders", &MIN_VAL_YELLOW, val_slider_max);
//    createTrackbar("YELLOW_MAX_HUE", "Calibration Sliders", &MAX_HUE_YELLOW, hue_slider_max);
//    createTrackbar("YELLOW_MAX_SAT", "Calibration Sliders", &MAX_SAT_YELLOW, sat_slider_max);
//    createTrackbar("YELLOW_MAX_VAL", "Calibration Sliders", &MAX_VAL_YELLOW, val_slider_max);
//
    createTrackbar("BLACK_MIN_HUE", "Calibration Sliders", &MIN_HUE_BLACK, hue_slider_max);
    createTrackbar("BLACK_MIN_SAT", "Calibration Sliders", &MIN_SAT_BLACK, sat_slider_max);
    createTrackbar("BLACK_MIN_VAL", "Calibration Sliders", &MIN_VAL_BLACK, val_slider_max);
    createTrackbar("BLACK_MAX_HUE", "Calibration Sliders", &MAX_HUE_BLACK, hue_slider_max);
    createTrackbar("BLACK_MAX_SAT", "Calibration Sliders", &MAX_SAT_BLACK, sat_slider_max);
    createTrackbar("BLACK_MAX_VAL", "Calibration Sliders", &MAX_VAL_BLACK, val_slider_max);
//
//    createTrackbar("WHITE_MIN_HUE", "Calibration Sliders", &MIN_HUE_WHITE, hue_slider_max);
//    createTrackbar("WHITE_MIN_SAT", "Calibration Sliders", &MIN_SAT_WHITE, sat_slider_max);
//    createTrackbar("WHITE_MIN_VAL", "Calibration Sliders", &MIN_VAL_WHITE, val_slider_max);
//    createTrackbar("WHITE_MAX_HUE", "Calibration Sliders", &MAX_HUE_WHITE, hue_slider_max);
//    createTrackbar("WHITE_MAX_SAT", "Calibration Sliders", &MAX_SAT_WHITE, sat_slider_max);
//    createTrackbar("WHITE_MAX_VAL", "Calibration Sliders", &MAX_VAL_WHITE, val_slider_max);

}

Calibration::~Calibration() {
}
