/* 
 * @file:   Calibration.hpp
 * @author: Mustafa Sabur
 * @author: Peter van Leeuwen
 */

#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include "Config.hpp"

#include "opencv2/highgui.hpp"

/*RED*/
extern int MIN_HUE_RED_LOWER;
extern int MIN_SAT_RED_LOWER;
extern int MIN_VAL_RED_LOWER;

extern int MAX_HUE_RED_LOWER;
extern int MAX_SAT_RED_LOWER;
extern int MAX_VAL_RED_LOWER;

extern int MIN_HUE_RED_HIGHER;
extern int MIN_SAT_RED_HIGHER;
extern int MIN_VAL_RED_HIGHER;

extern int MAX_HUE_RED_HIGHER;
extern int MAX_SAT_RED_HIGHER;
extern int MAX_VAL_RED_HIGHER;

/*GREEN*/;
extern int MIN_HUE_GREEN;
extern int MIN_SAT_GREEN;
extern int MIN_VAL_GREEN;

extern int MAX_HUE_GREEN;
extern int MAX_SAT_GREEN;
extern int MAX_VAL_GREEN;

/*BLUE*/;
extern int MIN_HUE_BLUE;
extern int MIN_SAT_BLUE;
extern int MIN_VAL_BLUE;

extern int MAX_HUE_BLUE;
extern int MAX_SAT_BLUE;
extern int MAX_VAL_BLUE;

/*YELLOW*/;
extern int MIN_HUE_YELLOW;
extern int MIN_SAT_YELLOW;
extern int MIN_VAL_YELLOW;

extern int MAX_HUE_YELLOW;
extern int MAX_SAT_YELLOW;
extern int MAX_VAL_YELLOW;

/*BLACK*/;
extern int MIN_HUE_BLACK;
extern int MIN_SAT_BLACK;
extern int MIN_VAL_BLACK;

extern int MAX_HUE_BLACK;
extern int MAX_SAT_BLACK;
extern int MAX_VAL_BLACK;

/*WHITE*/;
extern int MIN_HUE_WHITE;
extern int MIN_SAT_WHITE;
extern int MIN_VAL_WHITE;

extern int MAX_HUE_WHITE;
extern int MAX_SAT_WHITE;
extern int MAX_VAL_WHITE;

class Calibration {
public:
    Calibration();
    virtual ~Calibration();

private:
    const int hue_slider_max = 180;
    const int sat_slider_max = 255;
    const int val_slider_max = 255;

};

#endif /* CALIBRATION_HPP */

