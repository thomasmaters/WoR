/**
 * @file   Config.hpp
 * @author Peter van Leeuwen
 * @author Mustafa Sabur
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

/*GENERAL*/
#define SQUARE_WIDHT_HEIGHT_DIFFERENCE 15
#define NO_SHAPES_FOUND_MESSAGE "NO SHAPES FOUND"
#define MIN_SATURATION_VALUE 100
#define MIN_BRIGHTNESS_VALUE 20
#define CONTOUR_LINE_THICKNESS 8
#define APPROX_POLY_DP_VALUE 0.04
#define BOTTOM_BORDER 150

/*COLORS*/
#define CV_COLOR_BLUE Scalar(255, 0, 0, 255)
#define CV_COLOR_GREEN Scalar(0, 255, 0, 255)
#define CV_COLOR_RED Scalar(0, 0, 255, 255)
#define CV_COLOR_WHITE Scalar(255, 255, 255, 255)
#define CV_COLOR_BLACK Scalar(0, 0, 0, 255)

/*CIRCLES*/
#define DP_PARAMETER 1
#define PARAM1_PARAMETER 400
#define PARAM2_PARAMETER 50

//struct Constrains {
//    uint8_t hue;
//    uint8_t sat;
//    uint8_t val;
//};
//
//struct SomeColor {
//    ColorID id;
//    Scalar scalar;
//    Constrains min;
//    Constrains max;
//    
//};

#endif /* CONFIG_HPP */

