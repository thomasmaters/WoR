/** 
 * @file   Specification.hpp
 * @author Peter van Leeuwen
 * @author Mustafa Sabur
 */

#ifndef SPECIFICATION_HPP
#define SPECIFICATION_HPP

struct MyShape {

    enum ShapeID {
        UNKNOWN_SHAPE,
        CIRCLE,
        HALF_CIRCLE,
        SQUARE,
        RECTANGLE,
        TRIANGLE,
    };
    ShapeID id = UNKNOWN_SHAPE;
    uint8_t corners = 0;
};

enum ColorID {
    UNKNOWN_COLOR,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    BLACK,
    WHITE,
};

/**
 * @brief Contains a color and a shape, these can be used to detect shapes in an image.
 */
class Specification {
public:
    Specification();

    /**
     * The constructor of the Specification struct.
     * @param c The color that the specification needs to contain.
     * @param s The shape that the specification needs to contain.
     */
    Specification(ColorID c, MyShape s);

    /**
     * Converts a specification string to a Specification object.
     * @param userInput The string that contains the specification.
     * @return Returns true if the specification is successfully converted, otherwise returns false.
     */
    bool strToKnownSpecs(std::string& userInput);

    ColorID color; ///< The color of the specification that needs to be searched.
    MyShape shape; ///< The shape of the specification that needs to be searched.
    double width; ///< The width of the object.
    double height; ///< The height of the object.
private:
    /**
     * Converts a shape string to a shape.
     * @param s The string that contains the shape.
     * @return Returns a shape of type MyShape.
     */
    MyShape shapeFromString(std::string& s);

    /**
     * Converts a color string to a color.
     * @param c The string that contains the color.
     * @return Returns a color of type MyColor.
     */
    ColorID colorFromString(std::string& c);

};

#endif /* SPECIFICATION_HPP */

