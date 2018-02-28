#include <iostream>
#include "Specification.hpp"

using namespace std;

Specification::Specification()
: color(UNKNOWN_COLOR) {
}

MyShape Specification::shapeFromString(string& s) {
    MyShape ms;
    if (s == "cirkel") {
        ms.id = MyShape::CIRCLE;
    } else if (s == "halve cirkel") {
        ms.id = MyShape::HALF_CIRCLE;
    } else if (s == "vierkant") {
        ms.id = MyShape::SQUARE;
        ms.corners = 4;
    } else if (s == "rechthoek") {
        ms.id = MyShape::RECTANGLE;
        ms.corners = 4;
    } else if (s == "driehoek") {
        ms.id = MyShape::TRIANGLE;
        ms.corners = 3;
    }
    return ms;
}

ColorID Specification::colorFromString(string& c) {
    if (c == "rood") return RED;
    if (c == "groen") return GREEN;
    if (c == "blauw") return BLUE;
    if (c == "geel") return YELLOW;
    if (c == "zwart") return BLACK;
    if (c == "wit") return WHITE;
    return UNKNOWN_COLOR;
}

bool Specification::strToKnownSpecs(string& userInput) {
    const auto last_pos = userInput.find_last_of(" \t"); // locate the last white space
    string s = userInput.substr(0, last_pos);
    const auto one_last_pos = s.find_last_of(" \t");
    string specStr = s.substr(0, one_last_pos);
    const auto spec_pos = specStr.find_last_of(" \t");
    string shapeStr = specStr.substr(0, spec_pos);
    string colorStr = specStr.substr(spec_pos + 1);
    string widthStr = s.substr(one_last_pos + 1);
    string heightStr = userInput.substr(last_pos + 1);
    shape = shapeFromString(shapeStr);
    color = colorFromString(colorStr);

    if (shape.id == MyShape::UNKNOWN_SHAPE || color == UNKNOWN_COLOR) {
        cout << "\n\033[0;31mOnbekende specificatie.\033[0m" << endl;
        cout << "\033[0;31mGeef specificatie in vorm [shape][color][width][height]\033[0m\n" << endl;
        return false;
    }

    width = stod(widthStr);
    height = stod(heightStr);

    return true;
}

