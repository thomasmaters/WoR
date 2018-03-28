/*
 * ApplicationManager.hpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#ifndef APPLICATIONMANAGER_HPP_
#define APPLICATIONMANAGER_HPP_

#include <string>

#include "ColorFilter.hpp"
#include "ImageFilter.hpp"
#include "InputHandler.hpp"
#include "RosCommunication.hpp"
#include "ShapeFilter.hpp"

class ApplicationManager
{
  public:
    ApplicationManager();

    void start();

    virtual ~ApplicationManager();

  private:
    void specialCommandParser(std::string& command);
    static std::pair<ColorFilter::Color, ShapeFilter::Shape> findCommandParser(const std::string& command);

    bool findSingleObject(const std::string& command, std::vector<ShapeDetectResult>& result);

    ImageFilter imageFilter;
    InputHandler inputHandler;
    ColorFilter colorFinder;
    ShapeFilter shapeFinder;

    RosCommunication rosCommunication;

    std::mutex shapeFindMutex;

    ImageFilter::FilterType globalFilter;
    bool showFilterProgress;
};

#endif /* APPLICATIONMANAGER_HPP_ */
