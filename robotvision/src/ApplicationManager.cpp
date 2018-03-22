/*
 * ApplicationManager.cpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#include <iostream>
#include <regex>
#include <thread>

#include "robotapplication/PickAndPlace.h"
#include "ros/ros.h"

#include "ApplicationManager.hpp"

#define FPS 20

ApplicationManager::ApplicationManager()
  : imageFilter(ImageFilter())
  , imageDisplayer(ImageDisplayer())
  , inputHandler(InputHandler())
  , colorFinder()
  , shapeFinder()
  , rosCommunication(RosCommunication())
  , shapeFindMutex()
  , globalFilter(static_cast<ImageFilter::FilterType>(0))
  , showFilterProgress(false)
{
    if (!inputHandler.startVideoCapture(1))
    {
        inputHandler.startVideoCapture(0);
    }
    // shapeFinder.setRealLifeConversionRate(colorFinder.applyFilter(inputHandler.getVideoCaptureFrame(),ColorFilter::Color::WHITE),
    // 100);
}

void ApplicationManager::start()
{
    cv::waitKey(1000);
    while (1)
    {
        std::string command = inputHandler.getUserInput();
        cv::waitKey(500);
        specialCommandParser(command);
        cv::waitKey(100);
    }
}

ApplicationManager::~ApplicationManager()
{
}

void ApplicationManager::specialCommandParser(std::string& command)
{
    // To lower case.
    transform(command.begin(), command.end(), command.begin(), ::tolower);

    std::smatch match;
    std::regex regexConversion("sc\\s(\\d*)");
    std::regex regexGlobalFilter("\\s(\\d*)");
    std::regex regexShowFilterProgress("showsteps\\s(\\d*)");

    if (std::regex_search(command, match, regexConversion))
    {
        shapeFinder.setRealLifeConversionRate(
            colorFinder.applyFilter(inputHandler.getVideoCaptureFrame(), ColorFilter::Color::WHITE, showFilterProgress),
            std::stoi(match[1]), showFilterProgress);
        return;
    }
    if (command.find("applyfilter") != std::string::npos)
    {
        globalFilter = static_cast<ImageFilter::FilterType>(0);
        while (std::regex_search(command, match, regexGlobalFilter))
        {
            globalFilter = globalFilter | static_cast<ImageFilter::FilterType>(std::stoi(match[1]));
            command = match.prefix().str() + match.suffix().str();
        }
        return;
    }
    if (std::regex_search(command, match, regexShowFilterProgress))
    {
        showFilterProgress = static_cast<bool>(std::stoi(match[1]));
        std::cout << "ShowFilterProgress has been set to: " << showFilterProgress << std::endl;
        return;
    }
    if (command.find("filterstream") != std::string::npos)
    {
        std::pair<ColorFilter::Color, ShapeFilter::Shape> streamFindParameters = findCommandParser(command);

        std::thread test = std::thread([this, streamFindParameters] {
            while (true)
            {
                shapeFindMutex.lock();
                cv::Mat frame = inputHandler.getVideoCaptureFrame();

                frame = imageFilter.applyFilter(frame, globalFilter, false);
                frame = colorFinder.applyFilter(frame, streamFindParameters.first, true);
                shapeFinder.findShape(frame, streamFindParameters.second, true);
                shapeFindMutex.unlock();
            }
        });
        test.detach();
        return;
    }
    if (command.find("find") != std::string::npos)
    {
        shapeFindMutex.lock();
        std::vector<ShapeDetectResult> resultVector;
        if (findSingleObject(command, resultVector))
        {
            std::cout << "Found destenation" << std::endl;
            std::vector<ShapeDetectResult> targetVector;
            if (findSingleObject("white circle", targetVector))
            {
                robotapplication::PickAndPlace message;
                message.dest.x = resultVector.at(0).xPositionRL - 60;
                message.dest.y = resultVector.at(0).yPositionRL;
                message.dest.z = 70;
                message.destRotation = resultVector.at(0).rotation;
                message.target.x = targetVector.at(0).xPositionRL - 60;
                message.target.y = targetVector.at(0).yPositionRL;
                message.target.z = 70;
                message.targetRotation = 0;
                std::cout << "Going to: " << message.dest << " from: " << message.target << std::endl;
                rosCommunication.sendPickupLocation(message);
            }
            else
            {
                std::cout << "did not find white circle" << std::endl;
                shapeFindMutex.unlock();
                return;
            }
        }
        else
        {
            std::cout << "Did not find object" << std::endl;
        }
        shapeFindMutex.unlock();
        return;
    }
    if (command.find("help") != std::string::npos)
    {
        std::cout << "----------------------" << std::endl;
        std::cout << "List of commands:" << std::endl;
        std::cout << "-sc [mm]" << std::endl;
        std::cout << "-applyfilter [filternumber] [filternumber] ..." << std::endl;
        std::cout << "-showsteps [1/0]" << std::endl;
        std::cout << "-find [color*] [shape*]" << std::endl;
        std::cout << "-filterstream [color*] [shape*]" << std::endl;
        std::cout << "-showvideo" << std::endl;
        std::cout << "-help" << std::endl;
        std::cout << "-quit" << std::endl;
        std::cout << "----------------------" << std::endl;
        return;
    }
    if (command.find("quit") != std::string::npos)
    {
        exit(0);
    }
    if (command.find("showvideo") != std::string::npos)
    {
        std::thread video([this] { inputHandler.video_capture(); });
        video.detach();
        return;
    }
    std::cout << "Not a valid command." << std::endl;
}

std::pair<ColorFilter::Color, ShapeFilter::Shape> ApplicationManager::findCommandParser(const std::string& command)
{
    // Init enums with a 0 value.
    ColorFilter::Color colorToFind = static_cast<ColorFilter::Color>(0);
    ShapeFilter::Shape shapeToFind = static_cast<ShapeFilter::Shape>(0);

    // Parse colors.
    if (command.find("black") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::BLACK;
    if (command.find("blue") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::BLUE;
    if (command.find("green") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::GREEN;
    if (command.find("red") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::RED;
    if (command.find("white") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::WHITE;
    if (command.find("yellow") != std::string::npos)
        colorToFind = colorToFind | ColorFilter::Color::YELLOW;

    // Parse shapes.
    if (command.find("square") != std::string::npos)
        shapeToFind = shapeToFind | ShapeFilter::Shape::SQUARE;
    if (command.find("rectangle") != std::string::npos)
        shapeToFind = shapeToFind | ShapeFilter::Shape::RECTANGLE;
    if (command.find("circle") != std::string::npos)
        shapeToFind = shapeToFind | ShapeFilter::Shape::CIRCLE;
    if (command.find("triangle") != std::string::npos)
        shapeToFind = shapeToFind | ShapeFilter::Shape::TRIANGLE;

    std::cout << "Trying to find color: " << static_cast<int>(colorToFind) << std::endl;
    std::cout << "Trying to find shape: " << static_cast<int>(shapeToFind) << std::endl;

    return std::make_pair(colorToFind, shapeToFind);
}

bool ApplicationManager::findSingleObject(const std::string& command, std::vector<ShapeDetectResult>& result)
{
    cv::Mat frame = inputHandler.getVideoCaptureFrame();
    if (!frame.empty())
    {
        std::pair<ColorFilter::Color, ShapeFilter::Shape> findParameters = findCommandParser(command);

        frame = imageFilter.applyFilter(frame, globalFilter, showFilterProgress);
        frame = colorFinder.applyFilter(frame, findParameters.first, showFilterProgress);
        result = shapeFinder.findShape(frame, findParameters.second, showFilterProgress);
        if (result.size() == 0)
        {
            std::cerr << "No results found." << std::endl;
            return false;
        }
        if (result.size() > 1)
        {
            std::cerr << "Found more then one result." << std::endl;
            return false;
        }
        return true;
    }
    else
    {
        std::cerr << "Video capture frame is empty." << std::endl;
    }
    return false;
}
