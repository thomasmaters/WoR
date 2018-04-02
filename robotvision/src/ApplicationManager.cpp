/*
 * ApplicationManager.cpp
 *
 *  Created on: 17 jan. 2018
 *      Author: Thomas
 */

#include <iostream>
#include <regex>
#include <thread>

#include "robotapplication/pick_and_place.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

#include "ApplicationManager.hpp"

#define FPS 20

ApplicationManager::ApplicationManager()
  : imageFilter(ImageFilter())
  , inputHandler(InputHandler())
  , colorFinder()
  , shapeFinder()
  , rosCommunication(RosCommunication())
  , shapeFindMutex()
  , globalFilter(static_cast<ImageFilter::FilterType>(0))
  , showFilterProgress(false)
{
    if (!inputHandler.openVideoCapture(1))
    {
        std::cout << "Failed to open video capture." << std::endl;
        if (!inputHandler.loadImage("/home/thomas/catkin_ws_course/src/WoR/robotvision/doc/test.png"))
        {
            std::cout << "Failed to load image" << std::endl;
            if (!inputHandler.openVideoCapture(0))
            {
                std::cout << "Failed to open default video capture" << std::endl;
            }
        }
    }
    inputHandler.displayVideoCapture();
    shapeFinder.setRealLifeConversionRate(
        colorFinder.applyFilter(inputHandler.getVideoCaptureFrame(), ColorFilter::Color::WHITE), 100);
}

void ApplicationManager::start()
{
    cv::waitKey(1000);

    AsyncGetline ag;
    std::string input;

    std::cout << "Enter command:" << std::endl;
    while (1)
    {
        input = ag.GetLine();
        if (!input.empty())
        {
            specialCommandParser(input);
            std::cout << "Enter command:" << std::endl;
        }
        ImageDisplayer::getInst().updateWindows();
        cv::waitKey(1000 / FPS);
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

        std::thread tempThread = std::thread([this, streamFindParameters] {
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
        filterStreamThread.swap(tempThread);
        filterStreamThread.detach();
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
                robotapplication::pick_and_place message;
                message.target.x = -1 * resultVector.at(0).xPositionRL;
                message.target.y = resultVector.at(0).yPositionRL;
                message.target.z = 90;
                message.targetRotation = resultVector.at(0).rotation;
                message.dest.x = -1 * targetVector.at(0).xPositionRL;
                message.dest.y = targetVector.at(0).yPositionRL;
                message.dest.z = 90;
                message.destRotation = 0;
                std::cout << "Going to: " << message.dest << "rot: " << message.targetRotation << std::endl
                          << "from: " << message.target << std::endl;
                rosCommunication.sendPickupLocation(message);

                publishDebugMarkers(resultVector, targetVector);
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
        std::cout << "-help" << std::endl;
        std::cout << "-quit" << std::endl;
        std::cout << "----------------------" << std::endl;
        return;
    }
    if (command.find("quit") != std::string::npos)
    {
        exit(0);
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

void ApplicationManager::publishDebugMarkers(const std::vector<ShapeDetectResult>& resultVector,
                                             const std::vector<ShapeDetectResult>& targetVector)
{
    visualization_msgs::Marker target_marker;
    target_marker.header.frame_id = "world";
    target_marker.header.stamp = ros::Time::now();
    target_marker.ns = "vision";
    target_marker.id = 0;
    target_marker.type = visualization_msgs::Marker::CUBE;

    target_marker.pose.position.y = -1 * (double)resultVector.at(0).xPositionRL / 1000;
    target_marker.pose.position.x = (double)resultVector.at(0).yPositionRL / 1000;
    target_marker.pose.position.z = 0;

    target_marker.pose.orientation = tf::createQuaternionMsgFromYaw((resultVector.at(0).rotation - 90) * M_PI / 180);
    target_marker.pose.orientation.w *= -1;

    target_marker.scale.x = (double)resultVector.at(0).widthInRL / 1000;   // Length
    target_marker.scale.y = (double)resultVector.at(0).heightInRL / 1000;  // Width
    target_marker.scale.z = 0.02;                                          // Height

    target_marker.color.a = 1.0;  // Don't forget to set the alpha!
    target_marker.color.r = 0.0;
    target_marker.color.g = 0.0;
    target_marker.color.b = 1.0;

    visualization_msgs::Marker circle_marker;
    circle_marker.header.frame_id = "world";
    circle_marker.header.stamp = ros::Time::now();
    circle_marker.ns = "vision";
    circle_marker.id = 1;
    circle_marker.type = visualization_msgs::Marker::CYLINDER;

    circle_marker.pose.position.y = -1 * (double)targetVector.at(0).xPositionRL / 1000;
    circle_marker.pose.position.x = (double)targetVector.at(0).yPositionRL / 1000;
    circle_marker.pose.position.z = 0;
    circle_marker.pose.orientation.x = 0.0;
    circle_marker.pose.orientation.y = 0.0;
    circle_marker.pose.orientation.z = 0.0;
    circle_marker.pose.orientation.w = 1;

    circle_marker.scale.x = (double)targetVector.at(0).radiusInRL * 2 / 1000;  // Length
    circle_marker.scale.y = (double)targetVector.at(0).radiusInRL * 2 / 1000;  // Width
    circle_marker.scale.z = 0.005;                                             // Height

    circle_marker.color.a = 1.0;  // Don't forget to set the alpha!
    circle_marker.color.r = 1.0;
    circle_marker.color.g = 1.0;
    circle_marker.color.b = 1.0;
    rosCommunication.sendDebugMarker(target_marker);
    rosCommunication.sendDebugMarker(circle_marker);
}
