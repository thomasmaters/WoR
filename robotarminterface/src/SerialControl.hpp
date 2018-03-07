/*
 * SerialControl.hpp
 *
 *  Created on: 18 feb. 2017
 *      Author: Thomas
 */

#ifndef SERIALCONTROL_HPP_
#define SERIALCONTROL_HPP_

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <cstdint>
#include <iostream>
#include <string>

#define READ_BUFFER_SIZE 256
typedef boost::shared_ptr<boost::asio::serial_port> serial_port_ptr;

class SerialControl
{
  public:
    /**
     * Gets the a reference to a SerialControl instance.
     **/
    static SerialControl& getInstance();

    /**
     * Starts the serial communication.
     * @param portName	Portname on the pc the controller is connected to.
     * @param baudrate	Baudrate to run the serial communication on.
     * @return 			Returns true if the serial communication has
     * succesfully been started.
     **/
    bool start(const std::string& portName, uint32_t baudrate);

    /**
     * Changes the baudrate of the serial communication.
     * @param portName	Port name on the pc the controller is connected to.
     * @param baudrate	Baudrate to run the serial communication on.
     **/
    void changeBaudRate(const std::string& portName, uint32_t baudrate);

    /**
     * Stop and performs cleanup of the serial communication.
     **/
    void stop();

    /**
     * Checks if a connection to a serial port is open.
     **/
    bool isConnectionOpen();

    /**
     * Writes a string to the serial port.
     * @param message	String to write to the serial port.
     * @return			Returns the amount of bytes written.
     **/
    int write(const std::string& message);

    /**
     * Writes a message to the serial port and waits(blocking) for a response of
     *a specific size.
     * @param message			String to write to the serial port.
     * @param responseSize		The size of the expected response
     * (without '\\r' or '\n').
     * @return				Returns the response.
     **/
    std::string writeAndRead(const std::string& message, const size_t responseSize);

    /**
     * Writes a message to the serial port and waits(blocking) for a response of
     *a specific size.
     * @param message			String to write to the serial port.
     * @param responseSize		The size of the expected response
     * (without '\\r' or '\n').
     * @return				Returns the response as int.
     **/
    int writeAndReadAsInt(const std::string& message, const size_t responseSize);

    /**
     * Writes a message to the serial port and waits(blocking) for a response of
     *a specific size.
     * @param message		String to write to the serial port.
     * @param untilChar		Reads from string until it reads this character.
     * @return				Returns the response.
     **/
    std::string writeAndReadUntil(const std::string& message, const char untilChar);

    /**
     * Writes a message to the serial port and waits(blocking) for a response of
     *a specific size.
     * @param message			String to write to the serial port.
     * @param untilChar		Reads from string until it reads this character.
     * @return			Returns the response as int.
     **/
    int writeAndReadUntilAsInt(const std::string& message, const char untilChar);

    virtual ~SerialControl();

  private:
    SerialControl();

    /**
     * Writes a message to the serial port.
     * @param buf	A char* that represents the message.
     * @param size	The length of the message.
     **/
    int write_some(const char* buf, const int& size);

    static SerialControl instance;

    boost::asio::io_service io_service_;
    serial_port_ptr port_;
    boost::mutex mutex_;
};

#endif /* SERIALCONTROL_HPP_ */
