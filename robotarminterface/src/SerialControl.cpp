/*
 * SerialControl.cpp
 *
 *  Created on: 18 feb. 2017
 *      Author: Thomas
 */

#include "SerialControl.hpp"

SerialControl& SerialControl::getInstance()
{
    static SerialControl instance;
    return instance;
}

SerialControl::SerialControl()
{
    // TODO Auto-generated constructor stub
}

bool SerialControl::start(const std::string& portName, uint32_t baudrate)
{
    boost::system::error_code ec;

    if (port_) {
        std::cout << "error : port is already opened..." << std::endl;
        return false;
    }

    port_ = serial_port_ptr(new boost::asio::serial_port(io_service_));
    port_->open(portName, ec);
    if (ec) {
        std::cout << "error : port_->open() failed...com_port_name=" << portName
                  << ", e=" << ec.message().c_str() << std::endl;
        return false;
    }

    // option settings...
    port_->set_option(boost::asio::serial_port_base::baud_rate(baudrate));
    port_->set_option(boost::asio::serial_port_base::character_size(8));
    port_->set_option(boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    port_->set_option(boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    port_->set_option(boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    boost::thread t(
        boost::bind(static_cast<size_t (boost::asio::io_service::*)()>(
                        &boost::asio::io_service::run),
                    &io_service_));

    return true;
}

void SerialControl::stop()
{
    boost::mutex::scoped_lock look(mutex_);

    if (port_) {
        port_->cancel();
        port_->close();
        port_.reset();
    }
    io_service_.stop();
    io_service_.reset();
}

int SerialControl::write(const std::string& message)
{
    return write_some(message.c_str(), message.size());
}

std::string SerialControl::writeAndRead(const std::string& message,
                                        const size_t responseSize)
{
    if (!isConnectionOpen()) {
        throw;
    }
    // Write message to serial.
    write(message);
    // Read until our buffer is full.
    boost::asio::streambuf tempBuffer(responseSize);
    boost::asio::read(*port_.get(), tempBuffer);

    std::string s((std::istreambuf_iterator<char>(&tempBuffer)),
                  std::istreambuf_iterator<char>());
    return s.substr(0, responseSize);
}

int SerialControl::writeAndReadAsInt(const std::string& message,
                                     const size_t responseSize)
{
    return std::atoi(writeAndRead(message, responseSize).c_str());
}

void SerialControl::changeBaudRate(const std::string& portName,
                                   uint32_t baudrate)
{
    if (port_->is_open()) {
        stop();
    }

    start(portName, baudrate);
}

bool SerialControl::isConnectionOpen()
{
    return !port_ || port_->is_open();
}

std::string SerialControl::writeAndReadUntil(const std::string& message,
                                             const char untilChar)
{
    if (!isConnectionOpen()) {
        throw;
    }
    // Write the message to serial.
    write(message);
    // Read until a specific character appears or we received 512 bytes.
    boost::asio::streambuf tempBuffer(512);
    boost::asio::read_until(*port_.get(), tempBuffer, untilChar);

    std::string s((std::istreambuf_iterator<char>(&tempBuffer)),
                  std::istreambuf_iterator<char>());
    return s;
}

int SerialControl::writeAndReadUntilAsInt(const std::string& message,
                                          const char untilChar)
{
    return std::atoi(writeAndReadUntil(message, untilChar).c_str());
}

int SerialControl::write_some(const char* buf, const int& size)
{
    boost::system::error_code ec;

    if (!port_)
        return -1;
    if (size == 0)
        return 0;

    return port_->write_some(boost::asio::buffer(buf, size), ec);
}

SerialControl::~SerialControl()
{
    stop();
}
