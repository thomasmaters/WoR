/*
 * SmartQueue.hpp
 *
 *  Created on: 26 jan. 2017
 *      Author: Thomas
 */

#ifndef SMARTQUEUE_HPP_
#define SMARTQUEUE_HPP_

#include <condition_variable>
#include <functional>
#include <iostream>
#include <mutex>
#include <queue>
#include <thread>

#include <vector>

template <class T> class SmartQueue
{
  public:
    /**
     * Constructor.
     **/
    SmartQueue()
    {
        eventLoopRunning = false;

        std::thread newEventLoopThread([this] { consumeEvents(); });
        eventLoopThread.swap(newEventLoopThread);
    }

    /**
     * Adds an instance to the queue.
     * @param aEvent	Sharedpointer with the class instance.
     **/
    void addToQueue(const std::shared_ptr<T> aEvent)
    {
        eventQueue.push(aEvent);
//        notifyEventLoop();
    }

    /**
     * Removes all pending elements from the queue.
     **/
    void clearQueue()
    {
        while (eventQueue.size() > 0) {
            eventQueue.pop();
        }
    }

    /**
     * Returns the amount of elements in the queue.
     **/
    size_t size()
    {
    	return eventQueue.size();
    }

    /**
     * Sets a function to call when an element leaves the queue.
     * @param a A function matching the template.
     **/
    void setCallbackFunction(std::function<void(std::shared_ptr<T>)> a)
    {
        callback = a;
    }

    virtual ~SmartQueue(){};

    /**
     * Function that signals the event loop there is some work to do.
     **/
    void notifyEventLoop()
    {
        eventLoopRunning = true;
        eventLoopCondition.notify_one();
    }

  private:
    /**
     * Handles events from event queue.
     **/
    void consumeEvents()
    {
        while (1) {
            std::cout << "looping" << std::endl;
            // Do we have events in our queue?
            if (eventQueue.size() > 0) {
                // If we have a callback given trigger it. Remove it from memory
                // otherwise.
                if (callback) {
                    std::shared_ptr<T> val = eventQueue.front();
                    eventQueue.pop();
                    callback(val);
                } else {
                    eventQueue.pop();
                }

                // Wait with dispatching until we receive a notification we can
                // go again.
                eventLoopRunning = false;
                std::unique_lock<std::mutex> lk(eventLoopMutex);
                eventLoopCondition.wait(lk,
                                        [this] { return eventLoopRunning; });
            } else {
                // Stop looping and wait until something signals the eventloop
                // to run again.
                eventLoopRunning = false;
                std::unique_lock<std::mutex> lk(eventLoopMutex);
                eventLoopCondition.wait(lk,
                                        [this] { return eventLoopRunning; });
            }
        }
    }

    /**************************************************************
     **************************************************************
     **************************************************************/

    // Event queue and a vector of scheduled events, that dispatch there event
    // at a later point in time.
    std::queue<std::shared_ptr<T>> eventQueue;

    // Callback function.
    std::function<void(std::shared_ptr<T>)> callback;
    // Bool to trigger eventloop can run.
    bool eventLoopRunning;

    // Thread stuff.
    std::thread eventLoopThread;
    std::condition_variable eventLoopCondition;
    std::mutex eventLoopMutex;
};

#endif /* SMARTQUEUE_HPP_ */
