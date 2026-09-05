/* Copyright 2014-2015 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_PUBSUB_H_
#define _SLROS_GENERIC_PUBSUB_H_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <functional>

#include "slros_busmsg_conversion.h"

extern std::shared_ptr<rclcpp::Node> SLROSNodePtr;


/**
 * Class for subscribing to ROS2 messages in C++.
 */
template <class MsgType, class BusType>
class SimulinkSubscriber
{
public:

    void subscriberCallback(
        const std::shared_ptr<MsgType const> msgPtr);

    void createSubscriber(
        std::string const& topic,
        uint32_t queueSize);

    bool getLatestMessage(BusType* busPtr);


private:

    typename rclcpp::Subscription<MsgType>::SharedPtr _subscriber;

    bool _newMessageReceived = false;

    std::shared_ptr<MsgType const> _lastMsgPtr;
};



template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType,BusType>::subscriberCallback(
    const std::shared_ptr<MsgType const> msgPtr)
{
    _lastMsgPtr = msgPtr;
    _newMessageReceived = true;
}



template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType,BusType>::createSubscriber(
    std::string const& topic,
    uint32_t queueSize)
{

    _subscriber =
        SLROSNodePtr->create_subscription<MsgType>(
            topic,
            queueSize,
            std::bind(
                &SimulinkSubscriber<MsgType,BusType>::subscriberCallback,
                this,
                std::placeholders::_1));
}



template <class MsgType, class BusType>
bool SimulinkSubscriber<MsgType,BusType>::getLatestMessage(
    BusType* busPtr)
{

    if (_newMessageReceived)
    {
        convertToBus(
            busPtr,
            _lastMsgPtr.get());

        _newMessageReceived = false;

        return true;
    }

    return false;
}



/**
 * Class for publishing ROS2 messages in C++.
 */
template <class MsgType, class BusType>
class SimulinkPublisher
{

public:

    void createPublisher(
        std::string const& topic,
        uint32_t queueSize);

    void publish(
        BusType* busPtr);


private:

    typename rclcpp::Publisher<MsgType>::SharedPtr _publisher;

    MsgType _msg;
};



template <class MsgType, class BusType>
void SimulinkPublisher<MsgType,BusType>::createPublisher(
    std::string const& topic,
    uint32_t queueSize)
{

    _publisher =
        SLROSNodePtr->create_publisher<MsgType>(
            topic,
            queueSize);
}



template <class MsgType, class BusType>
void SimulinkPublisher<MsgType,BusType>::publish(
    BusType* busPtr)
{

    convertFromBus(
        &_msg,
        busPtr);

    _publisher->publish(_msg);
}


#endif
