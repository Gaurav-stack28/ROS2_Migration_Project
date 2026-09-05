/* Copyright 2014-2015 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_PUBSUB_H_
#define _SLROS_GENERIC_PUBSUB_H_

#include <iostream>
#include <memory>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>

extern std::shared_ptr<rclcpp::Node> SLROSNodePtr;  
///< The global node handle that is used by all ROS entities in the model


/**
* Class for subscribing to ROS messages in C++.
*
* This class is used by code generated from the Simulink ROS
* subscriber blocks and is templatized by the ROS message type and
* Simulink bus type.
*/
template <class MsgType, class BusType>
class SimulinkSubscriber
{
public:
    void subscriberCallback(std::shared_ptr<MsgType const>);
    void createSubscriber(std::string const& topic, uint32_t queueSize);
    bool getLatestMessage(BusType* busPtr);

private:
    typename rclcpp::Subscription<MsgType>::SharedPtr _subscriber;
    bool                                    _newMessageReceived;
    std::shared_ptr<MsgType const>          _lastMsgPtr;
};


/**
* Callback that is triggered when a new message is received
*/
template <class MsgType, class BusType>
void SimulinkSubscriber<MsgType,BusType>::subscriberCallback(
    std::shared_ptr<MsgType const> msgPtr)
{
    _lastMsgPtr = msgPtr;
    _newMessageReceived = true;
}


/**
* Create a C++ subscriber object
*/
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


/**
* Get the latest received message
*/
template <class MsgType, class BusType>
bool SimulinkSubscriber<MsgType,BusType>::getLatestMessage(
    BusType* busPtr)
{
    if (_newMessageReceived)
    {
        convertToBus(busPtr, _lastMsgPtr.get());
        _newMessageReceived = false;
        return true;
    }
    else
    {
        return false;
    }
}


/**
* Class for publishing ROS messages in C++.
*
* This class is used by code generated from the Simulink ROS
* publisher blocks and is templatized by the ROS message type and
* Simulink bus type.
*/
template <class MsgType, class BusType>
class SimulinkPublisher
{

public:
    void createPublisher(std::string const& topic, uint32_t queueSize);
    void publish(BusType* busPtr);

private:
    typename rclcpp::Publisher<MsgType>::SharedPtr _publisher;
    MsgType                               _msg;
};


/**
* Create a publisher to a topic
*/
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


/**
* Publish a message
*/
template <class MsgType, class BusType>
void SimulinkPublisher<MsgType,BusType>::publish(
    BusType* busPtr)
{
    convertFromBus(&_msg, busPtr);
    _publisher->publish(_msg);
}


#endif