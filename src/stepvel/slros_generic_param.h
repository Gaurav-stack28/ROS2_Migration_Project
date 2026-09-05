/* Copyright 2015 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_PARAM_H_
#define _SLROS_GENERIC_PARAM_H_

#include <iostream>
#include <string>
#include <memory>
#include <algorithm>
#include <cassert>

#include <rclcpp/rclcpp.hpp>


extern std::shared_ptr<rclcpp::Node> SLROSNodePtr;


/**
 * Base class for getting ROS2 parameters in C++.
 */
class SimulinkParameterGetterBase
{
public:

    void initialize(const std::string& pName);

    void initialize_error_codes(
        uint8_t codeSuccess,
        uint8_t codeNoParam,
        uint8_t codeTypeMismatch,
        uint8_t codeArrayTruncate);


protected:

    std::shared_ptr<rclcpp::Node> nodePtr;

    std::string paramName;

    bool hasValidValue;

    uint8_t errorCodeSuccess;
    uint8_t errorCodeNoParam;
    uint8_t errorCodeTypeMismatch;
    uint8_t errorCodeArrayTruncate;
};



/**
 * Scalar parameter getter
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterGetter :
        public SimulinkParameterGetterBase
{

public:

    void set_initial_value(
        const CppParamType initValue);

    uint8_t get_parameter(
        CppParamType* dataPtr);


private:

    CppParamType initialValue;

    CppParamType lastValidValue;


    uint8_t process_received_data(
        CppParamType* dataPtr,
        bool paramRetrieved);

};



template <class CppParamType, class ROSCppParamType>
void SimulinkParameterGetter<CppParamType,ROSCppParamType>
::set_initial_value(
        const CppParamType initValue)
{
    initialValue = initValue;
    lastValidValue = initValue;
}



template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterGetter<CppParamType,ROSCppParamType>
::get_parameter(
        CppParamType* dataPtr)
{

    ROSCppParamType paramValue;

    bool paramRetrieved = false;


    if(nodePtr->has_parameter(paramName))
    {

        rclcpp::Parameter param =
            nodePtr->get_parameter(paramName);


        paramValue =
            param.get_value<ROSCppParamType>();


        paramRetrieved = true;
    }


    if(paramRetrieved)
    {
        *dataPtr =
            static_cast<CppParamType>(paramValue);
    }


    return process_received_data(
        dataPtr,
        paramRetrieved);
}



template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterGetter<CppParamType,ROSCppParamType>
::process_received_data(
        CppParamType* dataPtr,
        bool paramRetrieved)
{

    uint8_t errorCode =
        errorCodeSuccess;


    if(!paramRetrieved)
    {

        errorCode =
            nodePtr->has_parameter(paramName) ?
            errorCodeTypeMismatch :
            errorCodeNoParam;
    }


    if(errorCode == errorCodeSuccess)
    {

        lastValidValue = *dataPtr;

        hasValidValue = true;

    }
    else
    {

        if(hasValidValue)
        {
            *dataPtr = lastValidValue;
        }
        else
        {
            *dataPtr = initialValue;
        }
    }


    return errorCode;
}

/**
 * Class for getting ROS2 array parameters in C++.
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterArrayGetter :
        public SimulinkParameterGetterBase
{

public:

    void set_initial_value(
        const CppParamType* initValue,
        const uint32_t length);


    uint8_t get_parameter_array(
        CppParamType* dataPtr,
        const uint32_t maxLength,
        uint32_t* receivedLength);


private:

    ROSCppParamType initialValue;

    ROSCppParamType lastValidValue;


    uint8_t process_received_data(
        const ROSCppParamType& retrievedValue,
        const uint32_t maxLength,
        bool paramRetrieved,
        CppParamType* dataPtr,
        uint32_t* receivedLength);

};



template <class CppParamType, class ROSCppParamType>
void SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>
::set_initial_value(
        const CppParamType* initValue,
        const uint32_t length)
{

    initialValue =
        ROSCppParamType(
            initValue,
            initValue + length);


    lastValidValue =
        initialValue;
}



template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>
::get_parameter_array(
        CppParamType* dataPtr,
        const uint32_t maxLength,
        uint32_t* receivedLength)
{

    ROSCppParamType paramValue;

    bool paramRetrieved = false;


    if(nodePtr->has_parameter(paramName))
    {

        rclcpp::Parameter param =
            nodePtr->get_parameter(paramName);


        paramValue =
            param.get_value<ROSCppParamType>();


        paramRetrieved = true;
    }



    return process_received_data(
        paramValue,
        maxLength,
        paramRetrieved,
        dataPtr,
        receivedLength);
}



template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>
::process_received_data(
        const ROSCppParamType& retrievedValue,
        const uint32_t maxLength,
        bool paramRetrieved,
        CppParamType* dataPtr,
        uint32_t* receivedLength)
{

    uint8_t errorCode =
        errorCodeSuccess;



    if(!paramRetrieved)
    {

        errorCode =
            nodePtr->has_parameter(paramName) ?
            errorCodeTypeMismatch :
            errorCodeNoParam;

    }



    if(errorCode == errorCodeSuccess)
    {

        if(retrievedValue.size() > maxLength)
        {
            errorCode =
                errorCodeArrayTruncate;
        }


        uint32_t copyLength =
            std::min(
                maxLength,
                static_cast<uint32_t>(
                    retrievedValue.size()));


        assert(copyLength <= maxLength);



        std::copy(
            retrievedValue.begin(),
            retrievedValue.begin() + copyLength,
            dataPtr);



        *receivedLength =
            copyLength;



        lastValidValue.resize(copyLength);



        std::copy(
            retrievedValue.begin(),
            retrievedValue.begin() + copyLength,
            lastValidValue.begin());



        hasValidValue = true;

    }
    else
    {

        if(hasValidValue)
        {

            assert(
                lastValidValue.size()
                <= maxLength);



            std::copy(
                lastValidValue.begin(),
                lastValidValue.begin()
                    + lastValidValue.size(),
                dataPtr);



            *receivedLength =
                static_cast<uint32_t>(
                    lastValidValue.size());

        }
        else
        {

            assert(
                initialValue.size()
                <= maxLength);



            std::copy(
                initialValue.begin(),
                initialValue.begin()
                    + initialValue.size(),
                dataPtr);



            *receivedLength =
                static_cast<uint32_t>(
                    initialValue.size());
        }

    }


    return errorCode;
}

/**
 * Class for setting ROS2 parameters in C++.
 *
 * This class is used by code generated from the Simulink ROS
 * parameter blocks.
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterSetter
{

public:

    void initialize(
        const std::string& pName);


    void set_parameter(
        const CppParamType& value);


    void set_parameter_array(
        const CppParamType* value,
        const uint32_t maxLength,
        const uint32_t lengthToWrite);


    void length_error(
        const std::string& modelName,
        const uint32_t lengthToWrite,
        const uint32_t arrayLength);



private:

    std::shared_ptr<rclcpp::Node> nodePtr;

    std::string paramName;

};



/**
 * Initialize parameter setter.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>
::initialize(
        const std::string& pName)
{

    nodePtr = SLROSNodePtr;

    paramName = pName;

}



/**
 * Set scalar parameter.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>
::set_parameter(
        const CppParamType& value)
{

    ROSCppParamType paramValue =
        static_cast<ROSCppParamType>(value);



    if(!nodePtr->has_parameter(paramName))
    {

        nodePtr->declare_parameter(
            paramName,
            paramValue);

    }
    else
    {

        nodePtr->set_parameter(
            rclcpp::Parameter(
                paramName,
                paramValue));

    }

}



/**
 * Set array parameter.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>
::set_parameter_array(
        const CppParamType* value,
        const uint32_t maxLength,
        const uint32_t lengthToWrite)
{

    assert(lengthToWrite <= maxLength);



    ROSCppParamType paramValue(
        value,
        value + lengthToWrite);



    if(!nodePtr->has_parameter(paramName))
    {

        nodePtr->declare_parameter(
            paramName,
            paramValue);

    }
    else
    {

        nodePtr->set_parameter(
            rclcpp::Parameter(
                paramName,
                paramValue));

    }

}



/**
 * Log parameter length error.
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>
::length_error(
        const std::string& modelName,
        const uint32_t lengthToWrite,
        const uint32_t arrayLength)
{

    RCLCPP_ERROR(
        nodePtr->get_logger(),
        "Error setting parameter '%s'. "
        "The number of array elements to write, %d, "
        "is larger than the length of the input array, %d.",
        paramName.c_str(),
        lengthToWrite,
        arrayLength);

}



#endif