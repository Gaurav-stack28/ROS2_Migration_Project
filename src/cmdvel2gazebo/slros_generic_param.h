/* Copyright 2015 The MathWorks, Inc. */

#ifndef _SLROS_GENERIC_PARAM_H_
#define _SLROS_GENERIC_PARAM_H_

#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cassert>

extern std::shared_ptr<rclcpp::Node> SLROSNodePtr;  ///< The global node handle that is used by all ROS entities in the model

// Namespace for parameter parsing code
namespace param_parser
{
    template <class T>
    bool getScalar(const rclcpp::Parameter& parameter, T& paramValue)
    {
        try
        {
            paramValue = parameter.get_value<T>();
            return true;
        }
        catch (const rclcpp::ParameterTypeException&)
        {
            return false;
        }
    }
}

/**
 * Base class for getting ROS parameters in C++.
 * 
 * This class is used by derived classes used for handling scalar and array 
 * parameter values.
 */
class SimulinkParameterGetterBase
{
public:
    void    initialize(const std::string& pName);
    void    initialize_error_codes(uint8_t codeSuccess, uint8_t codeNoParam, uint8_t codeTypeMismatch, uint8_t codeArrayTruncate);
    
protected:
    std::shared_ptr<rclcpp::Node>     nodePtr;                ///< Pointer to node handle (node will be used to connect to parameter server)
    std::string         paramName;              ///< The name of the parameter    
    bool                hasValidValue;          ///< Indicates if a valid value has been received yet. If TRUE, this last value will be stored in lastValidValue.
    
    uint8_t             errorCodeSuccess;       ///< Returned if parameter was retrieved successfully.
    uint8_t             errorCodeNoParam;       ///< Returned if parameter does not exist on server.
    uint8_t             errorCodeTypeMismatch;  ///< Returned if parameter data type did not match.
    uint8_t             errorCodeArrayTruncate; ///< Returned if received array was truncated.    
};


/**
 * Class for getting scalar ROS parameters in C++.
 * 
 * This class is used by code generated from the Simulink ROS
 * parameter blocks and is templatized by the expected C++ data type
 * for the parameter value.
 */
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterGetter : public SimulinkParameterGetterBase
{
    
public:
    void        set_initial_value(const CppParamType initValue);    
    uint8_t     get_parameter(CppParamType* dataPtr);
    
private:
    CppParamType        initialValue;   ///< The default value that should be returned by get_parameter if one of the error conditions occurs    
    CppParamType        lastValidValue; ///< The last valid value that was received from the parameter server

    uint8_t     process_received_data(CppParamType* dataPtr, bool paramRetrieved);    
};

/**
 * Set initial value for returned parameter value.
 *
 * This initial value will be returned if the parameter does not exist or does not have the correct data type when the node is started.
 * @param[in] initValue The initial value. 
 */
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterGetter<CppParamType,ROSCppParamType>::set_initial_value(const CppParamType initValue)
{
    initialValue = initValue;
    lastValidValue = initValue;
}

/**
 * Get the value for a named parameter from the parameter server.
 * @param[out] dataPtr Pointer to initialized data variable. The retrieved parameter value will be written to this location
 * @return Error code (=0 if value was read successfully, >0 if an error occurred)
 */
template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterGetter<CppParamType,ROSCppParamType>::get_parameter(CppParamType* dataPtr)
{
    rclcpp::Parameter parameter;
    ROSCppParamType paramValue;
    bool paramRetrieved = false;
 
    // Get parameter as XmlRpcValue and then parse it through our own function
    if (nodePtr->get_parameter(paramName, parameter))
    {
      paramRetrieved = param_parser::getScalar(parameter, paramValue);
    }    
    
    // Cast the returned value into the data type that Simulink is expecting
    *dataPtr = static_cast<CppParamType>(paramValue);    
    
    const uint8_t errorCode = process_received_data(dataPtr, paramRetrieved);
    return errorCode;
}


/**
 * Determine value and error code for retrieved parameter
 * @param[in,out] dataPtr Pointer to initialized data variable. The retrieved parameter value will be written to this location
 * @param[in] paramRetrieved Return value from ROS function for getting a parameter value
 * @return Error code (=0 if value was read successfully, >0 if an error occurred)
 */
template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterGetter<CppParamType,ROSCppParamType>::process_received_data(CppParamType* dataPtr, bool paramRetrieved)
{
    // By default, assume that parameter can be retrieved successfully
    uint8_t errorCode = errorCodeSuccess;

    if (!paramRetrieved) 
    {
        // An error code of "errorCodeNoParam" means that the parameter does not exist and
        // "errorCodeTypeMismatch" means that it exists, but the data types don't match
        errorCode = nodePtr->has_parameter(paramName) ? errorCodeTypeMismatch : errorCodeNoParam;
    }

    if (errorCode == errorCodeSuccess)
    {
        // Remember last valid value
        lastValidValue = *dataPtr;
        hasValidValue = true;
    }
    else
    {
        // An error occurred. If a valid value was previously
        // received, return it. Otherwise, return the
        // initial value.
        if (hasValidValue)
            *dataPtr = lastValidValue;
        else
            *dataPtr = initialValue;
    }

    return errorCode;
}


/**
* Class for getting array ROS parameters in C++.
*
* This class is used by code generated from the Simulink ROS
* parameter blocks.
* Note that the ROSCppParamType template parameter needs to refer to a container 
* type that supports the following operations:
* - resize
* - std::copy
* std::string (used for string parameters) and std::vector (used for numeric arrays)
* fall into this category.
*/
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterArrayGetter : public SimulinkParameterGetterBase
{
    
public:
    void    set_initial_value(const CppParamType* initValue, const uint32_t lengthToWrite);
    
    uint8_t get_parameter(const uint32_t maxLength, CppParamType* dataPtr, uint32_t* receivedLength);
    
private:
    ROSCppParamType        initialValue;   ///< The default value that should be returned by get_parameter if one of the error conditions occurs    
    ROSCppParamType        lastValidValue; ///< The last valid value that was received from the parameter server
    
    uint8_t process_received_data(const ROSCppParamType& retrievedValue, const uint32_t maxLength, bool paramRetrieved, CppParamType* dataPtr, uint32_t* receivedLength);
};


/**
* Set initial value for returned parameter value.
*
* This initial value will be returned if the parameter does not exist or does not have the correct data type when the node is started.
* @param[in] initValue The initial value.
* @param[in] lengthToWrite The number of elements in the @c initValue array. Since the array is passed as a pointer, the @c lengthToWrite argument is required to indicate how many elements the array has.
*/
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>::set_initial_value(const CppParamType* desiredInitialValue, const uint32_t lengthToWrite)
{
    initialValue.resize(lengthToWrite);

	// Store the initial value in a member variable. Note that std::copy will work on any
	// iterable array, e.g. std::string or std::vector
    std::copy(desiredInitialValue, desiredInitialValue + lengthToWrite, initialValue.begin());
}


/**
* Get the value for a named parameter from the parameter server.
* @param[in] maxLength The maximum length of the returned array (in elements). The array in @c dataPtr will have this many elements.
* @param[out] dataPtr Pointer to initialized data array. The retrieved parameter value will be written to this location
* @param[out] receivedLength The actual number of array elements that was received. This value will be <= than @c maxLength.
* @return Error code (=0 if value was read successfully, >0 if an error occurred)
*/
template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>::get_parameter( 
        const uint32_t maxLength, CppParamType* dataPtr, uint32_t* receivedLength)
{
    uint8_t errorCode = errorCodeSuccess;
    
    // Ensure that getParam is called with correct data type signature
	// This works for strings without any custom data type checking, but probably does not scale
	// to numeric arrays. See the data type checking code for scalars in SimulinkParameterGetter::get_parameter 
	// as an example.
    ROSCppParamType paramValue;
    bool paramRetrieved = nodePtr->get_parameter(paramName, paramValue);
    
    errorCode = process_received_data(paramValue, maxLength, paramRetrieved, dataPtr, receivedLength);    
    return errorCode;    
}


/**
* Determine value and error code for retrieved parameter
* @param[in] retrievedValue Retrieved parameter value as ROS C++ data type
* @param[in] maxLength The maximum length of the returned array (in elements). The array in @c dataPtr will have this many elements.
* @param[in] paramRetrieved Return value from ROS function for getting a parameter value
* @param[out] dataPtr Pointer to Simulink data array. The retrieved parameter value will be written to this location
* @param[out] receivedLength The actual number of array elements that was received. This value will be <= than @c maxLength.
* @return Error code (=0 if value was read successfully, >0 if an error occurred)
*/
template <class CppParamType, class ROSCppParamType>
uint8_t SimulinkParameterArrayGetter<CppParamType,ROSCppParamType>::process_received_data(
        const ROSCppParamType& retrievedValue, const uint32_t maxLength, bool paramRetrieved, 
        CppParamType* dataPtr, uint32_t* receivedLength)
{

    // By default, assume that parameter can be retrieved successfully
    uint8_t errorCode = errorCodeSuccess;

    if (!paramRetrieved) 
    {
        // An error code of "errorCodeNoParam" means that the parameter does not exist and
        // "errorCodeTypeMismatch" means that it exists, but the data types don't match
        errorCode = nodePtr->has_parameter(paramName) ? errorCodeTypeMismatch : errorCodeNoParam;
    }

    if (errorCode == errorCodeSuccess)
    {
		// Indicate truncation if received array has more elements than maxLength
        if (retrievedValue.size() > maxLength)
            errorCode = errorCodeArrayTruncate;        
        
        // Copy the received data into the Simulink variable location
        // We don't need to do zero-padding here, because GetParameterArrayState::codegenStepImpl 
        // initializes dataPtr to all zeros.
        uint32_t copyLength = std::min(maxLength, static_cast<uint32_t>(retrievedValue.size()));
        assert(copyLength <= maxLength);
        std::copy(retrievedValue.begin(), retrievedValue.begin() + copyLength, dataPtr);
        *receivedLength = copyLength;
    
        // Remember last valid value
        lastValidValue.resize(copyLength);
        std::copy(retrievedValue.begin(), retrievedValue.begin() + copyLength, lastValidValue.begin());
        hasValidValue = true;
    }
    else
    {                
        // An error occurred. If a valid value was previously
        // received, return it. Otherwise, return the
        // initial value.
        if (hasValidValue)
        {
            // lastValidValue is truncated when it is saved, so there is no possibility 
            // of a buffer overrun in dataPtr here 
            assert(lastValidValue.size() <= maxLength);
            std::copy(lastValidValue.begin(), lastValidValue.begin() + lastValidValue.size(), dataPtr);
            *receivedLength = uint32_t(lastValidValue.size());
        }
        else
        {
            // initialValue is truncated in GetParameterArrayState::setupSetInitialValue, 
            // so there is no possibility of a buffer overrun in dataPtr here 
            assert(initialValue.size() <= maxLength);
            std::copy(initialValue.begin(), initialValue.begin() + initialValue.size(), dataPtr);
            *receivedLength = uint32_t(initialValue.size());
        }
    }

    return errorCode;
}






/**
* Class for setting ROS parameters in C++.
* 
* This class is used by code generated from the Simulink ROS
* parameter blocks and is templatized by the expected C++ data type
* for the parameter value.
*/
template <class CppParamType, class ROSCppParamType>
class SimulinkParameterSetter
{

public:
    void    initialize(const std::string& pName);
    void    set_parameter(const CppParamType& value);
    void    set_parameter_array(const CppParamType* value, const uint32_t maxLength, const uint32_t lengthToWrite);
    void    length_error(const std::string& modelName, const uint32_t lengthToWrite, const uint32_t arrayLength);

private:
    std::shared_ptr<rclcpp::Node> nodePtr;        ///< Pointer to node handle (node will be used to connect to parameter server)
    std::string         paramName;      ///< The name of the parameter
};


/**
* Initialize the class.
* @param[in] pName The name of the ROS parameter
*/
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>::initialize(const std::string& pName)
{
    nodePtr = SLROSNodePtr;
    paramName = pName;
}

/**
* Set the value of a named scalar parameter.
* @param[in] value The value that should be set.
*/
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>::set_parameter(const CppParamType& value)
{
    // Cast from Simulink data type to data type that ROS expects
    ROSCppParamType paramValue = static_cast<ROSCppParamType>(value);
    nodePtr->set_parameter(paramName, paramValue);
}

/**
* Set the value of a named array parameter.
* @param[in] value The array that should be set.
* @param[in] maxLength The maximum length that can be written. 
* @param[in] lengthToWrite The number of elements in the @c value array that 
* should be written. The value of @c lengthToWrite needs to be less than or 
* equal to the value of @c maxLength.
*/
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>::set_parameter_array(const CppParamType* value, const uint32_t maxLength, const uint32_t lengthToWrite)
{
    // ROSCppParamType is either a std::vector or a std::string
    // The constructors of std::vector and std::string allow us to give a pointer 
    // and the number of elements, so make use of that.
    
    // The validation that lengthToWrite <= than the length of the array should 
    // occur in the calling code.
    
    assert(lengthToWrite <= maxLength);
    
    ROSCppParamType paramValue(value, lengthToWrite);    
    nodePtr->set_parameter(paramName, paramValue);
}

/**
* Log a length error via rosout.
*
* This error occurs when the user-specified length to write is bigger than the number of elements in the user-provided array.
* @param[in] modelName Name of the Simulink model in which the error occurred
* @param[in] lengthToWrite The number of elements that should be written.
* @param[in] arrayLength The number of elements that the array actually contains.
*/
template <class CppParamType, class ROSCppParamType>
void SimulinkParameterSetter<CppParamType,ROSCppParamType>::length_error(const std::string& modelName, const uint32_t lengthToWrite, const uint32_t arrayLength)
{
    RCLCPP_ERROR(
    SLROSNodePtr->get_logger(),
    "Error setting parameter '%s'. The number of array elements to write, %d, is larger than the length of the input array, %d.",
    paramName.c_str(),
    lengthToWrite,
    arrayLength
);
}

#endif
