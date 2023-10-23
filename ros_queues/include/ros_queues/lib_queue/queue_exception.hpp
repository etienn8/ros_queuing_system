#pragma once
#include <stdexcept>
#include <string>

/**
 * @brief Exception thrown when a arrival prediction gives an negative value which should neven happen.
 */
class NegativeArrivalPredictionException: public std::out_of_range
{
    public:
        NegativeArrivalPredictionException(const std::string& arg):out_of_range(arg) {};
        ~NegativeArrivalPredictionException() noexcept {};
        virtual const char * what() const  noexcept override
        {
                return "Arrival prediction gave an negative integer while it expects a positive integer.";
        }
};

/**
 * @brief Exception thrown when a transmission prediction gives an negative value which should neven happen.
 */
class NegativeDeparturePredictionException: public std::out_of_range
{
    public:
        NegativeDeparturePredictionException(const std::string& arg):out_of_range(arg) {};
        ~NegativeDeparturePredictionException() noexcept {};
        virtual const char * what() const  noexcept override
        {
                return "Departure prediction gave an negative integer while it expects a positive integer.";
        }
};

/**
 * @brief Exception thrown whenever a logic error occurs that is linked to the user-defined conversion function of an element into another size format.
 */
class BadConversionException: public std::logic_error
{
    public:
        BadConversionException(const std::string& arg):logic_error(arg) {};
        ~BadConversionException() noexcept {};
};