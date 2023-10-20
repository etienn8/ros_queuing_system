#pragma once
#include <stdexcept>
#include <string>

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

class BadConversionException: public std::logic_error
{
    public:
        BadConversionException(const std::string& arg):logic_error(arg) {};
        ~BadConversionException() noexcept {};
};