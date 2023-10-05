#pragma once

template<typename TQueueElementType>
class ElementWithConvertedSize
{
    public:
        ElementWithConvertedSize(TQueueElementType element, unsigned int converted_size):element_(element), converted_size_(converted_size){};
        TQueueElementType element_;
        unsigned int converted_size_=0;
};