#pragma once


namespace filter
{   
    // Interface class for various filter strategy 
    class IFilter
    { 
    public:
        virtual ~IFilter() {};
        virtual bool filter() = 0;
    };
}