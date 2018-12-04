#pragma once
#include"Ifilter.h"
namespace filter
{
    class cDistanceFilter : public IFilter
    {
    public:
        cDistanceFilter() {};
        ~cDistanceFilter() {};
        bool filter() {};
        void SetDistanceforRejection(float Distance)const;
        void GetFilteredData() const;
        void setNegative(bool negative) const;
    };

}