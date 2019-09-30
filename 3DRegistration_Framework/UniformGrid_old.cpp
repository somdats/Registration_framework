
//////
//
// Includes
//
// predefined headers
#include"pch.h"
// C++ STL
#include <cstdlib>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <limits>

// Implemented header
#include "UniformGrid.h"



//////
//
// Local helpers
//

// Infinity macro
#define INF(type) (std::numeric_limits<type>::infinity())



//////
//
// Class implementations
//

//
// CUniformGrid2D
CUniformGrid2D::CUniformGrid2D()
{
    grid_size = -INFINITY;
}

CUniformGrid2D::~CUniformGrid2D()
{
}

void CUniformGrid2D::init(float grid_size)
{
    points.clear();
    grid.clear();
    this->grid_size = grid_size;
}

void CUniformGrid2D::insertPoint(const Vec2 &point, size_t index)
{
    // Reference the point
    points.push_back(index);

    // Quantized position of the point
    SCell cell = SCell::get(point, grid_size);

    // Insert into grid
    grid[cell].push_back(index);
}

bool CUniformGrid2D::query(const Vec2 &query, std::vector<int> *indices, std::vector<float> *distances) const
{
    // Clear output buffer
    indices->clear();
    distances->clear();
    std::map<float, int>dist_idx_map;
    // Look in the 9-neighborhood for the nearest sample
    SCell _q = SCell::get(query, grid_size);
    for (unsigned i = 0; i < 9; i++)
    {
        auto cell = grid.find({/* x = */ _q.x - 1 + long(i % 3),
            /* y = */ _q.y - 1 + long(i / 3) });
        if (cell != grid.end())
        {
            for (int i = 0; i < cell->second.size(); i++)
            {
                int idx = static_cast<int>(cell->second[i]);
                Vec2 point = InputCloud->points[idx].getVector3fMap().head<2>();
                float dist = (query - point).squaredNorm();
                dist_idx_map[dist] = idx;
            }
        }
    }
        std::map<float, int>::iterator it;
        for (it = dist_idx_map.begin(); it != dist_idx_map.end(); it++)
        {
            indices->push_back(it->second);
            distances->push_back(it->first);
        }
    

    // Done!
    return !(indices->empty());
}

bool CUniformGrid2D::query(const Vec2 &query, std::vector<int> *indices)const
{
    // Clear output buffer
    indices->clear();
    
    // Look in the 9-neighborhood for the nearest sample
    SCell _q = SCell::get(query, grid_size);
    for (unsigned i = 0; i < 9; i++)
    {
        auto cell = grid.find({/* x = */ _q.x - 1 + long(i % 3),
            /* y = */ _q.y - 1 + long(i / 3) });
        if (cell != grid.end())
        {
            indices->insert(indices->end(), cell->second.begin(), cell->second.end());
        }
    }
   
    // Done!
    return !(indices->empty());
}
void CUniformGrid2D::SetInputCloud(CloudWithNormalPtr &Cloud)
{
    InputCloud = std::move(Cloud);
    size_t nPts = InputCloud->points.size();
    for (int itr = 0; itr < nPts; itr++)
    {
        // Quantized position of the point
        Vec2 point = InputCloud->points[itr].getVector3fMap().head<2>();
        insertPoint(point, itr);
       
    }
}
