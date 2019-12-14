
//////
//
// Includes
//

// C++ STL
#include"pch.h"
#include <cstdlib>
#include <vector>
#include <map>
#include <unordered_map>
#include <utility>
#include <limits>
#include <stdexcept>

// Implemented header
#include "UniformGrid.h"



//////
//
// Class implementations
//

//
// CUniformGrid2D

CUniformGrid2D::CUniformGrid2D(double grid_size, PointAccessor &&pointAccessor,
                               bool buildImmediatly)
	: grid_size(grid_size), pointAccess(std::move(pointAccessor))
{
	if (buildImmediatly)
		build();
}

CUniformGrid2D::~CUniformGrid2D()
{}

CUniformGrid2D& CUniformGrid2D:: operator=(const CUniformGrid2D & _grid)
{
    if (this == &_grid)
        return *this;
    grid_size = _grid.grid_size;
    grid = _grid.grid;
    pointAccess = _grid.pointAccess;
    return *this;

}

CUniformGrid2D::CUniformGrid2D(const CUniformGrid2D & _grid)
{
    grid = _grid.grid;
    grid_size = _grid.grid_size;
    pointAccess = _grid.pointAccess;
}

void CUniformGrid2D::build (void)
{
	// Check if already built
	if (!grid.empty())
		return;

	// Request all points from client, one by one
	double attrib_dummy; Vec2 point;
	for (unsigned i=0; pointAccess(&point, &attrib_dummy, i); i++)
	{
		// Quantized position of the point
		SCell cell = SCell::get(point, grid_size);
		// Insert into grid
		grid[cell].push_back(i);
	}
}

bool CUniformGrid2D::query (std::vector<size_t>* out, const Vec2& query,
                            ESortMode sort, double attribQuantization) const
{
	// Sorting helpers
	// TODO: Avoid constucting all three helpers regardless of sorting mode.
	auto quantize = [](double attrib, double quant) -> long {
		return (long)(attrib / quant) - long(attrib < 0);
	};
	std::multimap<double, size_t> orderedMap;
	std::map<long, std::multimap<double, size_t> > bins;
	attribQuantization = attribQuantization != 0.0 ? attribQuantization : grid_size;

	if (!sort)
		// We're outputting to the output buffer directly, so clear it right now
		out->clear();

	// Look for samples within the 9-neighborhood of the query point
	SCell _q = SCell::get(query, grid_size);
	for (unsigned i=0; i<9; i++)
	{
		auto cell = grid.find({/* x = */ _q.x-1 + long(i%3),
		                       /* y = */ _q.y-1 + long(i/3)});
		if (cell != grid.end())
		{
			switch (sort)
			{
				case SORT_NONE:
					// Unsorted output requested
					out->insert(out->end(), cell->second.begin(), cell->second.end());
					break;

				case SORT_DISTANCE:
					// Sorted insertion
					for (unsigned i=0; i<cell->second.size(); i++)
					{
						// Fetch point data from client
						Vec2 point; double dummy;
						pointAccess(&point, &dummy, cell->second[i]);

						// Insert in map for sorting
						orderedMap.emplace(std::move(std::pair<double, size_t>(
							(point-query).squaredNorm(), cell->second[i]
						)));
					}
					break;

				case SORT_ATTRIB:
					// Sorted insertion
					for (unsigned i=0; i<cell->second.size(); i++)
					{
						// Fetch point data from client
						Vec2 dummy; double attrib;
						pointAccess(&dummy, &attrib, cell->second[i]);

						// Insert in map for sorting
						orderedMap.emplace(std::move(std::pair<double, size_t>(
							attrib, cell->second[i]
						)));
					}
					break;

				case SORT_MULTI:
					// Sorted insertion
					for (unsigned i=0; i<cell->second.size(); i++)
					{
						// Fetch point data from client
						Vec2 point; double attrib;
						pointAccess(&point, &attrib, cell->second[i]);

						// Look for correct bin
						long pos = quantize(attrib, attribQuantization);

						// Insert in map for sorting
						bins[pos].emplace(std::move(std::pair<double, size_t>(
							(point-query).squaredNorm(), cell->second[i]
						)));
					}

					break;

				default:
					throw std::runtime_error("[UniformGrid]: invalid sort mode for query!");
			}
		}
	}

	if (!sort)
		// No sorting was done, return immediatly
		return !(out->empty());

	// Commit to output buffer
	out->clear();
	switch(sort)
	{
		case SORT_DISTANCE:
			for (auto pnt=orderedMap.begin(); pnt!=orderedMap.end(); pnt++)
				out->push_back(pnt->second);
			break;
		case SORT_ATTRIB:
			for (auto pnt=orderedMap.rbegin(); pnt!=orderedMap.rend(); pnt++)
				out->push_back(pnt->second);
			break;
		case SORT_MULTI:
			for (auto bin=bins.rbegin(); bin!=bins.rend(); bin++)
				for (auto pnt=bin->second.begin(); pnt!=bin->second.end(); pnt++)
					out->push_back(pnt->second);
			break;
		default:
			throw std::runtime_error("[UniformGrid]: INTERNAL STATE CORRUPTION!");
	}

	// Done!
	return !(out->empty());
}

cParameterGrid::cParameterGrid()
{

}

cParameterGrid::cParameterGrid(double grid_size, PtAccessor &&pointAccessor,
    bool buildImmediatly)
    : grid_size(grid_size), pointAccess(std::move(pointAccessor))
{
    if (buildImmediatly)
        build();
}
cParameterGrid& cParameterGrid:: operator=(const cParameterGrid & _grid)
{
    if (this == &_grid)
        return *this;
    grid_size = _grid.grid_size;
    grid = _grid.grid;
    pointAccess = _grid.pointAccess;
    return *this;

}

cParameterGrid::cParameterGrid(const cParameterGrid & _grid)
{
    grid = _grid.grid;
    grid_size = _grid.grid_size;
    pointAccess = _grid.pointAccess;
}



cParameterGrid ::~cParameterGrid()
{

}
void cParameterGrid::build(void)
{
    // Check if already built
    if (!grid.empty())
        return;

    // Request all points from client, one by one
    double attrib_dummy; Vec2 point;
    for (unsigned i = 0; pointAccess(&point, &attrib_dummy, i); i++)
    {
        // Quantized position of the point
        Cell cell = Cell::get(point, grid_size);
        // Insert into grid
        grid[cell].push_back(i);
    }
}
bool cParameterGrid::query(const Vec2 &query)const
{
    Cell _q = Cell::get(query, grid_size);
    auto cell = grid.find({ _q.x , _q.y });
    if (cell != grid.end())
    {
        return true;
    }
    else
        return false;
}