
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
        buildgrid();
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
void cParameterGrid::buildgrid(void)
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
////////////////////////////////////////////////////////
CUniformGrid3D::CUniformGrid3D(double grid_size, PointAccessor &&pointAccessor,
    bool buildImmediatly)
    : grid_size(grid_size), pointAccess(std::move(pointAccessor))
{
    if (buildImmediatly)
        build();
}

CUniformGrid3D::~CUniformGrid3D()
{}

CUniformGrid3D& CUniformGrid3D:: operator=(const CUniformGrid3D & _grid)
{
    if (this == &_grid)
        return *this;
    grid_size = _grid.grid_size;
    grid = _grid.grid;
    pointAccess = _grid.pointAccess;
    return *this;

}

CUniformGrid3D::CUniformGrid3D(const CUniformGrid3D & _grid)
{
    grid = _grid.grid;
    grid_size = _grid.grid_size;
    pointAccess = _grid.pointAccess;
}

void CUniformGrid3D::build(void)
{
    // Check if already built
    if (!grid.empty())
        return;

    // Request all points from client, one by one
    double attrib_dummy; Vec3 point;
    for (unsigned i = 0; pointAccess(&point,/* &attrib_dummy,*/ i); i++)
    {
        // Quantized position of the point
        SCell cell = SCell::get(point, grid_size);
        // Insert into grid
        grid.insert(cell);
    }
}

std::vector<Eigen::Vector3f> CUniformGrid3D::Compute()
{
    std::vector<Eigen::Vector3f>voxel_centers;
    float half_value = 0.5;
    for (std::unordered_set<SCell, SCell::hash>::iterator itr = grid.begin(); itr != grid.end(); ++itr )
    {
        SCell cell = *itr;
        std::cout << cell.x << "," << cell.y << "," << cell.z << std::endl;
        float x = (float)(itr->x)* grid_size + half_value * grid_size;
        float y = (float)(itr->y)* grid_size + half_value * grid_size;
        float z = (float)(itr->z)* grid_size + half_value * grid_size;
        /*long y = itr->y * static_cast<long>(grid_size) + half_value * static_cast<long>(grid_size);
        long z = itr->z * static_cast<long>(grid_size) + half_value * static_cast<long>(grid_size);*/
        Eigen::Vector3f cell_center(x,y,z); /*(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z))*/;
        std::cout << cell_center << std::endl;
        voxel_centers.push_back(cell_center);
    }
    return  voxel_centers;
}

void  CUniformGrid3D::GetSampledCloud(const CloudWithoutType &InputCloud,
    const std::vector<Eigen::Vector3f> &voxel_center, const std::string &OutPutFileName)
{
    CloudWithNormalPtr pTarget(new pcl::PointCloud <PointNormalType>);
    pcl::fromPCLPointCloud2(*InputCloud, *pTarget);
    pcl::KdTreeFLANN<PointNormalType>kt;
    kt.setInputCloud(pTarget);  // initialize kdtree with uniformly sampled cloud points
    std::vector<int>indxs;
    std::vector<float>dist;
    
    size_t posSlash = OutPutFileName.rfind('.');
    std::string subfileName = OutPutFileName.substr(0, posSlash);
    subfileName = subfileName + ".txt";
    FILE *pFile;
    pFile = fopen(subfileName.c_str(), "wb");

    CloudWithNormalPtr _filteredcloud(new pcl::PointCloud <PointNormalType>);
    _filteredcloud->points.reserve(voxel_center.size());

    for (const auto pt : voxel_center)
    {
        PointNormalType qPt;
        qPt.getVector3fMap() = pt;
        kt.nearestKSearch(qPt, 1, indxs, dist);
        _filteredcloud->points.emplace_back(pTarget->points[indxs[0]]);
        fprintf(pFile, "%d\n", indxs[0]);
    }
    fclose(pFile);
    _filteredcloud->width = voxel_center.size();
    _filteredcloud->height = 1;

    pcl::io::savePLYFile(OutPutFileName, *_filteredcloud);
}