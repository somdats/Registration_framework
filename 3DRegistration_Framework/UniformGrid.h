
#ifndef __UNIFORM_GRID_H__
#define __UNIFORM_GRID_H__


//////
//
// Includes
//

// C++ STL
#include <vector>
#include <unordered_map>
#include <limits>
#include"Config.h"
// Eigen library
#include <Eigen/Dense>



//////
//
// Macros and enums
//

#ifndef INF
	#define INF(type) (std::numeric_limits<type>::infinity())
#endif



/** @brief Query results sorting mode for use with @ref CUniformGrid2D::query . */
enum ESortMode
{
	/** @brief Apply no sorting to query results. */
	SORT_NONE = 0,

	/**
	 * @brief
	 *		Report query results in ascending order by their distance to the query point.
	 */
	SORT_DISTANCE,

	/** @brief Report query results in @b DESCENDING order by their attribute. */
	SORT_ATTRIB,

	/**
	 * @brief
	 *		Use a multi-criterion ordering of the query results. Each point will be
	 *		grouped in a "bin" according to their attribute, effectively forming
	 *		"classes". Within a bin, the points are ordered with ascending distance to
	 *		the query point. The "bins" grouping the points are sorted by their
	 *		representative attribute (the "class") in @b DESCENDING order.
	 */
	SORT_MULTI
};





//////
//
// Class definitions
//

/// ** PLACEHOLDER for dll import/export stuff **
//#define API_SPEC

/** @brief Class for filling a sparse 2D grid with, and querying it for, points. */
class REG3D_API CUniformGrid2D
{
public:

	////
	// Types

	/** @brief 2D vector type. */
	typedef Eigen::Vector2d Vec2;

	/** @brief Point accessor type */
	typedef std::function<bool(Vec2*, double*, size_t)> PointAccessor;

	/** @brief Grid cell functionality wrapper. */
	struct SCell
	{
		struct hash
		{
			size_t operator() (const SCell& key_value) const
			{
				// Uses the 3d vector hash function described in "Optimized Spatial
				// Hashing for Collision Detection of Deformable Objects" available here:
				// http://www.beosil.com/download/CollisionDetectionHashing_VMV03.pdf
				// We're discarding the z-value because this is a 2D-grid, which might
				// make the hashing function perform a bit less optimal for large point
				// clouds
				return size_t(
					key_value.x*73856093L ^ key_value.y*19349663L ^ 0*83492791L
					/* mod N is ommitted since it would be (size_t)-1 here, which is
					basically a no-op */
				);
			}
		};

		long x, y;

		inline bool operator == (const SCell &other) const
		{
			return x == other.x && y == other.y/* && z == other.z*/;
		}

		inline static SCell get (const Vec2 &position, double gridsize)
		{
			bool nx = position.x()<0, ny = position.y()<0;
			return {(long)(position.x() / gridsize) - long(nx),
			        (long)(position.y() / gridsize) - long(ny)};
		}
	};


protected:

	////
	// Data members

	/** @brief Point accessor. */
	PointAccessor pointAccess;

	/**
	 * @brief
	 *		Sparse grid organizing the @link #points points @endlink for fast
	 *		neighborhood query, implemented as a hash map.
	 */
	std::unordered_map<SCell, std::vector<size_t>, typename SCell::hash> grid;

	/** @brief Grid cell edge length of the @link #grid grid @endlink . */
	double grid_size;


public:

	////
	// Object construction / deconstruction

	/**
	 * @brief
	 *		Construct with given cell edge length and point accessor. If
	 *		@a buildImmediatly is set to @c false, the method @ref #build must be called
	 *		before the grid can be queried.
	 */
	CUniformGrid2D(double grid_size, PointAccessor &&pointAccessor,
	               bool buildImmediatly=true);

	/** @brief The destructor. */
	~CUniformGrid2D();

    /* assignment operator*/
    CUniformGrid2D& operator=(const CUniformGrid2D &grid);

    // copy constructor
    CUniformGrid2D(const CUniformGrid2D &grid);

	////
	// Methods

	/**
	 * @brief
	 *		Populates the grid with the data points provided by the @ref #PointAccessor .
	 *		Does not do anything if the grid was already built by the constructor.
	 */
	void build (void);

	/**
	 * @brief
	 *		Returns all points in the 9-neighborhood of the given query point. If sorting
	 *		mode is SORT_ATTRIB, @p attribQuantization sets the quantization step, which
	 *		translates to the width of each bin in units of the sorting attribute.
	 */
	bool query (
		std::vector<size_t> *out, const Vec2 &query, ESortMode sort=SORT_ATTRIB,
		double attribQuantization=0
	) const;
};

class REG3D_API cParameterGrid
{
public:
    // Types

    /** @brief 2D vector type. */
    typedef Eigen::Vector2d Vec2;

    /** @brief Point accessor type */
    typedef std::function<bool(Vec2*, double*, size_t)> PtAccessor;

    /** @brief Grid cell functionality wrapper. */
    struct Cell
    {

        long x, y;
        struct hash
        {
            size_t operator() (const Cell& key_value) const
            {
                return size_t(
                    key_value.x * 73856093L ^ key_value.y * 19349663L ^ 0 * 83492791L
                    /* mod N is ommitted since it would be (size_t)-1 here, which is
                    basically a no-op */
                );
            }
        };

        inline bool operator == (const Cell &other) const
        {
            return x == other.x && y == other.y;
        }

        inline static Cell get(const Vec2 &position, double gridsize)
        {
            bool nx = position.x()<0, ny = position.y()<0;
            return{ (long)(position.x() / gridsize) - long(nx),
                (long)(position.y() / gridsize) - long(ny) };
        }
    };
protected:

    ////
    // Data members

    /** @brief Point accessor. */
    PtAccessor pointAccess;

    /**
    * @brief
    *		Sparse grid organizing the @link #points points @endlink
    */
    std::unordered_map<Cell, std::vector<size_t>, typename Cell::hash> grid;

    /** @brief Grid cell edge length of the @link #grid grid @endlink . */
    double grid_size;


public:

    ////
    // Object construction / deconstruction

    cParameterGrid();
    /**
    * @brief
    *		Construct with given cell edge length and point accessor. If
    *		@a buildImmediatly is set to @c false, the method @ref #build must be called
    *		before the grid can be queried.
    */
    cParameterGrid(double grid_size, PtAccessor &&pointAccessor,
        bool buildImmediatly = true);


    /* assignment operator*/
    cParameterGrid& operator=(const cParameterGrid &grid);

    // copy constructor
    cParameterGrid(const cParameterGrid &grid);
    /** @brief The destructor. */

    

    ~cParameterGrid();
    ////
    // Methods

    /**
    * @brief
    *		Populates the grid with the data points provided by the @ref #PointAccessor .
    *		Does not do anything if the grid was already built by the constructor.
    */
    void buildgrid(void);

    /**
    * @brief
    *		Returns whether the given optimized parameter corresponds to a non-empty grid cell
    */
    bool query(const Vec2 &query
    ) const;

};
#endif // ifndef __UNIFORM_GRID_H__
