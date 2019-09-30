#pragma once
//dll config header
#include"Config.h"
// C++ STL
#include <vector>
#include <unordered_map>
#include <map>

// Eigen library
#include <Eigen/Dense>
//library header
#include"Datatypes.h"

class REG3D_API CUniformGrid2D
{
    public:
        typedef Eigen::Vector2f Vec2;
        // Object construction / deconstruction

        /** @brief Default constructor. */
        CUniformGrid2D();

        /** @brief Construct with given cell edge length. */
        CUniformGrid2D(float gridsize) :grid_size(gridsize)
        {};

        /** @brief The destructor. */
        ~CUniformGrid2D();


        ////
        // Methods

        /** @brief Initializes for given grid cell size */
        void init(float grid_size);

        /**
        * @brief
        *		Inserts a 2d point into the sparse grid. Only the index is actually being
        *		stored in the internal data structure.
        */
        void insertPoint(const Vec2 &point, size_t index);

        /**
        * @brief
        *		Returns all points in the 9-neighborhood of the given query point, in a
        *		sorted by distance order.
        */
        bool query(const Vec2 &query, std::vector<int> *indices, std::vector<float> *distances) const;

        /*Returns all points in the 9 - neighborhood of the given query point, in a
            *		random order*/
        bool query(const Vec2 &query, std::vector<int> *indices)const;

        // setInputCloud
        void SetInputCloud(CloudWithNormalPtr &Cloud);

        ////
        // Types

        /** @brief 2D vector type. */
      

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
                        key_value.x * 73856093L ^ key_value.y * 19349663L ^ 0 * 83492791L
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

            inline static SCell get(const Vec2 &position, float gridsize)
            {
                bool nx = position.x()<0, ny = position.y()<0;
                return{ (long)(position.x() / gridsize) - long(nx),
                    (long)(position.y() / gridsize) - long(ny) };
            }
        };


protected:

    ////
    // Data members


    /** @brief Array of all referenced points. */
    std::vector<size_t> points;

    /**
    * @brief
    *		Sparse grid organizing the @link #points points @endlink for fast
    *		neighborhood query, implemented as a hash map.
    */
    std::unordered_map<SCell, std::vector<size_t>, typename SCell::hash> grid;

    /** @brief Grid cell edge length of the @link #grid grid @endlink . */
    float grid_size;
    CloudWithNormalPtr InputCloud;

};