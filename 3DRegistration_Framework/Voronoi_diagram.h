#pragma once
#include <vector>
#include <Eigen/Core>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>

// includes for defining the Voronoi diagram adaptor
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>

class Voronoidiagram
{
public:
    // typedefs for defining the adaptor
    typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
    typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
    typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
    typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
    typedef CGAL::Voronoi_diagram_2<DT, AT, AP>                                    VD;
    // typedef for the result type of the point location
    typedef AT::Site_2                    Site_2;
    typedef AT::Point_2                   Point_2;
    typedef VD::Locate_result             Locate_result;
    typedef VD::Vertex_handle             Vertex_handle;
    typedef VD::Face_handle               Face_handle;
    typedef VD::Halfedge_handle           Halfedge_handle;
    typedef VD::Ccb_halfedge_circulator   Ccb_halfedge_circulator;

    typedef CGAL::Polygon_2<CGAL::Cartesian<double> > Polygon;

    /** \brief Compute the area of the Voronoi Cell corresponding to the first
    * point.
    *
    *  This function creates a Voronoi diagram from the given points using the
    *  CGAL library. Afterwards, it computes the area of the Voronoi Cell
    *  that contains the first point in the array when possible.
    *
    *  \param[in] points Array of 2D points representing the center of the Voronoi
    *  cells.
    *  \return The area of the Voronoi Cell containing the first point of the
    *  array.
    */
    static float Area(const std::vector<Eigen::Vector2f>& points);
};
