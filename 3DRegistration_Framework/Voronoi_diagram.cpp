#include"pch.h"
#include"Voronoi_diagram.h"

float Voronoidiagram::Area(const std::vector<Eigen::Vector2f>& points)
{
    assert(points.size() >= 4);

    VD vd;
    for (const auto& point : points) {
        vd.insert(Site_2(point(0), point(1)));
    }

    assert(vd.is_valid());

    Locate_result lr = vd.locate(Point_2(0.0, 0.0));

    if (Face_handle* f = boost::get<Face_handle>(&lr)) {

        Ccb_halfedge_circulator ec_start = (*f)->outer_ccb();
        Ccb_halfedge_circulator ec = ec_start;

        Polygon pl;
        do {
            if (!(ec->has_source())) {
               // std::cerr << "voronoi_cell_area : error : no source.\n";
                return 0.0;
            }

            double x = ec->source()->point().x();
            double y = ec->source()->point().y();
            pl.push_back(CGAL::Cartesian<double>::Point_2(x, y));

        } while (++ec != ec_start);

        return pl.area();
    }
    else {
        return 0.0;
    }
}
