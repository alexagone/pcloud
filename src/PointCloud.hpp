#pragma once

#include <iostream>
#include <cinttypes>
#include <vector>
#include <algorithm>
#include <random>
#include <memory>

using namespace std;

namespace Cloud {


/**
 * 2d point coordinate
 *
 * x coordinate for this point
 * y coordinate for this point
 */
struct Point
{
    Point(double x=0.0, double y=0.0) : x(x), y(y) {}

    Point(const Point& p)
    {
        x = p.x;
        y = p.y;
    }

    Point& operator=(const Point& p)
    {
        x = p.x;
        y = p.y;

        return *this;
    }

    double x;
    double y; 
};


/**
 * A dynamically allocated contiguous array of Point
 */
using PointArray = vector<Point>;

/**
 * Compute the Euclidean distance between two points
 * P1 and P2 where P1 = (x1, y1) and P2 = (x2, y2).
 *
 * distance(P1, P2) = sqrt((y2 - y1)^2 + (x2 - x1)^2)
 */
double distance2D(Point& P1, Point& P2)
{
    return sqrt((P2.y-P1.y)*(P2.y-P1.y) + (P2.x-P1.x)*(P2.x-P1.x));
}

/**
 * PointCloud initializer base class
 */
class PointCloudInitializer
{
public:
    virtual ~PointCloudInitializer() {}

    virtual inline uint64_t len(void) { return 0; };
    virtual inline void gen(Point& p) { p.x = 0.0; p.y = 0.0; };

    void operator()(Point& p) { gen(p); };

};

/**
 * PointCloud initialization algorithm using uniform distribution
 */
class PointCloudInitializerUniform : public PointCloudInitializer
{
public:

    constexpr static const uint64_t DEFAULT_M = 100;
    constexpr static const double DEFAULT_XMIN = 0.0;
    constexpr static const double DEFAULT_XMAX = 1.0;
    constexpr static const double DEFAULT_YMIN = 0.0;
    constexpr static const double DEFAULT_YMAX = 1.0;

    PointCloudInitializerUniform(
        const uint64_t M=DEFAULT_M,
        const double xmin=DEFAULT_XMIN,
        const double xmax=DEFAULT_XMAX,
        const double ymin=DEFAULT_YMIN,
        const double ymax=DEFAULT_YMAX
    ) :
        _M(M),
        _xmin(xmin),
        _xmax(xmax),
        _ymin(ymin),
        _ymax(ymax),
        _mt((random_device())()),
        _distx(_xmin, _xmax),
        _disty(_ymin, _ymax)
    {
    }

    virtual inline uint64_t len(void)
    {
        return _M;
    }

    virtual inline void gen(Point& p)
    {
        p.x = _distx(_mt); p.y = _disty(_mt);
    }

private:
    const uint64_t _M;
    const double _xmin;
    const double _xmax;
    const double _ymin;
    const double _ymax;

    mt19937 _mt;
    uniform_real_distribution<> _distx;
    uniform_real_distribution<> _disty;
};


/**
 * Main point cloud class that encapsulate memory management, containement,
 * and initialization.
 */
class PointCloud
{
public:
    /**
     * shared pointer type definition
     */ 
    typedef shared_ptr<PointCloud> PointCloudPtr;

    /**
     * Factory method to create a new point cloud using specified initializer
     */
    static PointCloudPtr CreatePointCloud(PointCloudInitializer& init)
    {
        return make_shared<PointCloud>(init);
    }

    /**
     * PointCloud  constructor
     */
    PointCloud(PointCloudInitializer& init)
    {
        _points.resize(init.len());

        for_each(_points.begin(), _points.end(), std::ref(init));
    }

    /**
     * Dump internal points array into provided output stream
     */
    void write(ostream& stream)
    {
        auto print = [&stream](Point& p) { stream << p.x << " " << p.y << endl; };

        for_each(_points.begin(), _points.end(), print);
    }

private:

    PointArray _points;
};

/**
 * Class representing extracted convex hull,
 * provided a corresponding point cloud
 */
class ConvexHull
{
public:

    /**
     * shared pointer type definition
     */
    typedef shared_ptr<ConvexHull> ConvexHullPtr;

    /**
     * Factory method to create a new convex hull out of a point cloud
     */
    static ConvexHullPtr CreateConvexHull(PointCloud::PointCloudPtr points)
    {
        return make_shared<ConvexHull>(points);
    }

    /**
     * ConvexHull constructor
     */
    ConvexHull(PointCloud::PointCloudPtr points) {}

    /**
     * Query the number of element that compose the hull
     */
    uint64_t getNbElements(void)
    {
        return _hull.size();
    }

    /**
     * Query the final length of the perimeter delimited by the hull
     */
    double getPerimeter(void)
    {
        return 0.0;
    }

private:

    PointArray _hull;
};

} // namespace Cloud
