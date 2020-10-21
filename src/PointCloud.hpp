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
 * Compute the distance of a point C = (x0, y0) to a line define 
 * by two points P1 and P2 where P1 = (x1, y1) and P2 = (x2, y2). 
 *
 * distanceToLine(P1, P2, C) = | (y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1 |
 *                             -------------------------------------------
 *                                   sqrt((y2 - y1)^2 + (x2 - x1)^2)
 */
double distanceToLine2D(Point& P1, Point& P2, Point& C)
{
    double num = abs((P2.y-P1.y)*C.x - (P2.x-P1.x)*C.y + P2.x*P1.y - P2.y*P1.x);
    double det = distance2D(P1, P2);

    return num / det;
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
     * Return the number of elements contained by this point cloud
     */
    uint64_t getNbElements(void)
    {
        return _points.size();
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
 * Base class for convex hull determiantion algorithm
 */
class ConvexHullAlgo
{
public:

    PointArray operator()(PointCloud::PointCloudPtr& points) { return _process(points); }

private:

    virtual PointArray _process(PointCloud::PointCloudPtr& points) { return PointArray(); }
};

/**
 * Quick hull implementation
 */
class QuickHull : public ConvexHullAlgo
{
public:

    PointArray operator()(PointCloud::PointCloudPtr& points) { return _process(points); }

private:

    virtual PointArray _process(PointCloud::PointCloudPtr& points)
    {     
        return PointArray();
    }
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
    template<typename Algo>
    static ConvexHullPtr CreateConvexHull(PointCloud::PointCloudPtr points)
    {
        Algo algo;
        return make_shared<ConvexHull>(algo, points);
    }

    /**
     * ConvexHull constructor
     */
    ConvexHull(ConvexHullAlgo& algo, PointCloud::PointCloudPtr points)
    {
        _hull = algo(points);
    }

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
