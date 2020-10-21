#pragma once

#include <iostream>
#include <cinttypes>
#include <vector>
#include <set>
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

    bool operator<(const Point& p) const
    {
        return x < p.x;
    }

    bool operator>(const Point& p) const
    {
        return x > p.x;
    }

    double x;
    double y; 
};


/**
 * A dynamically allocated contiguous array of Point
 */
using PointArray = vector<Point>;

/**
 * An ordered set of Point
 */
using PointSet = set<Point>;

/**
 * Compute the Euclidean distance between two points
 * P1 and P2 where P1 = (x1, y1) and P2 = (x2, y2).
 *
 * distance(P1, P2) = sqrt((y2 - y1)^2 + (x2 - x1)^2)
 */
double distance2D(const Point& P1, const Point& P2)
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
double distanceToLine2D(const Point& P1, const Point& P2, const Point& C)
{
    double num = abs((P2.y-P1.y)*C.x - (P2.x-P1.x)*C.y + P2.x*P1.y - P2.y*P1.x);
    double det = distance2D(P1, P2);

    return num / det;
}

typedef enum {
    SIDE_RIGHT = 0,
    SIDE_LEFT = 1,
    SIDE_UNKNOWN = 2
} SegmentSide;

/**
 * Compute the side of the points P0 given the segment (P1, P2)
 * where P0=(x0, y0), P1=(x1, y1), P2=(x2, y2)
 *
 * d = (x0-x1)(y2-y1) * (y0-y1)(x2-x1)
 * 
 * if d < 0 then P0 is on the right side
 * if d > 0 then P0 is on the left side
 */
SegmentSide determineSide2D(const Point& P1, const Point& P2, const Point& P0)
{
    double val = (P0.x-P1.x)*(P2.y-P1.y) - (P0.y-P1.y)*(P2.x-P1.x);

    if (val > 0)
        return SIDE_RIGHT;

    if (val < 0)
        return SIDE_LEFT;

    return SIDE_UNKNOWN;
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
     * PointCloud constructor using predefined point array
     */
    PointCloud(PointArray& pa) : _points(pa) {}

    /**
     * PointCloud constructor using initializer
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
     * Return the point array for this cloud
     */
    PointArray getPointArray(void) { return _points; }

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

    PointSet operator()(PointCloud::PointCloudPtr& points) { return _process(points); }

private:

    virtual PointSet _process(PointCloud::PointCloudPtr& points) { return PointSet(); }
};


/**
 * Quick hull implementation
 */
class QuickHull : public ConvexHullAlgo
{
public:

    QuickHull() : _hull() {}

    PointSet operator()(PointCloud::PointCloudPtr& points) { return _process(points); }

    inline uint64_t getXminIndex(void) { return _xmin; }
    inline uint64_t getXmaxIndex(void) { return _xmax; }

private:

    virtual PointSet _process(PointCloud::PointCloudPtr& points)
    {
        _pa = points->getPointArray();

        // determine the min and max value over x axis
        uint64_t n = _pa.size();
        for(uint64_t i = 1; i < n; ++i)
        {
            if(_pa[i].x < _pa[_xmin].x) {
                _xmin = i;
            }
            if(_pa[i].x > _pa[_xmax].x) {
                _xmax = i;
            }
        }

        // process recursively each side of the segment created by xmin, xmax
        _processRecurse(_pa[_xmin], _pa[_xmax], SIDE_RIGHT);
        _processRecurse(_pa[_xmin], _pa[_xmax], SIDE_LEFT);

        return _hull;
    }

    void _processRecurse(const Point& P1, const Point& P2, int side)
    {
        // index of most distant point from point array
        int i = 0;
        int index = -1;
        double max_distance = 0;

        // 1. Determine the left/right SegmentSide of each points contained in 
        //    the point array as segmented by (P1, P2) axis. Only the SegmentSide s
        //    Shall be considered
        // 2. For all point on the SegmentSide s, determine the Point P0 that is the
        //    furthest of the (P1, P2) axis.
        auto findFurthest = [&i, &index, &max_distance, P1, P2, side](Point& P0)
        { 
            auto s = determineSide2D(P1, P2, P0);
            if(s == side) {
                auto distance = distanceToLine2D(P1, P2, P0);
                if(distance > max_distance) {
                    index = i;
                    max_distance = distance;
                }
            } 

            ++i;
        };
        for_each(_pa.begin(), _pa.end(), findFurthest);

        // Stop condition for the recursion
        // Correspond to the condition where both Points are part of the convex hull
        if(index == -1) {
            _hull.insert(P1);
            _hull.insert(P2);
            return;
        }

        // 1. recursively process the right/left side of the segment for both segment
        //    (P1, P0)  and  (P0, P2)
        _processRecurse(_pa[index], P1, determineSide2D(_pa[index], P2, P1));
        _processRecurse(_pa[index], P2, determineSide2D(_pa[index], P1, P2));
    }

    PointArray _pa;
    PointSet _hull;

    uint64_t _xmin = 0;
    uint64_t _xmax = 0;
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

    /**
     * Dump internal points set into provided output stream
     */
    void write(ostream& stream)
    {
        auto print = [&stream](const Point& p) { stream << p.x << " " << p.y << endl; };

        for_each(_hull.begin(), _hull.end(), print);
    }

private:

    PointSet _hull;
};

} // namespace Cloud
