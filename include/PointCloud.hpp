/*
 *  Copyright (C) 2004-2020
 *
 *  Author: Alexandre Savard <alexagone@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.
 */

#pragma once

#include <set>
#include <stack>
#include <random>
#include <vector>
#include <memory>
#include <iostream>
#include <cinttypes>
#include <algorithm>
#include <cassert>

using namespace std;

namespace Cloud {

/**
 * Define PI for further computation
 */
constexpr double pi() { return std::atan(1)*4; }

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

    Point& operator+=(const Point& p)
    {
        x += p.x;
        y += p.y;

        return *this;
    }

    Point& operator-=(const Point& p)
    {
        x -= p.x;
        y -= p.y;

        return *this;
    }

    Point& operator*=(const Point& p)
    {
        x *= p.x;
        y *= p.y;

        return *this;
    }

    Point& operator*=(const double s)
    {
        x *= s;
        y *= s;

        return *this;  
    }

    Point& operator/=(const double s)
    {
        x /= s;
        y /= s;

        return *this;
    }

    double x;
    double y; 
};

ostream& operator<<(ostream& os, const Point& p)
{
    os << "(" << p.x << ',' << p.y << ")";
    
    return os;
}

/**
 * Define a binary minus operation for Point class
 */
Point operator-(const Point& lh, const Point& rh)
{
    return Point{lh.x-rh.x, lh.y-rh.y};
}

/**
 * Define a binary add operation for Point class
 */
Point operator+(const Point& lh, const Point& rh)
{
    return Point{lh.x+rh.x, lh.y+rh.y};
}

/**
 * Define a binary multiplication of a point by a scalar
 */
Point operator*(const Point& lh, const float alpha)
{
    return Point{lh.x*alpha, lh.y*alpha};
}

/**
 * Define a binary multiplication of a point by a scalar (reverse order)
 */
Point operator*(const float alpha, const Point& lh)
{
    return Point{lh.x*alpha, lh.y*alpha};
}

/**
 * Define comparison operator
 */
bool operator==(const Point& lh, const Point& rh)
{
    return (lh.x == lh.y) && (rh.x == rh.y);
}

/**
 * Compute the convex combination of two points P3, given P1 and P2
 *
 *  P3 = alpha * P1 + (1 - alpha) * P2
 */
Point convexCombination(const Point& P1, const Point& P2, const float alpha)
{
    return (P1 * alpha) + (P2 * (1.0 - alpha));
}

/**
 * Compute the cross product of two vector at origin
 *
 *   det ((x1, y1), (x2, y2))
 *
 * If p1 x p2 is positive, then p1 is clockwise from p2 with respect to the origin,
 * if cross product is negative, then p1 is counterclockwise from p2.
 *
 * We can also interpret the cross product p1 p2 as the signed area of the parallelogram 
 * formed by the points (0, 0), p1, p2, and p1 + p2 = (p1.x + p2.x, p1.y + p2.y).
 */
double crossProduct(const Point& P1, const Point& P2)
{
    return (P1.x*P2.y) - (P2.x*P1.y);
}

/**
 * Compute the cross product of two vector centered at (x0, y0)
 *
 *   det ((x1-x0, y1-y0), (x2-x0, y2-y0))
 */

double crossProduct(const Point& P1, const Point& P2, const Point& P0)
{
    return (P1.x-P0.x)*(P2.y-P0.y) - (P2.x-P0.x)*(P1.y-P0.y);
}

/**
 * A dynamically allocated contiguous array of Point
 */
using PointArray = vector<Point>;

/**
 * LIFO data structure for Points
 */
using PointStack = stack<Point>;

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

bool intersect2D(const Point& P1, const Point& P2, const Point& P3, const Point& P4)
{
    auto onSegment = [](const Point& P1, const Point& P2, const Point& P0)
    {
        if ( ((min(P1.x, P2.x) <= P0.x) && (max(P1.x, P2.x) >= P0.x)) &&
             ((min(P1.y, P2.y) <= P0.y) && (max(P1.y, P2.y) >= P0.y)) ) {
            return true;
        }
        else {
            return false;
        }
    };

    auto d1 = crossProduct(P4, P1, P3);
    auto d2 = crossProduct(P4, P2, P3);
    auto d3 = crossProduct(P2, P3, P1);
    auto d4 = crossProduct(P2, P4, P1);


    if( (((d1>0) && (d2<0)) || ((d1<0) && (d2>0))) &&
        (((d3>0) && (d4<0)) || ((d3<0) && (d4>0))) ) {
        return true;
    }
    else if( (d1 == 0) && onSegment(P3, P4, P1) ) {
        return true;
    }
    else if( (d2 == 0) && onSegment(P3, P4, P2) ) {
        return true;
    }
    else if( (d3 == 0) && onSegment(P1, P2, P3) ) {
        return true;
    } 
    else if( (d4 == 0) && onSegment(P1, P2, P4) ) {
        return true;
    }
    else
    {
        return false;
    }
}

class SortPolarCoord
{
public:

    SortPolarCoord(PointArray& pa) : _origin{0.0, 0.0}
    {
        // compute the center position of the provided PointArray
        for(auto& p : pa) {
            _origin += p;
        }

        _origin.x /= (double)pa.size();
        _origin.y /= (double)pa.size();
    }

    SortPolarCoord(Point& origin) : _origin(origin) {}

    bool operator()(const Point& r, const Point& l)
    {
        rr = r - _origin;
        ll = l - _origin;

        return _get_angle(rr) > _get_angle(ll);
    }

private:

    double _get_angle(Point& p)
    {
        if(p.y >= 0.0)
            return atan2(p.y, p.x);
        else
            return 2*pi() + atan2(p.y, p.x);
    }

    Point _origin;
    Point rr;
    Point ll;
};

/**
 * Side definition, used to determine if points are right or left from a reference position
 */ 
typedef enum {
    ORIENTATION_CLOCKWISE = 0,
    ORIENTATION_COUNTERCLOCKWISE = 1,
    ORIENTATION_UNKNOWN = 2 // ex: colinear vectors
} SegmentOrientation;

/**
 * Compute the side of the points P0 given the segment (P1, P2)
 * where P0=(x0, y0), P1=(x1, y1), P2=(x2, y2)
 *
 * d = (x0-x1)(y2-y1) * (y0-y1)(x2-x1)
 * 
 * if d < 0 then P0 is on the right side
 * if d > 0 then P0 is on the left side
 */
SegmentOrientation determineOrientation2D(const Point& P1, const Point& P2, const Point& P0)
{
    double val = crossProduct(P0, P2, P1);

    if (val > 0)
        return ORIENTATION_CLOCKWISE;

    if (val < 0)
        return ORIENTATION_COUNTERCLOCKWISE;

    return ORIENTATION_UNKNOWN;
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

struct CloudModeParams
{

    constexpr static const uint64_t DEFAULT_M = 100;
    constexpr static const double DEFAULT_XMIN = 0.0;
    constexpr static const double DEFAULT_XMAX = 1.0;
    constexpr static const double DEFAULT_YMIN = 0.0;
    constexpr static const double DEFAULT_YMAX = 1.0;

    const uint64_t M;
    const double xmin;
    const double xmax;
    const double ymin;
    const double ymax;

    uniform_real_distribution<> distx;
    uniform_real_distribution<> disty;

    CloudModeParams(
        const uint64_t M = DEFAULT_M,
        const double xmin = DEFAULT_XMIN,
        const double xmax = DEFAULT_XMAX,
        const double ymin = DEFAULT_YMIN,
        const double ymax = DEFAULT_YMAX
    ) :
        M(M),
        xmin(xmin),
        xmax(xmax),
        ymin(ymin),
        ymax(ymax),
        distx(xmin, xmax),
        disty(ymin, ymax)
    {}

};

typedef vector<CloudModeParams> CloudModeArray;


/**
 * PointCloud initialization algorithm using uniform distribution
 */
class PointCloudInitializerUniform : public PointCloudInitializer
{
public:

    PointCloudInitializerUniform(CloudModeArray modes) : _mt((random_device())()), _modes(modes)
    {
        for(const auto& m : _modes) {
            _M += m.M;
        }
    }

    PointCloudInitializerUniform(CloudModeParams cp=CloudModeParams()) : _mt((random_device())()), _modes{cp}
    {

        _M = _modes[0].M;

        if(_M == 0) {
            throw runtime_error("error, specified number to generate point cloud cannot be 0");
        }
    }

    virtual inline uint64_t len(void)
    {
        return _M;
    }

    virtual inline void gen(Point& p)
    {
        if(_count >= _modes[_index].M) {
            _count = 0;

            ++_index; 
            if(_index >= _modes.size()) {
                throw runtime_error("Error, mode index larger than the current number of modes");
            }
        }

        p.x = _modes[_index].distx(_mt);
        p.y = _modes[_index].disty(_mt);

        ++_count;
    }

private:

    mt19937 _mt;
    CloudModeArray _modes;

    uint64_t _M = 0;
    uint64_t _index = 0;
    uint64_t _current = 0;
    uint64_t _count = 0;
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

    template <typename Init>
    static PointCloudPtr CreatePointCloud(CloudModeArray modes)
    {
        Init init(modes);
        return make_shared<PointCloud>(init);
    }

    /**
     * Factory method to create a new point cloud using specified initializer
     */
    template <typename Init>
    static PointCloudPtr CreatePointCloud(CloudModeParams cp=CloudModeParams())
    {
        Init init(cp);
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

    QuickHull() : _hull() {}

    inline uint64_t getXminIndex(void) { return _xmin; }
    inline uint64_t getXmaxIndex(void) { return _xmax; }

private:

    virtual PointArray _process(PointCloud::PointCloudPtr& points)
    {
        PointArray pa = points->getPointArray();

        uint64_t i = 0;
        auto x_minmax = [this, &i, pa](Point& p)
        {
            if(p.x < pa[_xmin].x) {
                _xmin = i;
            }
            if(p.x > pa[_xmax].x) {
                _xmax = i;
            }
            ++i;
        };

        for_each(pa.begin(), pa.end(), x_minmax);

        // process recursively each side of the segment created by xmin, xmax
        _processRecurse(pa, pa[_xmin], pa[_xmax], ORIENTATION_CLOCKWISE);
        _processRecurse(pa, pa[_xmin], pa[_xmax], ORIENTATION_COUNTERCLOCKWISE);

        // sort the final result
        _sortClockWise();

        return _hullSorted;
    }

    void _processRecurse(PointArray& pa, const Point& P1, const Point& P2, int side)
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
            auto s = determineOrientation2D(P1, P2, P0);
            if(s == side) {
                auto distance = distanceToLine2D(P1, P2, P0);
                if(distance > max_distance) {
                    index = i;
                    max_distance = distance;
                }
            } 

            ++i;
        };
        for_each(pa.begin(), pa.end(), findFurthest);

        // Stop condition for the recursion
        // Correspond to the condition where both Points are part of the convex hull
        if(index == -1) {
            _hull.insert(P1);
            _hull.insert(P2);
            return;
        }

        // 1. recursively process the right/left side of the segment for both segment
        //    (P1, P0)  and  (P0, P2)
        _processRecurse(pa, pa[index], P1, 1-determineOrientation2D(pa[index], P1, P2));
        _processRecurse(pa, pa[index], P2, 1-determineOrientation2D(pa[index], P2, P1));
    }

    /**
     * Sort hull Point contained inside the PointArray
     */
    void _sortClockWise(void)
    {
        copy(_hull.begin(), _hull.end(), back_inserter(_hullSorted));

        sort(_hullSorted.begin(), _hullSorted.end(), SortPolarCoord(_hullSorted));
    }

    PointSet _hull;
    PointArray _hullSorted;

    uint64_t _xmin = 0;
    uint64_t _xmax = 0;
};

/**
 * Gift wrapping implementation
 */
class GiftWrapping : public ConvexHullAlgo
{
public:

    GiftWrapping() : _hull() {}

    inline uint64_t getXminIndex(void) { return _xmin; }

private:

    virtual PointArray _process(PointCloud::PointCloudPtr& points)
    {
        _hull.clear();

        PointArray pa = points->getPointArray();

        uint64_t i = 0;
        auto x_min = [this, &i, pa](Point& p)
        {
            if(p.x < pa[_xmin].x) {
                _xmin = i;
            }
            ++i;
        };

        for_each(pa.begin(), pa.end(), x_min);

        uint64_t n = pa.size();
        uint64_t p = _xmin, q;
        do
        {
            _hull.push_back(pa[p]);

            // selecting the next point
            q = (p+1)%n;

            for(uint64_t i = 0; i < n; ++i)
            {
                if(determineOrientation2D(pa[p], pa[i], pa[q]) == ORIENTATION_COUNTERCLOCKWISE)
                    q = i;
            }
            p = q;
        }
        while(p != _xmin);

        _sortClockWise();

        return _hull;
    }

    void _sortClockWise(void)
    {
        sort(_hull.begin(), _hull.end(), SortPolarCoord(_hull));
    }

    PointArray _hull;

    uint64_t _xmin = 0;
};

/**
 * GrahamScan
 */
class GrahamScan : public ConvexHullAlgo
{
public:

    GrahamScan() : _hull() {}

    inline uint64_t getYminIndex(void) { return _ymin; }

private:

    virtual PointArray _process(PointCloud::PointCloudPtr& points)
    {
        _hull.clear();

        PointArray pa = points->getPointArray();

        uint64_t i = 0;
        auto y_min = [this, &i, pa](Point& p)
        {
            if(p.x < pa[_ymin].x) {
                _ymin = i;
            }
            ++i;
        };

        // Find minimum over y axis
        for_each(pa.begin(), pa.end(), y_min);

        // Use the back inserter to populate the PointArray data structure
        copy(pa.begin(), pa.end(), back_inserter(_hull));

        // Sort counter clockwise points in reference to the y minimum location
        sort(_hull.begin(), _hull.end(), SortPolarCoord(pa[_ymin]));

        // Populate the stack, considering all points except minimum over the y axis
        std::stack<Point> hull_stack(std::deque<Point>(_hull.begin(), _hull.end()-1));


        return _hull;
    }

    PointArray _hull;

    uint64_t _ymin = 0;
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
        _computePerimeter();
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
        return _hullPerimeter;
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

    void _computePerimeter(void)
    {
        if(_hull.size() <= 2) {
            throw runtime_error("error, perimeter of 2 or less points is undefined");
        }

        _hullPerimeter = distance2D(_hull.front(), _hull.back());

        uint64_t n = _hull.size();
        for(uint64_t i = 1; i < n; ++i) {
            _hullPerimeter += distance2D(_hull[i], _hull[i-1]);
        }
    }

    PointArray _hull;
    double _hullPerimeter = 0.0;
};

} // namespace Cloud
