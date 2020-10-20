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
 * Cloud point initialization algorithm
 */
class PointCloudInitializerUniform
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

    inline uint64_t len(void)
    {
        return _M;
    }

    void operator()(Point& p)
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
    typedef shared_ptr<PointCloud> PointCloudPtr;

    static PointCloudPtr CreatePointCloud(PointCloudInitializerUniform& init)
    {
        return PointCloudPtr(new PointCloud(init));
    }

    PointCloud(PointCloudInitializerUniform& init)
    {
        _points.resize(init.len());

        for_each(_points.begin(), _points.end(), init);
    }

    void display(ostream& stream)
    {
        auto print = [&stream](Point& p) { stream << p.x << " " << p.y << endl; };

        for_each(_points.begin(), _points.end(), print);
    }

private:

    PointArray _points;
};

} // namespace Cloud
