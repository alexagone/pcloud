#include "PointCloud.hpp"

#include <gtest/gtest.h>

using namespace Cloud;

TEST(TestPoint, TestCopyAndAssignment)
{
    Point p1{23.0, 443.23};
    Point p2(p1);
    Point p3 = p2;

    EXPECT_EQ(p1.x, p2.x);
    EXPECT_EQ(p1.x, p3.x);
    EXPECT_EQ(p1.y, p2.y);
    EXPECT_EQ(p1.y, p3.y);
}

TEST(TestPoint, TestDistance2d)
{
    Point p1{0.0, 0.0};
    Point p2{2.0, 2.0};

    EXPECT_LT(abs(distance2D(p1, p2) - sqrt(8.0)), 1e-9);
}

TEST(TestPoint, TestDistanceToLine2d)
{
    Point p1{0.0, 4.0};
    Point p2{4.0, 0.0};
    Point c{4.0, 4.0};

    EXPECT_LT(abs(distanceToLine2D(p1, p2, c) - sqrt(8.0)), 1e-9);
}
