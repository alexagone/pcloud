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
