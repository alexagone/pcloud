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

TEST(TestPoint, TestSegmentSide)
{
    Point p1{0.0, 4.0};
    Point p2{4.0, 0.0};

    Point pleft{1.0, 1.0};
    Point pright{3.0, 3.0};

    EXPECT_EQ(determineSide2D(p1, p2, pright), SIDE_LEFT);
    EXPECT_EQ(determineSide2D(p1, p2, pleft), SIDE_RIGHT);
}

TEST(TestPointCloud, TestInitialization)
{
    auto init = PointCloudInitializerUniform();
    auto pc = PointCloud::CreatePointCloud(init);

    EXPECT_EQ(pc->getNbElements(), 100);
}

TEST(TestConvexHull, TestInitialization)
{
    auto init = PointCloudInitializerUniform();
    auto pc = PointCloud::CreatePointCloud(init);

    auto convexhull = ConvexHull::CreateConvexHull<QuickHull>(pc);
}

TEST(TestConvexHull, TestTrivialCase)
{
    PointArray pa{{0.59, 0.61},
                  {0.78, 0.54},
                  {0.33, 0.63},
                  {0.63, 0.59},
                  {0.34, 0.58},
                  {0.28, 0.52}};

    auto pc = make_shared<PointCloud>(pa);

    QuickHull qh;
    auto hull = qh(pc);

    EXPECT_EQ(qh.getXminIndex(), 5);
    EXPECT_EQ(qh.getXmaxIndex(), 1);

    Point p0{0.78, 0.54};
    Point p1{0.28, 0.52};
    Point p2{0.33, 0.63};
    Point p3{0.59, 0.61};

    EXPECT_EQ(hull[0].x, p0.x);
    EXPECT_EQ(hull[0].y, p0.y);
    EXPECT_EQ(hull[1].x, p1.x);
    EXPECT_EQ(hull[1].y, p1.y);
    EXPECT_EQ(hull[2].x, p2.x);
    EXPECT_EQ(hull[2].y, p2.y);
    EXPECT_EQ(hull[3].x, p3.x);
    EXPECT_EQ(hull[3].y, p3.y);
}


TEST(TestConvexHull, TestPerimeter)
{
    PointArray pa{{0.59, 0.61},
                  {0.78, 0.54},
                  {0.33, 0.63},
                  {0.63, 0.59},
                  {0.34, 0.58},
                  {0.28, 0.52}};

    auto pc = make_shared<PointCloud>(pa);

    auto convexhull = ConvexHull::CreateConvexHull<QuickHull>(pc);

    EXPECT_EQ(convexhull->getNbElements(), 4);
    EXPECT_LT(fabs(convexhull->getPerimeter() - 1.08448), 1e-4);
}
