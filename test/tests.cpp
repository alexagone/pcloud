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

TEST(TestPoint, TestMultiplicationByScalar)
{
    Point p1{4.0, 4.0};
    Point p2 = p1 * 0.5;

    EXPECT_EQ(p2.x, 2.0);
    EXPECT_EQ(p2.y, 2.0);

    Point p3 = 0.5 * p1;

    EXPECT_EQ(p3.x, 2.0);
    EXPECT_EQ(p3.y, 2.0);
}

TEST(TestPoint, TestConvexCombination)
{
    Point p1{2.0, 2.0};
    Point p2{6.0, 8.0};

    Point p3 = convexCombination(p1, p2, 0.5);

    EXPECT_EQ(p3.x, 4.0);
    EXPECT_EQ(p3.y, 5.0);
}

TEST(TestPoint, TestCrossProductOrigin)
{
    Point p1{2.0, 4.0};
    Point p2{8.0, 7.0};

    double cprod = crossProduct(p1, p2);

    EXPECT_EQ(cprod, -18.0);
}

TEST(TestPoint, TestCrossProduct)
{
    Point p1{4.0, 6.0};
    Point p2{10.0, 9.0};

    Point p0{2.0, 2.0};

    double cprod = crossProduct(p1, p2, p0);

    EXPECT_EQ(cprod, -18.0);
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

TEST(TestPoint, TestSegmentOrientation)
{
    Point p1{0.0, 4.0};
    Point p2{4.0, 0.0};

    Point pleft{1.0, 1.0};
    Point pright{3.0, 3.0};

    EXPECT_EQ(determineOrientation2D(p2, p1, pleft), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(p2, p1, pright), ORIENTATION_CLOCKWISE);

    EXPECT_EQ(determineOrientation2D(p1, p2, pright), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(p1, p2, pleft), ORIENTATION_CLOCKWISE);
}

TEST(TestPointCloud, TestInitialization)
{
    auto pc = PointCloud::CreatePointCloud<PointCloudInitializerUniform>();

    EXPECT_EQ(pc->getNbElements(), 100);
}

TEST(TestConvexHull, TestInitialization)
{
    auto pc = PointCloud::CreatePointCloud<PointCloudInitializerUniform>();

    auto convexhull = ConvexHull::CreateConvexHull<QuickHull>(pc);
}

TEST(TestConvexHull, TestQuickHullManual)
{
    PointArray pa{{0.59, 0.61},
                  {0.78, 0.54},
                  {0.33, 0.63},
                  {0.63, 0.59},
                  {0.34, 0.58},
                  {0.28, 0.52}};

    auto pc = make_shared<PointCloud>(pa);

    // xmin index is 5
    // xmax index is 1

    Point P1 = pa[5];
    Point P2 = pa[1];

    // determine side, all should be on side 1
    EXPECT_EQ(determineOrientation2D(P1, P2, pa[0]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(P1, P2, pa[2]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(P1, P2, pa[3]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(P1, P2, pa[4]), ORIENTATION_COUNTERCLOCKWISE);

    // furthest is index 2 with 0.107914 
    EXPECT_LT(abs(distanceToLine2D(P1, P2, pa[0]) - 0.077538), 1e-6);
    EXPECT_LT(abs(distanceToLine2D(P1, P2, pa[2]) - 0.107914), 1e-6);
    EXPECT_LT(abs(distanceToLine2D(P1, P2, pa[3]) - 0.0559553), 1e-6);
    EXPECT_LT(abs(distanceToLine2D(P1, P2, pa[4]) - 0.057554), 1e-6);

    // determineOrientation2d
    EXPECT_EQ(determineOrientation2D(pa[2], P2, P1), ORIENTATION_CLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P1, P2), ORIENTATION_COUNTERCLOCKWISE);

    // verify the side for every point of segment pa[2], P1
    EXPECT_EQ(determineOrientation2D(pa[2], P1, pa[0]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P1, pa[1]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P1, pa[3]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P1, pa[4]), ORIENTATION_COUNTERCLOCKWISE);
    // the algorithm should stop there as this segment contains only inner points

    // verify the side for every point of segment pa[2], P2
    EXPECT_EQ(determineOrientation2D(pa[2], P2, pa[0]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P2, pa[1]), ORIENTATION_CLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P2, pa[3]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[2], P2, pa[4]), ORIENTATION_CLOCKWISE);

    // determine the furthest point pa[2], P2, which is pa[0]
    EXPECT_LT(abs(distanceToLine2D(pa[2], P2, pa[0]) - 0.0313786), 1e-6);
    EXPECT_LT(abs(distanceToLine2D(pa[2], P2, pa[3]) - 0.0196116), 1e-6);

    EXPECT_EQ(determineOrientation2D(pa[0], pa[2], P2), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], P2, pa[2]), ORIENTATION_CLOCKWISE);

    EXPECT_EQ(determineOrientation2D(pa[0], P2, pa[2]), ORIENTATION_CLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], P2, pa[3]), ORIENTATION_CLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], P2, pa[4]), ORIENTATION_CLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], P2, pa[5]), ORIENTATION_CLOCKWISE);

    EXPECT_EQ(determineOrientation2D(pa[0], pa[2], pa[1]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], pa[2], pa[3]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], pa[2], pa[4]), ORIENTATION_COUNTERCLOCKWISE);
    EXPECT_EQ(determineOrientation2D(pa[0], pa[2], pa[5]), ORIENTATION_COUNTERCLOCKWISE);

    QuickHull qh;
    auto hull = qh(pc);

    EXPECT_EQ(qh.getXminIndex(), 5);
    EXPECT_EQ(qh.getXmaxIndex(), 1);

    Point p0(P2);
    Point p1(P1);
    Point p2(pa[2]);
    Point p3(pa[0]);

    EXPECT_EQ(hull[0].x, p0.x);
    EXPECT_EQ(hull[0].y, p0.y);
    EXPECT_EQ(hull[1].x, p1.x);
    EXPECT_EQ(hull[1].y, p1.y);
    EXPECT_EQ(hull[2].x, p2.x);
    EXPECT_EQ(hull[2].y, p2.y);
    EXPECT_EQ(hull[3].x, p3.x);
    EXPECT_EQ(hull[3].y, p3.y);
}

TEST(TestConvexHull, TestQuickHullTrivialCase)
{
    PointArray pa{{0.591107, 0.852247},
                  {0.771903, 0.0974454},
                  {0.104228, 0.75303},
                  {0.27302,  0.425654},
                  {0.273791, 0.173948},
                  {0.331152, 0.496003}};

    auto pc = make_shared<PointCloud>(pa);

    QuickHull qh;
    auto hull = qh(pc);

    Point p0(pa[1]);
    Point p1(pa[4]);
    Point p2(pa[2]);
    Point p3(pa[0]);

    EXPECT_EQ(hull[0].x, p0.x);
    EXPECT_EQ(hull[0].y, p0.y);
    EXPECT_EQ(hull[1].x, p1.x);
    EXPECT_EQ(hull[1].y, p1.y);
    EXPECT_EQ(hull[2].x, p2.x);
    EXPECT_EQ(hull[2].y, p2.y);
    EXPECT_EQ(hull[3].x, p3.x);
    EXPECT_EQ(hull[3].y, p3.y);
}

TEST(TestConvexHull, TestGiftWrappingTrivialCase)
{
    PointArray pa{{0.591107, 0.852247},
                  {0.771903, 0.0974454},
                  {0.104228, 0.75303},
                  {0.27302,  0.425654},
                  {0.273791, 0.173948},
                  {0.331152, 0.496003}};

    auto pc = make_shared<PointCloud>(pa);

    GiftWrapping gw;
    auto hull = gw(pc);

    Point p0(pa[1]);
    Point p1(pa[4]);
    Point p2(pa[2]);
    Point p3(pa[0]);

    EXPECT_EQ(hull[0].x, p0.x);
    EXPECT_EQ(hull[0].y, p0.y);
    EXPECT_EQ(hull[1].x, p1.x);
    EXPECT_EQ(hull[1].y, p1.y);
    EXPECT_EQ(hull[2].x, p2.x);
    EXPECT_EQ(hull[2].y, p2.y);
    EXPECT_EQ(hull[3].x, p3.x);
    EXPECT_EQ(hull[3].y, p3.y);
}

TEST(TestConvexHull, TestPerimeterEval)
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
