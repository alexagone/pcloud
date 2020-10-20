#include "PointCloud.hpp"

#include <iostream>
#include <fstream>

using namespace Cloud;
using namespace std;

int main()
{
    auto init = PointCloudInitializerUniform();
    auto pc = PointCloud::CreatePointCloud(init);

    auto fcloud = ofstream("/tmp/cloud.csv");
    pc->write(fcloud);

    Point a{0.0, 2.0};
    Point b(a);
    Point c = b;
    cout << b.x << " " << b.y << endl;
    cout << c.x << " " << c.y << endl;

    return 0;
}
