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

    QuickHull qh;
    qh(pc);

    return 0;
}
