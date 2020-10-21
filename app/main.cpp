#include "PointCloud.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>

using namespace Cloud;
using namespace std;

int main()
{
    cout << "*********************************************" << endl;
    cout << "*            nailedit demo app              *" << endl;
    cout << "*********************************************" << endl;
    cout << endl;

    const string CLOUD_CSV_PATH = "/tmp/nailedit_cloud.csv";
    const string HULL_CSV_PATH = "/tmp/nailedit_hull.csv";

    uint64_t nbPoints = 0;
    cout << "Enter number of nails to be inserted [3, ...)" << endl;
   
    cin >> noskipws; 
    cin >> nbPoints;
    cout << "Nb nails: " << nbPoints << endl << endl;

    auto init = PointCloudInitializerUniform(nbPoints);
    auto pc = PointCloud::CreatePointCloud(init);

    auto fcloud = ofstream(CLOUD_CSV_PATH);
    pc->write(fcloud);

    cout << "Writing generated point cloud in " << CLOUD_CSV_PATH << endl;

    auto hull = ConvexHull::CreateConvexHull<QuickHull>(pc);
    auto fhull = ofstream(HULL_CSV_PATH);
    hull->write(fhull);

    cout << "Writing computed convex hull in " << HULL_CSV_PATH << endl;

    cout << endl << "Elastic lenght: " << hull->getPerimeter() << endl;

    cout << endl << "Run ./python/plot_cloud.py for visualization" << endl;

    return 0;
}
