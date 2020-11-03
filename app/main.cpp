#include "PointCloud.hpp"

#include <iomanip>
#include <iostream>
#include <fstream>
#include <functional>
#include <array>

using namespace Cloud;
using namespace std;

int main()
{
    cout << "*********************************************" << endl;
    cout << "*              pcloud demo app              *" << endl;
    cout << "*********************************************" << endl;
    cout << endl;

    const string CLOUD_CSV_PATH = "/tmp/pcloud_cloud.csv";
    const string HULL_CSV_PATH = "/tmp/pcloud_hull.csv";

    cin >> noskipws; 

    cout << "Enter number of points to be inserted [3, ...)" << endl;
    uint64_t nbPoints = 0;
    cin >> nbPoints;
    cout << "Nb points: " << nbPoints << endl << endl;

    auto carray = CloudModeArray
    {
        CloudModeParams(nbPoints, 0.0, 0.25, 0.0, 0.25),
        CloudModeParams(nbPoints, 0.25, 0.4, 0.25, 0.6),
        CloudModeParams(nbPoints, 0.4, 0.70, 0.6, 0.8),
        CloudModeParams(nbPoints, 0.7, 1.0, 0.8, 1.0),
    };

    auto pc = PointCloud::CreatePointCloud<PointCloudInitializerUniform>(carray);

    auto fcloud = ofstream(CLOUD_CSV_PATH);
    pc->write(fcloud);

    cout << "Writing generated point cloud in " << CLOUD_CSV_PATH << endl;

    array<function<ConvexHull::ConvexHullPtr(PointCloud::PointCloudPtr)>, 2> arr = 
    {
        ConvexHull::CreateConvexHull<QuickHull>,
        ConvexHull::CreateConvexHull<GiftWrapping>
    };

    cout << "Enter algorithm selection:" << endl;
    cout << "0: QuickHull" << endl;
    cout << "1: GiftWrapping" << endl;
    uint64_t algo = 0;
    cin.clear();
    cin.ignore();
    cin >> algo;
   
    if(algo > 1)
        throw runtime_error("Algo cannot be larger than 1");
    
    // auto hull = ConvexHull::CreateConvexHull<QuickHull>(pc);
    // auto hull = ConvexHull::CreateConvexHull<GiftWrapping>(pc);
    auto hull = arr[algo](pc);
    auto fhull = ofstream(HULL_CSV_PATH);
    hull->write(fhull);

    cout << "Writing computed convex hull in " << HULL_CSV_PATH << endl;

    cout << endl << "Elastic lenght: " << hull->getPerimeter() << endl;

    cout << endl << "Run ./python/plot_cloud.py for visualization" << endl;

    return 0;
}
