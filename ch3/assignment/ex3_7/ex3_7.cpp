#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
int main()
{
    Eigen::Quaterniond q1(0.35,0.2,0.3,0.1);
    Eigen::Quaterniond q2(-0.5,0.4,-0.1,0.2);
    Eigen::Vector3d t1(0.3,0.1,0.1);
    Eigen::Vector3d t2(-0.1,0.5,0.3);
    Eigen::Vector3d p1(0.5,0,0.2);

    q1=q1.normalized();
    q2=q2.normalized();

    Eigen::Vector3d p0=q1.inverse()*(p1-t1);
    Eigen::Vector3d p2=q2*p0+t2;

    cout<<p2<<endl;
    return 0;
}

