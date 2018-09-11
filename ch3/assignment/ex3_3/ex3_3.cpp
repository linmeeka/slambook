#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;
int main()
{
    Eigen::Quaterniond q1(0.35,0.2,0.3,0.1);
    q1=q1.normalized();
    Eigen::Quaterniond p(0,0.1,0.2,0.3);
    cout<<"p"<<endl<<p.coeffs()<<endl;
    Eigen::Quaterniond p1=q1*p*q1.inverse();
    cout<<"p after rotate:"<<endl<<p1.coeffs()<<endl;

    return 0;
}

