#include <iostream>
using namespace std;
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>

int main()
{
    Eigen::Matrix<float,2,3>matrix_23;
    matrix_23<<1,2,3,4,5,6;
    //cout<<matrix_23<<endl;
    
    Eigen::Vector3d v_3d=Eigen::Vector3d::Zero();
    Eigen::Matrix<float,3,1> vd_3d;
    //cout<<v_3d<<endl;
    //cout<<v_3d(2,0)<<endl;
    v_3d<<3,2,1;
    vd_3d<<4,5,6;
    cout<<v_3d<<endl;
    cout<<vd_3d<<endl;
    
    Eigen::Matrix<double,2,1>result=matrix_23.cast<double>()*v_3d;
    cout<<result;
    Eigen::Matrix<float,2,1>result2=matrix_23*vd_3d;
    //cout<<result2<<endl;


    return 0;
}
