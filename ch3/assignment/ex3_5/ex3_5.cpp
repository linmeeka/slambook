#include <iostream>
#include <Eigen/Core>

using namespace std;

int main()
{
    Eigen::Matrix<float,7,9>mat0;
    mat0=Eigen::MatrixXf::Random(7,9);
    cout<<"mat0:"<<endl<<mat0<<endl;

    Eigen::Matrix<float,3,3>mat1=mat0.block(0,0,3,3);
    cout<<"mat1:"<<endl<<mat1<<endl;
    
    mat1=Eigen::MatrixXf::Identity(3,3);
    cout<<"mat1:"<<endl<<mat1<<endl;
    return 0;
}
