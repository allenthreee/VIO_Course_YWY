// 通过四元数和旋转矩阵两种不同的方式更新旋转
// 结果应该相差不大
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se3.hpp"
using namespace std;
/**
 *   1. 定义 q & R
 *  2. define rotaion vector w = [0.01, 0.02, 0.03]T
 * 3. 分别用两种方法更新旋转
 * 4. 两种方法都改为旋转矩阵显示 / 或者用欧拉角显示？？
*/


/**
 * eigen 中各种旋转表示方式
 * 1. rotation matrix(3x3) ----->  Eigen::Matrix3d
 * 2. 旋转向量(3x1) ----->  Eigen::AngleAxis 每个轴，对应转多少度
 * 3. quterniond(4x1) ------------> Eigen::Qaterniond
 * 4. translation vector(3x1)-----> Eigen::Vector3d
 * 5. transformation matrix(4x4)---> Eigen::Isometry3d
*/


int main(){
    Eigen::Matrix3d R;  // rotation matrix
    Eigen::Quaterniond q;   // quaterniond

    R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    q = Eigen::Quaterniond(R);

    Sophus::SO3d SO3_R(R);  //SO3李群，由旋转矩阵R构造

    cout << "R is = " << endl << R << endl;
    cout << "q is = " << q.coeffs().transpose() << endl; // 实部在后，虚部在前
    cout << "R  from SO3_R is = " << endl << SO3_R.matrix() << endl;  // 李群创建的旋转矩阵
    // SOPHUS中的SO3形式和eigen中一样吗？ 只是输出格式不同，还是数学的公式也不相同？

    Eigen::Vector3d so3 = SO3_R.log();

    // 旋转向量微小扰动用w表示
    Eigen::Vector3d w(0.01, 0.02, 0.03);

    // cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    cout << "w hat = \n" << Sophus::SO3d::hat(w) << endl;

    // 用四元数演示更新
     // 使用q方式储存
    Eigen::Quaterniond q_ = Eigen::Quaterniond (1, 0.5*w(0), 0.5*w(1), 0.5*w(2));
    // 更新q
    q = q * q_;
    q.normalize(); // 归一化
    cout << "============\n" << "updated_q = \n" << q.toRotationMatrix() << endl;

    // 用李代数演示更新
    Sophus::SO3d updated_SO3_R = SO3_R * Sophus::SO3d::exp(w);
    cout << "============\n" << "updated_SO3_R = \n" << updated_SO3_R.matrix() << endl;

    // 两种方法之间的差值
    Eigen::Matrix3d diff = q.toRotationMatrix() - updated_SO3_R.matrix();
    cout << "============\n" << " the difference between 2 methods is: \n" 
    << diff << endl;



    return 0;
}


