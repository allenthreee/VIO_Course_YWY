// 通过四元数和旋转矩阵两种不同的方式更新旋转
// 结果应该相差不大
#include <iostream>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "sophus/se3.hpp"
#include <math.h>
using namespace std;
/**
 *   1. 定义 q & R
 *  2. define rotaion vector w = [0.01, 0.02, 0.03]T
 * 
 * 3. 分别用两种方法更新旋转
 *       q = q * [1, 1/2 * w]T
 *       R = R * exp(w^)
 *       eigen中无法做矩阵乘法，或许用罗德里格斯公式
 *       exp(theta a) = cos(theta)I + (1 - cos(theta))aaT + sin(theta)a^
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
    R = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
    Eigen::Quaterniond q_eigen = Eigen::Quaterniond(R);   // quaterniond
    double trace_R = R(0,0) + R(1,1) + R(2,2);
    std::cout << "the trace of R is  " << trace_R << endl; 
    double w_quaterniond = sqrt(trace_R + 1) / 2;
    double z = (R(1,0) - R(0,1)) / (4 * w_quaterniond);
    Eigen::Quaterniond q_handmade( w_quaterniond, 0, 0, z );  // 实部在前，虚部在后？？
    // 为什么构造函数和显示的时候顺序不一样啊，因为Eigen::Quaterniond q(w, x, y, z) 的构造函数
    // 单数如果构造函数是 Eigen::Quaterniond q(Eigen::vector4d(x, y, z, w))
    // 但是显示就是 Eigen::Quaterniond q(x, y, z, w); 
    // 这个确实比较奇怪

    // Eigen::Quaterniond q_handmade((R(2,1)-R(1,2)), (R(0,2) - R(2,0)), (R(1,0)-R(0,1)), sqrt(trace_R+1)/2);

    // q_eigen = Eigen::Quaterniond(R);      // 由旋转矩阵构造四元数，其他形式到四元数的转换

    Sophus::SO3d SO3_R(R);  //SO3李群，由旋转矩阵R构造

    cout << "R is = " << endl << R << endl;
    cout << "q transformed by Eigen is = " << q_eigen.coeffs().transpose() << endl; // 实部在后，虚部在前
    cout << "q transformed by handmade is = " << q_handmade.coeffs().transpose() << endl; // 实部在后，虚部在前

    cout << "R  from SO3_R is = " << endl << SO3_R.matrix() << endl;  // 李群创建的旋转矩阵

    Eigen::Vector3d so3 = SO3_R.log();

    // 旋转向量微小扰动用w表示
    Eigen::Vector3d w(0.01, 0.02, 0.03);



    // 向量的模长
    double theta = sqrt(0.01*0.01 + 0.02*0.02 + 0.03*0.03);
    std::cout << "theta is : " << theta << endl;
    // 向量的方向
    Eigen::Vector3d a(0.01/theta, 0.02/theta, 0.03/theta);
    
    // ***********************罗德里格斯公式实现 update*******************************//
    Eigen::Matrix3d update_matrix;
    // Rodrigues Formula:
    // R = cos(theta)I + (1 - cos(theta)aaT  + sin(theta)a^)
    // 罗德里格斯公式 part 1
    Eigen::Matrix3d cos_theta_I;
    cos_theta_I << cos(theta), 0,             0,
                                    0,         cos(theta),     0,
                                    0,                    0,    cos(theta);
    // 罗德里格斯公式 part 2
    Eigen::Matrix3d aaT;
    aaT << a(0)*a(0),   0,               0,
                     0,          a(1)*a(1),      0,
                     0,                0,     a(2)*a(2);
    Eigen::Matrix3d second_part = (1-cos(theta)) * aaT;                                    

    // 罗德里格斯公式 part 3                                   
    Eigen::Matrix3d a_hat;
    a_hat << 0,    -a(2), a(1),
                    a(2),     0,   -a(0),
                    -a(1),  a(0),   0;
   
    Eigen::Matrix3d w_hat_exp = cos_theta_I + second_part + sin(theta) * a_hat;
    std::cout << " w_hat_exp is \n"<< w_hat_exp << endl;

    // 用handmade李代数演示更新
    R = R * w_hat_exp;
    cout << "============\n" << "handmade R by Rodrigues' Formula = \n" << R << endl;


    //***********************sophus库实现*****************************
    // cout << "so3 hat = \n" << Sophus::SO3d::hat(so3) << endl;
    // cout << "w hat = \n" << Sophus::SO3d::hat(w) << endl;
    // 用李代数演示更新
    Sophus::SO3d updated_SO3_R = SO3_R * Sophus::SO3d::exp(w);
    cout << "============\n" << "updated_SO3_R = \n" << updated_SO3_R.matrix() << endl;
    
    //***********************四元数实现*****************************
    // 用四元数演示更新
     // 使用q方式储存
    Eigen::Quaterniond q_ = Eigen::Quaterniond (1, 0.5*w(0), 0.5*w(1), 0.5*w(2));
    // 更新q
    q_eigen = q_eigen * q_;
    q_eigen.normalize(); // 归一化
    cout << "============\n" << "updated_q = \n" << q_eigen.toRotationMatrix() << endl;

    


    // // 三种方法之间的差值

    // // 1. sophus 和 四元数
    // Eigen::Matrix3d diff = q_eigen.toRotationMatrix() - updated_SO3_R.matrix();
    // cout << "============\n" << " the difference between 2 methods is: \n" 
    // << diff << endl;

    // 肉眼可以看出三者之间的差值较小，因此就步分别做差了，否则还要两两比对，输出三个矩阵

    



    return 0;
}


