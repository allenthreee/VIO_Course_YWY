//
// Created by hyj on 18-11-11.
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;

    Eigen::Vector2d uv;    // 这帧图像观测到的特征坐标
};
int main()
{

    int poseNums = 10;
    double radius = 8;
    double fx = 1.;
    double fy = 1.;
    std::vector<Pose> camera_pose;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // 1/4 圆弧
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成 1 个 三维特征点
    std::default_random_engine generator;
    std::uniform_real_distribution<double> xy_rand(-4, 4.0);
    std::uniform_real_distribution<double> z_rand(8., 10.);
    double tx = xy_rand(generator);
    double ty = xy_rand(generator);
    double tz = z_rand(generator);

    Eigen::Vector3d Pw(tx, ty, tz);
    // 这个特征从第三帧相机开始被观测，i=3
    int start_frame_id = 3;
    int end_frame_id = poseNums;
    for (int i = start_frame_id; i < end_frame_id; ++i) {
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);

        double x = Pc.x();
        double y = Pc.y();
        double z = Pc.z();

        const double mean = 0.0;         // 均值
        const double stddev = 0.005;       // 标准差 standard deviation
        std::normal_distribution<double> noise(mean,stddev);

        Pc = Pc/Pc[2];
        Pc[0] += noise(generator);      // 生成一个随机的误差
        Pc[1] += noise(generator);   

        camera_pose[i].uv = Eigen::Vector2d(Pc[0],Pc[1]);
    }
    
    /// TODO::homework; 请完成三角化估计深度的代码
    // 遍历所有的观测数据，并三角化
    Eigen::Vector3d P_est;           // 结果保存到这个变量
    P_est.setZero();
    /* your code begin */
    const auto D_rows = 2*(end_frame_id-start_frame_id);
    Eigen::MatrixXd D(Eigen::MatrixXd::Zero(D_rows, 4));
    // 构建矩阵 D
    for(auto i=start_frame_id;i<end_frame_id;++i)
    {
        // 我对这个下标的含义一直有点模糊，感觉好像是跟机器人学里面的不太一样
        // 可能机械臂那些更多是考虑机械臂末端相对世界坐标系
        // 而VIO则是考虑通过路标点得到camera 和 landmarks 在世界坐标系中的位置
        Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
        Eigen::Vector3d tcw = -Rcw *camera_pose[i].twc;
        D.block(2*(i-start_frame_id), 0,1,3).noalias() =    // u(像素坐标)*R第三行 - R第一行
                camera_pose[i].uv(0)*Rcw.block(2,0,1,3)-Rcw.block(0,0,1,3);
        D.block(2*(i-start_frame_id),3,1,1).noalias() =     // 最后一列是 translation
                camera_pose[i].uv(0)*tcw.segment(2,1)-tcw.segment(0,1);
        D.block(2*(i-start_frame_id)+1, 0,1,3).noalias() = 
                camera_pose[i].uv(1)*Rcw.block(2,0,1,3)-Rcw.block(1,0,1,3);
        D.block(2*(i-start_frame_id)+1,3,1,1).noalias() = 
                camera_pose[i].uv(1)*tcw.segment(2,1) - tcw.segment(1,1);
    }
    // 对矩阵DTD进行SVD特征值分解
    Eigen::JacobiSVD<Eigen::MatrixXd> 
            svd(D.transpose()*D,Eigen::ComputeThinU| Eigen::ComputeThinV);
    // 将四个特征值存入向量中
    Eigen::Vector4d lambda = svd.singularValues();

    if(lambda(2)/lambda(3)<1e-3) return -1;         // 比较sigma3 和 sigma4 的大小

    // SVD分解得到的结果 U 矩阵，取最后一列 u4 as we just proved
    Eigen::Vector4d u4 = svd.matrixU().block(0,3,4,1);
    if(u4(3)!=0 && u4(2)/u4(3)>0){
        P_est(0) = u4(0)/u4(3);
        P_est(1) = u4(1)/u4(3);
        P_est(2) = u4(2)/u4(3);
    }
    std::cout << "the result of lambda4/lambda3 is: " << lambda(3)/lambda(2) << std::endl;; 
    /* your code end */
    
    std::cout <<"ground truth: \n"<< Pw.transpose() <<std::endl;
    std::cout <<"your result: \n"<< P_est.transpose() <<std::endl;
    // TODO:: 请如课程讲解中提到的判断三角化结果好坏的方式，绘制奇异值比值变化曲线
    return 0;
}
