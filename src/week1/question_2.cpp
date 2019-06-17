#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.h>

using namespace std;

/**
 * 第二题编程验证对于一个小量的旋转omega,通过四元数或者旋转矩阵来更新旋转时实际效果一致
 * 运行结果见附图question_2.png
 */
int main(int argc, char **argv)
{
    //初始旋转矩阵以绕轴(1,1,3)旋转90度为例
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 1, 3).normalized()).toRotationMatrix();
    //旋转矩阵->四元数
    Eigen::Quaterniond q(R);

    //角速度
    Eigen::Vector3d omega(0.01, 0.02, 0.03);

    //R<-R * exp(omega_skew_symmetric_matrix)
    Eigen::Matrix3d updated_R = R * Sophus::SO3::exp(omega).matrix();
    Eigen::Quaterniond q_R(updated_R);

    cout << "通过旋转矩阵更新后的q:" << endl
         << q_R.coeffs() << endl;

    //q<-q * [1,0.5*omega].transpose()
    Eigen::AngleAxisd vector_r(1, omega / 2);
    Eigen::Quaterniond q_delta(vector_r);
    Eigen::Quaterniond q_q = q * q_delta;

    //以四元数来比较更新后旋转差异
    q_q.normalize();
    cout << "通过四元数更新后的q:" << endl
         << q_q.coeffs() << endl;
    Eigen::Vector4d diff = q_R.coeffs() - q_q.coeffs();
    cout << "以四元数形式来比较二者差距很小:" << endl
         << diff << endl;

    //以旋转矩阵来比较更新后的旋转差异
    cout << "R1: " << endl
         << updated_R << endl;
    cout << "R2: " << endl
         << q_q.matrix() << endl;

    cout << "以旋转矩阵来比较二者差距还是很小: " << endl
         << updated_R - q_q.matrix() << endl;
}