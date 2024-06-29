#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// 从文件中读取变换矩阵的函数
Eigen::Matrix4f readTransformationMatrixFromFile(const std::string& filename) {
    Eigen::Matrix4f matrix;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        throw std::runtime_error("无法打开文件");
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            file >> matrix(i, j);
        }
    }
    file.close();
    return matrix;
}

// 将变换矩阵转换为四元数的函数
Eigen::Quaternionf convertToQuaternion(const Eigen::Matrix4f& matrix) {
    return Eigen::Quaternionf(matrix.block<3,3>(0,0));
}

int main(int argc, char* argv[]) {
    // 检查命令行参数数量
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <transform_matrix_1.txt> <transform_matrix_2.txt>" << std::endl;
        return -1;
    }

    // 读取两个变换矩阵
    Eigen::Matrix4f transform_matrix_1 = readTransformationMatrixFromFile(argv[1]);
    Eigen::Matrix4f transform_matrix_2 = readTransformationMatrixFromFile(argv[2]);

    // 将变换矩阵转换为四元数
    Eigen::Quaternionf quat_1 = convertToQuaternion(transform_matrix_1);
    Eigen::Quaternionf quat_2 = convertToQuaternion(transform_matrix_2);

    // 输出四元数
    std::cout << "四元数1: " << quat_1.coeffs().transpose() << std::endl;
    std::cout << "四元数2: " << quat_2.coeffs().transpose() << std::endl;

    return 0;
}