#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
//从文件中读取真值
Eigen::Matrix4f readTransformationMatrixFromFile(const std::string& filename) {
    Eigen::Matrix4f matrix;
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        throw std::runtime_error("无法打开文件");
    }
    std::string line;
    int i = 0;
    while (std::getline(file, line)) {
        // Skip empty lines and comments starting with #
        if (line.empty() || line[0] == '#') {
            continue;
        }
        // Check if line contains descriptive text and skip it
        if (line.find("格式：PCD") != std::string::npos || 
            line.find("点数") != std::string::npos || 
            line.find("由源点云变换到目标点云的旋转平移矩阵") != std::string::npos || 
            line.find("在X轴方向上旋转了45°") != std::string::npos) {
            continue;
        }
        // Trim leading and trailing spaces
        line.erase(0, line.find_first_not_of(' '));
        line.erase(line.find_last_not_of(' ') + 1);
        // Parse the matrix data
        std::istringstream iss(line);
        float values[4];
        int j = 0;
        while (iss >> values[j] && j < 4) {
            j++;
        }
        if (j == 4) {
            for (int k = 0; k < 4; ++k) {
                matrix(i, k) = values[k];
            }
            ++i;
        }
        if (i == 4) break;
    }
    if (i != 4) {
        std::cerr << "文件格式错误: " << filename << " - Expected 4 lines but got " << i << std::endl;
        throw std::runtime_error("文件格式错误");
    }
    file.close();
    return matrix;
}

// 计算矩阵差的Frobenius范数
double computeMatrixDifference(const Eigen::Matrix4f &true_transformation, const Eigen::Matrix4f &estimated_transformation) {
    return (true_transformation - estimated_transformation).norm();
}
int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc != 6)
    {
        std::cout << "Usage: " << argv[0] << " <source_cloud.pcd> <target_cloud.pcd> <output_cloud.pcd> <output_matrix.txt> <truth.txt>" << std::endl;
        return -1;
    }

    // 读取源点云和目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *source_cloud) == -1 ||
        pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *target_cloud) == -1)
    {
        std::cout << "Couldn't read source or target point cloud file." << std::endl;
        return -1;
    }

    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(source_cloud);
    normal_estimator.setKSearch(80);
    normal_estimator.compute(*normals1);

    // 计算法向量
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator2;
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>);
    normal_estimator2.setSearchMethod(tree);
    normal_estimator2.setInputCloud(target_cloud);
    normal_estimator2.setKSearch(80);
    normal_estimator2.compute(*normals2);

    // 计算FPFH特征
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>);
    fpfh.setSearchMethod(tree);
    fpfh.setInputCloud(source_cloud);
    fpfh.setInputNormals(normals1);
    fpfh.setKSearch(100);
    fpfh.compute(*fpfhs1);

    fpfh.setInputCloud(target_cloud);
    fpfh.setInputNormals(normals2);
    fpfh.setKSearch(100);
    fpfh.compute(*fpfhs2);

    // 特征匹配
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> corr_est;
    corr_est.setInputSource(fpfhs1);
    corr_est.setInputTarget(fpfhs2);
    corr_est.determineReciprocalCorrespondences(*correspondences);

    // 计算变换矩阵
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    trans_est.estimateRigidTransformation(*source_cloud, *target_cloud, *correspondences, transformation);

    // 输出变换矩阵
    std::cout << "Transformation matrix:" << std::endl;
    std::cout << transformation << std::endl;

    // 计算变换后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*source_cloud, *transformed_cloud, transformation);

    // 使用FPFH特征匹配结果初始化ICP配准
    // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    // icp.setInputSource(transformed_cloud);
    // icp.setInputTarget(target_cloud);
    // icp.setMaximumIterations(500); // 设置最大迭代次数
    // icp.setTransformationEpsilon(1e-8); // 设置变换收敛阈值./
    // //icp.setEuclideanFitnessEpsilon(1e-8); // 设置欧几里得距离收敛阈值
    // //icp.setMaxCorrespondenceDistance(1.8); // 设置最大对应点之间的距离
    // icp.align(*transformed_cloud); // 使用FPFH得到的变换矩阵初始化ICP

    //计算范数距离
    Eigen::Matrix4f truth_matrx = readTransformationMatrixFromFile(argv[5]);
    double err = computeMatrixDifference(truth_matrx,transformation);
    cout<<"Difference"<<err<<endl;
    // 输出ICP配准后的变换矩阵
    std::cout << "ICP transformation matrix:" << std::endl;
    std::cout << transformation << std::endl;
    //点云之间均方根误差
    // double fitness_score = icp.getFitnessScore();
    // std::cout << "配准后的点云与目标点云之间的均方根误差(RMSE)为: " << fitness_score << std::endl;

    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("FPFH + ICP");
    // 设置背景颜色
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // 添加源点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 0, 0); // 红色
    viewer.addPointCloud(source_cloud, source_cloud_color_handler, "source_cloud");
   viewer.addText("Source Cloud", 10, 570, 25 ,1, 0, 0,"source_text"); // 红色字体，黑色背景
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "source_cloud");
    // 添加目标点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color_handler(target_cloud, 0, 0,255); // 蓝色
    viewer.addPointCloud(target_cloud, target_cloud_color_handler, "target_cloud");
    viewer.addText("Target Cloud", 10, 540, 25 ,0, 0, 1,"target_text"); // 蓝色字体，黑色背景
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "target_cloud");
    // 添加变换后的点云
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler(transformed_cloud, 0, 255, 0); // 绿色
    viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");
    viewer.addText("Transformed Cloud", 10, 510, 25, 0, 1, 0, "transformed_text"); //绿色字体，黑色背景
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "transformed_cloud");
    // 设置窗口大小
    viewer.setSize(800, 600);
    // 开始可视化循环
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return (0);
}

