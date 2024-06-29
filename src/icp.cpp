#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
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
int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 6)
    {
        std::cout << "Usage: " << argv[0] << " <source_cloud.pcd> <target_cloud.pcd> <output_cloud.pcd> <output_matrix.txt>  <truth.txt>" << std::endl;
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

    // 初始化ICP对象
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    // 设置ICP参数
    icp.setMaximumIterations(100);             // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-8);       // 设置变换收敛阈值
    icp.setEuclideanFitnessEpsilon(1e-8);     // 设置欧几里得距离收敛阈值
    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    //将变换矩阵重新赋值
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    // 输出ICP配准后的变换矩阵
    std::cout << "ICP transformation matrix:" << std::endl;
    std::cout << transformation << std::endl;
    // 真值变换矩阵
    Eigen::Matrix4f truth_matrx = readTransformationMatrixFromFile(argv[5]);
    double err = computeMatrixDifference(truth_matrx,transformation);
    cout<<"Difference"<<err<<endl;
    //点云之间均方根误差
    double fitness_score = icp.getFitnessScore();
    std::cout << "配准后的点云与目标点云之间的均方根误差(RMSE)为: " << fitness_score << std::endl;
    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("ICP Visualization");
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(&Final);
    // 添加源点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 255, 0, 0); // 红色
    viewer.addPointCloud(source_cloud, source_color, "source_cloud");
    // 添加文本标签并设置颜色为红色
   viewer.addText("Source Cloud", 10, 570, 25 ,1, 0, 0,"source_text"); // 红色字体，黑色背景
    
    // 添加目标点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 0, 0, 255); // 蓝色
    viewer.addPointCloud(target_cloud, target_color, "target_cloud");
    // 添加文本标签并设置颜色为红色
   viewer.addText("Target Cloud", 10, 540, 25 ,0, 0, 1,"target_text"); // 红色字体，黑色背景

    
    // 添加配准后的Final点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(result, 0, 255, 0); // 绿色
    viewer.addPointCloud(result, final_color, "Final");
    viewer.addText("Transformed Cloud", 10, 510,25,0,1,0, "transformed_text"); // 添加文本标签

    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "source_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "target_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5, "Final");
    // 设置窗口大小
    viewer.setSize(800, 600);
    // 开始可视化循环
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

}