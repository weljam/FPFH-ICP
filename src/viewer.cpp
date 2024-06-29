#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc != 4)
    {
        std::cout << "Usage: " << argv[0] << " <source_cloud.pcd> <target_cloud.pcd> <transformation_matrix.txt>" << std::endl;
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

    // 读取变换矩阵
    Eigen::Matrix4f transformation_matrix;
    std::ifstream matrix_file(argv[3]);
    if (matrix_file.is_open())
    {
        for (int i = 0; i < 4; ++i)
        {
            for (int j = 0; j < 4; ++j)
            {
                matrix_file >> transformation_matrix(i, j);
            }
        }
        matrix_file.close();
    }
    else
    {
        std::cout << "Couldn't open transformation matrix file." << std::endl;
        return -1;
    }

    // 应用变换矩阵到源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*target_cloud, *transformed_source, transformation_matrix);
    pcl::transformPointCloud(*source_cloud, *transformed_target, transformation_matrix);

    // 创建可视化对象
    pcl::visualization::PCLVisualizer viewer("Point Cloud Viewer");

    // 设置背景颜色为黑色
    viewer.setBackgroundColor(0.0, 0.0, 0.0);
    // 添加源点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(source_cloud, 255, 0, 0); // 红色
    viewer.addPointCloud(source_cloud, source_color, "source_cloud");
    // 添加目标点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 0, 255,0); // 绿色
    viewer.addPointCloud(target_cloud, target_color, "target_cloud");
    // 添加变换后的源点云到可视化对象
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_color(transformed_source, 0, 0, 255); // 蓝色
    viewer.addPointCloud(transformed_source, transformed_color, "transformed_source");

    // 设置窗口大小
    viewer.setSize(800, 600);

    // 开始可视化循环
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}