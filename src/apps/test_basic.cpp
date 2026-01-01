// test_basic.cpp - 修正目标位置标记
#include "targetSim.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

int main() {
    std::cout << "靶子仿真基础测试程序" << std::endl;
    std::cout << "=====================" << std::endl;
    
    // 创建靶子仿真器
    TargetSim sim(800, 600, cv::Scalar(220, 220, 220));
    
    std::cout << "1. 测试五角星靶子生成..." << std::endl;
    
    // 测试1：生成固定位置、默认颜色的五角星靶子
    cv::Mat frame1 = sim.generate_pentagon_frame(
        cv::Point2f(400, 300),  // 中心位置
        0.0f,                   // 无旋转
        0                       // 目标索引0
    );
    
    // 获取第一个帧的色块坐标
    auto centroids1 = sim.get_blob_centroids();
    
    // 测试2：生成随机位置、旋转30度的五角星靶子
    cv::Point2f random_pos1 = sim.get_random_position();
    cv::Mat frame2 = sim.generate_pentagon_frame(
        random_pos1,           // 随机位置
        M_PI / 6,             // 旋转30度
        2                      // 目标索引2
    );
    auto centroids2 = sim.get_blob_centroids();
    
    // 测试3：生成单色块目标
    std::cout << "2. 测试单色块目标生成..." << std::endl;
    cv::Point2f blob_pos(200, 150);
    cv::Mat frame3 = sim.generate_single_blob_frame(blob_pos, 40);
    auto centroids3 = sim.get_blob_centroids();
    
    // 测试4：生成另一个随机位置的五角星靶子
    cv::Point2f random_pos2 = sim.get_random_position();
    cv::Mat frame4 = sim.generate_pentagon_frame(
        random_pos2,           // 随机位置
        M_PI / 4,             // 旋转45度
        3                      // 目标索引3
    );
    auto centroids4 = sim.get_blob_centroids();
    
    // 测试5：使用指定目标索引生成五角星靶子
    cv::Mat frame5 = sim.generate_pentagon_frame(
        cv::Point2f(600, 450),  // 位置
        M_PI / 3,              // 旋转60度
        4                      // 目标索引4
    );
    auto centroids5 = sim.get_blob_centroids();
    
    // 输出所有色块中心坐标
    std::cout << "3. 色块中心坐标：" << std::endl;
    std::vector<std::vector<cv::Point2f>> all_centroids = {
        centroids1, centroids2, centroids3, centroids4, centroids5
    };
    
    for (size_t frame_idx = 0; frame_idx < all_centroids.size(); frame_idx++) {
        std::cout << "  帧" << (frame_idx + 1) << " (" << all_centroids[frame_idx].size() << "个色块):" << std::endl;
        for (size_t i = 0; i < all_centroids[frame_idx].size(); i++) {
            auto& pt = all_centroids[frame_idx][i];
            std::cout << "    色块" << i << ": (" << pt.x << ", " << pt.y << ")" << std::endl;
        }
    }
    
    // 测试获取训练用颜色组合
    std::cout << "4. 测试颜色组合生成..." << std::endl;
    cv::Scalar center_color;
    std::vector<cv::Scalar> surround_colors;
    int target_idx;
    
    TargetSim::get_training_colors(center_color, surround_colors, target_idx);
    
    std::cout << "  中心颜色: BGR(" << center_color[0] << ", " 
              << center_color[1] << ", " << center_color[2] << ")" << std::endl;
    std::cout << "  目标索引: " << target_idx << std::endl;
    std::cout << "  外围颜色数: " << surround_colors.size() << std::endl;
    
    // 创建窗口并显示图像
    cv::namedWindow("1. 中心位置五角星", cv::WINDOW_NORMAL);
    cv::namedWindow("2. 随机位置五角星(30度)", cv::WINDOW_NORMAL);
    cv::namedWindow("3. 单色块目标", cv::WINDOW_NORMAL);
    cv::namedWindow("4. 随机位置五角星(45度)", cv::WINDOW_NORMAL);
    cv::namedWindow("5. 指定位置五角星(60度)", cv::WINDOW_NORMAL);
    
    // 在图像上添加文字标签
    cv::putText(frame1, "1. 中心位置五角星", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame2, "2. 随机位置五角星(30度)", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame3, "3. 单色块目标", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame4, "4. 随机位置五角星(45度)", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    cv::putText(frame5, "5. 指定位置五角星(60度)", cv::Point(10, 30), 
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 0), 2);
    
    // 修正目标标记 - 使用保存的坐标数据
    // 帧1：目标索引0（第一个外围色块）
    if (centroids1.size() > 1) {
        cv::circle(frame1, centroids1[1], 10, cv::Scalar(0, 255, 255), 3);
        cv::putText(frame1, "TARGET", 
                   cv::Point(centroids1[1].x + 15, centroids1[1].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    
    // 帧2：目标索引2（第三个外围色块）
    if (centroids2.size() > 3) {
        cv::circle(frame2, centroids2[3], 10, cv::Scalar(0, 255, 255), 3);
        cv::putText(frame2, "TARGET", 
                   cv::Point(centroids2[3].x + 15, centroids2[3].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    
    // 帧3：单色块目标
    if (centroids3.size() > 0) {
        cv::circle(frame3, centroids3[0], 12, cv::Scalar(0, 255, 255), 3);
        cv::putText(frame3, "TARGET", 
                   cv::Point(centroids3[0].x + 15, centroids3[0].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    
    // 帧4：目标索引3（第四个外围色块）
    if (centroids4.size() > 4) {
        cv::circle(frame4, centroids4[4], 10, cv::Scalar(0, 255, 255), 3);
        cv::putText(frame4, "TARGET", 
                   cv::Point(centroids4[4].x + 15, centroids4[4].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    
    // 帧5：目标索引4（第五个外围色块）
    if (centroids5.size() > 5) {
        cv::circle(frame5, centroids5[5], 10, cv::Scalar(0, 255, 255), 3);
        cv::putText(frame5, "TARGET", 
                   cv::Point(centroids5[5].x + 15, centroids5[5].y - 10),
                   cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 2);
    }
    
    // 显示所有图像
    std::cout << "\n5. 显示图像（按任意键继续）..." << std::endl;
    
    cv::imshow("1. 中心位置五角星", frame1);
    cv::imshow("2. 随机位置五角星(30度)", frame2);
    cv::imshow("3. 单色块目标", frame3);
    cv::imshow("4. 随机位置五角星(45度)", frame4);
    cv::imshow("5. 指定位置五角星(60度)", frame5);
    
    cv::waitKey(0);
    
    // 逐个显示详细信息
    std::cout << "\n6. 逐个显示（按任意键显示下一个）..." << std::endl;
    
    const char* window_names[] = {
        "1. 中心位置五角星",
        "2. 随机位置五角星(30度)", 
        "3. 单色块目标",
        "4. 随机位置五角星(45度)",
        "5. 指定位置五角星(60度)"
    };
    
    cv::Mat frames[] = {frame1, frame2, frame3, frame4, frame5};
    
    for (int i = 0; i < 5; i++) {
        std::cout << "  显示: " << window_names[i] << std::endl;
        cv::imshow(window_names[i], frames[i]);
        cv::waitKey(0);
    }
    
    cv::destroyAllWindows();
    
    std::cout << "\n测试完成!" << std::endl;
    std::cout << "画布尺寸: " << sim.get_width() << "x" << sim.get_height() << std::endl;
    std::cout << "画布中心: (" << sim.get_center().x << ", " << sim.get_center().y << ")" << std::endl;
    
    return 0;
}