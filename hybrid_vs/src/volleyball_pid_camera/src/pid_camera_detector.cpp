/*******************************************************************************
 * @file detector.cpp
 * @brief 排球检测器实现，基于OpenVINO的YOLO模型推理与3D定位
 *******************************************************************************/

#include "volleyball_pid_camera/pid_camera_detector.hpp"

namespace volleyball
{
/*****************************************************************
 * @brief 构造函数：加载OpenVINO模型、设置预处理管道、编译模型
 *****************************************************************/
PidCameraDetector::PidCameraDetector(const rclcpp::Logger logger, const std::string& model_path, const cv::Size model_input_shape, 
                   const float confidence_threshold, const float NMS_threshold)
  : logger_(logger)
  , model_input_shape_(model_input_shape)
  , confidence_threshold_(confidence_threshold)
  , NMS_threshold_(NMS_threshold)
{
    // 加载模型
    std::shared_ptr<ov::Model> model = core_.read_model(model_path);
    if (model->is_dynamic())
    {
        model->reshape(
            { 1, 3, static_cast<long int>(model_input_shape_.height), static_cast<long int>(model_input_shape_.width) });
    }
    // 设置自动预处理：BGR u8 → RGB f32，除以255归一化
    ov::preprocess::PrePostProcessor ppp_(model);
    ppp_.input()
        .tensor()
        .set_element_type(ov::element::u8)
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);
    ppp_.input()
        .preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale({ 255, 255, 255 });
    ppp_.input().model().set_layout("NCHW");
    ppp_.output().tensor().set_element_type(ov::element::f32);
    // 重建模型并编译到CPU
    model = ppp_.build();
    // 编译模型
    compiled_model_ = core_.compile_model(model, "AUTO:CPU,GPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    // 请求推理
    infer_request_ = compiled_model_.create_infer_request();

    short width, height;
    // 读取模型的输入/输出shape
    const std::vector<ov::Output<ov::Node>> inputs = model->inputs();
    const ov::Shape input_shape = inputs[0].get_shape();
    height = input_shape[1];
    width = input_shape[2];
    model_input_shape_ = cv::Size2f(width, height);

    const std::vector<ov::Output<ov::Node>> outputs = model->outputs();
    const ov::Shape output_shape = outputs[0].get_shape();
    height = output_shape[1];
    width = output_shape[2];
    model_output_shape_ = cv::Size(width, height);
}

/*****************************************************************
 * @brief 图像预处理：缩放至模型输入尺寸并拷贝到输入张量
 *****************************************************************/
void PidCameraDetector::PreProcessing(const cv::Mat& img)
{
    auto start = std::chrono::system_clock::now();

    // 1.计算letterbox缩放比例
    letterbox_scale_ = std::min(model_input_shape_.width / img.cols, model_input_shape_.height / img.rows);

    // 2.计算缩放后的宽高
    const int new_w = static_cast<int>(img.cols * letterbox_scale_);
    const int new_h = static_cast<int>(img.rows * letterbox_scale_);

    // 3.等比例缩放
    cv::Mat resized;
    cv::resize(img, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    // 4.计算填充位置及大小
    padding_x_ = (model_input_shape_.width - new_w) * 0.5f;
    padding_y_ = (model_input_shape_.height - new_h) * 0.5f;

    // 5.创建画布
    cv::Mat letter_box_img(model_input_shape_,CV_8UC3, cv::Scalar(144,144,144));
    resized.copyTo(letter_box_img(cv::Rect(static_cast<int>(padding_x_), static_cast<int>(padding_y_), new_w, new_h)));

    ov::Tensor input_tensor = infer_request_.get_input_tensor();
    uint8_t* input_data = input_tensor.data<uint8_t>();
    size_t byte_to_cp = letter_box_img.total() * letter_box_img.elemSize();
    memcpy(input_data, letter_box_img.data, byte_to_cp);

    auto end = std::chrono::system_clock::now();
    auto time =  std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    RCLCPP_DEBUG(logger_, "预处理耗费时间：%.1ldus", time.count());
}

/*****************************************************************
 * @brief 后处理：解析模型输出，提取检测框并执行NMS
 *****************************************************************/
void PidCameraDetector::PostProcessing(std::vector<DetectionBox>& detection_box_list)
{
    auto start = std::chrono::system_clock::now();

    std::vector<int> class_list;
    std::vector<float> confidence_list;
    std::vector<cv::Rect> box_list;

    const int D = model_output_shape_.width; // 候选框
    const int C = model_output_shape_.height - 4; //类别数

    // 预分配内存
    class_list.reserve(D);
    confidence_list.reserve(D);
    box_list.reserve(D);

    // 获取推理结果
    const float* data = infer_request_.get_output_tensor().data<const float>();

    for (int i = 0; i < D; ++i)
    {
        // 现在为每一个候选框找到最大置信度类别
        float max_score = 0.0f;
        int best_class = -1;
        for (int c = 0; c < C; ++c) 
        {
            const float score = data[(4+c)*D + i];
            if (score > max_score) 
            {
                max_score = score;
                best_class = c;
            }
        }
        // 检查是否满足置信度阈值要求
        if (max_score > confidence_threshold_)
        {
            class_list.push_back(best_class);
            confidence_list.push_back(max_score);

            const float cx = data[i];
            const float cy = data[D + i];
            const float w = data[2 * D + i];
            const float h = data[3 * D + i];

            cv::Rect box;
            box.x = static_cast<int>((cx - w * 0.5f));
            box.y = static_cast<int>((cy - h * 0.5f));
            box.width = static_cast<int>(w);
            box.height = static_cast<int>(h);
            box_list.push_back(box);
        }
    }
    // 非极大值抑制，去除重叠框
    std::vector<int> NMS_result;
    cv::dnn::NMSBoxes(box_list, confidence_list, confidence_threshold_, NMS_threshold_, NMS_result);

    // 对NMS后保留的每个框，缩放回原始图像尺寸
    for (const int id : NMS_result)
    {
        DetectionBox result;
        result.class_id = class_list[id];
        result.confidence = confidence_list[id];
        result.box = GetBoundingBox(box_list[id]);
        result.cx = static_cast<float>(result.box.x + result.box.width * 0.5f);
        result.cy = static_cast<float>(result.box.y + result.box.height * 0.5f);
        detection_box_list.push_back(result);
    }

    auto end = std::chrono::system_clock::now();
    auto time =  std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    RCLCPP_DEBUG(logger_, "后处理耗费时间：%.1ldus", time.count());
}

/*****************************************************************
 * @brief 将检测框从模型输出尺寸缩放回原始图像尺寸
 *****************************************************************/
cv::Rect PidCameraDetector::GetBoundingBox(const cv::Rect& src) const
{
  cv::Rect box = src;
  box.x = static_cast<int>((box.x - padding_x_) / letterbox_scale_);
  box.y = static_cast<int>((box.y - padding_y_) / letterbox_scale_);
  box.width = static_cast<int>(box.width / letterbox_scale_);
  box.height = static_cast<int>(box.height / letterbox_scale_);
  return box;
}

/*****************************************************************
 * @brief 执行完整推理流程：预处理→推理→后处理
 *****************************************************************/
std::vector<DetectionBox> PidCameraDetector::infer(const cv::Mat& input_img)
{
  auto start = std::chrono::system_clock::now();
  PreProcessing(input_img);
  infer_request_.infer();
  std::vector<DetectionBox> detection_box_list;
  PostProcessing(detection_box_list);
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  if (!detection_box_list.empty())
  {
    for (auto detect_box : detection_box_list)
    {
        RCLCPP_DEBUG(logger_, 
            "推理时间:%.1fms, 检测到目标，置信度:%.2f, x:%d, y:%d, w:%d, h:%d",
            static_cast<double>(duration.count()), detect_box.confidence, detect_box.box.x, detect_box.box.y, detect_box.box.width, detect_box.box.height);
    }
  }
  return detection_box_list;
}

/*****************************************************************
 * @brief 在图像上绘制检测框和置信度
 *****************************************************************/
void PidCameraDetector::drawDetectResult(cv::Mat& color_img, const std::vector<DetectionBox>& detect_box_list)
{
    for (auto detect_box : detect_box_list)
    {
        //标出检测框
        cv::rectangle(color_img, detect_box.box, cv::Scalar(255, 0, 0));
        //显示置信度
        std::string text = "volleyball:" + std::to_string(detect_box.confidence);
        int font_face = cv::FONT_HERSHEY_COMPLEX;
        double font_scale = 0.5;
        int thickness = 1;
        int baseline;
        //获取文本框的长宽
        cv::Size text_size = cv::getTextSize(text, font_face, font_scale, thickness, &baseline);
        cv::Point origin;
        origin.x = detect_box.box.x;
        origin.y = detect_box.box.y + text_size.height;
        cv::putText(color_img, text, origin, font_face, font_scale, cv::Scalar(0, 255, 255), thickness);
    }
}

/*****************************************************************
 * @brief 执行检测：推理→获取检测框→计算3D位置
 *****************************************************************/
void PidCameraDetector::detect(const cv::Mat& color_img)
{
    detection_box_list_ = infer(color_img);
    ball_list_.clear();
}

}  // namespace volleyball
