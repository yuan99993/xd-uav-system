#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <sar_yolo_detector/inference_backend.hpp>

namespace sar_yolo_detector {
namespace {

TEST(InferenceBackend, TestBackendProducesRawYoloV8Rows) {
  InferenceBackendConfig config;
  config.backend = "test";
  config.class_count = 2;
  config.yolov5_layout = false;
  config.test_detections = {{320.0F, 300.0F, 100.0F, 200.0F, 0.9F, 1}};
  std::unique_ptr<InferenceBackend> backend = createInferenceBackend(config);

  int dimensions[4] = {1, 3, 640, 640};
  const cv::Mat blob(4, dimensions, CV_32F, cv::Scalar::all(0));
  const std::vector<cv::Mat> outputs = backend->infer(blob);
  ASSERT_EQ(outputs.size(), 1U);
  ASSERT_EQ(outputs.front().dims, 3);
  EXPECT_EQ(outputs.front().size[0], 1);
  EXPECT_EQ(outputs.front().size[1], 1);
  EXPECT_EQ(outputs.front().size[2], 6);
  const float* row = outputs.front().ptr<float>();
  EXPECT_FLOAT_EQ(row[0], 320.0F);
  EXPECT_FLOAT_EQ(row[1], 300.0F);
  EXPECT_FLOAT_EQ(row[2], 100.0F);
  EXPECT_FLOAT_EQ(row[3], 200.0F);
  EXPECT_FLOAT_EQ(row[4], 0.0F);
  EXPECT_FLOAT_EQ(row[5], 0.9F);
}

TEST(InferenceBackend, TestBackendProducesRawYoloV5Rows) {
  InferenceBackendConfig config;
  config.backend = "test";
  config.class_count = 2;
  config.yolov5_layout = true;
  config.test_detections = {{100.0F, 120.0F, 40.0F, 60.0F, 0.8F, 0}};
  std::unique_ptr<InferenceBackend> backend = createInferenceBackend(config);

  int dimensions[4] = {1, 3, 640, 640};
  const cv::Mat blob(4, dimensions, CV_32F, cv::Scalar::all(0));
  const std::vector<cv::Mat> outputs = backend->infer(blob);
  ASSERT_EQ(outputs.size(), 1U);
  const float* row = outputs.front().ptr<float>();
  EXPECT_FLOAT_EQ(row[4], 0.8F);
  EXPECT_FLOAT_EQ(row[5], 1.0F);
  EXPECT_FLOAT_EQ(row[6], 0.0F);
}

TEST(InferenceBackend, TensorRtFailureIsExplicitWhenNotBuilt) {
  if (tensorRtBackendAvailable()) GTEST_SKIP();
  InferenceBackendConfig config;
  config.backend = "tensorrt";
  config.engine_path = "/tmp/not-an-engine.plan";
  EXPECT_THROW(createInferenceBackend(config), std::runtime_error);
  EXPECT_NE(tensorRtBackendAvailabilityMessage().find("unavailable"),
            std::string::npos);
}

}  // namespace
}  // namespace sar_yolo_detector

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
