#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include <sar_yolo_detector/yolo_postprocessor.hpp>

namespace sar_yolo_detector {
namespace {

PostprocessorConfig baseConfig() {
  PostprocessorConfig config;
  config.input_width = 640;
  config.input_height = 640;
  config.confidence_threshold = 0.25F;
  config.nms_threshold = 0.45F;
  config.minimum_box_size_px = 1.0F;
  config.maximum_detections = 20;
  config.class_names = {"person", "boat"};
  return config;
}

TEST(YoloPostprocessor, LetterboxPreservesAspectRatio) {
  const cv::Mat image(480, 640, CV_8UC3, cv::Scalar::all(0));
  LetterboxTransform transform;
  const cv::Mat result = letterboxImage(image, 640, 640, &transform);
  EXPECT_EQ(result.cols, 640);
  EXPECT_EQ(result.rows, 640);
  EXPECT_DOUBLE_EQ(transform.scale, 1.0);
  EXPECT_EQ(transform.pad_x, 0);
  EXPECT_EQ(transform.pad_y, 80);
}

TEST(YoloPostprocessor, DecodesUltralyticsChannelFirstAndClassAwareNms) {
  PostprocessorConfig config = baseConfig();
  config.model_layout = ModelLayout::kUltralytics;
  YoloPostprocessor postprocessor(config);

  const int dimensions[3] = {1, 6, 3};
  cv::Mat output(3, dimensions, CV_32F, cv::Scalar::all(0));
  float *values = output.ptr<float>();
  const auto set = [values](const int feature, const int prediction,
                            const float value) {
    values[feature * 3 + prediction] = value;
  };
  // Two overlapping people and one boat. Person NMS keeps only the stronger
  // person while class-aware NMS retains the boat at the same location.
  for (int prediction = 0; prediction < 3; ++prediction) {
    set(0, prediction, 320.0F);
    set(1, prediction, 320.0F);
    set(2, prediction, 100.0F);
    set(3, prediction, 100.0F);
  }
  set(4, 0, 0.90F);
  set(5, 0, 0.05F);
  set(4, 1, 0.80F);
  set(5, 1, 0.05F);
  set(4, 2, 0.05F);
  set(5, 2, 0.85F);

  LetterboxTransform transform;
  transform.scale = 1.0;
  const std::vector<Detection> detections =
      postprocessor.decode({output}, transform, cv::Size(640, 640));
  ASSERT_EQ(detections.size(), 2U);
  EXPECT_EQ(detections[0].class_id, 0);
  EXPECT_NEAR(detections[0].confidence, 0.90F, 1e-6F);
  EXPECT_EQ(detections[0].box, cv::Rect(270, 270, 100, 100));
  EXPECT_EQ(detections[1].class_id, 1);
}

TEST(YoloPostprocessor, DecodesYoloV5ObjectnessAndFiltersClasses) {
  PostprocessorConfig config = baseConfig();
  config.model_layout = ModelLayout::kYoloV5;
  config.allowed_class_ids = {0};
  YoloPostprocessor postprocessor(config);

  const int dimensions[3] = {1, 2, 7};
  cv::Mat output(3, dimensions, CV_32F, cv::Scalar::all(0));
  float *row0 = output.ptr<float>();
  row0[0] = 100.0F;
  row0[1] = 120.0F;
  row0[2] = 40.0F;
  row0[3] = 60.0F;
  row0[4] = 0.80F;
  row0[5] = 0.75F;
  row0[6] = 0.10F;
  float *row1 = row0 + 7;
  row1[0] = 300.0F;
  row1[1] = 300.0F;
  row1[2] = 50.0F;
  row1[3] = 50.0F;
  row1[4] = 0.95F;
  row1[5] = 0.05F;
  row1[6] = 0.95F;

  LetterboxTransform transform;
  transform.scale = 1.0;
  const std::vector<Detection> detections =
      postprocessor.decode({output}, transform, cv::Size(640, 640));
  ASSERT_EQ(detections.size(), 1U);
  EXPECT_EQ(detections.front().class_id, 0);
  EXPECT_NEAR(detections.front().confidence, 0.60F, 1e-6F);
  EXPECT_EQ(detections.front().box, cv::Rect(80, 90, 40, 60));
}

TEST(YoloPostprocessor, RejectsMismatchedOutputClassCount) {
  PostprocessorConfig config = baseConfig();
  YoloPostprocessor postprocessor(config);
  const int dimensions[3] = {1, 8, 5};
  const cv::Mat output(3, dimensions, CV_32F, cv::Scalar::all(0));
  LetterboxTransform transform;
  transform.scale = 1.0;
  EXPECT_THROW(postprocessor.decode({output}, transform, cv::Size(640, 640)),
               std::runtime_error);
}

} // namespace
} // namespace sar_yolo_detector

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
