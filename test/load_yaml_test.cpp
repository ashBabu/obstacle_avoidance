#include <iostream>
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

TEST(TESTSuite, yamlTest)
{
    YAML::Node config = YAML::LoadFile("/home/ash/Ash/C++/test_ash/test/realsense_opencv/torchscript/config.yaml");
    std::string imagePath = config["image_params"]["IMG_PATH"].as<std::string>();
    std::string modelPath = config["model_params"]["MODEL_PATH"].as<std::string>();
    std::string labelPath = config["label_params"]["LABEL_PATH"].as<std::string>();

    int kIMAGE_SIZE = config["image_params"]["kIMAGE_SIZE"].as<int>();
    int kCHANNELS = config["image_params"]["kCHANNELS"].as<int>();
    int kTOP_K = config["image_params"]["kTOP_K"].as<int>();
    std::cout<<"imagePath: "<<imagePath<<std::endl;
    std::cout<<"kTOP_K: "<<kTOP_K<<std::endl;

    EXPECT_EQ(kTOP_K, 3);
    EXPECT_EQ(kCHANNELS, 3);
    EXPECT_EQ(kIMAGE_SIZE, 224);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}