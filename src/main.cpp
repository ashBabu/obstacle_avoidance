#include <iostream>
#include <algorithm>
#include <cv-helpers.hpp>
#include <librealsense2/rsutil.h>


// comparison function object
bool compareContourAreas ( const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

void detectObstacles(float & distance, const cv::Mat & depth_map, cv::Mat & output_canvas)
{
    float depth_thresh = 1; // Threshold for SAFE distance
    cv::Mat mask, mean, stddev, mask2;

    // Mask to segment regions with depth less than safe distance
    cv::inRange(depth_map, 0.3, depth_thresh, mask);
    cv::imshow("mask",mask);

    double s = (cv::sum(mask)[0])/255.0;
    double img_area = double(mask.rows * mask.cols);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;

    // Check if a significantly large obstacle is present and filter out smaller noisy regions
    if (s > 0.01*img_area)
    {
        // finding conoturs in the generated mask
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // sorting contours from largest to smallest
        std::sort(contours.begin(), contours.end(), compareContourAreas);

        // extracting the largest contour
        std::vector<cv::Point> cnt = contours[0];

        // Check if detected contour is significantly large (to avoid multiple tiny regions)
        double cnt_area = fabs( cv::contourArea(cv::Mat(cnt)));

        if (cnt_area > 0.01*img_area)
        {
            cv::Rect box;

            // Finding the bounding rectangle for the largest contour
            box = cv::boundingRect(cnt);

            // finding average depth of region represented by the largest contour
            mask2 = mask*0;
            cv::drawContours(mask2, contours, 0, (255), -1);

            // Calculating the average depth of the object closer than the safe distance
            cv::meanStdDev(depth_map, mean, stddev, mask2);

            // Printing the warning text with object distance
            char text[10];
            std::sprintf(text, "%.2f cm",mean.at<double>(0,0));
            std::cout<< '\a'<< std::endl;
            cv::putText(output_canvas, "WARNING!", cv::Point2f(box.x + 5, box.y-40), 1, 2, cv::Scalar(0,0,255), 2, 2);
            cv::putText(output_canvas, "Object at", cv::Point2f(box.x + 5, box.y), 1, 2, cv::Scalar(0,0,255), 2, 2);
            cv::putText(output_canvas, text, cv::Point2f(box.x + 5, box.y+40), 1, 2, cv::Scalar(0,0,255), 2, 2);

        }
    }
    else
    {
        // Printing SAFE if no obstacle is closer than the safe distance
        cv::putText(output_canvas, "SAFE!", cv::Point2f(200,200),1,2,cv::Scalar(0,255,0),2,2);
    }

// Displaying the output of the obstacle avoidance system
    cv::imshow("output_canvas",output_canvas);
}

int main(int argc, char** argv)
{
    std::cout<<"Program Started: "<< '\a'<< std::endl;
        // Start streaming from Intel RealSense Camera
    rs2::pipeline pipe;
    rs2_stream r = RS2_STREAM_DEPTH;
//    rs2_stream r = RS2_STREAM_COLOR;
    rs2::align align_to(r);
    auto config = pipe.start();
    auto profile = config.get_stream(r).as<rs2::video_stream_profile>();

    const auto window_name = "Display Image";
    namedWindow(window_name, cv::WINDOW_AUTOSIZE);
    float distance;

    while (getWindowProperty(window_name, cv::WND_PROP_AUTOSIZE) >= 0)
    {
        // Wait for the next set of frames
        auto data = pipe.wait_for_frames();
        // Make sure the frames are spatially aligned
        data = align_to.process(data);

        auto depth_frame = data.get_depth_frame();
        auto color_frame = data.get_color_frame();

        static int last_frame_number = 0;
        if (depth_frame.get_frame_number() == last_frame_number)
            continue;
        last_frame_number = static_cast<int>(depth_frame.get_frame_number());

        auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
        auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

//        auto sensor = config.get_device().first<rs2::depth_sensor>();

        // Convert RealSense frame to OpenCV matrix:
        auto depth_mat = depth_frame_to_meters(depth_frame);
        auto color_mat = frame_to_mat(color_frame);

//        cv::imshow(window_name, color_mat);

        detectObstacles(distance, depth_mat, color_mat);
        if (cv::waitKey(1) >= 0)
            break;
    }

    cv::destroyWindow(window_name);
    return EXIT_SUCCESS;
}