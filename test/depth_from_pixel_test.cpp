#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>


using namespace std;
using namespace cv;

Mat image;
Mat imgHSV;
Mat OutputImage;

int iLowH = 0;
int iHighH = 12;
int iLowS = 60;
int iHighS = 255;
int iLowV = 0;
int iHighV = 245;

int acc = 1;
int rows = 10;
int para1 = 5;
int para2 = 10;
int minRad = 5;
int maxRad = 70;

float x;
float y;

int width = 640;
int height = 480;

int input;

static void HSVthreshold(int, int, int, int, int, int, void*)
{
    inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), OutputImage);
}

static void Circle_detector(int, int, int, int, int, void*)
{
    vector<Vec3f> circles;
    HoughCircles(OutputImage, circles, HOUGH_GRADIENT, 1,
                 OutputImage.rows / rows,      //change to detect circles that are closer to eachother
                 para1, para2, minRad, maxRad);        //chang last to parameters to detect larger or smaller circles

    for (size_t i = 0; i < circles.size(); i++)
    {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        // circle center
        circle(imgHSV, center, 1, Scalar(0, 255, 0), 3, LINE_AA);
        // circle outline
        int radius = c[2];
        circle(imgHSV, center, radius, Scalar(255, 0, 0), 3, LINE_AA);

//        cout << "The center of the detection is located at pixel: " << Point(c[0], c[1]) << endl;

        x = c[0];
        y = c[1];
    }
}


int main()
{
    // Contructing piplines and other stuff to receive data from the realsense camera.

    //Contruct a pipeline which abstracts the device
    rs2::pipeline pipe;     //for color

    //Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;

    //Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);

    //Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile selection = pipe.start(cfg);    //for color

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;

    while (waitKey(1) < 0)
//    while (true)
    {
        frames = pipe.wait_for_frames();
        auto depth_frame = frames.get_depth_frame();
        auto color_frame = frames.get_color_frame();
        // Make sure that both depth and  color are present for calculations
        if (!depth_frame || !color_frame)
            continue;

        //Get color each frame
        //rs2::frame color_frame = frames.get_color_frame();

        // Creating OpenCV Matrix from a color image
        Mat color(Size(width, height), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);

        // cheking if an image was read
        if (color.empty())
        {
            cerr << "image was not generated !" << endl;
            return 1;
        }

        namedWindow("Display Image", WINDOW_AUTOSIZE);
        imshow("Display Image", color);

        //convert RGB to HSV
        cvtColor(color, imgHSV, COLOR_BGR2HSV);

        //showHSV image
        imshow("image", imgHSV);

        //Create windows
        namedWindow("image", WINDOW_AUTOSIZE); //window for original image
        namedWindow("Output", WINDOW_AUTOSIZE); //window for output mask

        //aplying color filter to HSV image
        HSVthreshold(iLowH, iHighH, iLowS, iHighS, iLowV, iHighV, 0);

        //Optional filter --> does not work properly at the moment needs more setup <--

        //morphological opening (remove small objects from the foreground)
        erode(OutputImage, OutputImage, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
        dilate(OutputImage, OutputImage, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));

        //morphological closing (fill small holes in the foreground)
        dilate(OutputImage, OutputImage, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));
        erode(OutputImage, OutputImage, getStructuringElement(MORPH_ELLIPSE, Size(2, 2)));

        imshow("Output", OutputImage);

        Circle_detector(rows, para1, para2, minRad, maxRad, 0);
        imshow("image", imgHSV);

        auto depth_profile = depth_frame.get_profile().as<rs2::video_stream_profile>();
        auto color_profile = color_frame.get_profile().as<rs2::video_stream_profile>();

        auto depth_intrin = depth_profile.get_intrinsics();
        auto color_intrin = color_profile.get_intrinsics();
        auto depth2color_extrin = depth_profile.get_extrinsics_to(color_profile);
        auto color2depth_extrin = color_profile.get_extrinsics_to(depth_profile);

        float rgb_src_pixel[2] = { x,y }; // The RGB coordinate for the center of the marble
        float dpt_tgt_pixel[2] = { 0 }; // The depth pixel that has the best match for that RGB coordinate

        auto sensor = selection.get_device().first<rs2::depth_sensor>();
        auto scale = sensor.get_depth_scale();

        // Search along a projected beam from 0.1m to 10 meter. This can be optimized to the concrete scenario, e.g. if you know that the data is bounded within [min..max] range
        rs2_project_color_pixel_to_depth_pixel(dpt_tgt_pixel, reinterpret_cast<const uint16_t*>(depth_frame.get_data()), scale, 0.1f, 2,
                                               &depth_intrin, &color_intrin,
                                               &color2depth_extrin, &depth2color_extrin, rgb_src_pixel);


        // Verify that the depth correspondence is valid, i.e within the frame boundaries
        std::cout<< "Width: "<<depth_frame.get_width()<<" Height: "<<depth_frame.get_height()<<std::endl;
        std::cout<< "dpt_tgt_pixel[0]: "<<dpt_tgt_pixel[0]<<" dpt_tgt_pixel[1]: "<<dpt_tgt_pixel[1]<<std::endl;
        if ((dpt_tgt_pixel[0] <= depth_frame.get_width()) && (dpt_tgt_pixel[1] <= depth_frame.get_height()))
        {
            auto distance = depth_frame.get_distance(dpt_tgt_pixel[0], dpt_tgt_pixel[1]);
            // Get the depth value for the pixel found

            cout << "The distance to the object is: " << distance << endl;
        }

        waitKey();
    }
    return 0;
}