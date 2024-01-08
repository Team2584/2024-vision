#include "nadjieb/mjpeg_streamer.hpp"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <librealsense2/h/rs_sensor.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>

#define WIDTH 640
#define HEIGHT 480
#define FPS 60

#define DEPTH_BLUE "017322071728"
#define DEPTH_RED "939622072805"

using namespace std;
using namespace cv;
using mjpgs = nadjieb::MJPEGStreamer;

int main(void)
{
    rs2::pipeline pipe;
    rs2::config cfg;

    // Select camera by serial number
    cfg.enable_device(DEPTH_RED);

    // Add desired streams to configuration
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, FPS);

    // Instruct pipeline to start streaming with the requested configuration
    rs2::pipeline_profile prof = pipe.start();//cfg);

    // Disgusting one-liner to disable laser
    prof.get_device().first<rs2::depth_sensor>().set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);

    vector<int> stream_params = {IMWRITE_JPEG_QUALITY, 90};
    mjpgs streamer;

    streamer.start(5802);

    int ring_min_hue;
    int ring_max_hue;
    int ring_min_sat;
    int ring_max_sat;
    int ring_min_val;
    int ring_max_val;

    while (true)
    {
        // Get frame
        rs2::frameset frames;
        frames = pipe.wait_for_frames();
        rs2::video_frame frame = frames.get_color_frame();
        Mat colorFrame(Size(WIDTH, HEIGHT), CV_8UC3, (void *)frame.get_data());
        cv::cvtColor(colorFrame, colorFrame, cv::COLOR_RGB2BGR);
        GaussianBlur(colorFrame, colorFrame, Size(17, 17), 1.2, 1.2, BORDER_DEFAULT);

        // Get parameters
        vector<int> ringParams;
        vector<int> cubeParams;

        string line;
        ifstream ringFile("/home/patriotrobotics/Documents/FRCCode/2024-vision/ring-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (ringFile)
                getline(ringFile, line);
            if (line != "")
                ringParams.push_back(stoi(line));
        }
        if (ringParams.size() > 0)
        {
            ring_min_hue = ringParams[0];
            cout << "ring_min_hue " << ring_min_hue << endl;
            ring_max_hue = ringParams[1];
            cout << "ring_max_hue " << ring_max_hue << endl;
            ring_min_sat = ringParams[2];
            cout << "ring_min_sat " << ring_min_sat << endl;
            ring_max_sat = ringParams[3];
            cout << "ring_max_sat " << ring_max_sat << endl;
            ring_min_val = ringParams[4];
            cout << "ring_min_val " << ring_min_val << endl;
            ring_max_val = ringParams[5];
            cout << "ring_max_val " << ring_max_val << endl << endl;
        }

        Mat hsvFrame;
        cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);
        Mat channels[3];
        split(hsvFrame, channels);

        // Ring filtering
        Mat ringHueMask;
        inRange(channels[0], ring_min_hue, ring_max_hue, ringHueMask);
        Mat ringSatMask;
        inRange(channels[1], ring_min_sat, ring_max_sat, ringSatMask);
        Mat ringValMask;
        inRange(channels[2], ring_min_val, ring_max_val, ringValMask);

        Mat ringMask;
        bitwise_and(ringHueMask, ringSatMask, ringMask);
        bitwise_and(ringMask, ringValMask, ringMask);
        cvtColor(ringMask, ringMask, COLOR_GRAY2BGR);

        Mat justRing(Size(WIDTH, HEIGHT), CV_8UC1);
        bitwise_and(colorFrame, ringMask, justRing);

        // Stream all the images
        vector<uchar> buf_colorFrame;
        imencode(".jpg", colorFrame, buf_colorFrame, stream_params);
        streamer.publish("/colorFrame", string(buf_colorFrame.begin(), buf_colorFrame.end()));

        vector<uchar> buf_ringValMask;
        imencode(".jpg", ringValMask, buf_ringValMask, stream_params);
        streamer.publish("/ringValMask", string(buf_ringValMask.begin(), buf_ringValMask.end()));

        vector<uchar> buf_ringHueMask;
        imencode(".jpg", ringHueMask, buf_ringHueMask, stream_params);
        streamer.publish("/ringHueMask", string(buf_ringHueMask.begin(), buf_ringHueMask.end()));

        vector<uchar> buf_ringSatMask;
        imencode(".jpg", ringSatMask, buf_ringSatMask, stream_params);
        streamer.publish("/ringSatMask", string(buf_ringSatMask.begin(), buf_ringSatMask.end()));

        vector<uchar> buf_justRing;
        imencode(".jpg", justRing, buf_justRing, stream_params);
        streamer.publish("/justRing", string(buf_justRing.begin(), buf_justRing.end()));

        colorFrame.release();
        hsvFrame.release();
        ringHueMask.release();
        ringValMask.release();
        ringSatMask.release();
        justRing.release();
    }
}
