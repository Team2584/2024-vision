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

    int disk_min_hue;
    int disk_max_hue;
    int disk_min_sat;
    int disk_max_sat;
    int disk_min_val;
    int disk_max_val;

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
        vector<int> diskParams;
        vector<int> cubeParams;

        string line;
        ifstream diskFile("/home/patriotrobotics/Documents/FRCCode/2024-vision/disk-params.txt");
        for (int i = 0; i < 6; i++)
        {
            if (diskFile)
                getline(diskFile, line);
            if (line != "")
                diskParams.push_back(stoi(line));
        }
        if (diskParams.size() > 0)
        {
            disk_min_hue = diskParams[0];
            cout << "disk_min_hue " << disk_min_hue << endl;
            disk_max_hue = diskParams[1];
            cout << "disk_max_hue " << disk_max_hue << endl;
            disk_min_sat = diskParams[2];
            cout << "disk_min_sat " << disk_min_sat << endl;
            disk_max_sat = diskParams[3];
            cout << "disk_max_sat " << disk_max_sat << endl;
            disk_min_val = diskParams[4];
            cout << "disk_min_val " << disk_min_val << endl;
            disk_max_val = diskParams[5];
            cout << "disk_max_val " << disk_max_val << endl << endl;
        }

        Mat hsvFrame;
        cvtColor(colorFrame, hsvFrame, COLOR_BGR2HSV);
        Mat channels[3];
        split(hsvFrame, channels);

        // Disk filtering
        Mat diskHueMask;
        inRange(channels[0], disk_min_hue, disk_max_hue, diskHueMask);
        Mat diskSatMask;
        inRange(channels[1], disk_min_sat, disk_max_sat, diskSatMask);
        Mat diskValMask;
        inRange(channels[2], disk_min_val, disk_max_val, diskValMask);

        Mat diskMask;
        bitwise_and(diskHueMask, diskSatMask, diskMask);
        bitwise_and(diskMask, diskValMask, diskMask);
        cvtColor(diskMask, diskMask, COLOR_GRAY2BGR);

        Mat justDisk(Size(WIDTH, HEIGHT), CV_8UC1);
        bitwise_and(colorFrame, diskMask, justDisk);

        // Stream all the images
        vector<uchar> buf_colorFrame;
        imencode(".jpg", colorFrame, buf_colorFrame, stream_params);
        streamer.publish("/colorFrame", string(buf_colorFrame.begin(), buf_colorFrame.end()));

        vector<uchar> buf_diskValMask;
        imencode(".jpg", diskValMask, buf_diskValMask, stream_params);
        streamer.publish("/diskValMask", string(buf_diskValMask.begin(), buf_diskValMask.end()));

        vector<uchar> buf_diskHueMask;
        imencode(".jpg", diskHueMask, buf_diskHueMask, stream_params);
        streamer.publish("/diskHueMask", string(buf_diskHueMask.begin(), buf_diskHueMask.end()));

        vector<uchar> buf_diskSatMask;
        imencode(".jpg", diskSatMask, buf_diskSatMask, stream_params);
        streamer.publish("/diskSatMask", string(buf_diskSatMask.begin(), buf_diskSatMask.end()));

        vector<uchar> buf_justDisk;
        imencode(".jpg", justDisk, buf_justDisk, stream_params);
        streamer.publish("/justDisk", string(buf_justDisk.begin(), buf_justDisk.end()));

        colorFrame.release();
        hsvFrame.release();
        diskHueMask.release();
        diskValMask.release();
        diskSatMask.release();
        justDisk.release();
    }
}
