#include "main.h"
#include "Cameras.h"
#include "globals.h"
#include "graphics_helpers.h"

using namespace std;
using namespace cv;
using mjpgs = nadjieb::MJPEGStreamer;

int disk_min_hue;
int disk_max_hue;
int disk_min_sat;
int disk_max_sat;
int disk_min_val;
int disk_max_val;

bool found_camera = true;

int main()
{
    /**********************************************************************************************
     * Network Tables Setup *
     ************************/

    // Create networktables instan8ce and a table for vision
    nt::NetworkTableInstance nt_inst = nt::NetworkTableInstance::GetDefault();

    // Setup networktable client
    nt_inst.StartClient4("raspi vision client");
    nt_inst.SetServerTeam(2584);
    nt_inst.StartDSClient();
    nt_inst.SetServer("host", NT_DEFAULT_PORT4);

    // Create tables
    shared_ptr<nt::NetworkTable> visionTbl = nt_inst.GetTable("vision");

    // Make a sanity check topic and an entry to publish/read from it; set initial
    // value
    nt::IntegerTopic sanitycheck = visionTbl->GetIntegerTopic("sanitycheck");
    nt::IntegerEntry sanitycheckEntry = sanitycheck.GetEntry(0, {.periodic = 0.01});
    sanitycheckEntry.Set(1);

    // Connected topic
    nt::BooleanTopic connectedTopic = visionTbl->GetBooleanTopic("connected");
    nt::BooleanSubscriber connectedSub = connectedTopic.Subscribe(false);

    // Info from Roborio
    nt::BooleanTopic in_match_Topic = visionTbl->GetBooleanTopic("inMatch");
    nt::BooleanSubscriber in_match_Sub = in_match_Topic.Subscribe({false});


    // Vision topics
    nt::DoubleArrayTopic disk_pos_Topic = visionTbl->GetDoubleArrayTopic("diskPos");
    nt::DoubleArrayPublisher disk_pos_Pub = disk_pos_Topic.Publish();

    /**********************************************************************************************
     * Camera & Stream Setup *
     *************************/

    // depthCamera *depth;
    // try
    //{
    depthCamera depth(DEPTH_RED, 640, 480, 60);
    // depth = &depth_obj;
    //}
    /*
    catch (const rs2::error &err)
    {
        if (!in_match_Sub.Get())
        {
            return (7);
        }
        cout << "RS2 ERROR CAUGHT: " << err.what() << endl;
        found_camera = false;
    }
    */

    // Position of Camera on Robot
    depth.setPosOffset(1 / INCH, 5 / INCH, 23.5 / INCH, 0, -0.349);
    // Red camera settings
    depth.setCamParams(599, 600, 334, 236);

    // Start steam on port 5802
    vector<int> params = {IMWRITE_JPEG_QUALITY, 80};
    mjpgs streamer;
    streamer.start(5802);

    /**********************************************************************************************
    * GET FILTER PARAMETERS FROM FILE *
    *************************/
    vector<int> diskParams;

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

            /**********************************************************************************************
         * THE LOOP *
         ************/
        int counter = 2;
        double diskNum = 0;

        while (true)
        {
            chrono::time_point loop_start = chrono::steady_clock::now();

            if (!connectedSub.Get())
            {
                printf("Reconnecting...\n");
                nt_inst.StartClient4("raspi vision client");
                nt_inst.SetServerTeam(2584);
                nt_inst.StartDSClient();
                nt_inst.SetServer("host", NT_DEFAULT_PORT4);
            }

            // Make sure networktables is working
            sanitycheckEntry.Set(counter);
            counter++;

            depth.getFrame();
            chrono::time_point frameTime = chrono::steady_clock::now();
            if (found_camera)
            {
                    // Print & send disk info
                    pair<double, double> diskPos = depth.findDisks();
                    cout << "Disk X: " << diskPos.first << endl;
                    cout << "Disk Y: " << diskPos.second << endl;
                    double micros = time_since(frameTime);
                    vector<double> diskVector = {diskPos.first, diskPos.second, micros, diskNum};
                    disk_pos_Pub.Set(diskVector);
            }

            nt_inst.Flush();

            // Graphics stuff
            // drawMargins(depth.colorFrame);

            // streamer
            vector<uchar> buf_bgr;
            imencode(".jpg", depth.colorFrame, buf_bgr, params);
            streamer.publish("/colorFrame", string(buf_bgr.begin(), buf_bgr.end()));

            // Make sure each loop takes at least 10 ms (for streamer library)
            auto loop_time = chrono::duration_cast<chrono::milliseconds>(
                chrono::steady_clock::now() - loop_start);
            if (loop_time.count() < 10)
                this_thread::sleep_for(chrono::milliseconds(10 - loop_time.count()));
	}
        streamer.stop();

        return 0;
}

 // Return time elapsed since passed time_point in microseconds
double time_since(std::chrono::time_point<std::chrono::steady_clock> start)
{
     chrono::time_point end = chrono::steady_clock::now();
     auto duration = chrono::duration_cast<chrono::microseconds>(end - start);
     return (double)duration.count();
}
