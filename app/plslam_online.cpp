/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#ifdef HAS_MRPT
#include <slamScene.h>
#endif

#include <stereoFrame.h>
#include <stereoFrameHandler.h>
#include <boost/filesystem.hpp>

#include <mapFeatures.h>
#include <mapHandler.h>

#include <dataset.h>
#include <timer.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>

using namespace StVO;
using namespace PLSLAM;

void showHelp();
bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file);

cv::Mat matFromImage(const sensor_msgs::CompressedImageConstPtr& source);

// rostopic订阅 回调函数
void leftImCallback(const sensor_msgs::ImageConstPtr& msg);
void rightImCallback(const sensor_msgs::ImageConstPtr& msg);
void leftCompressedImCallback(const sensor_msgs::CompressedImageConstPtr& msg);


cv::Mat leftIm(cvSize(1280, 720), CV_8UC3);
cv::Mat rightIm(cvSize(1280, 720), CV_8UC3);
cv::Mat leftCompressedIm(cvSize(1280, 720), CV_8UC3);
//cv::Mat leftDepth16uc1(cvSize(1280, 720), CV_16UC1);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "plslam_online");

	ros::NodeHandle nh;

	//ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
	ros::Subscriber sub_leftIm = nh.subscribe("/zed/left/image_rect_color", 1000, leftImCallback);
	ros::Subscriber sub_rightIm = nh.subscribe("/zed/right/image_rect_color", 1000, rightImCallback);
	ros::Subscriber sub_leftCompressedIm = nh.subscribe("zed/right/image_raw_color/compressed", 1000, leftCompressedImCallback);

    // read inputs
	string config_file, camera_params, scene_config;
	ros::param::get("config_file", config_file);
	ros::param::get("camera_params", camera_params);
	ros::param::get("scene_config", scene_config);
	cout << config_file << endl;
	cout << camera_params << endl;
	cout << scene_config << endl;

	ros::spin();


    if (!config_file.empty()) SlamConfig::loadFromFile(config_file);

    if (SlamConfig::hasPoints() &&
            (!boost::filesystem::exists(SlamConfig::dbowVocP()) || !boost::filesystem::is_regular_file(SlamConfig::dbowVocP()))) {
        cout << "Invalid vocabulary for points" << endl;
        return -1;
    }
    if (SlamConfig::hasLines() &&
            (!boost::filesystem::exists(SlamConfig::dbowVocL()) || !boost::filesystem::is_regular_file(SlamConfig::dbowVocL()))) {
        cout << "Invalid vocabulary for lines" << endl;
        return -1;
    }

    cout << endl << "Initializing PL-SLAM...." << flush;

    PinholeStereoCamera*  cam_pin = new PinholeStereoCamera(camera_params);
    // Dataset dataset(dataset_dir, *cam_pin, frame_offset, frame_number, frame_step);

    // create scene
    slamScene scene(scene_config);
    Matrix4d Tcw, Tfw = Matrix4d::Identity();
    Tcw = Matrix4d::Identity();
    scene.setStereoCalibration( cam_pin->getK(), cam_pin->getB() );
    scene.initializeScene(Tfw);

    // create PLSLAM object
    PLSLAM::MapHandler* map = new PLSLAM::MapHandler(cam_pin);

    cout << " ... done. " << endl;

    Timer timer;

    // initialize and run PL-StVO
    int frame_counter = 0;
    StereoFrameHandler* StVO = new StereoFrameHandler(cam_pin);
    Mat& img_l = leftIm;
    Mat& img_r = rightIm;
    // while (dataset.nextFrame(img_l, img_r))
    while (1)
    {
        if( frame_counter == 0 ) // initialize
        {
            StVO->initialize(img_l,img_r,0);
            PLSLAM::KeyFrame* kf = new PLSLAM::KeyFrame( StVO->prev_frame, 0 );
            map->initialize( kf );
            // update scene
            scene.initViewports( img_l.cols, img_r.rows );
            scene.setImage(StVO->prev_frame->plotStereoFrame());
            scene.updateSceneSafe( map );
        }
        else // run
        {
            // PL-StVO
            timer.start();
            StVO->insertStereoPair( img_l, img_r, frame_counter );
            StVO->optimizePose();
            double t1 = timer.stop(); //ms
            cout << "------------------------------------------   Frame #" << frame_counter
                 << "   ----------------------------------------" << endl;
            cout << endl << "VO Runtime: " << t1 << endl;

            // check if a new keyframe is needed
            if( StVO->needNewKF() )
            {
                cout <<         "#KeyFrame:     " << map->max_kf_idx + 1;
                cout << endl << "#Points:       " << map->map_points.size();
                cout << endl << "#Segments:     " << map->map_lines.size();
                cout << endl << endl;

                // grab StF and update KF in StVO (the StVO thread can continue after this point)
                PLSLAM::KeyFrame* curr_kf = new PLSLAM::KeyFrame( StVO->curr_frame );
                // update KF in StVO
                StVO->currFrameIsKF();
                map->addKeyFrame( curr_kf );
                // update scene
                scene.setImage(StVO->curr_frame->plotStereoFrame());
                scene.updateSceneSafe( map );
            }
            else
            {
                scene.setImage(StVO->curr_frame->plotStereoFrame());
                scene.setPose( StVO->curr_frame->DT );
                scene.updateScene();
            }

            // update StVO
            StVO->updateFrame();
        }

        frame_counter++;
    }

    // finish SLAM
    map->finishSLAM();
    scene.updateScene( map );

    // perform GBA
    cout << endl << "Performing Global Bundle Adjustment..." ;
    map->globalBundleAdjustment();
    cout << " ... done." << endl;
    scene.updateSceneGraphs( map );

    // wait until the scene is closed
    while( scene.isOpen() );

    return 0;
}

void showHelp() {
    cout << endl << "Usage: ./imgPLSLAM <dataset_name> [options]" << endl
         << "Options:" << endl
         << "\t-o Offset (number of frames to skip in the dataset directory" << endl
         << "\t-n Number of frames to process the sequence" << endl
         << "\t-s Parameter to skip s-1 frames (default 1)" << endl
         << "\t-c Config file" << endl
         << endl;
}

bool getInputArgs(int argc, char **argv, std::string &dataset_name, int &frame_offset, int &frame_number, int &frame_step, std::string &config_file) {

    if( argc < 2 || argc > 10 || (argc % 2) == 1 )
        return false;

    dataset_name = argv[1];
    int nargs = argc/2 - 1;
    for( int i = 0; i < nargs; i++ )
    {
        int j = 2*i + 2;
        if( string(argv[j]) == "-o" )
            frame_offset = stoi(argv[j+1]);
        else if( string(argv[j]) == "-n" )
            frame_number = stoi(argv[j+1]);
        else if( string(argv[j]) == "-s" )
            frame_step = stoi(argv[j+1]);
        else if (string(argv[j]) == "-c")
            config_file = string(argv[j+1]);
        else
            return false;
    }

    return true;
}

cv::Mat matFromImage(const sensor_msgs::CompressedImageConstPtr& source)
{
	cv::Mat jpegData(1,source->data.size(),CV_8UC1);
	jpegData.data 	= const_cast<uchar*>(&source->data[0]);
	cv::InputArray data(jpegData);
	cv::Mat bgrMat 	= cv::imdecode(data,cv::IMREAD_COLOR);
	return bgrMat;
}

void leftImCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	leftIm = cv_bridge::toCvCopy(msg, "8UC3")->image;
  	// ROS_INFO("leftIm");
  	cv::imwrite("/home/lab404/Documents/leftIm.jpg",leftIm);
  }
  catch (cv_bridge::Exception& e)
  {
  	ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
  }
}
void rightImCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
  	rightIm = cv_bridge::toCvCopy(msg, "8UC3")->image;
  	// ROS_INFO("rightIm");
  	cv::imwrite("/home/lab404/Documents/rightIm.jpg",rightIm);
  }
  catch (cv_bridge::Exception& e)
  {
  	ROS_ERROR("Could not convert from '%s' to '8UC3'.", msg->encoding.c_str());
  }
}

void leftCompressedImCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
  try
  {
  	leftCompressedIm = matFromImage(msg);
  	// ROS_INFO("leftCompressedImCallback");
  	cv::imwrite("/home/lab404/Documents/leftCompressedIm.jpg",leftCompressedIm);
  }
  catch (cv_bridge::Exception& e)
  {
  	ROS_ERROR("Could not convert from leftCompressedIm to cvMat.");
  }
}
