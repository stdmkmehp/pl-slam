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

#include "rosImg2cvMat.h"

using namespace StVO;
using namespace PLSLAM;

cv::Mat leftIm(cvSize(1280, 720), CV_8UC3);
cv::Mat rightIm(cvSize(1280, 720), CV_8UC3);

int main(int argc, char **argv)
{

	ros::init(argc, argv, "plslam_online");

	ros::NodeHandle nh;

	//ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);
	// ros::Subscriber sub_leftIm = nh.subscribe("/zed/left/image_rect_color", 1000, leftImCallback);
	// ros::Subscriber sub_rightIm = nh.subscribe("/zed/right/image_rect_color", 1000, rightImCallback);
	ros::Subscriber sub_leftCompressedIm = nh.subscribe("/zed/left/image_raw_color/compressed", 1000, leftCompressedImCallback);
	ros::Subscriber sub_rightCompressedIm = nh.subscribe("/zed/right/image_raw_color/compressed", 1000, rightCompressedImCallback);

    // read inputs
	string config_file, camera_params, scene_config;
	ros::param::get("config_file", config_file);
	ros::param::get("camera_params", camera_params);
	ros::param::get("scene_config", scene_config);
	cout << config_file << endl;
	cout << camera_params << endl;
	cout << scene_config << endl;

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
    cv::Mat& img_l = leftIm;
    cv::Mat& img_r = rightIm;
    // while (dataset.nextFrame(img_l, img_r))
	ros::Rate loop_rate(10);	// 10Hz
    while (ros::ok())
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

        ros::spinOnce();
        loop_rate.sleep();
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
