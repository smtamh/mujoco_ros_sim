/*  Copyright © 2018, Roboti LLC

    This file is licensed under the MuJoCo Resource License (the "License").
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.roboti.us/resourcelicense.txt
*/

#include "mjros.h"
#include "mujoco_rgbd_camera.hpp"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <mujoco_ros_msgs/ImageRequest.h>
#include <mujoco_ros_msgs/ImgReqAction.h>
#include "actionlib/server/simple_action_server.h"
#include "actionlib/server/action_server.h"

// MuJoCo basic data structures
mjModel* model_ = NULL;
mjData* data_ = NULL;

mjvCamera camera;
mjvScene scene;
mjvOption option;

// image_transport::Publisher camera_image_pub;
// image_transport::Publisher depth_image_pub;
image_transport::Publisher camL_pub;
image_transport::Publisher camR_pub;

// cv::Mat pub_img;
// cv::Mat depth_img;

ros::Publisher color_cloud_pub;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::ImagePtr img_msg;
sensor_msgs::ImagePtr depth_msg;
sensor_msgs::PointCloud2 color_cloud_msg;
bool img_updated = false;

ros::Publisher obj_pose_pub;
geometry_msgs::Pose obj_pose_msg_;
int body_id;
int body_id1;

int geomIndex;

// drop file callback
void drop(GLFWwindow *window, int count, const char **paths)
{
    // make sure list is non-empty
    if (count > 0)
    {
        mju_strncpy(filename, paths[0], 1000);
        settings.loadrequest = 1;
        ROS_INFO("DROP REQUEST");
    }
}

// load mjb or xml model
void loadmodel(void)
{
    // clear request
    settings.loadrequest = 0;

    // make sure filename is not empty
    if (!filename[0])
        return;

    // load and compile
    char error[500] = "";
    mjModel *mnew = 0;
    if (strlen(filename) > 4 && !strcmp(filename + strlen(filename) - 4, ".mjb"))
    {
        mnew = mj_loadModel(filename, NULL);
        if (!mnew)
            strcpy(error, "could not load binary model");
    }
    else
    {
        mnew = mj_loadXML(filename, NULL, error, 500);
    }
    if (!mnew)
    {
        printf("%s\n", error);
        return;
    }

    // compiler warning: print and pause
    if (error[0])
    {
        // mj_forward() below will print the warning message
        printf("Model compiled, but simulation warning (paused):\n  %s\n\n",
               error);
        settings.run = 0;
    }

    // delete old model, assign new
    mj_deleteData(d);
    mj_deleteModel(m);
    m = mnew;
    d = mj_makeData(m);

    int i = settings.key;
    d->time = m->key_time[i];
    mju_copy(d->qpos, m->key_qpos + i * m->nq, m->nq);
    mju_copy(d->qvel, m->key_qvel + i * m->nv, m->nv);
    mju_copy(d->act, m->key_act + i * m->na, m->na);

    if (m->actuator_biastype[0])
    {
        mju_copy(d->ctrl, m->key_qpos + 7 + i * m->nq, m->nu);
    }

    mj_forward(m, d);

    ros_sim_started = true;
    ctrl_command = mj_stackAlloc(d, (int)m->nu);
    ctrl_command2 = mj_stackAlloc(d, (int)(m->nbody * 6));

    // re-create scene and context
    mjv_makeScene(m, &scn, maxgeom);
    mjr_makeContext(m, &con, 50 * (settings.font + 1));

    // clear perturbation state
    pert.active = 0;
    pert.select = 0;
    pert.skinselect = -1;

    // align and scale view, update scene
    alignscale();
    mjv_updateScene(m, d, &vopt, &pert, &cam, mjCAT_ALL, &scn);

    // set window title to model name
    if (window && m->names)
    {
        char title[200] = "Simulate : ";
        strcat(title, m->names);
        strcat(title, ros::this_node::getNamespace().c_str());
        glfwSetWindowTitle(window, title);
    }

    // rebuild UI sections
    makesections();

    // full ui update
    uiModify(window, &ui0, &uistate, &con);
    uiModify(window, &ui1, &uistate, &con);

    updatesettings();
    mujoco_ros_connector_init();
    std::cout << " MODEL LOADED " << std::endl;
}

// render and publish image from specified camera
void renderAndPublish(
    const std::string& cam_name,
    mjModel* model,
    mjData* data,
    mjvCamera& rgbd_camera,
    mjvOption& sensor_option,
    mjvScene& sensor_scene,
    mjrContext& sensor_context,
    mjrRect& viewport,
    RGBD_mujoco& mj_RGBD,
    image_transport::Publisher& cam_pub)
{
    rgbd_camera.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, cam_name.c_str());
    mj_RGBD.set_camera_intrinsics(model, rgbd_camera, viewport);
    mjv_updateScene(model, data, &sensor_option, NULL, &rgbd_camera, mjCAT_ALL, &sensor_scene);
    mjr_render(viewport, &sensor_scene, &sensor_context);
    mj_RGBD.get_RGBD_buffer(model, viewport, &sensor_context);

    cv::Mat pub_img = mj_RGBD.get_color_image();
    cv::Mat resized;
    cv::resize(pub_img, resized, cv::Size(640, 480));

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", resized).toImageMsg();
    msg->header.stamp = ros::Time::now();
    msg->header.frame_id = cam_name;

    cam_pub.publish(msg);
}

// void RGBD_sensor(mjModel* model, mjData* data, string* camera_name, string* pub_topic_name, string* sub_topic_name)
void RGBD_sensor(mjModel* model, mjData* data, const std::string& cam_name)
{
  glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
  GLFWwindow* window = glfwCreateWindow(640, 480, cam_name.c_str(), NULL, NULL);

  // glfwSetWindowAttrib(window, GLFW_RESIZABLE, GLFW_FALSE);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // setup camera
  mjvCamera rgbd_camera;
  rgbd_camera.type = mjCAMERA_FIXED;
//   rgbd_camera.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "camera");

//   std::cout << "debugging111" << std::endl;
  
  mjvOption sensor_option;
  mjvPerturb sensor_perturb;
  mjvScene sensor_scene;
  mjrContext sensor_context;

  mjv_defaultOption(&sensor_option);
  mjv_defaultScene(&sensor_scene);
  mjr_defaultContext(&sensor_context);

  // create scene and context
  mjv_makeScene(model, &sensor_scene, 1000);
  mjr_makeContext(model, &sensor_context, mjFONTSCALE_150);

  RGBD_mujoco mj_RGBD;

  while (!glfwWindowShouldClose(window))
  {
    ros::Time init_ = ros::Time::now();
    // get framebuffer viewport
    mjrRect viewport = {0,0,0,0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
    
    renderAndPublish(cam_name.c_str(), model, data, rgbd_camera, sensor_option, sensor_scene, sensor_context, viewport, mj_RGBD, camL_pub);

    // // for 1 camera
    // mtx.lock();

    // pub_img = mj_RGBD.get_color_image();
    // depth_img = mj_RGBD.get_depth_image();
    // ros::Time img_capture_time = ros::Time::now();
    // if(pub_img.empty())
    // {
    //     ROS_ERROR("Could not read the image.");
    //     // return -1;
    // }
    // else
    // {
    //     ////CAMERA img publish
    //     // cv::Mat resized_img;
    //     // cv::Size desired_size(480, 640); // 원하는 크기 지정하세요 FOV는 .xml에서..
    //     // cv::resize(pub_img, resized_img, desired_size);
    //     // img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", resized_img).toImageMsg();
    //     img_msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", pub_img).toImageMsg();
    //     depth_msg = cv_bridge::CvImage(std_msgs::Header(), "32FC1", depth_img).toImageMsg();
    //     img_msg->header.stamp = img_capture_time;
    //     depth_msg->header.stamp = img_capture_time;
    //     camera_image_pub.publish(img_msg);
    //     depth_image_pub.publish(depth_msg);
    //     img_updated = true;
    // }

    ////OBJ POSE
    body_id = -1;
    body_id = mj_name2id(model, mjOBJ_BODY, "obj");


    if (body_id >= 0)
    {
        geomIndex = model->body_geomadr[body_id];

        obj_pose_msg_.position.x = data->geom_xpos[3*geomIndex];
        obj_pose_msg_.position.y = data->geom_xpos[3*geomIndex + 1];
        obj_pose_msg_.position.z = data->geom_xpos[3*geomIndex + 2];

        double rot_vec[9];  // column vectors
        for(int i = 0; i < 9; i++){
            rot_vec[i] = data->geom_xmat[9*geomIndex + i];
        }
        Eigen::Map<Eigen::Matrix3d> rotationMatrix(rot_vec);
        Eigen::Quaterniond quaternion(rotationMatrix.transpose());  // eigen uses row vectors, so transpose
        obj_pose_msg_.orientation.x = quaternion.coeffs()[0];
        obj_pose_msg_.orientation.y = quaternion.coeffs()[1];
        obj_pose_msg_.orientation.z = quaternion.coeffs()[2];
        obj_pose_msg_.orientation.w = quaternion.coeffs()[3];
    }
    else
    {
        ROS_WARN("NO OBJ POS");
    }
    obj_pose_pub.publish(obj_pose_msg_);

    mtx.unlock();

    // mtx.lock();
    // *color_cloud = mj_RGBD.generate_color_pointcloud();
    // pcl::toROSMsg(*color_cloud, color_cloud_msg);
    // color_cloud_msg.header.frame_id = "Camera";
    // color_cloud_msg.header.stamp = img_capture_time;
    // color_cloud_pub.publish(color_cloud_msg);
    // mtx.unlock();

    // Swap OpenGL buffers
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();

    int elapsed_time_nano = int((ros::Time::now() - init_).toSec() * 1e9);
    int dt = int(1e9/30);
    std::this_thread::sleep_for(std::chrono::nanoseconds(dt - elapsed_time_nano));

    // Do not forget to release buffer to avoid memory leak
    mj_RGBD.release_buffer();
  }

  mjv_freeScene(&sensor_scene);
  mjr_freeContext(&sensor_context);
}

class ImageRequestAction {
    protected:
        ros::NodeHandle nh_;

        actionlib::SimpleActionServer<mujoco_ros_msgs::ImgReqAction> as_;

        std::string action_name_;

        mujoco_ros_msgs::ImgReqResult result_;
        mujoco_ros_msgs::ImgReqFeedback feedback_;
    public:
        bool isSuccess = false;
        bool action_called = false;

        mujoco_ros_msgs::ImgReqGoal goal_;

        ImageRequestAction(std::string name) :
            as_(nh_, name, boost::bind(&ImageRequestAction::executeCB, this, _1), false), action_name_(name)
        {
            as_.start();
        }

        ~ImageRequestAction(){
        }

        void executeCB(const mujoco_ros_msgs::ImgReqGoalConstPtr &goal){
            ROS_INFO("action called");
            ros::Rate r(1);
            mujoco_ros_msgs::ImgReqFeedback feedback;
            mujoco_ros_msgs::ImgReqResult result;

            if (goal->request==true){
                img_updated = false;
            } 
            
            while(!isSuccess){
                // Check that preempt has not been requested by the client
                // if(!ros::ok()) ROS_INFO("ROS:: NOT OK");
                if (as_.isPreemptRequested() || !ros::ok()){
                    ROS_INFO("NONONONONO");
                    as_.setPreempted();
                    isSuccess = false;
                    break;
                }

                if(img_updated!=true){
                    feedback.isDone = false;
                }
                else{
                    feedback.isDone = true;
                    isSuccess = true;
                    img_updated = false;
                }
                feedback.isDone = false;
                as_.publishFeedback(feedback);
                r.sleep();
            }

            if(isSuccess){
                ROS_INFO("Before Succeeded");
                result.image = *img_msg;
                result.dimage = *depth_msg;
                ROS_INFO("Succeeded");
                as_.setSucceeded(result);
            }
        }
};

// run event loop
int main(int argc, char **argv)
{
    // :: ROS CUSTUM :: initialize ros
    ros::init(argc, argv, "mujoco_ros");
    ros::NodeHandle nh("~");
    std::string key_file;
    nh.param<std::string>("license", key_file, "mjkey.txt");

    nh.param("use_shm", use_shm, false);
    sim_command_sub = nh.subscribe<std_msgs::String>("/mujoco_ros_interface/sim_command_con2sim", 100, sim_command_callback);
    sim_command_pub = nh.advertise<std_msgs::String>("/mujoco_ros_interface/sim_command_sim2con", 1);

    image_transport::ImageTransport it(nh);
    camL_pub = it.advertise("/mujoco_ros_interface/cam_L/image", 1);
    camR_pub = it.advertise("/mujoco_ros_interface/cam_R/image", 1);
    // camera_image_pub = it.advertise("/mujoco_ros_interface/camera/image", 1);
    // depth_image_pub = it.advertise("/mujoco_ros_interface/camera/depth", 1);
    // color_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/mujoco_ros_interface/camera/color_cloud", 1);

    std::string actionServerName = "/imageRequestAction";
    ImageRequestAction action(actionServerName);

    obj_pose_pub = nh.advertise<geometry_msgs::Pose>("/obj_pose", 1);
    new_obj_pose_sub = nh.subscribe<geometry_msgs::Pose>("/new_obj_pose", 1, NewObjPoseCallback);

    if (!use_shm)
    {
        nh.param("pub_mode", pub_total_mode, false);
        std::cout<<"Name Space: " << ros::this_node::getNamespace() << std::endl;

        //register publisher & subscriber
        char prefix[200] = "/mujoco_ros_interface";
        char joint_set_name[200];
        char sim_status_name[200];
        char joint_state_name[200];
        char sim_time_name[200];
        char sensor_state_name[200];

        strcpy(joint_set_name, prefix);
        strcpy(sim_status_name, prefix);
        strcpy(joint_state_name, prefix);
        strcpy(sim_time_name, prefix);
        strcpy(sensor_state_name, prefix);
        if (ros::this_node::getNamespace() != "/")
        {
            strcat(joint_set_name, ros::this_node::getNamespace().c_str());
            strcat(sim_status_name, ros::this_node::getNamespace().c_str());
            strcat(joint_state_name, ros::this_node::getNamespace().c_str());
            strcat(sim_time_name, ros::this_node::getNamespace().c_str());
            strcat(sensor_state_name, ros::this_node::getNamespace().c_str());
        }
        strcat(joint_set_name, "/joint_set");
        strcat(sim_status_name, "/sim_status");
        strcat(joint_state_name, "/joint_states");
        strcat(sim_time_name, "/sim_time");
        strcat(sensor_state_name, "/sensor_states");

        joint_set = nh.subscribe<mujoco_ros_msgs::JointSet>(joint_set_name, 1, jointset_callback, ros::TransportHints().tcpNoDelay(true));

        if (pub_total_mode)
        {
            sim_status_pub = nh.advertise<mujoco_ros_msgs::SimStatus>(sim_status_name, 1);
        }
        else
        {
            joint_state_pub = nh.advertise<sensor_msgs::JointState>(joint_state_name, 1);
            sim_time_pub = nh.advertise<std_msgs::Float32>(sim_time_name, 1);
            sensor_state_pub = nh.advertise<mujoco_ros_msgs::SensorState>(sensor_state_name, 1);
        }
    }
    else
    {
#ifdef COMPILE_SHAREDMEMORY
        init_shm(shm_msg_key, shm_msg_id, &mj_shm_);
#endif
    }

    //ROS_INFO("ROS initialize complete");
    sim_time_ros = ros::Duration(0);
    sim_time_run = ros::Time::now();
    sim_time_now_ros = ros::Duration(0);

    // initialize everything
    init();

    std::string model_file;
    // request loadmodel if file given (otherwise drag-and-drop)
    if (nh.getParam("model_file", model_file))
    {
        mju_strncpy(filename, model_file.c_str(), 1000);
        settings.loadrequest = 2;
        ROS_INFO("model is at %s", model_file.c_str());
    }

    // start simulation thread
    ROS_INFO("debugging--Hi");
    // // start simulation thread
    std::thread simthread(simulate);
    std::thread visual_thread1;
    std::thread visual_thread2;

    // event loop
    while ((!glfwWindowShouldClose(window) && !settings.exitrequest) && ros::ok())
    {
        // start exclusive access (block simulation thread)
        mtx.lock();
        // load model (not on first pass, to show "loading" label)
        if (settings.loadrequest == 1)
        {
            ROS_INFO("Load Request");
            loadmodel();
            visual_thread1 = std::thread(RGBD_sensor, m, d, "cam_L");
            visual_thread2 = std::thread(RGBD_sensor, m, d, "cam_R");
        }
        else if (settings.loadrequest > 1)
            settings.loadrequest = 1;

        // handle events (calls all callbacks)
        glfwPollEvents();

        // prepare to render
        prepare();

        // ros events
        rosPollEvents();

        // end exclusive access (allow simulation thread to run)
        mtx.unlock();

        // render while simulation is running
        render(window);
    }

    // stop simulation thread
    settings.exitrequest = 1;
    simthread.join();
    visual_thread1.join();
    visual_thread2.join();

    // delete everything we allocated
    uiClearCallback(window);
    mj_deleteData(d);
    mj_deleteModel(m);
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // deactive MuJoCo
    // mj_deactivate();

    std_msgs::String pmsg;
    pmsg.data = std::string("terminate");
    sim_command_pub.publish(pmsg);

#ifdef COMPILE_SHAREDMEMORY
        deleteSharedMemory(shm_msg_id, mj_shm_);
#endif
// terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
    glfwTerminate();
#endif

    return 0;
}
