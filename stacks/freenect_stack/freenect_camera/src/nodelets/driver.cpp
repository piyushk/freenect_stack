/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011 Willow Garage, Inc.
 *    Suat Gedikli <gedikli@willowgarage.com>
 *    Patrick Michelich <michelich@willowgarage.com>
 *    Radu Bogdan Rusu <rusu@willowgarage.com>
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "driver.h" /// @todo Get rid of this header entirely?
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <boost/algorithm/string/replace.hpp>

using namespace std;

DriverNodelet::~DriverNodelet () {
  /// @todo Test watchdog timer for race conditions. May need to use a separate callback queue
  /// controlled by the driver nodelet.
}

void DriverNodelet::onInit ()

  ros::NodeHandle& nh       = getNodeHandle();        // topics
  ros::NodeHandle& param_nh = getPrivateNodeHandle(); // parameters

  // Allow remapping namespaces rgb, ir, depth, depth_registered
  image_transport::ImageTransport it(nh);
  ros::NodeHandle rgb_nh(nh, "rgb");
  image_transport::ImageTransport rgb_it(rgb_nh);
  ros::NodeHandle depth_nh(nh, "depth");
  image_transport::ImageTransport depth_it(depth_nh);
  ros::NodeHandle depth_registered_nh(nh, "depth_registered");
  image_transport::ImageTransport depth_registered_it(depth_registered_nh);

  rgb_frame_counter_ = depth_frame_counter_ = 0;
  publish_rgb_ = = publish_depth_ = true;

  // Initialize the sensor, but don't start any streams yet. That happens in the connection callbacks.
  setupDevice();

  // Initialize dynamic reconfigure
  reconfigure_server_.reset( new ReconfigureServer(param_nh) );
  reconfigure_server_->setCallback(boost::bind(&DriverNodelet::configCb, this, _1, _2));

  // Camera TF frames
  param_nh.param("rgb_frame_id",   rgb_frame_id_,   string("/openni_rgb_optical_frame"));
  param_nh.param("depth_frame_id", depth_frame_id_, string("/openni_depth_optical_frame"));
  NODELET_INFO("rgb_frame_id = '%s' ",   rgb_frame_id_.c_str());
  NODELET_INFO("depth_frame_id = '%s' ", depth_frame_id_.c_str());

  // Pixel offset between depth and IR images.
  // By default assume offset of (5,4) from 9x7 correlation window.
  // NOTE: These are now (temporarily?) dynamically reconfigurable, to allow tweaking.
  //param_nh.param("depth_ir_offset_x", depth_ir_offset_x_, 5.0);
  //param_nh.param("depth_ir_offset_y", depth_ir_offset_y_, 4.0);

  // The camera names are set to [rgb|depth]_[serial#], e.g. depth_B00367707227042B.
  // camera_info_manager substitutes this for ${NAME} in the URL.
  std::string serial_number = device_->getSerialNumber();
  std::string rgb_name, ir_name;
  if (serial_number.empty())
  {
    rgb_name = "rgb";
    ir_name  = "depth"; /// @todo Make it ir instead?
  }
  else
  {
    rgb_name = "rgb_"   + serial_number;
    ir_name  = "depth_" + serial_number;
  }

  std::string rgb_info_url, ir_info_url;
  param_nh.param("rgb_camera_info_url", rgb_info_url, std::string());
  param_nh.param("depth_camera_info_url", ir_info_url, std::string());

  // Suppress some of the output from loading camera calibrations (kinda hacky)
  log4cxx::LoggerPtr logger_ccp = log4cxx::Logger::getLogger("ros.camera_calibration_parsers");
  log4cxx::LoggerPtr logger_cim = log4cxx::Logger::getLogger("ros.camera_info_manager");
  logger_ccp->setLevel(log4cxx::Level::getFatal());
  logger_cim->setLevel(log4cxx::Level::getWarn());
  // Also suppress sync warnings from image_transport::CameraSubscriber. When subscribing to
  // depth_registered/foo with OpenNI registration disabled, the rectify nodelet for depth_registered/
  // will complain because it receives depth_registered/camera_info (from the register nodelet), but
  // the driver is not publishing depth_registered/image_raw.
  log4cxx::LoggerPtr logger_its = log4cxx::Logger::getLogger("ros.image_transport.sync");
  logger_its->setLevel(log4cxx::Level::getError());
  ros::console::notifyLoggerLevelsChanged();
  
  // Load the saved calibrations, if they exist
  rgb_info_manager_ = boost::make_shared<camera_info_manager::CameraInfoManager>(rgb_nh, rgb_name, rgb_info_url);
  ir_info_manager_  = boost::make_shared<camera_info_manager::CameraInfoManager>(ir_nh,  ir_name,  ir_info_url);

  if (!rgb_info_manager_->isCalibrated())
    NODELET_WARN("Using default parameters for RGB camera calibration.");
  if (!ir_info_manager_->isCalibrated())
    NODELET_WARN("Using default parameters for IR camera calibration.");

  // Advertise all published topics
  {
    // Prevent connection callbacks from executing until we've set all the publishers. Otherwise
    // connectCb() can fire while we're advertising (say) "depth/image_raw", but before we actually
    // assign to pub_depth_. Then pub_depth_.getNumSubscribers() returns 0, and we fail to start
    // the depth generator.
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    
    // Asus Xtion PRO does not have an RGB camera
    if (device_->hasImageStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::rgbConnectCb, this);
      pub_rgb_ = rgb_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }

    if (device_->hasIRStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::irConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::irConnectCb, this);
      pub_ir_ = ir_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }

    if (device_->hasDepthStream())
    {
      image_transport::SubscriberStatusCallback itssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      ros::SubscriberStatusCallback rssc = boost::bind(&DriverNodelet::depthConnectCb, this);
      pub_depth_ = depth_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
      pub_projector_info_ = projector_nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1, rssc, rssc);
      
      if (device_->isDepthRegistrationSupported())
        pub_depth_registered_ = depth_registered_it.advertiseCamera("image_raw", 1, itssc, itssc, rssc, rssc);
    }
  }

  // Create watch dog timer callback
  if (param_nh.getParam("time_out", time_out_) && time_out_ > 0.0)
  {
    time_stamp_ = ros::Time(0,0);
    watch_dog_timer_ = nh.createTimer(ros::Duration(time_out_), &DriverNodelet::watchDog, this);
  }
}

void DriverNodelet::setupDevice ()
{
  // Initialize the driver
  freenect_init(&driver_, NULL);
  freenect_set_log_level(driver_, FREENECT_LOG_NOTICE);
  freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

  do {

    if (freenect_num_devices(driver_) == 0)
    {
      NODELET_INFO ("No devices connected.... waiting for devices to be connected");
      boost::this_thread::sleep(boost::posix_time::seconds(3));
      continue;
    }

    NODELET_INFO ("Number devices connected: %d", freenect_num_devices(driver_));
    freenect_device_attributes* attr_list;
    freenect_device_attributes* item;
    freenect_list_device_attributes(&driver_->usb, &attr_list);

    int serial_index = 0;
    for (item = attrlist; item != NULL; item = item->next, ++serial_index)
    {
      NODELET_INFO("%u. device with with serial id \'%s\'",
                   serial_index, item->camera_serial)
    }

    string device_id;
    int index = 0;

    if (!getPrivateNodeHandle().getParam("device_id", device_id))
    {
      NODELET_WARN("~device_id is not set! Using first device (index = 0).");
      if (freenect_open_device(driver_, &device_, index) < 0) {
        index = -1;
      }
    }
    else if (device_id.find ('@') != string::npos)
    {
      NODELET_ERROR("device selected by bus@address unsupported");
      index = -1;
    }
    else if (device_id[0] == '#')
    {
      index = atoi(device_id.c_str());
      NODELET_INFO ("Searching for device with index = %d", index);
      if (freenect_open_device(driver_, &device_, index) < 0) {
        index = -1;
      }
    }
    else
    {
      NODELET_INFO ("Searching for device with serial number = '%s'", device_id.c_str());
      if (freenect_open_device_by_camera_serial(driver_, &device_, device_id.c_str()) < 0) {
        index = -1;
      }
    }

    if (index == -1) {
      NODELET_FATAL ("Unable to capture specified device.");
      exit(-1);
    }

  } while (!device_);

  freenect_set_user(device_, this);
  freenect_set_video_callback(device_, rgbCbWrapper);
  freenect_set_depth_callback(device_, depthCbWrapper);
}

void DriverNodelet::rgbConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  bool need_rgb = pub_rgb_.getNumSubscribers() > 0;
  
  if (need_rgb && !device_->isImageStreamRunning())
  {
    // Can't stream IR and RGB at the same time. Give RGB preference.
    if (device_->isIRStreamRunning())
    {
      NODELET_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
      device_->stopIRStream();
    }
    
    device_->startImageStream();
    startSynchronization();
    time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
  }
  else if (!need_rgb && device_->isImageStreamRunning())
  {
    stopSynchronization();
    device_->stopImageStream();

    // Start IR if it's been blocked on RGB subscribers
    bool need_ir = pub_ir_.getNumSubscribers() > 0;
    if (need_ir && !device_->isIRStreamRunning())
    {
      device_->startIRStream();
      time_stamp_ = ros::Time(0,0);
    }
  }
}

void DriverNodelet::depthConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  /// @todo pub_projector_info_? Probably also subscribed to a depth image if you need it
  bool need_depth =
    device_->isDepthRegistered() ? pub_depth_registered_.getNumSubscribers() > 0 : pub_depth_.getNumSubscribers() > 0;
  /// @todo Warn if requested topics don't agree with OpenNI registration setting

  if (need_depth && !device_->isDepthStreamRunning())
  {
    device_->startDepthStream();
    startSynchronization();
    time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
  }
  else if (!need_depth && device_->isDepthStreamRunning())
  {
    stopSynchronization();
    device_->stopDepthStream();
  }
}

void DriverNodelet::irConnectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  bool need_ir = pub_ir_.getNumSubscribers() > 0;
  
  if (need_ir && !device_->isIRStreamRunning())
  {
    // Can't stream IR and RGB at the same time
    if (device_->isImageStreamRunning())
    {
      NODELET_ERROR("Cannot stream RGB and IR at the same time. Streaming RGB only.");
    }
    else
    {
      device_->startIRStream();
      time_stamp_ = ros::Time(0,0); // starting an additional stream blocks for a while, could upset watchdog
    }
  }
  else if (!need_ir)
  {
    device_->stopIRStream();
  }
}

// If any stream is ready, publish next available image from all streams
// This publishes all available data with a maximum time offset of one frame between any two sources
// Need to have lock to call this, since callbacks can be in different threads
void DriverNodelet::checkFrameCounters()
{
    if (max(rgb_frame_counter_, max(depth_frame_counter_, ir_frame_counter_)) > config_.data_skip) {
        // Reset all counters after we trigger publish
        rgb_frame_counter_   = 0;
        depth_frame_counter_ = 0;
        ir_frame_counter_    = 0;

        // Trigger publish on all topics
        publish_rgb_   = true;
        publish_depth_ = true;
        publish_ir_    = true;
    }
}

void DriverNodelet::rgbCb(boost::shared_ptr<openni_wrapper::Image> image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.image_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      rgb_frame_counter_++;
      checkFrameCounters();
      publish = publish_rgb_;

      if (publish)
          rgb_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishRgbImage(*image, time);

  publish_rgb_ = false;
}

void DriverNodelet::depthCb(boost::shared_ptr<openni_wrapper::DepthImage> depth_image, void* cookie)
{
  ros::Time time = ros::Time::now () + ros::Duration(config_.depth_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      depth_frame_counter_++;
      checkFrameCounters();
      publish = publish_depth_;

      if (publish)
          depth_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishDepthImage(*depth_image, time);

  publish_depth_ = false;
}

void DriverNodelet::irCb(boost::shared_ptr<openni_wrapper::IRImage> ir_image, void* cookie)
{
  ros::Time time = ros::Time::now() + ros::Duration(config_.depth_time_offset);
  time_stamp_ = time; // for watchdog

  bool publish = false;
  {
      boost::unique_lock<boost::mutex> counter_lock(counter_mutex_);
      ir_frame_counter_++;
      checkFrameCounters();
      publish = publish_ir_;

      if (publish)
          ir_frame_counter_ = 0; // Reset counter if we publish this message to avoid under-throttling
  }

  if (publish)
      publishIrImage(*ir_image, time);
  publish_ir_ = false;
}

void DriverNodelet::publishRgbImage(const openni_wrapper::Image& image, ros::Time time) const
{
  /// @todo image_width_, image_height_ may be wrong here if downsampling is on?
  sensor_msgs::ImagePtr rgb_msg = boost::make_shared<sensor_msgs::Image >();
  rgb_msg->header.stamp = time;
  rgb_msg->header.frame_id = rgb_frame_id_;
  if (image.getEncoding() == openni_wrapper::Image::BAYER_GRBG)
  {
    rgb_msg->encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
    rgb_msg->step = image_width_;
  }
  else if (image.getEncoding() == openni_wrapper::Image::YUV422)
  {
    rgb_msg->encoding = sensor_msgs::image_encodings::YUV422;
    rgb_msg->step = image_width_ * 2; // 4 bytes for 2 pixels
  }
  rgb_msg->height = image_height_;
  rgb_msg->width = image_width_;
  rgb_msg->data.resize(rgb_msg->height * rgb_msg->step);
  
  image.fillRaw(&rgb_msg->data[0]);
  
  pub_rgb_.publish(rgb_msg, getRgbCameraInfo(time));
}

void DriverNodelet::publishDepthImage(const openni_wrapper::DepthImage& depth, ros::Time time) const
{
  bool registered = device_->isDepthRegistered();
  
  /// @todo Get rid of depth_height_, depth_width_
  sensor_msgs::ImagePtr depth_msg = boost::make_shared<sensor_msgs::Image>();
  depth_msg->header.stamp    = time;
  depth_msg->encoding        = sensor_msgs::image_encodings::TYPE_16UC1;
  depth_msg->height          = depth_height_;
  depth_msg->width           = depth_width_;
  depth_msg->step            = depth_msg->width * sizeof(short);
  depth_msg->data.resize(depth_msg->height * depth_msg->step);

  depth.fillDepthImageRaw(depth_width_, depth_height_, reinterpret_cast<unsigned short*>(&depth_msg->data[0]),
                          depth_msg->step);

  if (z_offset_mm_ != 0)
  {
    uint16_t* data = reinterpret_cast<uint16_t*>(&depth_msg->data[0]);
    for (unsigned int i = 0; i < depth_msg->width * depth_msg->height; ++i)
      if (data[i] != 0)
	data[i] += z_offset_mm_;
  }

  if (registered)
  {
    // Publish RGB camera info and raw depth image to depth_registered/ ns
    depth_msg->header.frame_id = rgb_frame_id_;
    pub_depth_registered_.publish(depth_msg, getRgbCameraInfo(time));
  }
  else
  {
    // Publish depth camera info and raw depth image to depth/ ns
    depth_msg->header.frame_id = depth_frame_id_;
    pub_depth_.publish(depth_msg, getDepthCameraInfo(time));
  }

  // Projector "info" probably only useful for working with disparity images
  if (pub_projector_info_.getNumSubscribers() > 0)
  {
    pub_projector_info_.publish(getProjectorCameraInfo(time));
  }
}

void DriverNodelet::publishIrImage(const openni_wrapper::IRImage& ir, ros::Time time) const
{
  sensor_msgs::ImagePtr ir_msg = boost::make_shared<sensor_msgs::Image>();
  ir_msg->header.stamp    = time;
  ir_msg->header.frame_id = depth_frame_id_;
  ir_msg->encoding        = sensor_msgs::image_encodings::MONO16;
  ir_msg->height          = ir.getHeight();
  ir_msg->width           = ir.getWidth();
  ir_msg->step            = ir_msg->width * sizeof(uint16_t);
  ir_msg->data.resize(ir_msg->height * ir_msg->step);

  ir.fillRaw(ir.getWidth(), ir.getHeight(), reinterpret_cast<unsigned short*>(&ir_msg->data[0]));

  pub_ir_.publish(ir_msg, getIrCameraInfo(time));
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDefaultCameraInfo(int width, int height, double f) const
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;

  // No distortion
  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  // Simple camera matrix: square pixels (fx = fy), principal point at center
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  // Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
  // This formula keeps the principal point the same in VGA and SXGA modes
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;

  // No separate rectified image plane, so R = I
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;

  // Then P=K(I|0) = (K|0)
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; // fx, fy
  info->P[2]  = info->K[2];     // cx
  info->P[6]  = info->K[5];     // cy
  info->P[10] = 1.0;

  return info;
}

/// @todo Use binning/ROI properly in publishing camera infos
sensor_msgs::CameraInfoPtr DriverNodelet::getRgbCameraInfo(ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (rgb_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(rgb_info_manager_->getCameraInfo());
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(image_width_, image_height_, device_->getImageFocalLength(image_width_));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = rgb_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getIrCameraInfo(ros::Time time) const
{
  sensor_msgs::CameraInfoPtr info;

  if (ir_info_manager_->isCalibrated())
  {
    info = boost::make_shared<sensor_msgs::CameraInfo>(ir_info_manager_->getCameraInfo());
  }
  else
  {
    // If uncalibrated, fill in default values
    info = getDefaultCameraInfo(depth_width_, depth_height_, device_->getDepthFocalLength(depth_width_));
  }

  // Fill in header
  info->header.stamp    = time;
  info->header.frame_id = depth_frame_id_;

  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getDepthCameraInfo(ros::Time time) const
{
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7). See http://www.ros.org/wiki/kinect_calibration/technical

  sensor_msgs::CameraInfoPtr info = getIrCameraInfo(time);
  info->K[2] -= depth_ir_offset_x_; // cx
  info->K[5] -= depth_ir_offset_y_; // cy
  info->P[2] -= depth_ir_offset_x_; // cx
  info->P[6] -= depth_ir_offset_y_; // cy

  /// @todo Could put this in projector frame so as to encode the baseline in P[3]
  return info;
}

sensor_msgs::CameraInfoPtr DriverNodelet::getProjectorCameraInfo(ros::Time time) const
{
  // The projector info is simply the depth info with the baseline encoded in the P matrix.
  // It's only purpose is to be the "right" camera info to the depth camera's "left" for
  // processing disparity images.
  sensor_msgs::CameraInfoPtr info = getDepthCameraInfo(time);
  // Tx = -baseline * fx
  info->P[3] = -device_->getBaseline() * info->P[0];
  return info;
}

void DriverNodelet::configCb(Config &config, uint32_t level)
{
  z_offset_mm_ = config.z_offset_mm;

  XnMapOutputMode old_depth_mode = device_->getDepthOutputMode ();
  
  // We need this for the ASUS Xtion Pro
  XnMapOutputMode old_image_mode = old_depth_mode, image_mode, compatible_image_mode;
  if (device_->hasImageStream ())
  {
    old_image_mode = device_->getImageOutputMode ();
     
    // does the device support the new image mode?
    image_mode = mapConfigMode2XnMode (config.image_mode);

    if (!device_->findCompatibleImageMode (image_mode, compatible_image_mode))
    {
      XnMapOutputMode default_mode = device_->getDefaultImageMode();
      NODELET_WARN("Could not find any compatible image output mode for %d x %d @ %d. "
                   "Falling back to default image output mode %d x %d @ %d.",
                    image_mode.nXRes, image_mode.nYRes, image_mode.nFPS,
                    default_mode.nXRes, default_mode.nYRes, default_mode.nFPS);

      config.image_mode = mapXnMode2ConfigMode(default_mode);
      image_mode = compatible_image_mode = default_mode;
    }
  }
  
  XnMapOutputMode depth_mode, compatible_depth_mode;
  depth_mode = mapConfigMode2XnMode (config.depth_mode);
  if (!device_->findCompatibleDepthMode (depth_mode, compatible_depth_mode))
  {
    XnMapOutputMode default_mode = device_->getDefaultDepthMode();
    NODELET_WARN ("Could not find any compatible depth output mode for %d x %d @ %d. "
                  "Falling back to default depth output mode %d x %d @ %d.",
                  depth_mode.nXRes, depth_mode.nYRes, depth_mode.nFPS,
                  default_mode.nXRes, default_mode.nYRes, default_mode.nFPS);
    
    config.depth_mode = mapXnMode2ConfigMode(default_mode);
    depth_mode = compatible_depth_mode = default_mode;
  }

  // here everything is fine. Now make the changes
  if ( (device_->hasImageStream () && compatible_image_mode != old_image_mode) ||
       compatible_depth_mode != old_depth_mode)
  {
    // streams need to be reset!
    stopSynchronization();

    if (device_->hasImageStream () && compatible_image_mode != old_image_mode)
      device_->setImageOutputMode (compatible_image_mode);

    if (compatible_depth_mode != old_depth_mode)
      device_->setDepthOutputMode (compatible_depth_mode);

    startSynchronization ();
  }

  // Desired dimensions may require decimation from the hardware-compatible ones
  image_width_  = image_mode.nXRes;
  image_height_ = image_mode.nYRes;
  depth_width_  = depth_mode.nXRes;
  depth_height_ = depth_mode.nYRes;

  /// @todo Run connectCb if registration setting changes
  if (device_->isDepthRegistered () && !config.depth_registration)
  {
    device_->setDepthRegistration (false);
  }
  else if (!device_->isDepthRegistered () && config.depth_registration)
  {
    device_->setDepthRegistration (true);
  }

  // now we can copy
  config_ = config;
}

void DriverNodelet::mapConfigMode2XnMode(int config_mode, freenect_video_format& mode, freenect_resolution& res) {
  switch(config_mode) {
    case OpenNI_SXGA_15Hz:
      mode = 
      res = 

  }
}

void DriverNodelet::mapConfigMode2XnMode(int config_mode, freenect_depth_format& mode, freenect_resolution& res) {
}

void DriverNodelet::watchDog (const ros::TimerEvent& event)
{
  /// @todo Also watch IR
  if ( !time_stamp_.isZero() && (device_->isDepthStreamRunning() || device_->isImageStreamRunning()) )
  {
    ros::Duration duration = ros::Time::now() - time_stamp_;
    if (duration.toSec() >= time_out_)
    {
      NODELET_ERROR("Timeout");
      watch_dog_timer_.stop();
      throw std::runtime_error("Timeout occured in DriverNodelet");
    }
  }
}

}

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_DECLARE_CLASS (freenect_camera, driver, freenect_camera::DriverNodelet, nodelet::Nodelet);
