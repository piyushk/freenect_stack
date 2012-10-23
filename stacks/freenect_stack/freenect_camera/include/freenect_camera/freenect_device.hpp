#ifndef FREENECT_DEVICE_T01IELX0
#define FREENECT_DEVICE_T01IELX0

#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <boost/thread/mutex.hpp>
#include <stdexcept>

#include <freenect/libfreenect.h>
#include <freenect/libfreenect-registration.h>
#include <freenect_camera/image.hpp>

namespace freenect_camera {

  static const std::string PRODUCT_NAME = "Xbox NUI Camera";
  static const unsigned PRODUCT_ID = 0x2ae;
  static const std::string VENDOR_NAME = "Microsoft";
  static const unsigned VENDOR_ID = 0x45e;

  typedef freenect_frame_mode OutputMode;

  class FreenectDriver;

  class FreenectDevice : public boost::noncopyable {

    private:

      typedef boost::lock_guard<boost::recursive_mutex> LockGuard;

    public:

      FreenectDevice(freenect_context* driver, std::string serial) {
        LockGuard depth_lock(m_depth_);
        LockGuard video_lock(m_video_);
        if (freenect_open_device_by_camera_serial(driver, &device_, serial.c_str()) < 0) {
          throw std::runtime_error("Unable to open specified kinect");
        }
        freenect_set_user(device_, this);
        freenect_set_depth_callback(device_, freenectDepthCallback);
        freenect_set_video_callback(device_, freenectVideoCallback);
        device_serial_ = serial;
        registration_ = freenect_copy_registration(device_);

        //Initialize default variables
        video_mode_.resolution = FREENECT_RESOLUTION_MEDIUM;
        video_mode_.video_format = FREENECT_VIDEO_RGB;
        depth_mode_.resolution = FREENECT_RESOLUTION_MEDIUM;
        depth_mode_.depth_format = FREENECT_DEPTH_MM;
        streaming_video_ = false;
        streaming_depth_ = false;
      }

      ~FreenectDevice() {
      }

      void shutdown() {
        LockGuard depth_lock(m_depth_);
        LockGuard video_lock(m_video_);
        freenect_close_device(device_);
        freenect_destroy_registration(&registration_);
      }

      /** Unsupported */
      unsigned getBus() {
        return 0;
      }

      /** Unsupported */
      unsigned getAddress() {
        return 0;
      }

      const char* getProductName() {
        return PRODUCT_NAME.c_str();
      }

      const char* getSerialNumber() {
        return device_serial_.c_str();
      }

      template<typename T> void registerImageCallback (void (T::*callback)(boost::shared_ptr<Image> image, void* cookie), T& instance, void* cookie = NULL) {
        image_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerDepthCallback (void (T::*callback)(boost::shared_ptr<ImageBuffer> depth_image, void* cookie), T& instance, void* cookie = NULL) {
        depth_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerIRCallback (void (T::*callback)(boost::shared_ptr<ImageBuffer> ir_image, void* cookie), T& instance, void* cookie = NULL) {
        ir_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      bool isImageStreamRunning() {
        LockGuard video_lock(m_video_);
        return streaming_video_ && _isImageModeEnabled();
      }

      bool isIRStreamRunning() {
        LockGuard video_lock(m_video_);
        return streaming_video_ && !_isImageModeEnabled();
      }

      bool isDepthStreamRunning() {
        LockGuard depth_lock(m_depth_);
        return streaming_depth_;
      }

      bool hasImageStream() {
        return true;
      }

      bool hasDepthStream() {
        return true;
      }

      bool hasIRStream() {
        return true;
      }

      bool isDepthRegistrationSupported() {
        return true;
      }

      bool isSynchronizationSupported() {
        return false;
      }

      bool isSynchronized() {
        return false;
      }

      bool setSynchronization(bool on_off) {
        throw std::runtime_error("the kinect does not support hardware synchronization");
      }

      void _stopVideoStream() {
        std::cout << "stop video start" << std::endl;
        LockGuard video_lock(m_video_);
        if (streaming_video_) {
          streaming_video_ = false;
          freenect_stop_video(device_);
        }
        std::cout << "stop video fin" << std::endl;
      }

      void _startVideoStream(freenect_frame_mode mode) {
        std::cout << "start video start" << std::endl;
        LockGuard video_lock(m_video_);
        if (!streaming_video_) {
          streaming_video_ = true;
          mode = freenect_find_video_mode(mode.resolution, mode.video_format);
          if (!mode.is_valid)
            std::cout << "mode not valid" << std::endl;
          if (freenect_set_video_mode(device_, mode) < 0)
            std::cout << "set video mode failed" << std::endl;
          video_mode_ = mode;
          if (freenect_start_video(device_) < 0)
            std::cout << "could not start video" << std::endl;
        }
        std::cout << "start video stop" << std::endl;
      }

      void stopImageStream() {
        LockGuard video_lock(m_video_);
        if (_isImageModeEnabled()) {
          _stopVideoStream();
        }
      }

      void startImageStream() {
        LockGuard video_lock(m_video_);
        freenect_frame_mode mode = video_mode_;
        mode.video_format = FREENECT_VIDEO_RGB;
        _startVideoStream(mode);
      }

      void stopIRStream() {
        LockGuard video_lock(m_video_);
        if (!_isImageModeEnabled()) {
          _stopVideoStream();
        }
      }

      void startIRStream() {
        LockGuard video_lock(m_video_);
        freenect_frame_mode mode = video_mode_;
        mode.video_format = FREENECT_VIDEO_IR_8BIT;
        _startVideoStream(mode);
      }

      void stopDepthStream() {
        LockGuard depth_lock(m_depth_);
        streaming_depth_ = false;
        freenect_stop_depth(device_);
      }

      void startDepthStream() {
        LockGuard depth_lock(m_depth_);
        streaming_depth_ = true;
        freenect_frame_mode mode = 
          freenect_find_depth_mode(depth_mode_.resolution, depth_mode_.depth_format);
        freenect_set_depth_mode(device_, mode);
        depth_mode_ = mode;
        freenect_start_depth(device_);
      }

      OutputMode getImageOutputMode() {
        LockGuard video_lock(m_video_);
        return video_mode_;
      }

      void setImageOutputMode(OutputMode mode) {
        LockGuard video_lock(m_video_);
        bool restartVideoStream = false;
        if (isImageStreamRunning() || isIRStreamRunning()) {
          _stopVideoStream();
          restartVideoStream = true;
        }
        video_mode_.resolution = mode.resolution;
        if (restartVideoStream) {
          _startVideoStream(video_mode_);
        }
      }

      OutputMode getDefaultImageMode() {
        OutputMode mode;
        mode.resolution = FREENECT_RESOLUTION_MEDIUM;
        return mode;
      }

      bool findCompatibleImageMode(const OutputMode& mode, OutputMode& compatible_mode) {
        OutputMode new_mode = freenect_find_video_mode(mode.resolution, video_mode_.video_format);
        if (!new_mode.is_valid) {
          return false;
        }
        compatible_mode = new_mode;
        return true;
      }

      OutputMode getDepthOutputMode() {
        LockGuard depth_lock(m_depth_);
        return depth_mode_;
      }

      void setDepthOutputMode(OutputMode mode) {
        LockGuard depth_lock(m_depth_);
        bool restartDepthMode = false;
        if (isDepthStreamRunning()) {
          _stopVideoStream();
          restartDepthMode = true;
        }
        depth_mode_.resolution = mode.resolution;
        if (restartDepthMode) {
          _startVideoStream(video_mode_);
        }
      }

      OutputMode getDefaultDepthMode() {
        OutputMode mode;
        mode.resolution = FREENECT_RESOLUTION_MEDIUM;
        return mode;
      }

      bool findCompatibleDepthMode(const OutputMode& mode, OutputMode& compatible_mode) {
        OutputMode new_mode = freenect_find_depth_mode(mode.resolution, depth_mode_.depth_format);
        if (!new_mode.is_valid) {
          return false;
        }
        compatible_mode = new_mode;
        return true;
      }

      bool isDepthRegistered() {
        LockGuard depth_lock(m_depth_);
        return depth_mode_.depth_format == FREENECT_DEPTH_REGISTERED;
      }

      void setDepthRegistration(bool enable) {
        LockGuard depth_lock(m_depth_);
        bool restartDepthStream = false;
        if (isDepthStreamRunning()) {
          stopDepthStream();
          restartDepthStream = true;
        }
        if (enable) {
          depth_mode_.depth_format = FREENECT_DEPTH_REGISTERED;
        } else {
          depth_mode_.depth_format = FREENECT_DEPTH_MM;
        }
        if (restartDepthStream) {
          startDepthStream();
        }
      }

      float getImageFocalLength(unsigned width) {
        float scale = width / WIDTH_SXGA;
        return RGB_FOCAL_LENGTH_SXGA * scale;
      }

      float getDepthFocalLength(unsigned width) {
        float depth_focal_length_sxga = 
          registration_.zero_plane_info.reference_distance /
          registration_.zero_plane_info.reference_pixel_size;
        float scale = width / WIDTH_SXGA;
        if (isDepthRegistered()) 
          return RGB_FOCAL_LENGTH_SXGA * scale;
        return depth_focal_length_sxga * scale;
      }

      float getBaseline() {
        return 0.01 * registration_.zero_plane_info.dcmos_emitter_dist; //cm to m
      }

      static void freenectDepthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->depthCallback(depth);
      }

      static void freenectVideoCallback(freenect_device *dev, void *video, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->videoCallback(video);
      }

    private:

      friend class FreenectDriver;

      freenect_device* device_;
      std::string device_serial_;
      freenect_registration registration_;

      boost::function<void(boost::shared_ptr<ImageBuffer>)> image_callback_;
      boost::function<void(boost::shared_ptr<ImageBuffer>)> depth_callback_;
      boost::function<void(boost::shared_ptr<ImageBuffer>)> ir_callback_;

      bool streaming_video_;
      bool change_video_settings_;
      bool streaming_depth_;
      bool change_depth_settings_;

      ImageBuffer depth_buffer_;
      OutputMode depth_mode_;
      
      ImageBuffer video_buffer_;
      OutputMode video_mode_;

      /* Prevents changing settings unless the freenect thread in the driver
       * is ready */
      boost::recursive_mutex m_settings_;

      void executeChanges() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        if (change_



      }

      bool _isImageModeEnabled() {
        boost::lock_guard<boost::recursive_mutex> lock(m_settings_);
        return isImageBuffervideo_mode_.video_format == FREENECT_VIDEO_RGB;
      }

      void depthCallback(void* depth) {
        boost::lock_guard<boost::mutex> buffer_lock(depth_buffer_->mutex);
        assert(depth == depth_buffer_->image_buffer.get());
        depth_callback_.operator()(depth_buffer_);
      }

      void videoCallback(void* video) {
        boost::lock_guard<boost::mutex> buffer_lock(video_buffer_->mutex);
        assert(video == video_buffer_->image_buffer.get());
        if (isImageMode(video_buffer_)) {
          image_callback_.operator()(video_buffer_);
        } else {
          ir_callback_.operator()(video_buffer_);
        }
      }

  };
}

#endif /* end of include guard: FREENECT_DEVICE_T01IELX0 */
