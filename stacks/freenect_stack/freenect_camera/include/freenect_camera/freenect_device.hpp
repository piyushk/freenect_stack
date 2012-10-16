#include <freenect/libfreenect.h>
#include <freenect/libfreenect-registration.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <stdexcept>

namespace freenect_camera {

  static const std::string PRODUCT_NAME = "Xbox NUI Camera";
  static const unsigned PRODUCT_ID = 0x2ae;
  static const std::string VENDOR_NAME = "Microsoft";
  static const unsigned VENDOR_ID = 0x45e;
  static const std::string UNKNOWN = "unknown";

  static const float RGB_FOCAL_LENGTH_SXGA_ = 1050;
  static const float WIDTH_SXGA = 1280;

  typedef freenect_frame_mode OutputMode;

  class Image {
    public:
      static const freenect_video_format BAYER_GRBG = FREENECT_VIDEO_BAYER;
      static const freenect_video_format YUV422 = FREENECT_VIDEO_YUV_RGB;
      OutputMode metadata;
      void* data;

      freenect_video_format getEncoding() const {
        return metadata.video_format;
      }

      void fillRaw(unsigned char* image_data) const {
        //std::cout << "size: " << metadata.bytes << std::endl;
        memcpy(image_data, data, metadata.bytes);
      }

      void fillDepthImageRaw(unsigned short* depth_data) const {
        memcpy(depth_data, data, metadata.bytes);
      }

      int getHeight() const {
        return metadata.height;
      }

      int getWidth() const {
        return metadata.width;
      }
  };

  typedef Image DepthImage;
  typedef Image IRImage;

  class FreenectDevice : public boost::noncopyable {

    public:

      FreenectDevice(freenect_context* driver, std::string serial) {
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

      template<typename T> void registerDepthCallback (void (T::*callback)(boost::shared_ptr<DepthImage> depth_image, void* cookie), T& instance, void* cookie = NULL) {
        depth_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerIRCallback (void (T::*callback)(boost::shared_ptr<IRImage> ir_image, void* cookie), T& instance, void* cookie = NULL) {
        ir_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      bool isImageStreamRunning() {
        return streaming_video_ && _isImageModeEnabled();
      }

      bool isIRStreamRunning() {
        return streaming_video_ && !_isImageModeEnabled();
      }

      bool isDepthStreamRunning() {
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
        streaming_video_ = false;
        freenect_stop_video(device_);
      }

      void _startVideoStream(freenect_frame_mode mode) {
        streaming_video_ = true;
        mode = freenect_find_video_mode(mode.resolution, mode.video_format);
        freenect_set_video_mode(device_, mode);
        video_mode_ = mode;
        freenect_start_video(device_);
      }

      void stopImageStream() {
        if (_isImageModeEnabled()) {
          _stopVideoStream();
        }
      }

      void startImageStream() {
        freenect_frame_mode mode = video_mode_;
        mode.video_format = FREENECT_VIDEO_RGB;
        _startVideoStream(mode);
      }

      void stopIRStream() {
        if (!_isImageModeEnabled()) {
          _stopVideoStream();
        }
      }

      void startIRStream() {
        freenect_frame_mode mode = video_mode_;
        mode.video_format = FREENECT_VIDEO_IR_8BIT;
        _startVideoStream(mode);
      }

      void stopDepthStream() {
        streaming_depth_ = false;
        freenect_stop_depth(device_);
      }

      void startDepthStream() {
        streaming_depth_ = true;
        freenect_frame_mode mode = 
          freenect_find_depth_mode(depth_mode_.resolution, depth_mode_.depth_format);
        freenect_set_depth_mode(device_, mode);
        depth_mode_ = mode;
        freenect_start_depth(device_);
      }

      OutputMode getImageOutputMode() {
        return video_mode_;
      }

      void setImageOutputMode(OutputMode mode) {
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
        return depth_mode_;
      }

      void setDepthOutputMode(OutputMode mode) {
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
        return depth_mode_.depth_format == FREENECT_DEPTH_REGISTERED;
      }

      void setDepthRegistration(bool enable) {
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
        return RGB_FOCAL_LENGTH_SXGA_ * scale;
      }

      float getDepthFocalLength(unsigned width) {
        float depth_focal_length_sxga = 
          registration_.zero_plane_info.reference_distance /
          registration_.zero_plane_info.reference_pixel_size;
        float scale = depth_mode_.width / WIDTH_SXGA;
        return depth_focal_length_sxga * scale;
      }

      float getBaseline() {
        return 0.01 * registration_.zero_plane_info.dcmos_emitter_dist; //cm to m
      }

      static void freenectDepthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        boost::shared_ptr<DepthImage> depth_ptr = _getDepth(device, depth);
        device->depth_callback_.operator()(depth_ptr);
      }

      static void freenectVideoCallback(freenect_device *dev, void *video, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        if (device->isImageStreamRunning()) {
          boost::shared_ptr<Image> image_ptr = _getImage(device, video);
          device->image_callback_.operator()(image_ptr);
        } else {
          boost::shared_ptr<IRImage> ir_ptr = _getIR(device, video);
          device->ir_callback_.operator()(ir_ptr);
        }
      }

    private:

      freenect_device* device_;
      freenect_frame_mode video_mode_;
      freenect_frame_mode depth_mode_;
      std::string device_serial_;
      freenect_registration registration_;

      boost::function<void(boost::shared_ptr<Image>)> image_callback_;
      boost::function<void(boost::shared_ptr<DepthImage>)> depth_callback_;
      boost::function<void(boost::shared_ptr<IRImage>)> ir_callback_;

      bool streaming_video_;
      bool streaming_depth_;

      bool _isImageModeEnabled() {
        return video_mode_.video_format == FREENECT_VIDEO_RGB;
      }

      static boost::shared_ptr<Image> _getImage(FreenectDevice* device, void* video) {
        boost::shared_ptr<Image> image_ptr;
        image_ptr.reset(new Image());
        image_ptr->data = video;
        image_ptr->metadata = device->video_mode_;
        return image_ptr;
      }

      static boost::shared_ptr<DepthImage> _getDepth(FreenectDevice* device, void* depth) {
        boost::shared_ptr<DepthImage> depth_ptr;
        depth_ptr.reset(new DepthImage());
        depth_ptr->data = depth;
        depth_ptr->metadata = device->depth_mode_;
        return depth_ptr;
      }

      static boost::shared_ptr<IRImage> _getIR(FreenectDevice* device, void* video) {
        boost::shared_ptr<IRImage> ir_ptr;
        ir_ptr.reset(new IRImage());
        ir_ptr->data = video;
        ir_ptr->metadata = device->video_mode_;
        return ir_ptr;
      }

  };

  class FreenectDriver {

    public:

      static FreenectDriver& getInstance() {
        static FreenectDriver instance;
        return instance;
      }

      void shutdown() {
        stop = true;
        freenect_thread->join();
        freenect_shutdown(driver_);
      }

      void updateDeviceList() {
        device_serials.clear();
        freenect_device_attributes* attr_list;
        freenect_device_attributes* item;
        freenect_list_device_attributes(driver_, &attr_list);
        for (item = attr_list; item != NULL; item = item->next) {
          device_serials.push_back(std::string(item->camera_serial));
        }
      }

      unsigned getNumberDevices() {
        return device_serials.size();
      }

      /** Unsupported */
      unsigned getBus(unsigned device_idx) {
        return 0;
      }

      /** Unsupported */
      unsigned getAddress(unsigned device_idx) {
        return 0;
      }

      const char* getProductName(unsigned device_idx) {
        return PRODUCT_NAME.c_str();
      }

      unsigned getProductID(unsigned device_idx) {
        return PRODUCT_ID;
      }

      const char* getVendorName(unsigned device_idx) {
        return VENDOR_NAME.c_str();
      }

      unsigned getVendorID(unsigned device_idx) {
        return VENDOR_ID;
      }

      const char* getSerialNumber(unsigned device_idx) {
        if (device_idx < getNumberDevices())
          return device_serials[device_idx].c_str();
        return UNKNOWN.c_str();
      }

      boost::shared_ptr<FreenectDevice> getDeviceByIndex(unsigned device_idx) {
        return getDeviceBySerialNumber(std::string(getSerialNumber(device_idx)));
      }

      boost::shared_ptr<FreenectDevice> getDeviceBySerialNumber(std::string serial) {
        boost::shared_ptr<FreenectDevice> device;
        device.reset(new FreenectDevice(driver_, serial));
        return device;
      }

      boost::shared_ptr<FreenectDevice> getDeviceByAddress(unsigned bus, unsigned address) {
        throw std::runtime_error("freenect_camera does not support searching for device by bus/address");
      }

      void process() {
        while (!stop) {
          freenect_process_events(driver_);
        }
      }

    private:
      FreenectDriver() {
        freenect_init(&driver_, NULL);
        freenect_set_log_level(driver_, FREENECT_LOG_NOTICE);
        freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
        // start freenect thread
        stop = false;
        freenect_thread.reset(new boost::thread(boost::bind(&FreenectDriver::process, this)));
      }

      freenect_context* driver_;
      std::vector<std::string> device_serials;
      boost::shared_ptr<boost::thread> freenect_thread;
      bool stop;
  };

}

