#include <freenect/libfreenect.h>
#include <boost/shared_ptr.hpp>
#include <boost/noncopyable.hpp>
#include <stdexcept>

namespace freenect_camera {

  static const std::string PRODUCT_NAME = "Xbox NUI Camera";
  static const std::string PRODUCT_ID = 0x2ae;
  static const std::string VENDOR_NAME = "Microsoft";
  static const std::string VENDOR_ID = 0x45e;
  static const std::string UNKNOWN = "unknown";

  typedef freenect_frame_mode
  typedef freenect_frame_mode OutputMode;

  class FreenectDevice : public boost::noncopyable {

    public:

      FreenectDevice(freenect_context* driver, std::string serial) {
        if (freenect_open_device_by_serial(driver, &device_, serial.c_str()) < 0) {
          throw std::runtime_error("Unable to open specified kinect");
        }
        freenect_set_user(device_, this);
        freenect_set_depth_callback(m_dev, freenectDepthCallback);
        freenect_set_video_callback(m_dev, freenectVideoCallback);
        device_serial_ = serial;
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

      template<typename T> void registerImageCallback (void (T::*callback)(void* image, void* cookie), T& instance, void* cookie = NULL) {
        image_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerDepthCallback (void (T::*callback)(void* image, void* cookie), T& instance, void* cookie = NULL) {
        depth_callback_ = boost::bind(callback, boost::ref(instance), _1, cookie);
      }

      template<typename T> void registerIRCallback (void (T::*callback)(void* image, void* cookie), T& instance, void* cookie = NULL) {
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

      bool isSynchronizedSupported() {
        return false;
      }

      bool isSynchronized() {
        return false;
      }

      bool setSynchronization(bool on_off) {
        throw std::runtime_error("the kinect does not support hardware synchronization");
      }

      void _stopVideoStream() {
        freenect_stop_video(device_);
      }

      void _startVideoStream(freenect_frame_mode mode) {
        freenect_frame_mode mode = 
          freenect_find_video_mode(mode.resolution, mode.format);
        if (!mode.is_valid) {
          NODELET_WARN("Invalid image mode provided");
        } else if (freenect_set_video_mode(device_, mode) < 0) {
          NODELET_ERROR("Unable to update image mode."); 
        } else {
          // Everything went fine. 
          mode_ = mode;
        }
        freenect_start_video(device_);
      }

      void stopImageStream() {
        if (isImageModeEnabled_()) {
          _stopVideoStream();
        }
      }

      void startImageStream() {
        freenect_frame_mode mode = image_mode_;
        mode.format = FREENECT_VIDEO_RGB;
        _startVideoStream(mode);
      }

      void stopIRStream() {
        if (!isImageModeEnabled_()) {
          _stopVideoStream();
        }
      }

      void startIRStream() {
        freenect_frame_mode mode = image_mode_;
        mode.format = FREENECT_VIDEO_IR_8BIT;
        _stopVideoStream(mode);
      }

      void stopDepthStream() {
        freenect_stop_depth(device_);
      }

      void startDepthStream() {
        freenect_frame_mode mode = 
          freenect_find_depth_mode(depth_mode_.resolution, depth_mode_.format);
        if (!mode.is_valid) {
          NODELET_WARN("Invalid depth mode provided");
        } else if (freenect_set_depth_mode(device_, mode) < 0) {
          NODELET_ERROR("Unable to update depth mode."); 
        } else {
          // Everything went fine. 
          depth_mode_ = depth_mode;
        }
        freenect_start_depth(device_);
      }

      setImageOutputMode()
      getDefaultImageMode()
      findCompatibleImageMode()

      setDepthOutputMode()
      getDefaultDepthMode()
      findCompatibleDepthMode()

      setDepthRegistration() {
        if (streaming_depth_)
          stopDepthStream()
        if (streaming_depth_)
          startDepthStream()
      }

    private:

      freenect_device* device_;
      freenect_frame_mode video_mode_;
      freenect_frame_mode depth_mode_;
      std_string device_serial_;

      boost::function<void(void*)> image_callback_;
      boost::function<void(void*)> depth_callback_;
      boost::function<void(void*)> ir_callback_;

      bool streaming_video_;
      bool streaming_depth_;

      static void freenectDepthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        // Do any processing here
        depth_callback_->operator()(depth);
        device->depthCallback(depth, timestamp);
      }
      static void freenectVideoCallback(freenect_device *dev, void *video, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->videoCallback(video, timestamp);
      }

      bool _isImageModeEnabled() {
        return video_mode_.format == FREENECT_VIDEO_RGB;
      }

  }

  class FreenectDriver {

    public:

      static FreenectDriver& getInstance() {
        static FreenectDriver instance;
        return instance;
      }

      void shutdown() {
        freenect_shutdown(driver_);
      }

      void updateDeviceList() {
        device_serials.clear()

        freenect_device_attributes* attr_list;
        freenect_device_attributes* item;
        freenect_list_device_attributes(&driver_->usb, &attr_list);

        for (item = attrlist; item != NULL; item = item->next, ++serial_index) {
          device_serials.push_back(std::string(item->camera_serials));
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

      unsigned getProductId(unsigned device_idx) {
        return PRODUCT_ID;
      }

      const char* getVendorName(unsigned device_idx) {
        return VENDOR_NAME.c_str();
      }

      unsigned getVendorId(unsigned device_idx) {
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

    private:
      FreenectDriver() {
        freenect_init(&driver_, NULL);
        freenect_set_log_level(driver_, FREENECT_LOG_NOTICE);
        freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

      }

      freenect_context* driver_;
      std::vector<std::string> device_serials;
  };

}

