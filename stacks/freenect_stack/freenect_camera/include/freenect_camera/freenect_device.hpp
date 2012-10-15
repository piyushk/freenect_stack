#include <freenect/libfreenect.h>
#include <boost/shared_ptr.hpp>
#include <stdexcept>

namespace freenect_camera {

  static const std::string PRODUCT_NAME = "Xbox NUI Camera";
  static const std::string PRODUCT_ID = 0x2ae;
  static const std::string VENDOR_NAME = "Microsoft";
  static const std::string VENDOR_ID = 0x45e;
  static const std::string UNKNOWN = "unknown";

  class FreenectDevice {

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

      void registerImageCallback()
      void registerDepthCallback()
      void registerIRCallback()

    private:

      freenect_device* device_;
      freenect_frame_mode video_mode_;
      freenect_frame_mode depth_mode_;
      std_string device_serial_;

      static void freenectDepthCallback(freenect_device *dev, void *depth, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->depthCallback(depth, timestamp);
      }
      static void freenectVideoCallback(freenect_device *dev, void *video, uint32_t timestamp) {
        FreenectDevice* device = static_cast<FreenectDevice*>(freenect_get_user(dev));
        device->videoCallback(video, timestamp);
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

