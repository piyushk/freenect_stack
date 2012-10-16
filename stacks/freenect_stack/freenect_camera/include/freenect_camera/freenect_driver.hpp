#ifndef FREENECT_DRIVER_K8EEAIBB
#define FREENECT_DRIVER_K8EEAIBB

#include <freenect/libfreenect.h>
#include <freenect_camera/freenect_device.hpp>

namespace freenect_camera {

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
        // freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
        freenect_select_subdevices(driver_, (freenect_device_flags)(FREENECT_DEVICE_CAMERA));
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

#endif /* end of include guard: FREENECT_DRIVER_K8EEAIBB */
