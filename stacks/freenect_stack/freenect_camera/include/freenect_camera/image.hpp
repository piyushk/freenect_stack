#ifndef IMAGE_VWSJG3T7
#define IMAGE_VWSJG3T7

#include <freenect/libfreenect.h>

namespace freenect_camera {

  class Image {
    public:
      static const freenect_video_format BAYER_GRBG = FREENECT_VIDEO_BAYER;
      static const freenect_video_format YUV422 = FREENECT_VIDEO_YUV_RGB;
      freenect_frame_mode metadata;
      void* data;

      freenect_video_format getEncoding() const {
        return metadata.video_format;
      }

      void fillRaw(unsigned char* image_data) const {
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
}


#endif /* end of include guard: IMAGE_VWSJG3T7 */
