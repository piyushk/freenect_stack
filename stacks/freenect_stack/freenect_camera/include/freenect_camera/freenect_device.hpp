#include <freenect/libfreenect.hpp>
#include <boost/thread/mutex.hpp>

namespace freenect_camera {

  class FreenectDevice : public Freenect::FreenectDevice {

    public:
      MyFreenectDevice(freenect_context *_ctx, int _index)
        : Freenect::FreenectDevice(_ctx, _index), m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes),m_buffer_video(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), m_new_rgb_frame(false), m_new_depth_frame(false)
      {}
      //~MyFreenectDevice(){}
      // Do not call directly even in child
      void VideoCallback(void* _rgb, uint32_t timestamp) {
        boost::mutex::scoped_lock lock(m_rgb_mutex);
        uint8_t* rgb = static_cast<uint8_t*>(_rgb);
        std::copy(rgb, rgb+getVideoBufferSize(), m_buffer_video.begin());
        m_new_rgb_frame = true;
      };
      // Do not call directly even in child
      void DepthCallback(void* _depth, uint32_t timestamp) {
        boost::mutex::scoped_lock lock(m_depth_mutex);
        uint16_t* depth = static_cast<uint16_t*>(_depth);
        std::copy(depth, depth+getDepthBufferSize(), m_buffer_depth.begin());
        m_new_depth_frame = true;
      }
      bool getRGB(std::vector<uint8_t> &buffer) {
        boost::mutex::scoped_lock lock(m_rgb_mutex);
        if (!m_new_rgb_frame)
          return false;
        buffer.swap(m_buffer_video);
        m_new_rgb_frame = false;
        return true;
      }
      bool getDepth(std::vector<uint16_t> &buffer) {
        boost::mutex::scoped_lock lock(m_depth_mutex);
        if (!m_new_depth_frame)
          return false;
        buffer.swap(m_buffer_depth);
        m_new_depth_frame = false;
        return true;
      }

    private:
      std::vector<uint16_t> m_buffer_depth;
      std::vector<uint8_t> m_buffer_video;
      boost::Mutex m_rgb_mutex;
      boost::Mutex m_depth_mutex;
      bool m_new_rgb_frame;
      bool m_new_depth_frame;
}

