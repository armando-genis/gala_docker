#include <chrono>
#include <cmath>
#include <mutex>
#include <regex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

using namespace std::chrono_literals;

class ImageAcquisitionNode : public rclcpp::Node
{
public:
  ImageAcquisitionNode()
  : Node("image_acquisition")
  {
    // ─────────────── Declare parameters ───────────────
    declare_parameter("camera_ip",  "10.2.105.189");
    declare_parameter("username",   "admin");
    declare_parameter("password",   "cv+sg-34a1");
    declare_parameter("profile",    "profile4");
    declare_parameter<int>("fps",   30);     // 0 ⇒ auto-detect
    declare_parameter("debug_mode", false);
    declare_parameter("video_file", "");     // empty ⇒ use RTSP

    // ─────────────── Read parameters ───────────────
    camera_ip_  = get_parameter("camera_ip").as_string();
    username_   = get_parameter("username").as_string();
    password_   = get_parameter("password").as_string();
    profile_    = get_parameter("profile").as_string();
    fps_        = std::clamp<int>(get_parameter("fps").as_int(), 0, 120);
    debug_mode_ = get_parameter("debug_mode").as_bool();
    video_file_ = get_parameter("video_file").as_string();

    if (!std::regex_match(profile_, std::regex(R"(profile[1-9]\d*)")))
      throw std::invalid_argument(
          "profile must look like 'profile<N>' (N ≥ 1), got '" + profile_ + "'");

    if (video_file_.empty()) {
      rtsp_url_ = "rtsp://" + username_ + ":" + password_ + "@" + camera_ip_
                + ":554/"  + profile_ + "/media.smp";
    }
  }

  // Call *once*, after the node is wrapped in a shared_ptr
  void init()
  {
    // Force RTSP-over-TCP and disable FFmpeg buffering
    ::setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
             "rtsp_transport;tcp|fifo_size;0", 1);

    // image_transport publisher (raw + compressed)
    pub_ = image_transport::create_publisher(this, "/image_raw");

    // Worker that reads frames
    capturing_ = true;
    capture_thread_ = std::thread(&ImageAcquisitionNode::captureLoop, this);

    // Placeholder timer – replaced if FPS gets auto-detected
    const double period = (fps_ == 0) ? 0.001
                                      : 1.0 / static_cast<double>(fps_);
    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&ImageAcquisitionNode::publishLoop, this));

    RCLCPP_INFO(get_logger(), "Image acquisition node initialised");
  }

  ~ImageAcquisitionNode() override
  {
    capturing_ = false;
    if (capture_thread_.joinable())
      capture_thread_.join();
  }

private:
  // ─────────────── Capture thread ───────────────
  void captureLoop()
  {
    while (capturing_ && rclcpp::ok()) {
      cv::VideoCapture cap;

      /* ───── 1) open file or RTSP stream ───── */
      if (!video_file_.empty()) {
        cap.open(video_file_);
        if (!cap.isOpened()) {
          RCLCPP_ERROR(get_logger(),
                      "Could not open video file '%s'", video_file_.c_str());
          return;
        }
        RCLCPP_INFO(get_logger(), "Replaying file '%s'", video_file_.c_str());
      } else {
        const std::size_t max_attempts = 5;
        std::size_t attempt = 0;
        while (capturing_ && rclcpp::ok() && attempt < max_attempts) {
          cap.open(rtsp_url_, cv::CAP_FFMPEG);
          cap.set(cv::CAP_PROP_BUFFERSIZE, 5);
          if (cap.isOpened()) {
            RCLCPP_INFO(get_logger(), "Connected to RTSP stream %s",
                        maskPassword(rtsp_url_).c_str());
            break;
          }
          ++attempt;
          RCLCPP_ERROR(get_logger(),
                      "RTSP open failed (%zu/%zu). Retrying in 2 s…",
                      attempt, max_attempts);
          std::this_thread::sleep_for(2s);
        }
        if (!cap.isOpened()) {
          RCLCPP_ERROR(get_logger(),
                      "Failed to open RTSP after %zu attempts – exiting",
                      max_attempts);
          return;
        }
      }

      /* ───── 2) auto-detect FPS once (if requested) ───── */
      if (fps_ == 0) {
        double reported = cap.get(cv::CAP_PROP_FPS);
        if (reported >= 1.0 && reported <= 120.0) {
          fps_ = static_cast<int>(std::lround(reported));
          RCLCPP_INFO(get_logger(),
                      "Detected stream FPS: %.2f  → using %d fps",
                      reported, fps_);
          rebuildPublishTimer();
        } else {
          RCLCPP_WARN(get_logger(),
                      "Could not determine FPS (CAP_PROP_FPS = %.2f) – "
                      "publishing as fast as possible.",
                      reported);
        }
      }

      /* ───── 3) grab loop: wait until buffer is empty ───── */
      while (capturing_ && rclcpp::ok()) {
        {
          std::scoped_lock lock(frame_mutex_);
          if (!latest_frame_.empty())
            continue;                       // publisher hasn’t consumed it yet
        }

        cv::Mat frame;
        if (!cap.read(frame) || frame.empty()) {
          RCLCPP_WARN(get_logger(),
                      "Failed to read frame – attempting to reconnect");
          break;                            // leave inner loop ⇒ reopen stream
        }

        {
          std::scoped_lock lock(frame_mutex_);
          latest_frame_ = std::move(frame); // hand off to publisher
        }
      }

      cap.release();
      std::this_thread::sleep_for(500ms);   // brief pause before reconnect
    }
  }

  // ─────────────── Publisher timer callback ───────────────
  void publishLoop()
  {
    cv::Mat frame;
    {
      std::scoped_lock lock(frame_mutex_);
      if (latest_frame_.empty())
        return;
      frame = latest_frame_.clone();
      latest_frame_.release();
    }

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
                 .toImageMsg();
    msg->header.stamp = now();
    msg->header.frame_id = "camera_frame";
    pub_.publish(msg);
  }

  // ─────────────── Helpers ───────────────
  void rebuildPublishTimer()
  {
    const double period = 1.0 / static_cast<double>(fps_);
    publish_timer_->cancel();
    publish_timer_ = create_wall_timer(
        std::chrono::duration<double>(period),
        std::bind(&ImageAcquisitionNode::publishLoop, this));
  }

  static std::string maskPassword(const std::string& url)
  {
    return std::regex_replace(url, std::regex(R"(:(.*?)@)"), ":******@");
  }

  // ─────────────── Data members ───────────────
  // parameters
  std::string camera_ip_, username_, password_, profile_;
  int         fps_;                // 0 ⇒ auto
  bool        debug_mode_;
  std::string video_file_;
  std::string rtsp_url_;

  // publishers / timers / threads
  image_transport::Publisher   pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  std::thread                  capture_thread_;
  std::atomic<bool>            capturing_{false};

  // shared frame buffer
  cv::Mat    latest_frame_;
  std::mutex frame_mutex_;
};

// ───────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageAcquisitionNode>();
  node->init();                      // finish set-up after shared_ptr exists
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}