// ロボットの走行経路上に waypoint を自動生成する node。
//
// TF frame_id -> robot_frame を周期ポーリングし、基準点からの移動距離
// または進行方位の変化がしきい値を超えたら現在位置に waypoint を打点する。
// ただし基準点から min_move 未満の移動では自動打点しない。
// 記録開始直後は waypoint を打たず、動き出してしきい値を超えてから
// 最初の点が置かれる。記録先は output_path の 1 ファイルで、
// 終了時（Ctrl-C）には停止位置に終端点を打ってから保存する。
//
// wait_for_initialpose が true（emcl2 で自己位置推定するとき）は
// /initialpose を受けるまで打点しない。/initialpose を受けるたびに
// 基準点をリセットし、推定位置の飛びを移動とみなして打点しないようにする。
//
// 記録中の waypoint は RViz の interactive marker でそのままドラッグ編集・
// 右クリックメニュー操作でき、生成結果は waypoint_follower 形式の YAML
// （waypoints: リスト）で保存できる。
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "waypoint_tools/interactive_waypoints.hpp"
#include "waypoint_tools/node_params.hpp"
#include "waypoint_tools/waypoint_edit_markers.hpp"
#include "waypoint_tools/waypoint_yaml.hpp"

namespace waypoint_tools
{

using namespace std::chrono_literals;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using std_srvs::srv::Trigger;

// TF をポーリングする周期。
constexpr auto kPollPeriod = 100ms;
// RViz の "2D Pose Estimate" が publish する topic。
constexpr char kInitialPoseTopic[] = "/initialpose";
// /initialpose 受信後、推定が落ち着くまで TF を使わない時間 [s]。
constexpr double kInitialPoseSettleSec = 1.0;

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double rad2deg(double rad)
{
  return rad * 180.0 / M_PI;
}

struct XY
{
  double x;
  double y;
};

struct RobotPose
{
  double x;
  double y;
  double yaw;
};

class WaypointRecorderNode : public rclcpp::Node
{
public:
  WaypointRecorderNode()
  : Node("waypoint_recorder_node")
  {
    require_parameters(
      *this, {
        {"output_path", rclcpp::PARAMETER_STRING},
        {"frame_id", rclcpp::PARAMETER_STRING},
        {"robot_frame", rclcpp::PARAMETER_STRING},
        {"distance_interval", rclcpp::PARAMETER_DOUBLE},
        {"yaw_interval_deg", rclcpp::PARAMETER_DOUBLE},
        {"min_move", rclcpp::PARAMETER_DOUBLE},
        {"wait_for_initialpose", rclcpp::PARAMETER_BOOL},
      });

    output_path_ = expand_user(get_parameter("output_path").as_string());
    map_frame_ = get_parameter("frame_id").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    distance_interval_ = get_parameter("distance_interval").as_double();
    yaw_interval_ = get_parameter("yaw_interval_deg").as_double() * M_PI / 180.0;
    min_move_ = get_parameter("min_move").as_double();
    waiting_initialpose_ = get_parameter("wait_for_initialpose").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    routes_.emplace(*this, map_frame_, 0.05);
    initialpose_sub_ = create_subscription<PoseWithCovarianceStamped>(
      kInitialPoseTopic, 10,
      [this](PoseWithCovarianceStamped::ConstSharedPtr) {initialposeCallback();});

    edit_markers_.emplace(
      *this, map_frame_, file_.waypoints,
      WaypointEditMarkers::Callbacks{
        [this]() {
          try {
            saveWaypoints();
          } catch (const std::exception & e) {
            RCLCPP_ERROR(get_logger(), "Save failed: %s", e.what());
          }
        },
        [this](size_t) {rebuildHeadings();},
        [this](size_t) {onDelete();},
        [this](size_t index) {onMove(index);},
        [this]() {publishRoutes();}},
      WaypointEditMarkers::Options{false, false});
    recording_handle_ = edit_markers_->addCheckEntry(
      "recording", recording_, [this]() {setRecording(!recording_);});

    add_service_ = createTrigger("~/add_waypoint", [this]() {return addWaypoint();});
    save_service_ = createTrigger(
      "~/save", [this]() -> Result {
        try {
          saveWaypoints();
        } catch (const std::exception & e) {
          return {false, e.what()};
        }
        return {true, "Saved " + std::to_string(waypoints().size()) + " waypoints: " +
          output_path_};
      });
    undo_service_ = createTrigger("~/undo", [this]() {return undo();});
    clear_service_ = createTrigger(
      "~/clear", [this]() -> Result {
        waypoints().clear();
        ref_xy_.reset();
        last_heading_.reset();
        rebuildMarkers();
        return {true, "Cleared all waypoints."};
      });
    pause_service_ = createTrigger(
      "~/pause", [this]() -> Result {
        setRecording(false);
        return {true, "Recording paused."};
      });
    resume_service_ = createTrigger(
      "~/resume", [this]() -> Result {
        setRecording(true);
        return {true, "Recording resumed."};
      });

    poll_timer_ = rclcpp::create_timer(this, get_clock(), kPollPeriod, [this]() {pollCallback();});
    route_timer_ = rclcpp::create_timer(this, get_clock(), 500ms, [this]() {publishRoutes();});

    RCLCPP_INFO(get_logger(), "Recording waypoints -> %s", output_path_.c_str());
    RCLCPP_INFO(
      get_logger(), "distance_interval=%g m, yaw_interval=%.1f deg, min_move=%g m",
      distance_interval_, rad2deg(yaw_interval_), min_move_);
    if (waiting_initialpose_) {
      RCLCPP_INFO(
        get_logger(), "Waiting for %s (RViz \"2D Pose Estimate\") before recording.",
        kInitialPoseTopic);
    }
  }

  // 停止位置で経路を終端させて保存する。rclcpp のコンテキストは終了済みなので
  // publish はしない。
  void onShutdown()
  {
    try {
      placeFinalWaypoint();
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Final waypoint not placed: %s", e.what());
    }
    if (!waypoints().empty()) {
      try {
        saveWaypoints();
      } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Save on shutdown failed: %s", e.what());
      }
    }
  }

private:
  using Result = std::pair<bool, std::string>;

  rclcpp::Service<Trigger>::SharedPtr createTrigger(
    const std::string & name, std::function<Result()> handler)
  {
    return create_service<Trigger>(
      name, [handler = std::move(handler)](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::tie(response->success, response->message) = handler();
      });
  }

  Waypoints & waypoints() {return file_.waypoints;}

  // ------------------------------------------------------------------
  // interactive marker
  // ------------------------------------------------------------------
  void rebuildMarkers()
  {
    edit_markers_->show();
    publishRoutes();
  }

  void resetRefToLast()
  {
    if (waypoints().empty()) {
      ref_xy_.reset();
    } else {
      const auto last = get_pose(waypoints().back());
      ref_xy_ = XY{last.x, last.y};
    }
  }

  void onMove(size_t index)
  {
    if (index + 1 == waypoints().size()) {
      resetRefToLast();
      rebuildHeadings();
    }
  }

  void onDelete()
  {
    resetRefToLast();
    rebuildHeadings();
  }

  void setRecording(bool value)
  {
    recording_ = value;
    edit_markers_->setChecked(recording_handle_, value);
    RCLCPP_INFO(get_logger(), "Recording %s.", value ? "resumed" : "paused");
  }

  // ------------------------------------------------------------------
  // 自己位置推定
  // ------------------------------------------------------------------
  void initialposeCallback()
  {
    // 推定位置が飛ぶので基準点を捨て、落ち着いてから測り直す。
    ref_xy_.reset();
    settle_until_ = now() + rclcpp::Duration::from_seconds(kInitialPoseSettleSec);
    if (waiting_initialpose_) {
      waiting_initialpose_ = false;
      RCLCPP_INFO(get_logger(), "Initial pose received. Recording starts.");
    } else {
      RCLCPP_INFO(get_logger(), "Initial pose updated. Reference reset.");
    }
  }

  bool poseReady()
  {
    if (waiting_initialpose_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "Waiting for initial pose; waypoints are not placed yet.");
      return false;
    }
    if (settle_until_) {
      if (now() < *settle_until_) {
        return false;
      }
      settle_until_.reset();
    }
    return true;
  }

  // ------------------------------------------------------------------
  // ポーリング
  // ------------------------------------------------------------------
  std::optional<RobotPose> lookupPose()
  {
    if (!poseReady()) {
      return std::nullopt;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "TF %s->%s unavailable: %s",
        map_frame_.c_str(), robot_frame_.c_str(), e.what());
      return std::nullopt;
    }
    const auto & t = tf.transform.translation;
    return RobotPose{t.x, t.y, quaternion_to_yaw(tf.transform.rotation)};
  }

  void pollCallback()
  {
    if (!recording_) {
      return;
    }
    const auto pose = lookupPose();
    if (!pose) {
      return;
    }

    if (!ref_xy_) {
      // 基準点を覚えるだけ。記録開始直後は waypoint を打たない。
      ref_xy_ = XY{pose->x, pose->y};
      return;
    }

    const double dx = pose->x - ref_xy_->x;
    const double dy = pose->y - ref_xy_->y;
    const double distance = std::hypot(dx, dy);
    if (distance < min_move_) {
      return;
    }

    const double heading = std::atan2(dy, dx);
    const bool heading_changed =
      last_heading_ && std::abs(normalize_angle(heading - *last_heading_)) >= yaw_interval_;

    if (distance >= distance_interval_ || heading_changed) {
      placeWaypoint(pose->x, pose->y, heading);
      last_heading_ = heading;
      RCLCPP_INFO(
        get_logger(), "Recorded waypoint %zu at (%.2f, %.2f), yaw=%.1f deg [total %zu]",
        waypoints().size() - 1, pose->x, pose->y, rad2deg(heading), waypoints().size());
    }
  }

  // ------------------------------------------------------------------
  // 打点処理
  // ------------------------------------------------------------------
  void placeWaypoint(double x, double y, double yaw, bool publish = true)
  {
    waypoints().push_back(make_waypoint({x, y, 0.0, yaw}));
    ref_xy_ = XY{x, y};
    if (publish) {
      edit_markers_->update(waypoints().size() - 1);
      publishRoutes();
    }
  }

  // 基準点から現在位置への進行方位。動いていなければ TF の yaw。
  double headingFromRef(const RobotPose & pose) const
  {
    if (ref_xy_ && std::hypot(pose.x - ref_xy_->x, pose.y - ref_xy_->y) >= 1e-3) {
      return std::atan2(pose.y - ref_xy_->y, pose.x - ref_xy_->x);
    }
    return pose.yaw;
  }

  // 現在位置に終端 waypoint を打つ（終了時用。publish しない）。
  // 既存の最終点と min_move 未満しか離れていなければ重複を避けて打点しない。
  void placeFinalWaypoint()
  {
    const auto pose = lookupPose();
    if (!pose) {
      return;
    }
    if (!waypoints().empty()) {
      const auto last = get_pose(waypoints().back());
      if (std::hypot(pose->x - last.x, pose->y - last.y) < min_move_) {
        return;
      }
    }
    placeWaypoint(pose->x, pose->y, headingFromRef(*pose), false);
  }

  Result addWaypoint()
  {
    const auto pose = lookupPose();
    if (!pose) {
      return {false, "Pose unavailable (TF or initial pose)."};
    }
    const double heading = headingFromRef(*pose);
    placeWaypoint(pose->x, pose->y, heading);
    last_heading_ = heading;
    char message[128];
    std::snprintf(
      message, sizeof(message), "Added waypoint %zu (%.2f, %.2f).", waypoints().size() - 1,
      pose->x, pose->y);
    return {true, message};
  }

  Result undo()
  {
    if (waypoints().empty()) {
      return {false, "No waypoints to undo."};
    }
    waypoints().pop_back();
    resetRefToLast();
    rebuildHeadings();
    rebuildMarkers();
    return {true, "Removed last waypoint. " + std::to_string(waypoints().size()) + " left."};
  }

  // 編集・undo 後に last_heading を直近セグメントから復元する。
  void rebuildHeadings()
  {
    const size_t n = waypoints().size();
    if (n < 2) {
      last_heading_.reset();
      return;
    }
    const auto p1 = get_pose(waypoints()[n - 2]);
    const auto p2 = get_pose(waypoints()[n - 1]);
    last_heading_ = std::atan2(p2.y - p1.y, p2.x - p1.x);
  }

  void saveWaypoints()
  {
    const auto directory = std::filesystem::path(output_path_).parent_path();
    if (!directory.empty()) {
      std::filesystem::create_directories(directory);
    }
    save_waypoint_file(output_path_, file_);
    RCLCPP_INFO(
      get_logger(), "Saved %zu waypoints: %s", waypoints().size(), output_path_.c_str());
  }

  // ------------------------------------------------------------------
  // 可視化
  // ------------------------------------------------------------------
  void publishRoutes()
  {
    routes_->publish(waypoints());
  }

  std::string output_path_;
  std::string map_frame_;
  std::string robot_frame_;
  double distance_interval_;
  double yaw_interval_;
  double min_move_;
  bool waiting_initialpose_;
  // /initialpose 直後は自己位置推定が飛ぶので、この時刻までは TF を使わない。
  std::optional<rclcpp::Time> settle_until_;

  WaypointFile file_;

  // しきい値判定の基準点（打点済みとは限らない）と直近セグメントの方位
  std::optional<XY> ref_xy_;
  std::optional<double> last_heading_;
  bool recording_ = true;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::optional<RouteMarkers> routes_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  std::optional<WaypointEditMarkers> edit_markers_;
  WaypointEditMarkers::EntryHandle recording_handle_ = 0;

  rclcpp::Service<Trigger>::SharedPtr add_service_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr undo_service_;
  rclcpp::Service<Trigger>::SharedPtr clear_service_;
  rclcpp::Service<Trigger>::SharedPtr pause_service_;
  rclcpp::Service<Trigger>::SharedPtr resume_service_;

  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr route_timer_;
};

}  // namespace waypoint_tools

int main(int argc, char ** argv)
{
  // Ctrl-C では端末と launch の両方から SIGINT が届く。rclcpp::shutdown() は init 前の
  // ハンドラに戻すので、ここで無視にしておくと保存後の 2 回目の SIGINT で落ちない。
  std::signal(SIGINT, SIG_IGN);
  rclcpp::init(argc, argv);
  std::shared_ptr<waypoint_tools::WaypointRecorderNode> node;
  int code = 0;
  try {
    node = std::make_shared<waypoint_tools::WaypointRecorderNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_recorder_node"), "%s", e.what());
    code = 1;
  }
  if (node) {
    node->onShutdown();
  }
  rclcpp::shutdown();
  return code;
}
