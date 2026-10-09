// waypoint YAML を読み込み、1 点ずつ Nav2 の NavigateToPose に送って走行させる node。
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "waypoint_tools/interactive_waypoints.hpp"
#include "waypoint_tools/node_params.hpp"
#include "waypoint_tools/waypoint_edit_markers.hpp"
#include "waypoint_tools/waypoint_yaml.hpp"

namespace waypoint_tools
{

using namespace std::chrono_literals;
using nav2_msgs::action::NavigateToPose;
using std_msgs::msg::Int32;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

// Nav2 の NavigateToPose action。経由点の管理はこの node が行う。
constexpr char kActionName[] = "/navigate_to_pose";

// 送信する waypoint の表示 topic。Nav2 の RViz 設定
// （orne_box_navigation_executor/config/rviz/nav2_TC2024_view2.rviz）が表示する。
constexpr char kMarkerTopic[] = "/waypoints";

// 円盤の色。停止点は赤、現在の目標点は緑。
const auto kSenderDiscColor = rgba(0.1f, 0.7f, 1.0f, 0.6f);
const auto kSenderStopDiscColor = rgba(1.0f, 0.1f, 0.1f, 0.7f);
const auto kTargetDiscColor = rgba(0.1f, 0.9f, 0.2f, 0.8f);

enum class State
{
  kIdle,      // 未送信（send_all / next_wp で走行開始）
  kRunning,   // 走行中
  kStopped,   // stop: true の点に到達し、next_wp 待ち
  kFinished,  // 最後の点に到達
  kFailed,    // 再送しても到達できず停止
  kPaused,    // 編集モードに入って goal を取り消した。next_wp で同じ点から再開
};

// サービスの応答（success, message）。
using Result = std::pair<bool, std::string>;

class WaypointSenderNode : public rclcpp::Node
{
public:
  using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  WaypointSenderNode()
  : Node("waypoint_sender_node")
  {
    require_parameters(
      *this, {
        {"yaml_path", rclcpp::PARAMETER_STRING},
        {"frame_id", rclcpp::PARAMETER_STRING},
        {"robot_frame", rclcpp::PARAMETER_STRING},
        {"switch_radius", rclcpp::PARAMETER_DOUBLE},
        {"max_retries", rclcpp::PARAMETER_INTEGER},
        {"skip_on_failure", rclcpp::PARAMETER_BOOL},
      });

    yaml_path_ = expand_user(get_parameter("yaml_path").as_string());
    if (!std::filesystem::is_regular_file(yaml_path_)) {
      throw std::runtime_error("yaml_path must be a file: " + yaml_path_);
    }
    frame_id_ = get_parameter("frame_id").as_string();
    robot_frame_ = get_parameter("robot_frame").as_string();
    switch_radius_ = get_parameter("switch_radius").as_double();
    max_retries_ = get_parameter("max_retries").as_int();
    skip_on_failure_ = get_parameter("skip_on_failure").as_bool();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    action_client_ = rclcpp_action::create_client<NavigateToPose>(this, kActionName);

    send_all_service_ = create_service<Trigger>(
      "~/send_all", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::tie(response->success, response->message) = start();
      });
    next_wp_service_ = create_service<Trigger>(
      "~/next_wp", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        std::tie(response->success, response->message) = nextWp();
      });
    edit_service_ = create_service<SetBool>(
      "~/edit", [this](
        const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        std::tie(response->success, response->message) = setEditing(request->data);
      });
    // 番号指定の開始は応答を返せないので、結果はログに出す。
    start_from_sub_ = create_subscription<Int32>(
      "~/start_from", 10, [this](const Int32::SharedPtr msg) {
        const auto [success, message] = startFrom(msg->data);
        if (success) {
          RCLCPP_INFO(get_logger(), "%s", message.c_str());
        } else {
          RCLCPP_WARN(get_logger(), "%s", message.c_str());
        }
      });

    marker_pub_ = create_publisher<MarkerArray>(kMarkerTopic, 10);

    // 編集モード中だけ円盤を interactive marker にして編集できるようにする。
    edit_markers_.emplace(
      *this, frame_id_, file_.waypoints,
      WaypointEditMarkers::Callbacks{
        [this]() {saveWaypoints();},
        [this](size_t index) {onInsert(index);},
        [this](size_t index) {onDelete(index);},
        nullptr,
        [this]() {publishMarkers();}});

    // 起動時は YAML を読んで RViz に表示するだけ。走行は ~/send_all
    // （または ~/next_wp）で開始する。
    load();
    RCLCPP_INFO(
      get_logger(), "Waiting for ~/send_all (or ~/next_wp, ~/start_from) to start navigation.");

    control_timer_ = rclcpp::create_timer(
      this, get_clock(), 100ms, [this]() {controlCallback();});
    // RViz の後起動でも表示されるよう定期的に publish する。
    marker_timer_ = rclcpp::create_timer(
      this, get_clock(), 1s, [this]() {publishMarkers();});
  }

private:
  Waypoints & waypoints() {return file_.waypoints;}

  // 到達（Nav2 の SUCCEEDED）まで待つ点か。stop 指定の点と最後の点。
  bool isHoldPoint(size_t index)
  {
    return is_stop(waypoints()[index]) || index + 1 == waypoints().size();
  }

  // -----------------------------------------------------------
  // 読み込みと送信開始
  // -----------------------------------------------------------
  // YAML を読み込んで RViz に表示する（走行はしない）。
  void load()
  {
    auto file = load_waypoint_file(yaml_path_);
    if (file.waypoints.empty()) {
      throw std::runtime_error("No waypoints in " + yaml_path_ + ".");
    }
    file_ = std::move(file);
    wp_index_ = 0;

    std::string stops;
    for (size_t i = 0; i < waypoints().size(); ++i) {
      if (is_stop(waypoints()[i])) {
        stops += (stops.empty() ? "" : ", ") + std::to_string(i);
      }
    }
    RCLCPP_INFO(
      get_logger(), "Loaded %zu waypoints from %s (stop: [%s]).", waypoints().size(),
      yaml_path_.c_str(), stops.c_str());
    publishMarkers();
  }

  // 指定した waypoint から走行を始める（編集後の waypoint を使う）。
  void sendFrom(size_t index)
  {
    if (!action_client_->wait_for_action_server(10s)) {
      throw std::runtime_error(std::string(kActionName) + " is not available.");
    }
    goTo(index);
  }

  // -----------------------------------------------------------
  // 走行制御
  // -----------------------------------------------------------
  void goTo(size_t index)
  {
    wp_index_ = index;
    retries_ = 0;
    sendCurrent();
    publishMarkers();
  }

  void sendCurrent()
  {
    const auto generation = ++goal_generation_;
    const auto pose = get_pose(waypoints()[wp_index_]);

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = frame_id_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = pose.x;
    goal.pose.pose.position.y = pose.y;
    goal.pose.pose.position.z = pose.z;
    goal.pose.pose.orientation = yaw_to_quaternion(pose.yaw);

    state_ = State::kRunning;
    goal_handle_.reset();
    RCLCPP_INFO(
      get_logger(), "Waypoint %zu/%zu%s", wp_index_, waypoints().size() - 1,
      is_stop(waypoints()[wp_index_]) ? " (stop)" : "");

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, generation](GoalHandle::SharedPtr handle) {
        goalResponseCallback(handle, generation);
      };
    options.result_callback = [this, generation](const GoalHandle::WrappedResult & result) {
        resultCallback(result, generation);
      };
    action_client_->async_send_goal(goal, options);
  }

  void advance()
  {
    if (wp_index_ + 1 >= waypoints().size()) {
      finish();
      return;
    }
    goTo(wp_index_ + 1);
  }

  void finish()
  {
    state_ = State::kFinished;
    RCLCPP_INFO(get_logger(), "Reached the last waypoint.");
  }

  void cancelGoal(const GoalHandle::SharedPtr & handle)
  {
    try {
      action_client_->async_cancel_goal(handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError &) {
      // 既に終わった goal。
    }
  }

  void cancelCurrent()
  {
    // 世代を進めて、キャンセルした goal の結果を無視させる。
    ++goal_generation_;
    if (goal_handle_) {
      cancelGoal(goal_handle_);
      goal_handle_.reset();
    }
  }

  void goalResponseCallback(const GoalHandle::SharedPtr & handle, uint64_t generation)
  {
    if (generation != goal_generation_) {
      // 応答待ちの間に取り消された goal。受理されていたら止める。
      if (handle) {
        cancelGoal(handle);
      }
      return;
    }
    if (!handle) {
      RCLCPP_ERROR(get_logger(), "Goal for waypoint %zu rejected.", wp_index_);
      onFailure();
      return;
    }
    goal_handle_ = handle;
  }

  void resultCallback(const GoalHandle::WrappedResult & result, uint64_t generation)
  {
    if (generation != goal_generation_) {
      return;
    }
    goal_handle_.reset();
    if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_WARN(
        get_logger(), "Waypoint %zu failed (status %d).", wp_index_,
        static_cast<int>(result.code));
      onFailure();
      return;
    }

    if (wp_index_ + 1 == waypoints().size()) {
      finish();
    } else if (is_stop(waypoints()[wp_index_])) {
      state_ = State::kStopped;
      RCLCPP_INFO(
        get_logger(), "Stopped at waypoint %zu. Call ~/next_wp to continue.", wp_index_);
    } else {
      // switch_radius に入る前に Nav2 が到達判定した場合。
      advance();
    }
  }

  void onFailure()
  {
    if (retries_ < max_retries_) {
      ++retries_;
      RCLCPP_WARN(
        get_logger(), "Retry waypoint %zu (%ld/%ld).", wp_index_, retries_, max_retries_);
      sendCurrent();
    } else if (skip_on_failure_) {
      RCLCPP_WARN(get_logger(), "Skip waypoint %zu.", wp_index_);
      advance();
    } else {
      state_ = State::kFailed;
      RCLCPP_ERROR(
        get_logger(), "Gave up waypoint %zu. Call ~/next_wp to go to the next waypoint.",
        wp_index_);
    }
  }

  // 通過点では switch_radius に入った時点で次の点を送る（停止しない）。
  void controlCallback()
  {
    if (state_ != State::kRunning || isHoldPoint(wp_index_)) {
      return;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(frame_id_, robot_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const auto pose = get_pose(waypoints()[wp_index_]);
    const double dist = std::hypot(
      pose.x - tf.transform.translation.x, pose.y - tf.transform.translation.y);
    if (dist <= switch_radius_) {
      advance();
    }
  }

  // -----------------------------------------------------------
  // RViz 表示
  // -----------------------------------------------------------
  Marker newMarker(const std::string & ns, size_t index, int32_t type)
  {
    Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = static_cast<int32_t>(index);
    marker.type = type;
    marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  // 緑で表示する現在の目標点。走行していなければ無し。
  std::optional<size_t> targetIndex() const
  {
    switch (state_) {
      case State::kRunning:
      case State::kStopped:
      case State::kFailed:
      case State::kPaused:
        return wp_index_;
      default:
        return std::nullopt;
    }
  }

  // waypoint を円盤 + 向きの矢印 + 番号で、経路を線で表示する。
  void publishMarkers()
  {
    MarkerArray marker_array;
    // 編集モード中は円盤・矢印・番号を interactive marker が出す。
    const size_t shown = editing_ ? 0 : waypoints().size();
    const auto target = targetIndex();
    for (size_t index = 0; index < shown; ++index) {
      const auto pose = get_pose(waypoints()[index]);
      const bool stop = is_stop(waypoints()[index]);

      auto disc = newMarker("waypoints", index, Marker::CYLINDER);
      disc.pose.position.x = pose.x;
      disc.pose.position.y = pose.y;
      disc.scale.x = disc.scale.y = 1.0;
      disc.scale.z = 0.04;
      if (target && index == *target) {
        disc.color = kTargetDiscColor;
      } else {
        disc.color = stop ? kSenderStopDiscColor : kSenderDiscColor;
      }
      marker_array.markers.push_back(disc);

      auto arrow = newMarker("waypoint_arrows", index, Marker::ARROW);
      arrow.pose.position.x = pose.x;
      arrow.pose.position.y = pose.y;
      arrow.pose.orientation = yaw_to_quaternion(pose.yaw);
      arrow.scale.x = 0.6;
      arrow.scale.y = arrow.scale.z = 0.08;
      arrow.color = rgba(1.0f, 0.2f, 0.1f, 1.0f);
      marker_array.markers.push_back(arrow);

      auto text = newMarker("waypoint_labels", index, Marker::TEXT_VIEW_FACING);
      text.pose.position.x = pose.x;
      text.pose.position.y = pose.y;
      text.pose.position.z = 0.6;
      text.scale.z = 0.5;
      text.color = rgba(0.0f, 0.0f, 0.0f, 1.0f);
      text.text = std::to_string(index) + (stop ? " (stop)" : "");
      marker_array.markers.push_back(text);
    }

    auto route = newMarker("waypoint_route", 0, Marker::LINE_STRIP);
    route.scale.x = 0.04;
    route.color = rgba(1.0f, 0.9f, 0.0f, 1.0f);
    for (const auto & waypoint : waypoints()) {
      const auto pose = get_pose(waypoint);
      geometry_msgs::msg::Point point;
      point.x = pose.x;
      point.y = pose.y;
      route.points.push_back(point);
    }
    if (route.points.size() >= 2) {
      marker_array.markers.push_back(route);
    }

    for (size_t index = shown; index < published_count_; ++index) {
      for (const char * ns : {"waypoints", "waypoint_arrows", "waypoint_labels"}) {
        auto stale = newMarker(ns, index, Marker::CYLINDER);
        stale.action = Marker::DELETE;
        marker_array.markers.push_back(stale);
      }
    }
    published_count_ = shown;

    marker_pub_->publish(marker_array);
  }

  // -----------------------------------------------------------
  // 編集モード
  // -----------------------------------------------------------
  Result setEditing(bool enabled)
  {
    const std::string on_off = enabled ? "on" : "off";
    if (enabled == editing_) {
      return {true, "Edit mode is already " + on_off + "."};
    }
    editing_ = enabled;
    if (enabled) {
      if (state_ == State::kRunning) {
        cancelCurrent();
        state_ = State::kPaused;
        RCLCPP_INFO(
          get_logger(), "Canceled the goal to waypoint %zu for editing. Call ~/next_wp to resume.",
          wp_index_);
      }
      edit_markers_->show();
    } else {
      edit_markers_->hide();
    }
    publishMarkers();
    return {true, "Edit mode " + on_off + "."};
  }

  void onInsert(size_t index)
  {
    // 目標点より前に挿入したら、目標点の番号をずらす。
    if (state_ != State::kIdle && index <= wp_index_) {
      ++wp_index_;
    }
  }

  void onDelete(size_t index)
  {
    if (state_ != State::kIdle && index < wp_index_) {
      --wp_index_;
    }
    wp_index_ = std::min(wp_index_, waypoints().size() - 1);
  }

  void saveWaypoints()
  {
    save_waypoint_file(yaml_path_, file_);
    RCLCPP_INFO(get_logger(), "Saved waypoints: %s", yaml_path_.c_str());
  }

  // -----------------------------------------------------------
  // サービス
  // -----------------------------------------------------------
  Result trySendFrom(size_t index)
  {
    try {
      sendFrom(index);
    } catch (const std::exception & e) {
      return {false, e.what()};
    }
    return {true, "Sent " + yaml_path_ + " from waypoint " + std::to_string(index) + "."};
  }

  // 最初の waypoint から走行を開始する（走行中ならやり直す）。
  Result start()
  {
    cancelCurrent();
    return trySendFrom(0);
  }

  // 指定した waypoint から走行を開始する（どの状態からでも。走行中ならやり直す）。
  Result startFrom(int64_t index)
  {
    if (index < 0 || static_cast<size_t>(index) >= waypoints().size()) {
      return {
        false, "Waypoint " + std::to_string(index) + " is out of range (0.." +
        std::to_string(waypoints().size() - 1) + ")."};
    }
    cancelCurrent();
    return trySendFrom(static_cast<size_t>(index));
  }

  // 次の waypoint へ進む。
  // 未送信 -> 走行開始 / 走行中・停止点・失敗 -> 現在の点をやめて次の点へ /
  // 編集で一時停止 -> 向かっていた点から再開。
  Result nextWp()
  {
    switch (state_) {
      case State::kIdle:
        return trySendFrom(0);
      case State::kPaused:
        goTo(wp_index_);
        return {true, "Resumed to waypoint " + std::to_string(wp_index_) + "."};
      case State::kFinished:
        return {false, "Already reached the last waypoint."};
      default:
        break;
    }
    cancelCurrent();
    advance();
    if (state_ == State::kFinished) {
      return {true, "Reached the last waypoint."};
    }
    return {true, "Go to waypoint " + std::to_string(wp_index_) + "."};
  }

  std::string yaml_path_;
  std::string frame_id_;
  std::string robot_frame_;
  double switch_radius_;
  int64_t max_retries_;
  bool skip_on_failure_;

  WaypointFile file_;
  size_t wp_index_ = 0;
  State state_ = State::kIdle;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  // goal を送るたびに増やす。古い（先行 goal に置き換えられた）goal の
  // 結果は世代が合わないので無視する。
  uint64_t goal_generation_ = 0;
  GoalHandle::SharedPtr goal_handle_;
  int64_t retries_ = 0;

  rclcpp::Service<Trigger>::SharedPtr send_all_service_;
  rclcpp::Service<Trigger>::SharedPtr next_wp_service_;
  rclcpp::Service<SetBool>::SharedPtr edit_service_;
  rclcpp::Subscription<Int32>::SharedPtr start_from_sub_;

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  // 直近で publish した waypoint 数。減ったときに古い marker を消す。
  size_t published_count_ = 0;

  bool editing_ = false;
  std::optional<WaypointEditMarkers> edit_markers_;

  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;
};

}  // namespace waypoint_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<waypoint_tools::WaypointSenderNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_sender_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
