#ifndef WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_
#define WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>
#include <QSpinBox>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/config.hpp>
#include <rviz_common/panel.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace waypoint_tools
{

// waypoint_sender_node のサービスを呼ぶボタンと、番号指定で開始する欄を並べたパネル。
// 3D 表示のクリック判定を使わないので、重い RViz でも確実に押せる。
class WaypointSenderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointSenderPanel(QWidget * parent = nullptr);
  void onInitialize() override;
  void save(rviz_common::Config config) const override;
  void load(const rviz_common::Config & config) override;

private:
  using Int32 = std_msgs::msg::Int32;
  using SetBool = std_srvs::srv::SetBool;
  using Trigger = std_srvs::srv::Trigger;

  void call(const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label);
  void setEditing(bool enabled);
  void startFrom();
  void setStatus(const QString & text);

  QPushButton * start_button_;
  QPushButton * next_wp_button_;
  QPushButton * edit_button_;
  QSpinBox * start_index_spin_;
  QPushButton * start_from_button_;
  QLabel * status_label_;

  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr next_wp_client_;
  rclcpp::Client<SetBool>::SharedPtr edit_client_;
  rclcpp::Publisher<Int32>::SharedPtr start_from_pub_;
};

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_
