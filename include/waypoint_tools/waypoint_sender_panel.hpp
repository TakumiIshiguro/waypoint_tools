#ifndef WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_
#define WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_

#include <QLabel>
#include <QPushButton>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace waypoint_tools
{

// waypoint_sender_node の Trigger サービスを呼ぶボタンを並べたパネル。
// 3D 表示のクリック判定を使わないので、重い RViz でも確実に押せる。
class WaypointSenderPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit WaypointSenderPanel(QWidget * parent = nullptr);
  void onInitialize() override;

private:
  using SetBool = std_srvs::srv::SetBool;
  using Trigger = std_srvs::srv::Trigger;

  void call(const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label);
  void setEditing(bool enabled);
  void setStatus(const QString & text);

  QPushButton * start_button_;
  QPushButton * next_wp_button_;
  QPushButton * edit_button_;
  QLabel * status_label_;

  rclcpp::Client<Trigger>::SharedPtr start_client_;
  rclcpp::Client<Trigger>::SharedPtr next_wp_client_;
  rclcpp::Client<SetBool>::SharedPtr edit_client_;
};

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__WAYPOINT_SENDER_PANEL_HPP_
