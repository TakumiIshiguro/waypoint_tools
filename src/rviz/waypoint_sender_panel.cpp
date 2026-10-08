#include "waypoint_tools/waypoint_sender_panel.hpp"

#include <QHBoxLayout>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace waypoint_tools
{

// waypoint_sender_node（send.launch.py）のサービス。
constexpr char kStartService[] = "/waypoint_sender_node/send_all";
constexpr char kNextWpService[] = "/waypoint_sender_node/next_wp";
constexpr char kEditService[] = "/waypoint_sender_node/edit";

WaypointSenderPanel::WaypointSenderPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  start_button_ = new QPushButton("Start");
  start_button_->setToolTip("最初の waypoint から走行開始（走行中ならやり直す）");
  next_wp_button_ = new QPushButton("Next WP");
  next_wp_button_->setToolTip("次の waypoint へ進む（未送信なら走行開始 / 編集で止めたら再開）");
  edit_button_ = new QPushButton("Edit");
  edit_button_->setCheckable(true);
  edit_button_->setToolTip("編集モード（走行中なら goal を取り消して止まる）");
  status_label_ = new QLabel("waypoint_sender_node: -");
  status_label_->setWordWrap(true);

  auto * buttons = new QHBoxLayout;
  buttons->addWidget(start_button_);
  buttons->addWidget(next_wp_button_);
  buttons->addWidget(edit_button_);
  auto * layout = new QVBoxLayout;
  layout->addLayout(buttons);
  layout->addWidget(status_label_);
  setLayout(layout);
}

void WaypointSenderPanel::onInitialize()
{
  // RViz 自身の node を使う。RViz の更新ループで spin されるので、
  // 応答のコールバックは GUI スレッドで呼ばれる。
  auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
  start_client_ = node->create_client<Trigger>(kStartService);
  next_wp_client_ = node->create_client<Trigger>(kNextWpService);
  edit_client_ = node->create_client<SetBool>(kEditService);

  connect(start_button_, &QPushButton::clicked, this, [this]() {call(start_client_, "Start");});
  connect(
    next_wp_button_, &QPushButton::clicked, this,
    [this]() {call(next_wp_client_, "Next WP");});
  connect(edit_button_, &QPushButton::clicked, this, &WaypointSenderPanel::setEditing);
}

void WaypointSenderPanel::call(
  const rclcpp::Client<Trigger>::SharedPtr & client, const QString & label)
{
  if (!client->service_is_ready()) {
    setStatus(label + ": " + client->get_service_name() + " が見つかりません");
    return;
  }
  setStatus(label + ": 送信中...");
  // 応答を待って GUI を止めないよう非同期で呼ぶ。
  client->async_send_request(
    std::make_shared<Trigger::Request>(),
    [this, label](rclcpp::Client<Trigger>::SharedFuture future) {
      const auto response = future.get();
      setStatus(
        label + (response->success ? ": OK " : ": NG ") +
        QString::fromStdString(response->message));
    });
}

void WaypointSenderPanel::setEditing(bool enabled)
{
  if (!edit_client_->service_is_ready()) {
    edit_button_->setChecked(!enabled);
    setStatus(QString("Edit: ") + edit_client_->get_service_name() + " が見つかりません");
    return;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = enabled;
  edit_client_->async_send_request(
    request, [this, enabled](rclcpp::Client<SetBool>::SharedFuture future) {
      const auto response = future.get();
      if (!response->success) {
        edit_button_->setChecked(!enabled);
      }
      setStatus("Edit: " + QString::fromStdString(response->message));
    });
}

void WaypointSenderPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

}  // namespace waypoint_tools

PLUGINLIB_EXPORT_CLASS(waypoint_tools::WaypointSenderPanel, rviz_common::Panel)
