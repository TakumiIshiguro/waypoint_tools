#include "waypoint_tools/waypoint_sender_panel.hpp"

#include <QHBoxLayout>
#include <QLabel>
#include <QVBoxLayout>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>

namespace waypoint_tools
{

// waypoint_sender_node（send.launch.py）のサービス。
constexpr char kStartService[] = "/waypoint_sender_node/send_all";
constexpr char kNextWpService[] = "/waypoint_sender_node/next_wp";
constexpr char kEditService[] = "/waypoint_sender_node/edit";
// 番号指定の開始。整数を渡せる標準のサービス型が無いので topic で送る。
constexpr char kStartFromTopic[] = "/waypoint_sender_node/start_from";
// .rviz に保存する開始番号のキー。
constexpr char kStartIndexKey[] = "StartIndex";

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
  start_index_spin_ = new QSpinBox;
  start_index_spin_->setRange(0, 9999);
  start_index_spin_->setToolTip("開始する waypoint 番号（範囲外なら node が無視してログに出す）");
  start_from_button_ = new QPushButton("Start from");
  start_from_button_->setToolTip("指定した番号の waypoint から走行開始（走行中ならやり直す）");
  status_label_ = new QLabel("waypoint_sender_node: -");
  status_label_->setWordWrap(true);

  auto * buttons = new QHBoxLayout;
  buttons->addWidget(start_button_);
  buttons->addWidget(next_wp_button_);
  buttons->addWidget(edit_button_);
  auto * start_from = new QHBoxLayout;
  start_from->addWidget(new QLabel("WP"));
  start_from->addWidget(start_index_spin_, 1);
  start_from->addWidget(start_from_button_);
  auto * layout = new QVBoxLayout;
  layout->addLayout(buttons);
  layout->addLayout(start_from);
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
  start_from_pub_ = node->create_publisher<Int32>(kStartFromTopic, 10);

  connect(start_button_, &QPushButton::clicked, this, [this]() {call(start_client_, "Start");});
  connect(
    next_wp_button_, &QPushButton::clicked, this,
    [this]() {call(next_wp_client_, "Next WP");});
  connect(edit_button_, &QPushButton::clicked, this, &WaypointSenderPanel::setEditing);
  connect(start_from_button_, &QPushButton::clicked, this, &WaypointSenderPanel::startFrom);
  // 開始番号を .rviz に保存できるよう、変更を RViz に知らせる。
  connect(
    start_index_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
    &WaypointSenderPanel::configChanged);
}

void WaypointSenderPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue(kStartIndexKey, start_index_spin_->value());
}

void WaypointSenderPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  int index;
  if (config.mapGetInt(kStartIndexKey, &index)) {
    start_index_spin_->setValue(index);
  }
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

void WaypointSenderPanel::startFrom()
{
  if (start_from_pub_->get_subscription_count() == 0) {
    setStatus(QString("Start from: ") + kStartFromTopic + " の購読者がいません");
    return;
  }
  Int32 msg;
  msg.data = start_index_spin_->value();
  start_from_pub_->publish(msg);
  // topic なので応答は無い。結果は node のログと RViz の緑の円盤で確認する。
  setStatus(QString("Start from: WP %1 を送信").arg(msg.data));
}

void WaypointSenderPanel::setStatus(const QString & text)
{
  status_label_->setText(text);
}

}  // namespace waypoint_tools

PLUGINLIB_EXPORT_CLASS(waypoint_tools::WaypointSenderPanel, rviz_common::Panel)
