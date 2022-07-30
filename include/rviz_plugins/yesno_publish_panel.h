#pragma once

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif

#include <rviz_common/panel.hpp>
#include <std_msgs/msg/bool.hpp>
#include <QVBoxLayout>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#  include <QtWidgets>
#else
#  include <QtGui>
#endif

namespace Ui {
class YesNoUI;
}

namespace rviz_plugins
{
class YesNoPublishPanel: public rviz_common::Panel
{
  Q_OBJECT
 public:
  YesNoPublishPanel(QWidget* parent = nullptr);
  ~YesNoPublishPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  /* void dialValueChanged(int value); */
  void lineEditChanged();
  void respondYes();
  void respondNo();

protected:
  Ui::YesNoUI* ui_;
  int value_{0};
  std::string topic_name_{"yes_no"};
  QHBoxLayout* layout_;
  QPushButton* yes_button_;
  QPushButton* no_button_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};
} // end namespace rviz_plugins;
