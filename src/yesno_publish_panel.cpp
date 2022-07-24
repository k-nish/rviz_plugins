#include <rviz_plugins/yesno_publish_panel.h>
#include <class_loader/class_loader.hpp>

#include "ui_yesno_panel.h"

namespace rviz_plugins
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("rviz_plugin.topic_publish_panel");


YesNoPublishPanel::YesNoPublishPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::YesNoUI())
{
  node_ = std::make_shared<rclcpp::Node>("yesno_publisher_node");
  ui_->setupUi(this);
}

YesNoPublishPanel::~YesNoPublishPanel() = default;

void YesNoPublishPanel::onInitialize()
{
    yes_button_ = ui_->yes;
    yes_button_->setDefault(false);

    no_button_ = ui_->no;
    no_button_->setDefault(false);

    ui_->line_edit->setPlaceholderText("Input topic name (Default : yes_no)");

    connect(yes_button_, SIGNAL(clicked()), this, SLOT(respondYes()));
    connect(no_button_, SIGNAL(clicked()), this, SLOT(respondNo()));
    connect(ui_->line_edit,  SIGNAL(textChanged(const QString &)), this, SLOT(lineEditChanged()));

    pub_ = node_->create_publisher<std_msgs::msg::Bool>(topic_name_, 1);
}

void YesNoPublishPanel::onEnable()
{
  show();
  parentWidget()->show();
}

void YesNoPublishPanel::onDisable()
{
  hide();
  parentWidget()->hide();
}

void YesNoPublishPanel::lineEditChanged()
{
  std::string old_topic_name = topic_name_;
  if(ui_->line_edit->text().isEmpty())
    topic_name_ = "yes_no";
  else
    topic_name_ = ui_->line_edit->text().toStdString();

  RCLCPP_INFO(LOGGER, "You set the topic name : %s", topic_name_.c_str());

  if(old_topic_name != topic_name_)
    pub_ = node_->create_publisher<std_msgs::msg::Bool>(topic_name_, 1);
}

void YesNoPublishPanel::respondYes()
{
    RCLCPP_INFO(LOGGER, "You pushed the Yes button.");
    yes_button_->setDown(true);
    no_button_->setDown(false);
    std_msgs::msg::Bool msg;
    msg.data = static_cast<bool>(1);
    pub_->publish(msg);
}

void YesNoPublishPanel::respondNo()
{
    yes_button_->setDown(false);
    no_button_->setDown(true);
    std_msgs::msg::Bool msg;
    msg.data = static_cast<bool>(0);
    pub_->publish(msg);
    RCLCPP_INFO(LOGGER, "You pushed the No button.");
}

}  // namespace rviz_plugins

CLASS_LOADER_REGISTER_CLASS(rviz_plugins::YesNoPublishPanel, rviz_common::Panel)
