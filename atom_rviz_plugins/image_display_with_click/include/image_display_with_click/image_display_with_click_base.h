#ifndef IMAGE_DISPLAY_WITH_CLICK_BASE_H
#define IMAGE_DISPLAY_WITH_CLICK_BASE_H

#include <QObject>

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/int_property.h"

#include "rviz/display.h"
#include "rviz/rviz_export.h"
#endif

using namespace rviz;

namespace atom_rviz
{
/** @brief Display subclass for subscribing and displaying to image messages.
 *
 * This class brings together some common things used for subscribing and displaying image messages in
 * Display
 * types.  It has a tf2_ros::MessageFilter and image_tranport::SubscriberFilter to filter incoming image
 * messages, and
 * it handles subscribing and unsubscribing when the display is
 * enabled or disabled.  */

class RVIZ_EXPORT ImageDisplayWithClickBase : public Display
{
  Q_OBJECT
public:
  /** @brief Constructor. */
  ImageDisplayWithClickBase();
  ~ImageDisplayWithClickBase() override;

  void setTopic(const QString& topic, const QString& datatype) override;

protected Q_SLOTS:
  /** @brief Update topic and resubscribe */
  virtual void updateTopic();

  /** @brief Update queue size of tf filter  */
  virtual void updateQueueSize();

  /** @brief Fill list of available and working transport options */
  void fillTransportOptionList(EnumProperty* property);

protected:
  void onInitialize() override;

  /** @brief Reset display. */
  void reset() override;

  /** @brief Enabling TF filtering by defining a target frame. */
  void enableTFFilter(std::string& targetFrame)
  {
    targetFrame_ = targetFrame;
    reset();
  }

  /** @brief ROS topic management. */
  virtual void subscribe();
  virtual void unsubscribe();

  void fixedFrameChanged() override;

  /** @brief Incoming message callback.  Checks if the message pointer
   * is valid, increments messages_received_, then calls
   * processMessage(). */
  void incomingMessage(const sensor_msgs::Image::ConstPtr& msg);

  /** @brief Callback for messages, whose frame_id cannot resolved */
  void failedMessage(const sensor_msgs::Image::ConstPtr& msg, tf2_ros::FilterFailureReason reason);

  /** @brief Implement this to process the contents of a message.
   *
   * This is called by incomingMessage(). */
  virtual void processMessage(const sensor_msgs::Image::ConstPtr& msg) = 0;

  void scanForTransportSubscriberPlugins();

  boost::scoped_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<image_transport::SubscriberFilter> sub_;
  boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Image> > tf_filter_;

  std::string targetFrame_;

  uint32_t messages_received_;

  RosTopicProperty* topic_property_;
  EnumProperty* transport_property_;
  IntProperty* queue_size_property_;

  std::string transport_;

  std::set<std::string> transport_plugin_types_;

  BoolProperty* unreliable_property_;
};

} // end namespace rviz

#endif // IMAGE_DISPLAY_BASE_H