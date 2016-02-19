/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <rviz/bit_allocator.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/load_resource.h>
#include <rviz/ogre_helpers/axes.h>
#include <rviz/properties/display_group_visibility_property.h>
#include <rviz/properties/enum_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/render_panel.h>
#include <rviz/uniform_string_stream.h>
#include <rviz/validate_floats.h>
#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <boost/bind.hpp>
#include <image_transport/camera_common.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <tf/transform_listener.h>

#include "rviz_camera_stream/camera_display.h"

namespace video_export
{
class VideoPublisher
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher pub_;
  uint image_id_;
public:
  VideoPublisher() :
    it_(nh_),
    image_id_(0)
  {
  }
  void shutdown()
  {
    if (pub_.getTopic() != "")
    {
      pub_.shutdown();
    }
  }

  void advertise(std::string topic)
  {
    pub_ = it_.advertise(topic, 1);
  }

  bool publishFrame(Ogre::RenderWindow * render_window, const std::string frame_id)
  {
    if (pub_.getTopic() == "")
    {
      return false;
    }
    if (frame_id == "")
    {
      return false;
    }
    // RenderTarget::writeContentsToFile() used as example
    int height = render_window->getHeight();
    int width = render_window->getWidth();
    Ogre::PixelFormat pf = render_window->suggestPixelFormat();
    uint pixelsize = Ogre::PixelUtil::getNumElemBytes(pf);
    uint datasize = width * height * pixelsize;

    // 1.05 multiplier is to avoid crash when the window is resized.
    // There should be a better solution.
    uchar *data = OGRE_ALLOC_T(uchar, datasize * 1.05, Ogre::MEMCATEGORY_RENDERSYS);
    Ogre::PixelBox pb(width, height, 1, pf, data);
    render_window->copyContentsToMemory(pb);

    sensor_msgs::Image image;
    image.header.stamp = ros::Time::now();
    image.header.seq = image_id_++;
    image.header.frame_id = frame_id;
    image.height = height;
    image.width = width;
    image.step = pixelsize * width;
    image.encoding = sensor_msgs::image_encodings::RGB8;  // would break if pf changes
    image.is_bigendian = (OGRE_ENDIAN == OGRE_ENDIAN_BIG);
    image.data.resize(datasize);
    memcpy(&image.data[0], data, datasize);
    pub_.publish(image);

    OGRE_FREE(data, Ogre::MEMCATEGORY_RENDERSYS);
  }
};
}  // namespace video_export


namespace rviz
{

const QString CameraPub::BACKGROUND("background");
const QString CameraPub::OVERLAY("overlay");
const QString CameraPub::BOTH("background and overlay");

bool validateFloats(const sensor_msgs::CameraInfo& msg)
{
  bool valid = true;
  valid = valid && validateFloats(msg.D);
  valid = valid && validateFloats(msg.K);
  valid = valid && validateFloats(msg.R);
  valid = valid && validateFloats(msg.P);
  return valid;
}

CameraPub::CameraPub()
  : Display()
  , render_panel_(0)
  // , caminfo_tf_filter_(0)
  , new_caminfo_(false)
  , force_render_(false)
  , caminfo_ok_(false)
  , video_publisher_(0)
{
  topic_property_ = new RosTopicProperty("Image Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::Image>()),
      "sensor_msgs::Image topic to publish to.", this, SLOT(updateTopic()));

  camera_info_property_ = new RosTopicProperty("Camera Info Topic", "",
      QString::fromStdString(ros::message_traits::datatype<sensor_msgs::CameraInfo>()),
      "sensor_msgs::CameraInfo topic to subscribe to.", this, SLOT(updateTopic()));

  queue_size_property_ = new IntProperty( "Queue Size", 2,
      "Advanced: set the size of the incoming message queue.  Increasing this "
      "is useful if your incoming TF data is delayed significantly from your"
      " image data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT(updateQueueSize()));
  queue_size_property_->setMin(1);
}

CameraPub::~CameraPub()
{
  if (initialized())
  {
    render_panel_->getRenderWindow()->removeListener(this);

    unsubscribe();
    // caminfo_tf_filter_->clear();

    // TODO(lucasw) is this why the panel doesn't go away entirely, just looks minimized?
    // workaround. delete results in a later crash
    render_panel_->hide();
    // delete render_panel_;

    // delete caminfo_tf_filter_;

    context_->visibilityBits()->freeBits(vis_bit_);
  }
}

void CameraPub::onInitialize()
{
  Display::onInitialize();

  video_publisher_ = new video_export::VideoPublisher();

  // caminfo_tf_filter_ = new tf::MessageFilter<sensor_msgs::CameraInfo>(
  //    *context_->getTFClient(), fixed_frame_.toStdString(),
  //    queue_size_property_->getInt(), update_nh_);

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->addListener(this);
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_->getSceneManager(), context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  // caminfo_tf_filter_->connectInput(caminfo_sub_);
  // caminfo_tf_filter_->registerCallback(boost::bind(&CameraPub::caminfoCallback, this, _1));
  // context_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

  vis_bit_ = context_->visibilityBits()->allocBit();
  render_panel_->getViewport()->setVisibilityMask(vis_bit_);

  visibility_property_ = new DisplayGroupVisibilityProperty(
    vis_bit_, context_->getRootDisplayGroup(), this, "Visibility", true,
    "Changes the visibility of other Displays in the camera view.");

  visibility_property_->setIcon(loadPixmap("package://rviz/icons/visibility.svg", true));

  this->addChild(visibility_property_, 0);
}

void CameraPub::updateTopic()
{
  unsubscribe();
  reset();
  subscribe();
  context_->queueRender();
}

void CameraPub::preRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // set view flags on all displays
  visibility_property_->update();
}

void CameraPub::postRenderTargetUpdate(const Ogre::RenderTargetEvent& evt)
{
  // TODO(lucasw) allow this to be throttled down
  // Publish the rendered window video stream
  std::string frame_id;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);
    if (!current_caminfo_)
      return;
    frame_id = current_caminfo_->header.frame_id;
  }
  video_publisher_->publishFrame(render_panel_->getRenderWindow(), frame_id);
}

void CameraPub::onEnable()
{
  subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void CameraPub::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  unsubscribe();
  clear();
}

void CameraPub::subscribe()
{
  if (!isEnabled())
    return;

  if (topic_property_->getTopicStd().empty())
  {
    setStatus(StatusProperty::Error, "Output Topic", "No topic set");
    return;
  }
  if (camera_info_property_->getTopicStd().empty())
  {
    setStatus(StatusProperty::Error, "Camera Info", "No topic set");
    return;
  }

  // std::string target_frame = fixed_frame_.toStdString();
  // Display::enableTFFilter(target_frame);

  std::string topic = topic_property_->getTopicStd();
  std::string caminfo_topic = camera_info_property_->getTopicStd();

  try
  {
    // caminfo_sub_.subscribe(update_nh_, caminfo_topic, 1);
    caminfo_sub_ = update_nh_.subscribe(caminfo_topic, 1, &CameraPub::caminfoCallback, this);
    setStatus(StatusProperty::Ok, "Camera Info", "OK");
  }
  catch (ros::Exception& e)
  {
    setStatus(StatusProperty::Error, "Camera Info", QString("Error subscribing: ") + e.what());
  }

  video_publisher_->advertise(topic);
  setStatus(StatusProperty::Ok, "Output Topic", "Topic set");
}

void CameraPub::unsubscribe()
{
  video_publisher_->shutdown();
  caminfo_sub_.shutdown();
}

void CameraPub::forceRender()
{
  force_render_ = true;
  context_->queueRender();
}

void CameraPub::updateQueueSize()
{
  // caminfo_tf_filter_->setQueueSize((uint32_t) queue_size_property_->getInt());
}

void CameraPub::clear()
{
  force_render_ = true;
  context_->queueRender();

  new_caminfo_ = false;
  current_caminfo_.reset();

  setStatus(StatusProperty::Warn, "Camera Info",
            "No CameraInfo received on [" +
            QString::fromStdString(caminfo_sub_.getTopic()) +
            "].  Topic may not exist.");
  setStatus(StatusProperty::Warn, "Camera Info", "No CameraInfo received");

  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

void CameraPub::update(float wall_dt, float ros_dt)
{
#if 0
  try
  {
#endif
    if (force_render_)
    {
      caminfo_ok_ = updateCamera();
      force_render_ = false;
    }
#if 0
  }
  catch (UnsupportedImageEncoding& e)
  {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
#endif

  render_panel_->getRenderWindow()->update();
}

bool CameraPub::updateCamera()
{
  sensor_msgs::CameraInfo::ConstPtr info;
  {
    boost::mutex::scoped_lock lock(caminfo_mutex_);
    info = current_caminfo_;
  }

  if (!info)
  {
    return false;
  }

  if (!validateFloats(*info))
  {
    setStatus(StatusProperty::Error, "Camera Info", "Contains invalid floating point values (nans or infs)");
    return false;
  }

  // if we're in 'exact' time mode, only show image if the time is exactly right
  ros::Time rviz_time = context_->getFrameManager()->getTime();
  if (context_->getFrameManager()->getSyncMode() == FrameManager::SyncExact &&
      rviz_time != info->header.stamp)
  {
    std::ostringstream s;
    s << "Time-syncing active and no info at timestamp " << rviz_time.toSec() << ".";
    setStatus(StatusProperty::Warn, "Time", s.str().c_str());
    return false;
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  context_->getFrameManager()->getTransform(info->header.frame_id, info->header.stamp, position, orientation);

  // printf( "CameraPub:updateCamera(): pos = %.2f, %.2f, %.2f.\n", position.x, position.y, position.z );

  // convert vision (Z-forward) frame to ogre frame (Z-out)
  orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

  float img_width = info->width;
  float img_height = info->height;

  // If the image width is 0 due to a malformed caminfo, try to grab the width from the image.
  if (img_width == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], width = 0", qPrintable(getName()));
    img_width = 640;
  }

  if (img_height == 0)
  {
    ROS_DEBUG("Malformed CameraInfo on camera [%s], height = 0", qPrintable(getName()));
    img_height = 480;
  }

  if (img_height == 0.0 || img_width == 0.0)
  {
    setStatus(StatusProperty::Error, "Camera Info",
              "Could not determine width/height of image due to malformed CameraInfo (either width or height is 0)");
    return false;
  }

  double fx = info->P[0];
  double fy = info->P[5];

  float win_width = render_panel_->width();
  float win_height = render_panel_->height();
  float zoom_x = 1.0;
  float zoom_y = zoom_x;

  // Preserve aspect ratio
  if (win_width != 0 && win_height != 0)
  {
    float img_aspect = (img_width / fx) / (img_height / fy);
    float win_aspect = win_width / win_height;

    if (img_aspect > win_aspect)
    {
      zoom_y = zoom_y / img_aspect * win_aspect;
    }
    else
    {
      zoom_x = zoom_x / win_aspect * img_aspect;
    }
  }

  // Add the camera's translation relative to the left camera (from P[3]);
  double tx = -1 * (info->P[3] / fx);
  Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
  position = position + (right * tx);

  double ty = -1 * (info->P[7] / fy);
  Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
  position = position + (down * ty);

  if (!validateFloats(position))
  {
    setStatus(StatusProperty::Error, "Camera Info",
        "CameraInfo/P resulted in an invalid position calculation (nans or infs)");
    return false;
  }

  render_panel_->getCamera()->setPosition(position);
  render_panel_->getCamera()->setOrientation(orientation);

  // calculate the projection matrix
  double cx = info->P[2];
  double cy = info->P[6];

  double far_plane = 100;
  double near_plane = 0.01;

  Ogre::Matrix4 proj_matrix;
  proj_matrix = Ogre::Matrix4::ZERO;

  proj_matrix[0][0] = 2.0 * fx / img_width * zoom_x;
  proj_matrix[1][1] = 2.0 * fy / img_height * zoom_y;

  proj_matrix[0][2] = 2.0 * (0.5 - cx / img_width) * zoom_x;
  proj_matrix[1][2] = 2.0 * (cy / img_height - 0.5) * zoom_y;

  proj_matrix[2][2] = -(far_plane + near_plane) / (far_plane - near_plane);
  proj_matrix[2][3] = -2.0 * far_plane * near_plane / (far_plane - near_plane);

  proj_matrix[3][2] = -1;

  render_panel_->getCamera()->setCustomProjectionMatrix(true, proj_matrix);

  setStatus(StatusProperty::Ok, "Camera Info", "OK");

#if 0
  static Axes* debug_axes = new Axes(scene_manager_, 0, 0.2, 0.01);
  debug_axes->setPosition(position);
  debug_axes->setOrientation(orientation);
#endif

  setStatus(StatusProperty::Ok, "Time", "ok");

  return true;
}

void CameraPub::caminfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  boost::mutex::scoped_lock lock(caminfo_mutex_);
  current_caminfo_ = msg;
  new_caminfo_ = true;
}

void CameraPub::fixedFrameChanged()
{
  std::string targetFrame = fixed_frame_.toStdString();
  // caminfo_tf_filter_->setTargetFrame(targetFrame);
  Display::fixedFrameChanged();
}

void CameraPub::reset()
{
  Display::reset();
  clear();
}

}  // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::CameraPub, rviz::Display)
