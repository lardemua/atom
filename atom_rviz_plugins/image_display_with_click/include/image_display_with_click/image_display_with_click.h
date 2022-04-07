#ifndef IMAGE_DISPLAY_WITH_CLICK_H
#define IMAGE_DISPLAY_WITH_CLICK_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include "../../../../../../../usr/include/x86_64-linux-gnu/qt5/QtCore/QObject"
#include "../../../../../../../usr/include/OGRE/OgreMaterial.h"
#include "../../../../../../../usr/include/OGRE/OgreRenderTargetListener.h"
#include "../../../../../../../usr/include/OGRE/OgreSharedPtr.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/image/image_display_base.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/image/ros_image_texture.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/render_panel.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/bool_property.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/float_property.h"
#include "../../../../../../../opt/ros/noetic/include/rviz/properties/int_property.h"
#endif

#include <iostream>
#include <QMouseEvent>

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"

#include <image_display_with_click/mouse_watcher.h>
#include <image_display_with_click/image_display_with_click_base.h>

namespace Ogre
{
    class SceneNode;
    class Rectangle2D;
} // namespace Ogre

using namespace std;
using namespace rviz;

namespace atom_rviz
{
    class ImageDisplayWithClick : public atom_rviz::ImageDisplayWithClickBase
    {
    Q_OBJECT
    public:
        ImageDisplayWithClick();
        ~ImageDisplayWithClick() override;

        // Overrides from Display
        void onInitialize() override;
        void update(float wall_dt, float ros_dt) override;
        void reset() override;

    public Q_SLOTS:
        virtual void updateNormalizeOptions();



    protected:
        // overrides from Display
        void onEnable() override;
        void onDisable() override;

        /* This is called by incomingMessage(). */
        void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;

        Ogre::SceneManager* img_scene_manager_;

        rviz::ROSImageTexture texture_;

        rviz::RenderPanel* render_panel_;


    private:
        Ogre::SceneNode* img_scene_node_;
        Ogre::Rectangle2D* screen_rect_;
        Ogre::MaterialPtr material_;

        BoolProperty* normalize_property_;
        rviz::FloatProperty* min_property_;
        rviz::FloatProperty* max_property_;
        rviz::IntProperty* median_buffer_size_property_;
        bool got_float_image_;

        MouseWatcher* mouse_watcher;
    };

} // namespace rviz


#endif
