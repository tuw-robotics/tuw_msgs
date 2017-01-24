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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/frame_manager.h>

#include "LineSegments2D/LineSegments2DDisplay.h"
#include "LineSegments2D/LineSegments2DVisual.h"

namespace tuw_geometry_rviz_plugin
{
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
LineSegments2DDisplay::LineSegments2DDisplay()
{
  property_scale_segments_ =
      new rviz::FloatProperty("Scale Segments", 0.4, "Scale of the line segments.", this, SLOT(updateScaleSegments()));
  property_scale_segments_->setMin(0);
  property_scale_segments_->setMax(1);
  property_color_segments_ = new rviz::ColorProperty(
      "Color Segments", QColor(204, 51, 0), "Color to draw the linesegments.", this, SLOT(updateColorSegments()));
  property_width_segments_ =
      new rviz::FloatProperty("Width Segments", 0.05, "Width of the line segments.", this, SLOT(updateWidthSegments()));
  property_width_segments_->setMin(0);
  property_width_segments_->setMax(0.5);
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void LineSegments2DDisplay::onInitialize()
{
  MFDClass::onInitialize();
  visual_.reset(new LineSegments2DVisual(context_->getSceneManager(), scene_node_));
}

LineSegments2DDisplay::~LineSegments2DDisplay()
{
}

// Clear the visual by deleting its object.
void LineSegments2DDisplay::reset()
{
  MFDClass::reset();
}

// Set the current color for the visual's pose.
void LineSegments2DDisplay::updateColorSegments()
{
  Ogre::ColourValue color = property_color_segments_->getOgreColor();
  visual_->setColorSegments(color);
}

// Set the current scale for the visual's pose.
void LineSegments2DDisplay::updateScaleSegments()
{
  float scale = property_scale_segments_->getFloat();
  visual_->setScaleSegments(scale);
}

void LineSegments2DDisplay::updateWidthSegments()
{
  float width = property_width_segments_->getFloat();
  visual_->setWidthSegments(width);
}

// This is our callback to handle an incoming message.
void LineSegments2DDisplay::processMessage(const tuw_geometry_msgs::LineSegments::ConstPtr& msg)
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if (!context_->getFrameManager()->getTransform(msg->header.frame_id, msg->header.stamp, position, orientation))
  {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(),
              qPrintable(fixed_frame_));
    return;
  }

  // Now set or update the contents of the visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
  visual_->setColorSegments(property_color_segments_->getOgreColor());
  visual_->setScaleSegments(property_scale_segments_->getFloat());
  visual_->setWidthSegments(property_width_segments_->getFloat());
}

}  // end namespace tuw_geometry_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(tuw_geometry_rviz_plugin::LineSegments2DDisplay, rviz::Display)
