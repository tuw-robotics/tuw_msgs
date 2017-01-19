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

#include <ros/ros.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "LineSegments2D/LineSegments2DVisual.h"

namespace tuw_geometry_rviz_plugin {

LineSegments2DVisual::LineSegments2DVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the MarkerDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // We create the visual objects within the frame node so that we can
    // set thier position and direction relative to their header frame.
    for(int i = 0; i < linesegments_.size(); i++)
    {
      linesegments_[i].reset(new rviz::Line(scene_manager_, frame_node_));
    }
}

LineSegments2DVisual::~LineSegments2DVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void LineSegments2DVisual::setMessage ( const tuw_geometry_msgs::LineSegments::ConstPtr& msg ) {
    linesegments_.resize(msg->segments.size());
    for(int i = 0; i < linesegments_.size(); i++)
    {
      Ogre::Vector3 start = Ogre::Vector3(msg->segments[i].p0.x, msg->segments[i].p0.y, msg->segments[i].p0.z);
      Ogre::Vector3 end = Ogre::Vector3(msg->segments[i].p1.x, msg->segments[i].p1.y, msg->segments[i].p1.z);
      
      linesegments_[i].reset(new rviz::Line(scene_manager_, frame_node_));
      linesegments_[i]->setPoints(start, end);
    }
}

// Position is passed through to the SceneNode.
void LineSegments2DVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void LineSegments2DVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Color is passed through to the pose Shape object.
void LineSegments2DVisual::setColorSegments ( Ogre::ColourValue color ) {
    for(int i = 0; i < linesegments_.size(); i++)
    {
      linesegments_[i]->setColor(color);
    }
    color_segment_ = color;
}

} // end namespace marker_rviz_plugin
