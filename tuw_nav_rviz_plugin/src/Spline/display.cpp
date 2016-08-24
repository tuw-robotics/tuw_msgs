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
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <Spline/visual.h>
#include <Spline/display.h>

namespace tuw_nav_rviz_plugin {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
SplineDisplay::SplineDisplay() {
    color_path_property_ = new rviz::ColorProperty ( "Path Color", QColor ( 50, 51, 204 ),
            "Color to draw the spline.",
            this, SLOT ( updatePathColor() ) );

    shape_property_ = new rviz::EnumProperty ( "Path Shape", QString::fromStdString ( "Sphere" ),
            "Shape of the spline.",
            this, SLOT ( updateShape() ) );
    shape_property_->addOptionStd ( "Sphere"  , rviz::Shape::Sphere );
    shape_property_->addOptionStd ( "Cube"    , rviz::Shape::Cube );
    shape_property_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );
    
    points_nr_path_property_ = new rviz::IntProperty ( "Path Points Nr", 100,
            "Number of points to display along path.",
            this, SLOT ( updatePathPointsNr() ) );
    points_nr_path_property_->setMin ( 1 );
    points_nr_path_property_->setMax ( 100000 );

    scale_path_property_ = new rviz::FloatProperty ( "Path Points Scale", 0.1,
            "Scale of the spline contour.",
            this, SLOT ( updatePathScale() ) );
    scale_path_property_->setMin ( 0 );
    scale_path_property_->setMax ( 1 );
    
    color_orient_property_ = new rviz::ColorProperty ( "Orientation Color", QColor ( 50, 51, 204 ),
            "Color to draw the spline orientation arrows.",
            this, SLOT ( updateOrientColor() ) );
    points_nr_orient_property_ = new rviz::IntProperty ( "Orientation Points Nr", 50,
            "Number of arrows to display along path.",
            this, SLOT ( updateOrientPointsNr() ) );
    points_nr_orient_property_->setMin ( 1 );
    points_nr_orient_property_->setMax ( 100000 );

    scale_orient_property_ = new rviz::FloatProperty ( "Orientation Points Scale", 0.1,
            "Scale of the spline orientation arrows.",
            this, SLOT ( updateOrientScale() ) );
    scale_orient_property_->setMin ( 0 );
    scale_orient_property_->setMax ( 1 );
    
    history_length_property_ = new rviz::IntProperty ( "History Length", 1,
            "Number of prior measurements to display.",
            this, SLOT ( updateHistoryLength() ) );
    history_length_property_->setMin ( 1 );
    history_length_property_->setMax ( 100000 );
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
void SplineDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

SplineDisplay::~SplineDisplay() {
}

// Clear the visuals by deleting their objects.
void SplineDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

// Set the current color values for each visual.
void SplineDisplay::updatePathColor() {
    Ogre::ColourValue color = color_path_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathColor ( color ); }
}
void SplineDisplay::updateOrientColor() {
    Ogre::ColourValue color = color_orient_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientColor ( color ); }
}

// Set the current shape for each visual.
void SplineDisplay::updateShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_property_->getOptionInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setShape ( shape_type ); }
}

// Set the current scale for each visual.
void SplineDisplay::updatePathScale() {
    float scale = scale_path_property_->getFloat();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathScale ( scale ); }
}
void SplineDisplay::updateOrientScale() {
    float scale = scale_orient_property_->getFloat();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientScale ( scale ); }
}

// Set the number of past visuals to show.
void SplineDisplay::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}

void SplineDisplay::updatePathPointsNr() {
    int pointsNr = points_nr_path_property_->getInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathPointsNr ( pointsNr ); }
}
void SplineDisplay::updateOrientPointsNr() {
    int pointsNr = points_nr_orient_property_->getInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientPointsNr ( pointsNr ); }
}


// This is our callback to handle an incoming message.
void SplineDisplay::processMessage ( const tuw_nav_msgs::Spline::ConstPtr& msg ) {
    // Here we call the rviz::FrameManager to get the transform from the
    // fixed frame to the frame in the header of this Imu message.  If
    // it fails, we can't do anything else so we return.
    Ogre::Quaternion orientation;
    Ogre::Vector3 position;
    if ( !context_->getFrameManager()->getTransform ( msg->header.frame_id,
            msg->header.stamp,
            position, orientation ) ) {
        ROS_DEBUG ( "Error transforming from frame '%s' to frame '%s'",
                    msg->header.frame_id.c_str(), qPrintable ( fixed_frame_ ) );
        return;
    }

    // We are keeping a circular buffer of visual pointers.  This gets
    // the next one, or creates and stores it if the buffer is not full
    boost::shared_ptr<SplineVisual> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new SplineVisual ( context_->getSceneManager(), scene_node_ ) );
    }

    // Now set or update the contents of the chosen visual.
    visual->setMessage          ( msg );
    visual->setFramePosition    ( position );
    visual->setFrameOrientation ( orientation );
    
    visual->setPathColor     ( color_path_property_->getOgreColor() );
    visual->setShape         ( ( rviz::Shape::Type ) shape_property_->getOptionInt() );
    visual->setPathScale     ( scale_path_property_->getFloat() );
    visual->setPathPointsNr  ( points_nr_path_property_->getInt() );
    
    visual->setOrientColor   ( color_orient_property_->getOgreColor() );
    visual->setOrientScale   ( scale_orient_property_->getFloat() );
    visual->setOrientPointsNr( points_nr_orient_property_->getInt() );

    // And send it to the end of the circular buffer
    visuals_.push_back ( visual );
}

} // end namespace tuw_nav_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( tuw_nav_rviz_plugin::SplineDisplay,rviz::Display )
