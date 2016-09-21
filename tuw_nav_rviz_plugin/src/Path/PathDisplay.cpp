/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by George Todoran <george.todoran@tuwien.ac.at>        *
 *                                                                         *
 *   Redistribution and use in source and binary forms, with or without    *
 *   modification, are permitted provided that the following conditions    *
 *   are met:                                                              *
 *                                                                         *
 *   1. Redistributions of source code must retain the above copyright     *
 *      notice, this list of conditions and the following disclaimer.      *
 *   2. Redistributions in binary form must reproduce the above copyright  *
 *      notice, this list of conditions and the following disclaimer in    *
 *      the documentation and/or other materials provided with the         *
 *      distribution.                                                      *
 *   3. Neither the name of the copyright holder nor the names of its      *
 *      contributors may be used to endorse or promote products derived    *
 *      from this software without specific prior written permission.      *
 *                                                                         *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS   *
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT     *
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS     *
 *   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE        *
 *   COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,  *
 *   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,  *
 *   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;      *
 *   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER      *
 *   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT    *
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY *
 *   WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE           *
 *   POSSIBILITY OF SUCH DAMAGE.                                           *
 ***************************************************************************/

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include <Path/PathVisual.h>
#include <Path/PathDisplay.h>

namespace tuw_nav_rviz_plugin {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
PathDisplay::PathDisplay() {
    color_path_property_ = new rviz::ColorProperty ( "Path Color", QColor ( 50, 51, 204 ),
            "Color to draw the path.",
            this, SLOT ( updatePathColor() ) );

    shape_property_ = new rviz::EnumProperty ( "Path Shape", QString::fromStdString ( "Sphere" ),
            "Shape of the path.",
            this, SLOT ( updateShape() ) );
    shape_property_->addOptionStd ( "Sphere"  , rviz::Shape::Sphere );
    shape_property_->addOptionStd ( "Cube"    , rviz::Shape::Cube );
    shape_property_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );

    scale_path_property_ = new rviz::FloatProperty ( "Path Points Scale", 0.1,
            "Scale of the path contour.",
            this, SLOT ( updatePathScale() ) );
    scale_path_property_->setMin ( 0 );
    scale_path_property_->setMax ( 1 );
    
    color_orient_property_ = new rviz::ColorProperty ( "Orientation Color", QColor ( 50, 51, 204 ),
            "Color to draw the path orientation arrows.",
            this, SLOT ( updateOrientColor() ) );
    path_delta_s_orient_property_ = new rviz::FloatProperty ( "Distance between arrows", 0.5,
            "Distance between pose arrows [m]",
            this, SLOT ( updateOrientPointsNr() ) );
    path_delta_s_orient_property_->setMin ( 0 );
    path_delta_s_orient_property_->setMax ( 100 );

    scale_orient_property_ = new rviz::FloatProperty ( "Orientation Points Scale", 0.1,
            "Scale of the path orientation arrows.",
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
void PathDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

PathDisplay::~PathDisplay() {
}

// Clear the visuals by deleting their objects.
void PathDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

// Set the current color values for each visual.
void PathDisplay::updatePathColor() {
    Ogre::ColourValue color = color_path_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathColor ( color ); }
}
void PathDisplay::updateOrientColor() {
    Ogre::ColourValue color = color_orient_property_->getOgreColor();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientColor ( color ); }
}

// Set the current shape for each visual.
void PathDisplay::updateShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_property_->getOptionInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setShape ( shape_type ); }
}

// Set the current scale for each visual.
void PathDisplay::updatePathScale() {
    float scale = scale_path_property_->getFloat();
    for ( auto& visualsI: visuals_ ) { visualsI->setPathScale ( scale ); }
}
void PathDisplay::updateOrientScale() {
    float scale = scale_orient_property_->getFloat();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientScale ( scale ); }
}

// Set the number of past visuals to show.
void PathDisplay::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}

void PathDisplay::updateOrientDeltaS() {
    double deltaS = path_delta_s_orient_property_->getFloat();
    for ( auto& visualsI: visuals_ ) { visualsI->setOrientDeltaS ( deltaS ); }
}


// This is our callback to handle an incoming message.
void PathDisplay::processMessage ( const nav_msgs::Path::ConstPtr& msg ) {
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
    boost::shared_ptr<PathVisual> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new PathVisual ( context_->getSceneManager(), scene_node_ ) );
    }

    // Now set or update the contents of the chosen visual.
    visual->setMessage          ( msg );
    visual->setFramePosition    ( position );
    visual->setFrameOrientation ( orientation );
    
    visual->setPathColor     ( color_path_property_->getOgreColor() );
    visual->setShape         ( ( rviz::Shape::Type ) shape_property_->getOptionInt() );
    visual->setPathScale     ( scale_path_property_->getFloat() );
    
    visual->setOrientColor   ( color_orient_property_->getOgreColor() );
    visual->setOrientScale   ( scale_orient_property_->getFloat() );
    visual->setOrientDeltaS  ( path_delta_s_orient_property_->getFloat() );

    // And send it to the end of the circular buffer
    visuals_.push_back ( visual );
}

} // end namespace tuw_nav_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( tuw_nav_rviz_plugin::PathDisplay,rviz::Display )
