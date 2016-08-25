/***************************************************************************
 *   Software License Agreement (BSD License)                              *
 *   Copyright (C) 2016 by Markus Bader <markus.bader@tuwien.ac.at>        *
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

#include <RouteSegments/RouteSegmentsVisual.h>
#include <RouteSegments/RouteSegmentsDisplay.h>

namespace tuw_nav_rviz_plugin {

// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
RouteSegmentsDisplay::RouteSegmentsDisplay() {
  
    
    color_lines_ = new rviz::ColorProperty ( "Line Segment Color", QColor ( 170, 0, 170 ),
            "Color Line Segment",  this, SLOT ( updateColorLines() ) );    
    show_lines_property_ = new rviz::BoolProperty ( "Show lines", true,
            "Shows line segments", this, SLOT ( updateShowLines() ) );
    
    color_arcs_ = new rviz::ColorProperty ( "Arc Segment Color", QColor ( 0, 170, 170 ),
            "Color Arc Segment",  this, SLOT ( updateColorArcs() ) );
    show_arcs_property_ = new rviz::BoolProperty ( "Show arcs", true,
            "Shows arc segments", this, SLOT ( updateShowArcs() ) );
    
    show_start_point_ = new rviz::BoolProperty ( "Show Start Point", true,
            "Shows segments start", this, SLOT ( updateShowStartPoints() ) );    
    color_start_point_ = new rviz::ColorProperty ( "Start Point Color", QColor ( 170, 0, 0 ),
            "Color to start point", this, SLOT ( updateStartPointColor() ) );
    shape_start_point_ = new rviz::EnumProperty ( "Start Point Shape", QString::fromStdString ( "Sphere" ),
            "Shape of start point",  this, SLOT ( updateStartPointShape() ) );
    shape_start_point_->addOptionStd ( "Sphere"  , rviz::Shape::Sphere );
    shape_start_point_->addOptionStd ( "Cube"    , rviz::Shape::Cube );
    shape_start_point_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );
    shape_start_point_->addOptionStd ( "Cone", rviz::Shape::Cone );
    scale_start_point_ = new rviz::FloatProperty ( "Start Points Scale", 0.05,
            "Scale start point",  this, SLOT ( updateStartPointScale() ) );
    scale_start_point_->setMin ( 0 );
    scale_start_point_->setMax ( 1 );
    
    
    show_end_point_ = new rviz::BoolProperty ( "Show End Point", false,
            "Shows segments end", this, SLOT ( updateShowEndPoints() ) );
    color_end_point_ = new rviz::ColorProperty ( "End Point Color", QColor ( 0, 170, 0 ),
            "Color to end point",  this, SLOT ( updateEndPointColor() ) );
    shape_end_point_ = new rviz::EnumProperty ( "End Point Shape", QString::fromStdString ( "Cube" ),
            "Shape of end point",  this, SLOT ( updateEndPointShape() ) );
    shape_end_point_->addOptionStd ( "Sphere"  , rviz::Shape::Sphere );
    shape_end_point_->addOptionStd ( "Cube"    , rviz::Shape::Cube );
    shape_end_point_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );
    shape_end_point_->addOptionStd ( "Cone", rviz::Shape::Cone );
    scale_end_point_ = new rviz::FloatProperty ( "End Points Scale", 0.04,
            "Scale end point",  this, SLOT ( updateEndPointScale() ) );
    scale_end_point_->setMin ( 0 );
    scale_end_point_->setMax ( 1 );
    
    
    show_center_point_ = new rviz::BoolProperty ( "Show Center Point", false,
            "Shows segments center", this, SLOT ( updateShowCenterPoints() ) );
    color_center_point_ = new rviz::ColorProperty ( "Center Point Color", QColor ( 0, 0, 170 ),
            "Color to center point",  this, SLOT ( updateCenterPointColor() ) );
    shape_center_point_ = new rviz::EnumProperty ( "Center Point Shape", QString::fromStdString ( "Sphere" ),
            "Shape of center point",  this, SLOT ( updateCenterPointShape() ) );
    shape_center_point_->addOptionStd ( "Sphere"  , rviz::Shape::Sphere );
    shape_center_point_->addOptionStd ( "Cube"    , rviz::Shape::Cube );
    shape_center_point_->addOptionStd ( "Cylinder", rviz::Shape::Cylinder );
    shape_center_point_->addOptionStd ( "Cone", rviz::Shape::Cone );
    scale_center_point_ = new rviz::FloatProperty ( "Center Points Scale", 0.05,
            "Scale center point",  this, SLOT ( updateCenterPointScale() ) );
    scale_center_point_->setMin ( 0 );
    scale_center_point_->setMax ( 1 );
    
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
void RouteSegmentsDisplay::onInitialize() {
    MFDClass::onInitialize();
    updateHistoryLength();
}

RouteSegmentsDisplay::~RouteSegmentsDisplay() {
}

// Clear the visuals by deleting their objects.
void RouteSegmentsDisplay::reset() {
    MFDClass::reset();
    visuals_.clear();
}

// Set the current color values for each visual.
void RouteSegmentsDisplay::updateStartPointColor() {
    for ( auto& visualsI: visuals_ ) { visualsI->setStartPointColor ( color_start_point_->getOgreColor() ); }
}

// Set the current shape for each visual.
void RouteSegmentsDisplay::updateStartPointShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_start_point_->getOptionInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setStartPointShape ( shape_type ); }
}

// Set the current scale for each visual.
void RouteSegmentsDisplay::updateStartPointScale() {
    for ( auto& visualsI: visuals_ ) { visualsI->setStartPointScale ( scale_start_point_->getFloat() ); }
}


// Set the current color values for each visual.
void RouteSegmentsDisplay::updateEndPointColor() {
    for ( auto& visualsI: visuals_ ) { visualsI->setEndPointColor ( color_end_point_->getOgreColor() ); }
}

// Set the current shape for each visual.
void RouteSegmentsDisplay::updateEndPointShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_end_point_->getOptionInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setEndPointShape ( shape_type ); }
}

// Set the current scale for each visual.
void RouteSegmentsDisplay::updateEndPointScale() {
    for ( auto& visualsI: visuals_ ) { visualsI->setEndPointScale ( scale_end_point_->getFloat() ); }
}

// Set the current color values for each visual.
void RouteSegmentsDisplay::updateCenterPointColor() {
    for ( auto& visualsI: visuals_ ) { visualsI->setEndPointColor ( color_center_point_->getOgreColor() ); }
}

// Set the current shape for each visual.
void RouteSegmentsDisplay::updateCenterPointShape() {
    rviz::Shape::Type shape_type = ( rviz::Shape::Type ) shape_center_point_->getOptionInt();
    for ( auto& visualsI: visuals_ ) { visualsI->setCenterPointShape ( shape_type ); }
}

// Set the current scale for each visual.
void RouteSegmentsDisplay::updateCenterPointScale() {
    for ( auto& visualsI: visuals_ ) { visualsI->setCenterPointScale ( scale_center_point_->getFloat() ); }
}


// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateHistoryLength() {
    visuals_.rset_capacity ( history_length_property_->getInt() );
}


// Set the current color values for each visual.
void RouteSegmentsDisplay::updateColorLines() {
    for ( auto& visualsI: visuals_ ) { visualsI->setLineColor ( color_lines_->getOgreColor() ); }
}

// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateShowLines() {
    for ( auto& visualsI: visuals_ ) { visualsI->setShowLines ( show_lines_property_->getBool() ); }
}

// Set the current color values for each visual.
void RouteSegmentsDisplay::updateColorArcs() {
    for ( auto& visualsI: visuals_ ) { visualsI->setArcColor ( color_arcs_->getOgreColor() ); }
}

// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateShowArcs() {
    for ( auto& visualsI: visuals_ ) { visualsI->setShowArcs ( show_arcs_property_->getBool() ); }
}
// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateShowStartPoints() {
    for ( auto& visualsI: visuals_ ) { visualsI->setShowStartPoints ( show_start_point_->getBool() ); }
}
// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateShowEndPoints() {
    for ( auto& visualsI: visuals_ ) { visualsI->setShowEndPoints ( show_end_point_->getBool() ); }
}
// Set the number of past visuals to show.
void RouteSegmentsDisplay::updateShowCenterPoints() {
    for ( auto& visualsI: visuals_ ) { visualsI->setShowCenterPoints ( show_center_point_->getBool() ); }
}


// This is our callback to handle an incoming message.
void RouteSegmentsDisplay::processMessage ( const tuw_nav_msgs::RouteSegments::ConstPtr& msg ) {
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
    boost::shared_ptr<RouteSegmentsVisual> visual;
    if ( visuals_.full() ) {
        visual = visuals_.front();
    } else {
        visual.reset ( new RouteSegmentsVisual ( context_->getSceneManager(), scene_node_ ) );
    }

    // Now set or update the contents of the chosen visual.
    visual->setMessage          ( msg );
    visual->setFramePosition    ( position );
    visual->setFrameOrientation ( orientation );
    
    visual->setStartPointColor     ( color_start_point_->getOgreColor() );
    visual->setStartPointShape     ( ( rviz::Shape::Type ) shape_start_point_->getOptionInt() );
    visual->setStartPointScale     ( scale_start_point_->getFloat() );
    
    
    visual->setEndPointColor     ( color_end_point_->getOgreColor() );
    visual->setEndPointShape     ( ( rviz::Shape::Type ) shape_end_point_->getOptionInt() );
    visual->setEndPointScale     ( scale_end_point_->getFloat() );
    
    visual->setCenterPointColor     ( color_center_point_->getOgreColor() );
    visual->setCenterPointShape     ( ( rviz::Shape::Type ) shape_center_point_->getOptionInt() );
    visual->setCenterPointScale     ( scale_center_point_->getFloat() );
    
    visual->setShowArcs(show_arcs_property_->getBool());
    visual->setShowLines(show_lines_property_->getBool());

    // And send it to the end of the circular buffer
    visuals_.push_back ( visual );
}

} // end namespace tuw_nav_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS ( tuw_nav_rviz_plugin::RouteSegmentsDisplay,rviz::Display )
