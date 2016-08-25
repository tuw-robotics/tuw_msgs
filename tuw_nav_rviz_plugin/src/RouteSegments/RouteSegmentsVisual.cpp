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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <RouteSegments/RouteSegmentsVisual.h>
#include <eigen3/unsupported/Eigen/Splines>
#include <tuw_nav/route_segments.h>

namespace tuw_nav_rviz_plugin {

RouteSegmentsVisual::RouteSegmentsVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
    scene_manager_ = scene_manager;

    // Ogre::SceneNode s form a tree, with each node storing the
    // transform (position and orientation) of itself relative to its
    // parent.  Ogre does the math of combining those transforms when it
    // is time to render.
    //
    // Here we create a node to store the pose of the MarkerDetection's header frame
    // relative to the RViz fixed frame.
    frame_node_ = parent_node->createChildSceneNode();

    // initialize global variables
    color_start_point_ = Ogre::ColourValue ( 170, 0, 0 );
    color_end_point_ = Ogre::ColourValue ( 0, 170, 0 );
    color_center_point_ = Ogre::ColourValue ( 0, 0, 170 );
    
    shape_start_point_ = rviz::Shape::Sphere;
    shape_end_point_ = rviz::Shape::Cube;
    shape_center_point_ = rviz::Shape::Sphere;
    
    scale_start_point_ = 0.04;
    scale_end_point_ = 0.08;
    scale_center_point_ = 0.01;
}

RouteSegmentsVisual::~RouteSegmentsVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void RouteSegmentsVisual::setMessage ( const tuw_nav_msgs::RouteSegments::ConstPtr& msg ) {
    static double timeOld_;
    if ( timeOld_ == msg->header.stamp.toSec() ) {
        return;
    }
    timeOld_ = msg->header.stamp.toSec();

    startPts_.resize ( msg->segments.size() );
    endPts_.resize ( msg->segments.size() );
    centerPts_.resize ( msg->segments.size() );
    for ( size_t i = 0; i < msg->segments.size(); i++ ) {
        {
            if ( !startPts_[i] ) startPts_[i].reset ( new rviz::Shape ( shape_start_point_, scene_manager_, frame_node_ ) );
            const geometry_msgs::Pose &p = msg->segments[i].start;
            startPts_[i]->setColor ( color_start_point_ );
            startPts_[i]->setPosition ( Ogre::Vector3 ( p.position.x, p.position.y, p.position.z ) );
            startPts_[i]->setOrientation ( Ogre::Quaternion ( p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w ) );
            startPts_[i]->setScale ( Ogre::Vector3 ( scale_start_point_, scale_start_point_, scale_start_point_ ) );
        }

        {
            if ( !endPts_[i] ) endPts_[i].reset ( new rviz::Shape ( shape_end_point_, scene_manager_, frame_node_ ) );
            const geometry_msgs::Pose &p = msg->segments[i].end;
            endPts_[i]->setColor ( color_end_point_ );
            endPts_[i]->setPosition ( Ogre::Vector3 ( p.position.x, p.position.y, p.position.z ) );
            endPts_[i]->setOrientation ( Ogre::Quaternion ( p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w ) );
            endPts_[i]->setScale ( Ogre::Vector3 ( scale_end_point_, scale_end_point_, scale_end_point_ ) );
        }
        {
            if ( !centerPts_[i] ) centerPts_[i].reset ( new rviz::Shape ( shape_center_point_, scene_manager_, frame_node_ ) );
            const geometry_msgs::Pose &p = msg->segments[i].center;
            centerPts_[i]->setColor ( color_center_point_ );
            centerPts_[i]->setPosition ( Ogre::Vector3 ( p.position.x, p.position.y, p.position.z ) );
            centerPts_[i]->setOrientation ( Ogre::Quaternion ( p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w ) );
            centerPts_[i]->setScale ( Ogre::Vector3 ( scale_center_point_, scale_center_point_, scale_center_point_ ) );
        }

    }
    
    lines_.clear ();
    for ( size_t i = 0; i < msg->segments.size(); i++ ) {
        if (msg->segments[i].type == tuw_nav::RouteSegment::LINE){
          lines_.push_back( boost::shared_ptr<rviz::Line>(new rviz::Line ( scene_manager_, frame_node_ )) );
          boost::shared_ptr<rviz::Line> line = lines_.back();
          line->setColor ( color_center_point_ );
            const geometry_msgs::Pose &p0 = msg->segments[i].start;
            const geometry_msgs::Pose &p1 = msg->segments[i].end;
            line->setPoints(Ogre::Vector3 ( p0.position.x, p0.position.y, p0.position.z ), Ogre::Vector3 ( p1.position.x, p1.position.y, p1.position.z ));
            line->setScale ( Ogre::Vector3 ( 1, 1, 1 ) );
          
        /*
        if(msg->segments.type == 
        double p_x = (*spline_)(i / (double)pointsNrOrient_ )(0);
        double p_y = (*spline_)(i / (double)pointsNrOrient_ )(1);
        Eigen::SplineTraits< Eigen::Spline3d >::DerivativeType diff = spline_->derivatives(i / (double)pointsNrOrient_, 1);
        double v_x = diff(0,1);
        double v_y = diff(1,1);
//      double p_z = (*spline_)(i / (double)pointsNrOrient_ )(2);
        
        Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrOrient_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
        Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
        splinePtsTheta_[i].reset ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
        splinePtsTheta_[i]->setColor ( colorOrient_ );
        splinePtsTheta_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, 0 ) );
        splinePtsTheta_[i]->setOrientation ( rotation*rotation2 );
        splinePtsTheta_[i]->setScale ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
        */
        }
    }
}

// Position is passed through to the SceneNode.
void RouteSegmentsVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void RouteSegmentsVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Color is passed through to the Shape object.
void RouteSegmentsVisual::setStartPointColor ( Ogre::ColourValue color ) {
    color_start_point_ = color;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : startPts_ ) {
        pnt   ->setColor ( color_start_point_ );
    }
}
// Color is passed through to the Shape object.
void RouteSegmentsVisual::setEndPointColor ( Ogre::ColourValue color ) {
    color_end_point_ = color;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : endPts_ ) {
        pnt   ->setColor ( color_end_point_ );
    }
}
// Color is passed through to the Shape object.
void RouteSegmentsVisual::setCenterPointColor ( Ogre::ColourValue color ) {
    color_center_point_ = color;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : centerPts_ ) {
        pnt   ->setColor ( color_center_point_ );
    }
}



// Shape type is passed through to the Shape object.
void RouteSegmentsVisual::setStartPointShape ( rviz::Shape::Type shape_type ) {
    shape_start_point_ = shape_type;
    for ( boost::shared_ptr <rviz::Shape > &pnt: startPts_ ) {
        Ogre::Vector3       posOld = pnt->getPosition();
        Ogre::Quaternion orientOld = pnt->getOrientation();
        pnt.reset ( new rviz::Shape ( shape_start_point_, scene_manager_, frame_node_ ) );
        pnt->setColor ( color_center_point_ );
        pnt->setPosition ( posOld );
        pnt->setOrientation ( orientOld );
        pnt->setScale ( Ogre::Vector3 ( scale_start_point_, scale_start_point_, scale_start_point_ ) );
    }
}
// Shape type is passed through to the Shape object.
void RouteSegmentsVisual::setEndPointShape ( rviz::Shape::Type shape_type ) {
    shape_end_point_ = shape_type;
    for ( boost::shared_ptr <rviz::Shape > &pnt: endPts_ ) {
        Ogre::Vector3       posOld = pnt->getPosition();
        Ogre::Quaternion orientOld = pnt->getOrientation();
        pnt.reset ( new rviz::Shape ( shape_end_point_, scene_manager_, frame_node_ ) );
        pnt->setColor ( color_end_point_ );
        pnt->setPosition ( posOld );
        pnt->setOrientation ( orientOld );
        pnt->setScale ( Ogre::Vector3 ( scale_end_point_, scale_end_point_, scale_end_point_ ) );
    }
}
// Shape type is passed through to the Shape object.
void RouteSegmentsVisual::setCenterPointShape ( rviz::Shape::Type shape_type ) {
    shape_center_point_ = shape_type;
    for ( boost::shared_ptr <rviz::Shape > &pnt: centerPts_ ) {
        Ogre::Vector3       posOld = pnt->getPosition();
        Ogre::Quaternion orientOld = pnt->getOrientation();
        pnt.reset ( new rviz::Shape ( shape_center_point_, scene_manager_, frame_node_ ) );
        pnt->setColor ( color_center_point_ );
        pnt->setPosition ( posOld );
        pnt->setOrientation ( orientOld );
        pnt->setScale ( Ogre::Vector3 ( scale_center_point_, scale_center_point_, scale_center_point_ ) );
    }
}

// Scale is passed through to the Shape object.
void RouteSegmentsVisual::setStartPointScale ( float scale ) {
    scale_start_point_ = scale;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : startPts_ ) {
        pnt   ->setScale ( Ogre::Vector3 ( scale_start_point_, scale_start_point_, scale_start_point_ ) );
    }
}
// Scale is passed through to the Shape object.
void RouteSegmentsVisual::setEndPointScale ( float scale ) {
    scale_end_point_ = scale;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : endPts_ ) {
        pnt   ->setScale ( Ogre::Vector3 ( scale_end_point_, scale_end_point_, scale_end_point_ ) );
    }
}
// Scale is passed through to the Shape object.
void RouteSegmentsVisual::setCenterPointScale ( float scale ) {
    scale_center_point_ = scale;
    for ( boost::shared_ptr <rviz::Shape > &pnt   : centerPts_ ) {
        pnt   ->setScale ( Ogre::Vector3 ( scale_center_point_, scale_center_point_, scale_center_point_ ) );
    }
}

} // end namespace tuw_nav_rviz_plugin

