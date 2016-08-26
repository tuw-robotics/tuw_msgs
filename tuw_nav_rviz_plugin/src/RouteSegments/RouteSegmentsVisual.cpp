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
#include <tuw_nav_msgs/route_segment.h>
#include <tuw_nav_msgs/route_segments.h>

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
    color_lines_ = Ogre::ColourValue ( 170, 0, 170 );
    color_arcs_ = Ogre::ColourValue ( 0, 170,  170 );

    shape_start_point_ = rviz::Shape::Sphere;
    shape_end_point_ = rviz::Shape::Cube;
    shape_center_point_ = rviz::Shape::Sphere;

    scale_start_point_ = 0.05;
    scale_end_point_ = 0.04;
    scale_center_point_ = 0.05;

    show_acrs_ = true;
    show_lines_ = true;
    show_start_points_ = true;
    show_end_points_ = false;
    show_center_points_ = false;
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

    startPts_.clear();
    endPts_.clear();
    centerPts_.clear();
    lines_.clear ();
    arcs_.clear ();
    for ( size_t i = 0; i < msg->segments.size(); i++ ) {
        const tuw_nav_msgs::obj::RouteSegment &segment = (const tuw_nav_msgs::obj::RouteSegment &) msg->segments[i];
        const geometry_msgs::Pose &p0 = segment.start;
        const geometry_msgs::Pose &pc = segment.center;
        const geometry_msgs::Pose &p1 = segment.end;

        if ( show_start_points_ ) {
            startPts_.push_back ( boost::shared_ptr<rviz::Shape> ( new rviz::Shape ( shape_start_point_, scene_manager_, frame_node_ ) ) );
            boost::shared_ptr<rviz::Shape> startShape = startPts_.back();
            startShape->setColor ( color_start_point_ );
            startShape->setPosition ( Ogre::Vector3 ( p0.position.x, p0.position.y, p0.position.z ) );
            startShape->setOrientation ( Ogre::Quaternion ( p0.orientation.x, p0.orientation.y, p0.orientation.z, p0.orientation.w ) );
            startShape->setScale ( Ogre::Vector3 ( scale_start_point_, scale_start_point_, scale_start_point_ ) );
        }

        if ( show_end_points_ ) {
            endPts_.push_back ( boost::shared_ptr<rviz::Shape> ( new rviz::Shape ( shape_end_point_, scene_manager_, frame_node_ ) ) );
            boost::shared_ptr<rviz::Shape> endShape = endPts_.back();
            endShape->setColor ( color_end_point_ );
            endShape->setPosition ( Ogre::Vector3 ( p1.position.x, p1.position.y, p1.position.z ) );
            endShape->setOrientation ( Ogre::Quaternion ( p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w ) );
            endShape->setScale ( Ogre::Vector3 ( scale_end_point_, scale_end_point_, scale_end_point_ ) );
        }
        if ( show_center_points_ && (segment.type == tuw_nav_msgs::obj::RouteSegment::ARC ))  {
            centerPts_.push_back ( boost::shared_ptr<rviz::Shape> ( new rviz::Shape ( shape_center_point_, scene_manager_, frame_node_ ) ) );
            boost::shared_ptr<rviz::Shape> centerShape = centerPts_.back();
            centerShape->setColor ( color_center_point_ );
            centerShape->setPosition ( Ogre::Vector3 ( pc.position.x, pc.position.y, pc.position.z ) );
            centerShape->setOrientation ( Ogre::Quaternion ( pc.orientation.x, pc.orientation.y, pc.orientation.z, pc.orientation.w ) );
            centerShape->setScale ( Ogre::Vector3 ( scale_center_point_, scale_center_point_, scale_center_point_ ) );
        }
        if ( show_lines_ && ( segment.type == tuw_nav_msgs::obj::RouteSegment::LINE ) ) {
            lines_.push_back ( boost::shared_ptr<rviz::Line> ( new rviz::Line ( scene_manager_, frame_node_ ) ) );
            boost::shared_ptr<rviz::Line> line = lines_.back();
            line->setColor ( color_lines_ );
            line->setPoints ( Ogre::Vector3 ( p0.position.x, p0.position.y, p0.position.z ), Ogre::Vector3 ( p1.position.x, p1.position.y, p1.position.z ) );
            line->setScale ( Ogre::Vector3 ( 1, 1, 1 ) );
        }

        if ( show_acrs_ && ( segment.type == tuw_nav_msgs::obj::RouteSegment::ARC ) ) {
            double angle_resolution = M_PI/45.;
            std::vector<geometry_msgs::PosePtr> poses;
            
            segment.sample_equal_angle(poses, angle_resolution, 0);
            
            Ogre::Vector3 v0( p0.position.x, p0.position.y, p0.position.z);
            Ogre::Vector3 v1;
            for ( int i = 1; i < poses.size(); i++) {
                arcs_.push_back ( boost::shared_ptr<rviz::Line> ( new rviz::Line ( scene_manager_, frame_node_ ) ) );
                boost::shared_ptr<rviz::Line> line = arcs_.back();
                v1 = Ogre::Vector3 ( poses[i]->position.x, poses[i]->position.y, poses[i]->position.z);
                line->setColor ( color_arcs_ );
                line->setPoints ( v0, v1 );
                line->setScale ( Ogre::Vector3 ( 1, 1, 1 ) );
                v0 = v1;
            }
            arcs_.push_back ( boost::shared_ptr<rviz::Line> ( new rviz::Line ( scene_manager_, frame_node_ ) ) );
            boost::shared_ptr<rviz::Line> line = arcs_.back();
            v1 = Ogre::Vector3 ( p1.position.x, p1.position.y, p1.position.z);
            line->setColor ( color_arcs_ );
            line->setPoints ( v0, v1 );
            line->setScale ( Ogre::Vector3 ( 1, 1, 1 ) );
            
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


void RouteSegmentsVisual::setLineColor ( Ogre::ColourValue color ) {
    color_lines_ = color;
    for ( boost::shared_ptr <rviz::Line > &pnt   : lines_ ) {
        pnt   ->setColor ( color_lines_ );
    }
}
void RouteSegmentsVisual::setArcColor ( Ogre::ColourValue color ) {
    color_arcs_ = color;
    for ( boost::shared_ptr <rviz::Line > &pnt   : arcs_ ) {
        pnt   ->setColor ( color_arcs_ );
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

void RouteSegmentsVisual::setShowArcs ( bool visible ) {
    show_acrs_ = visible;
    if ( !show_acrs_ ) arcs_.clear();
}

void RouteSegmentsVisual::setShowLines ( bool visible ) {
    show_lines_ = visible;
    if ( !show_lines_ ) lines_.clear();
}
void RouteSegmentsVisual::setShowStartPoints ( bool visible ) {
    show_start_points_ = visible;
    if ( !show_start_points_ ) startPts_.clear();
}
void RouteSegmentsVisual::setShowEndPoints ( bool visible ) {
    show_end_points_ = visible;
    if ( !show_end_points_ ) endPts_.clear();
}
void RouteSegmentsVisual::setShowCenterPoints ( bool visible ) {
    show_center_points_ = visible;
    if ( !show_center_points_ ) centerPts_.clear();
}


} // end namespace tuw_nav_rviz_plugin

