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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <Path/PathVisual.h>
#include <eigen3/unsupported/Eigen/Splines>

namespace tuw_nav_rviz_plugin {

PathVisual::PathVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
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
    colorPath_ = Ogre::ColourValue ( 255, 0, 0 );
    colorOrient_ = Ogre::ColourValue ( 255, 255, 0 );
    shape_type_ = rviz::Shape::Sphere;
    scalePath_ = 0.01;
    scaleOrient_ = 0.1;
    deltaSOrient_ = 0.1;
}

PathVisual::~PathVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void PathVisual::setMessage ( const nav_msgs::Path::ConstPtr& msg ) {
    static double timeOld_;
    if( timeOld_ == msg->header.stamp.toSec() ){ return; }
    timeOld_ = msg->header.stamp.toSec();
    
    pathPtsXY_   .resize ( msg->poses.size() );
    for( size_t i = 0; i < pathPtsXY_.size(); ++i) { 
	double p_x = msg->poses[i].pose.position.x;
	double p_y = msg->poses[i].pose.position.y;
	double p_z = msg->poses[i].pose.position.z;
	
// 	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrPath_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
	
	
	Ogre::Quaternion rotation;
	rotation.x = msg->poses[i].pose.orientation.x;
	rotation.y = msg->poses[i].pose.orientation.y;
	rotation.z = msg->poses[i].pose.orientation.z;
	rotation.w = msg->poses[i].pose.orientation.w;
	Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
	
	pathPtsXY_[i].reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        pathPtsXY_[i]->setColor ( colorPath_ );
        pathPtsXY_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, p_z ) );
        pathPtsXY_[i]->setOrientation ( rotation *rotation2 );
        pathPtsXY_[i]->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) );
    }
    
    pathPtsTheta_.clear();
    
    
    if(pathPtsXY_.size() > 0) {
	pathPtsTheta_.reserve ( pathPtsXY_.size() );
	double d = 0;
	pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	pathPtsTheta_.back()->setColor       ( colorOrient_ );
	pathPtsTheta_.back()->setPosition    ( pathPtsXY_[0]->getPosition   () );
	pathPtsTheta_.back()->setOrientation ( pathPtsXY_[0]->getOrientation() );
	pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
	for( size_t i = 1; i < pathPtsXY_.size(); ++i) { 
	    
	    d += pathPtsXY_[i]->getPosition().distance(pathPtsXY_[i-1]->getPosition());
	    if ( d < deltaSOrient_ * (pathPtsTheta_.size() /*+ 1*/) ) { continue; }
// 	    d = d - (int)(d / deltaSOrient_) * deltaSOrient_;
	    
	    pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	    pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	    pathPtsTheta_.back()->setColor       ( colorOrient_ );
	    pathPtsTheta_.back()->setPosition    ( pathPtsXY_[i]->getPosition   () );
	    pathPtsTheta_.back()->setOrientation ( pathPtsXY_[i]->getOrientation() );
	    pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
	}
	pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	pathPtsTheta_.back()->setColor       ( colorOrient_ );
	pathPtsTheta_.back()->setPosition    ( pathPtsXY_.back()->getPosition   () );
	pathPtsTheta_.back()->setOrientation ( pathPtsXY_.back()->getOrientation() );
	pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
    }
}

// Position is passed through to the SceneNode.
void PathVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void PathVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Color is passed through to the Shape object.
void PathVisual::setPathColor ( Ogre::ColourValue color ) {
    colorPath_ = color;
    for ( auto& pathXYi   : pathPtsXY_    ) { pathXYi   ->setColor ( colorPath_ ); }
}

// Color is passed through to the Shape object.
void PathVisual::setOrientColor ( Ogre::ColourValue color ) {
    colorOrient_ = color;
    for ( auto& pathThetai: pathPtsTheta_ ) { pathThetai->setColor ( colorOrient_ ); }
}

// Shape type is passed through to the Shape object.
void PathVisual::setShape ( rviz::Shape::Type shape_type ) {
    shape_type_ = shape_type;
    for ( auto& splineXYi: pathPtsXY_ ) { 
	Ogre::Vector3       posOld = splineXYi->getPosition();
	Ogre::Quaternion orientOld = splineXYi->getOrientation();
	splineXYi.reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        splineXYi->setColor       (    colorPath_ );
        splineXYi->setPosition    (    posOld     );
        splineXYi->setOrientation ( orientOld );
        splineXYi->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) );
    }
}

// Scale is passed through to the Shape object.
void PathVisual::setPathScale ( float scale ) {
    scalePath_ = scale;
    for ( auto& pathXYi   : pathPtsXY_    ) { pathXYi   ->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) ); }
}

// Scale is passed through to the Shape object.
void PathVisual::setOrientScale ( float scale ) {
    scaleOrient_ = scale;
    for ( auto& pathThetai: pathPtsTheta_ ) { pathThetai->setScale ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) ); }
}

void PathVisual::setOrientDeltaS ( double deltaS ) {
    deltaSOrient_ = fmax(1e-5, deltaS);
    
    pathPtsTheta_.clear();
    if(pathPtsXY_.size() > 0) {
	pathPtsTheta_.reserve ( pathPtsXY_.size() );
	double d = 0;
	pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	pathPtsTheta_.back()->setColor       ( colorOrient_ );
	pathPtsTheta_.back()->setPosition    ( pathPtsXY_[0]->getPosition   () );
	pathPtsTheta_.back()->setOrientation ( pathPtsXY_[0]->getOrientation() );
	pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
	for( size_t i = 1; i < pathPtsXY_.size(); ++i) { 
	    
	    d += pathPtsXY_[i]->getPosition().distance(pathPtsXY_[i-1]->getPosition());
	    if ( d < deltaSOrient_ * (pathPtsTheta_.size() /*+ 1*/) ) { continue; }
// 	    d = d - (int)(d / deltaSOrient_) * deltaSOrient_;
	    
	    pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	    pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	    pathPtsTheta_.back()->setColor       ( colorOrient_ );
	    pathPtsTheta_.back()->setPosition    ( pathPtsXY_[i]->getPosition   () );
	    pathPtsTheta_.back()->setOrientation ( pathPtsXY_[i]->getOrientation() );
	    pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
	}
	pathPtsTheta_.emplace_back(boost::shared_ptr<rviz::Arrow>());
	pathPtsTheta_.back().reset           ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	pathPtsTheta_.back()->setColor       ( colorOrient_ );
	pathPtsTheta_.back()->setPosition    ( pathPtsXY_.back()->getPosition   () );
	pathPtsTheta_.back()->setOrientation ( pathPtsXY_.back()->getOrientation() );
	pathPtsTheta_.back()->setScale       ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
    }
}


} // end namespace tuw_nav_rviz_plugin

