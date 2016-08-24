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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <spline/spline_visual.h>
#include <eigen3/unsupported/Eigen/Splines>

namespace tuw_nav_rviz_plugin {

SplineVisual::SplineVisual ( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) {
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
    pointsNrPath_ = 100;
    pointsNrOrient_ = 50;
}

SplineVisual::~SplineVisual() {
    // Destroy the frame node since we don't need it anymore.
    scene_manager_->destroySceneNode ( frame_node_ );
}

void SplineVisual::setMessage ( const tuw_nav_msgs::Spline::ConstPtr& msg ) {
    static double timeOld_;
    if( timeOld_ == msg->header.stamp.toSec() ){ return; }
    timeOld_ = msg->header.stamp.toSec();
    
    Eigen::MatrixXd vKnots(1, msg->knots.size() );
    Eigen::MatrixXd mCtrls(msg->ctrls.size(), msg->ctrls[0].val.size() );
    for( int i = 0; i < vKnots.cols(); ++i) {                                              vKnots(i) = msg->knots[i];          }
    for( int i = 0; i < mCtrls.rows(); ++i) {  for( int j = 0; j < mCtrls.cols(); ++j) { mCtrls(i,j) = msg->ctrls[i].val[j]; } }
    spline_ = boost::shared_ptr<Eigen::Spline3d>( new Eigen::Spline3d(vKnots, mCtrls) ); 
    
    splinePtsXY_   .resize ( pointsNrPath_ + 1 );
    for( size_t i = 0; i <= pointsNrPath_; ++i) { 
	double p_x = (*spline_)(i / (double)pointsNrPath_ )(0);
	double p_y = (*spline_)(i / (double)pointsNrPath_ )(1);
	Eigen::SplineTraits< Eigen::Spline3d >::DerivativeType diff = spline_->derivatives(i / (double)pointsNrPath_, 1);
	double v_x = diff(0,1);
	double v_y = diff(1,1);
// 	double p_z = (*spline_)(i / (double)pointsNrPath_ )(2);
	
	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrPath_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
	Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
	splinePtsXY_[i].reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        splinePtsXY_[i]->setColor ( colorPath_ );
        splinePtsXY_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, 0 ) );
        splinePtsXY_[i]->setOrientation ( rotation*rotation2 );
        splinePtsXY_[i]->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) );
    }
    
    splinePtsTheta_.resize ( pointsNrOrient_ + 1 );
//     if( !plotArrows_ ) { splinePtsTheta_.resize ( 0 ); return; }
    for( size_t i = 0; i <= pointsNrOrient_; ++i) { 
	double p_x = (*spline_)(i / (double)pointsNrOrient_ )(0);
	double p_y = (*spline_)(i / (double)pointsNrOrient_ )(1);
	Eigen::SplineTraits< Eigen::Spline3d >::DerivativeType diff = spline_->derivatives(i / (double)pointsNrOrient_, 1);
	double v_x = diff(0,1);
	double v_y = diff(1,1);
// 	double p_z = (*spline_)(i / (double)pointsNrOrient_ )(2);
	
	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( (*spline_)(i / (double)pointsNrOrient_ )(2) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
	Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
	splinePtsTheta_[i].reset ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
	splinePtsTheta_[i]->setColor ( colorOrient_ );
	splinePtsTheta_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, 0 ) );
	splinePtsTheta_[i]->setOrientation ( rotation*rotation2 );
	splinePtsTheta_[i]->setScale ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
    }
}

// Position is passed through to the SceneNode.
void SplineVisual::setFramePosition ( const Ogre::Vector3& position ) {
    frame_node_->setPosition ( position );
}

// Orientation is passed through to the SceneNode.
void SplineVisual::setFrameOrientation ( const Ogre::Quaternion& orientation ) {
    frame_node_->setOrientation ( orientation );
}

// Color is passed through to the Shape object.
void SplineVisual::setPathColor ( Ogre::ColourValue color ) {
    colorPath_ = color;
    for ( auto& splineXYi   : splinePtsXY_    ) { splineXYi   ->setColor ( colorPath_ ); }
}

// Color is passed through to the Shape object.
void SplineVisual::setOrientColor ( Ogre::ColourValue color ) {
    colorOrient_ = color;
    for ( auto& splineThetai: splinePtsTheta_ ) { splineThetai->setColor ( colorOrient_ ); }
}

// Shape type is passed through to the Shape object.
void SplineVisual::setShape ( rviz::Shape::Type shape_type ) {
    shape_type_ = shape_type;
    for ( auto& splineXYi: splinePtsXY_ ) { 
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
void SplineVisual::setPathScale ( float scale ) {
    scalePath_ = scale;
    for ( auto& splineXYi   : splinePtsXY_    ) { splineXYi   ->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) ); }
}

// Scale is passed through to the Shape object.
void SplineVisual::setOrientScale ( float scale ) {
    scaleOrient_ = scale;
    for ( auto& splineThetai: splinePtsTheta_ ) { splineThetai->setScale ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) ); }
}


void SplineVisual::setPathPointsNr ( int pointsNr ) {
    pointsNrPath_ = pointsNr;
    if(!spline_){ return; }
    
    splinePtsXY_.resize ( pointsNrPath_ + 1 );
    for( size_t i = 0; i <= pointsNrPath_; ++i) { 
	auto splineEval = spline_->derivatives(i / (double)pointsNrPath_, 1);
	double p_x = splineEval(0,0), p_y = splineEval(1,0);
	double v_x = splineEval(0,1), v_y = splineEval(1,1);
	
	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( splineEval(2,0) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
	Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
	
	splinePtsXY_[i].reset ( new rviz::Shape ( shape_type_, scene_manager_, frame_node_ ) );
        splinePtsXY_[i]->setColor ( colorPath_ );
        splinePtsXY_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, 0 ) );
        splinePtsXY_[i]->setOrientation ( rotation*rotation2 );
        splinePtsXY_[i]->setScale ( Ogre::Vector3 ( scalePath_, scalePath_, scalePath_ ) );
    }
}

void SplineVisual::setOrientPointsNr ( int pointsNr ) {
    pointsNrOrient_ = pointsNr;
    if(!spline_){ return; }
    
    splinePtsTheta_.resize ( pointsNrOrient_ + 1 );
    for( size_t i = 0; i <= pointsNrOrient_; ++i) { 
	auto splineEval = spline_->derivatives(i / (double)pointsNrOrient_, 1);
	double p_x = splineEval(0,0), p_y = splineEval(1,0);
	double v_x = splineEval(0,1), v_y = splineEval(1,1);
	
	Ogre::Quaternion rotation  = Ogre::Quaternion ( Ogre::Radian( splineEval(2,0) + atan2(v_y, v_x) ), Ogre::Vector3::UNIT_Z );
	Ogre::Quaternion rotation2 = Ogre::Quaternion ( Ogre::Radian( -Ogre::Math::PI/2.), Ogre::Vector3::UNIT_Y );
	
	splinePtsTheta_[i].reset ( new rviz::Arrow ( scene_manager_, frame_node_ ) );
        splinePtsTheta_[i]->setColor ( colorOrient_ );
        splinePtsTheta_[i]->setPosition ( Ogre::Vector3 ( p_x, p_y, 0 ) );
        splinePtsTheta_[i]->setOrientation ( rotation*rotation2 );
        splinePtsTheta_[i]->setScale ( Ogre::Vector3 ( scaleOrient_, scaleOrient_, scaleOrient_ ) );
    }
}


} // end namespace tuw_nav_rviz_plugin

