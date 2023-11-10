// Software License Agreement (BSD License)
//
// Copyright (c) 2021, National Institute of Advanced Industrial Science and Technology (AIST)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of National Institute of Advanced Industrial
//    Science and Technology (AIST) nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Toshio Ueshiba
//
/*!
  \file		textured_mesh_display.h
  \author	Toshio Ueshiba
*/
#pragma once

#include <QObject>

#ifndef Q_MOC_RUN
#  include <mutex>
#  include <OGRE/OgreRenderTargetListener.h>
#  include <OGRE/OgreRenderQueueListener.h>
#  include <OGRE/OgreRoot.h>
#  include <OGRE/OgreSceneNode.h>
#  include <OGRE/OgreManualObject.h>
#  include <OGRE/OgreVector3.h>
#  include <OGRE/OgreWindowEventUtilities.h>
#  include <rviz/display.h>
#  include <rviz/frame_manager.h>
#  include <rviz/image/image_display_base.h>
#  include <rviz/image/ros_image_texture.h>
#  include <aist_visualization/TexturedMeshStamped.h>
#endif  // Q_MOC_RUN

namespace rviz
{
/************************************************************************
*  class TexturedMeshDisplay						*
************************************************************************/
class TexturedMeshDisplay: public rviz::Display,
			   public Ogre::RenderTargetListener,
			   public Ogre::RenderQueueListener
{
    Q_OBJECT
  public:
			TexturedMeshDisplay()				;
    virtual		~TexturedMeshDisplay()				;

  // Overrides from Display
    virtual void	onInitialize()					;
    virtual void	onEnable()					;
    virtual void	onDisable()					;
    virtual void	update(float wall_dt, float ros_dt)		;
    virtual void	reset()						;

  protected:
    virtual void	subscribe()					;
    virtual void	unsubscribe()					;

  private:
    using image_t  = sensor_msgs::Image;
    using image_cp = sensor_msgs::Image::ConstPtr;
    using mesh_t   = aist_visualization::TexturedMeshStamped;
    using mesh_cp  = aist_visualization::TexturedMeshStampedPtr;

    void		updateImage(const image_cp& image)		;
    void		updateMesh(const mesh_cp& mesh)			;

    void		createTexture()					;
    void		createMesh()					;

    void		updateMeshProperties()				;
    void		updateCamera()					;

  private Q_SLOTS:
    void		updateDisplayImages()				;

  private:
    std::unique_ptr<RosTopicProperty>	image_topic_property_;
    std::unique_ptr<RosTopicProperty>	mesh_topic_property_;

    ros::NodeHandle			nh_;
    ros::Subscriber			image_sub_;
    ros::Subscriber			mesh_sub_;

    image_cp				cur_image_;
    mesh_cp				cur_mesh_;

    std::unique_ptr<ROSImageTexture>	texture_;
    std::unique_ptr<Ogre::SceneNode>	mesh_node_;
    std::unique_ptr<Ogre::ManualObject>	manual_object_;
    Ogre::MaterialPtr			mesh_material_;

    std::mutex				image_mutex_;
    std::mutex				mesh_mutex_;
};

}  // namespace rviz
