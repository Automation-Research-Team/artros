/****************************************************************************
 *
 * $Id: file.h 3496 2011-11-22 15:14:32Z fnovotny $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Calibrator node
 *
 * Authors:
 * Filip Novotny
 *
 *
 *****************************************************************************/

/*!
  \file		Calibrator.h
  \brief	Calibrator node implementing a quick compute service, a compute service and 2 subscribers to world_effector_topic and camera_object_topic.
*/

#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "ros/ros.h"

#include "Transform.h"
#include "visp_hand2eye_calibration/TransformArray.h"
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h"
#include "visp_hand2eye_calibration/compute_effector_camera.h"
#include "visp_hand2eye_calibration/reset.h"
#include "image_proc/advertisement_checker.h"

#include <vector>

namespace visp_hand2eye_calibration
{
/************************************************************************
*  class Calibrator							*
************************************************************************/
class Calibrator
{
  public:
    using transform_t	= TU::Transform<double>;
    
  public:
  //! advertises services and subscribes to topics
		Calibrator();
		~Calibrator();

  //! spins the ros node
    void	spin();
    
  private:
  /*!
    \brief callback corresponding to the camera->object topic.

    Adds a geometry_msgs::Transform to the internal queue. 
    A service may compute the calibration on all recieved elements later.
    \param trans: camera->object transformation
  */
    void	cameraObjectCallback(
		    const geometry_msgs::Transform::ConstPtr& trans);
  /*!
    \brief callback corresponding to the world->effector topic.

    Adds a geometry_msgs::Transform to the internal queue. 
    A service may compute the calibration on all recieved elements later.
    \param trans: world->effector transformation
  */
    void	worldEffectorCallback(
		    const geometry_msgs::Transform::ConstPtr& trans);

  /*!
    \brief service computing world->effector transformation from
    accumulated data. The service expects the number of recorded
    camera->object transformation to be equal to the number
    of recorded world->effector transformations.
    If it is not equal, the service fails.
  */
    bool	computeEffectorCameraCallback(
		    compute_effector_camera::Request&  req,
		    compute_effector_camera::Response& res)		;

  /*!
    \brief service computing world->effector transformation
    from parameter-passed data. The service expects the number
    of recorded camera->object transformation to be equal
    to the number of recorded world->effector transformations.
    If it is not equal, the service fails.
  */
    bool	computeEffectorCameraQuickCallback(
		    compute_effector_camera_quick::Request&  req,
		    compute_effector_camera_quick::Response& res)	;
  /*!
    \brief service reseting the acumulated data
  */
    bool	resetCallback(reset::Request& req, reset::Response& res);
    
  private:    
  //subscribers. Must be class-persistant
    ros::NodeHandle			_node;
    
    const ros::ServiceServer		_computeEffectorCamera;
    const ros::ServiceServer		_computeEffectorCameraQuick;
    const ros::ServiceServer		_reset;
    ros::Subscriber			_cMo_subscriber;
    ros::Subscriber			_wMe_subscriber;
    image_proc::AdvertisementChecker	_check_inputs;

    std::vector<transform_t>		_cMo;
    std::vector<transform_t>		_wMe;

    size_t				_queue_size;
};
}	// namespace visp_hnad2eye_calibration
#endif	// !CALIBRATOR_H

