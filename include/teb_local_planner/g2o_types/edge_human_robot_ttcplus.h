/*/
 * Copyright (c) 2016 LAAS/CNRS
 * All rights reserved.
 *
 * Redistribution and use  in source  and binary  forms,  with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Notes:
 * The following class is derived from a class defined by the
 * g2o-framework. g2o is licensed under the terms of the BSD License.
 * Refer to the base class source for detailed licensing information.
 *
 * Author: Michele Imparato (mimparat@laas.fr)
 */

#ifndef EDGE_HUMAN_ROBOT_TTCplus_H_
#define EDGE_HUMAN_ROBOT_TTCplus_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <std_msgs/Header.h>
#include "sstream"
#include <teb_local_planner/g2o_types/base_teb_edges.h>

// #include "g2o/core/base_multi_edge.h"

namespace teb_local_planner {

 class EdgeHumanRobotTTCplus : public BaseTebMultiEdge<1, double> {
public:
  EdgeHumanRobotTTCplus() {
    this->resize(6);
  }


   void computeError() {
    ROS_ASSERT_MSG(cfg_ && (radius_sum_ < std::numeric_limits<double>::infinity()), "You must call setParameters() on EdgeHumanRobotTTCplus()");
    const VertexPose *robot_bandpt =
        static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *robot_bandpt_nxt =
        static_cast<const VertexPose *>(_vertices[1]);
    const VertexTimeDiff *dt_robot =
        static_cast<const VertexTimeDiff *>(_vertices[2]);
    const VertexPose *human_bandpt =
        static_cast<const VertexPose *>(_vertices[3]);
    const VertexPose *human_bandpt_nxt =
        static_cast<const VertexPose *>(_vertices[4]);
    const VertexTimeDiff *dt_human =
        static_cast<const VertexTimeDiff *>(_vertices[5]);

    Eigen::Vector2d diff_robot = robot_bandpt_nxt->position() - robot_bandpt->position();
    Eigen::Vector2d robot_vel = diff_robot / dt_robot->dt();
    Eigen::Vector2d diff_human = human_bandpt_nxt->position() - human_bandpt->position();
    Eigen::Vector2d human_vel = diff_human / dt_human->dt();

    Eigen::Vector2d C = human_bandpt->position() - robot_bandpt->position();


    static double ttcplus = std::numeric_limits<double>::infinity();
    static double C_sq = C.dot(C);
    static double i =0;
    static double j =0;
    static double d=20;
    
    if (C_sq <= radius_sum_sq_) {
      ttcplus = 0.0;
    } 
    else {      

      Eigen::Vector2d V = robot_vel - human_vel;
      double C_dot_V = C.dot(V);
      if (C_dot_V > 0) { // otherwise ttcplus is infinite
        double V_sq = V.dot(V);
        double f = (C_dot_V * C_dot_V) - (V_sq * (C_sq - radius_sum_sq_));
        if (f > 0) {         // otherwise ttcplus is infinite
        	//if(i==0){ d = C_sq;}
        	i = i+1;
        	ttcplus = (C_dot_V - std::sqrt(f)) / V_sq;

         }
       }
      }


     if (ttcplus < std::numeric_limits<double>::infinity()) {
     	if( i > cfg_->hateb.ttcplus_timer ){              // timer in tenth of second
    	  j=j+1 ;
    	  i=0 ;
      _error[0] = penaltyBoundFromBelow(ttcplus, cfg_->hateb.ttcplus_threshold, cfg_->optim.penalty_epsilon) * j / C_sq ;
      std::cout << "ttcplus" <<ttcplus<< '\n';
      if (cfg_->hateb.scale_human_robot_ttcplus_c) {
        _error[0] = _error[0] * cfg_->optim.human_robot_ttcplus_scale_alpha / C_sq;
      }
     }
    }

    else {
      // no collision possible
    	 if(C_sq > 2){
        i=0;
        j=0;
        d=20;
        _error[0] = 0.0;
     	}
    }

     ROS_DEBUG_THROTTLE(0.5, "ttcplus value : %f", ttcplus);

     ROS_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeHumanRobot::computeError() _error[0]=%f\n", _error[0]);
  }


  void setParameters(const TebConfig &cfg, const double &robot_radius,
                     const double &human_radius) {
    cfg_ = &cfg;
    radius_sum_ = robot_radius + human_radius;
    radius_sum_sq_ = radius_sum_ * radius_sum_;
  }

 protected:
  double radius_sum_ = std::numeric_limits<double>::infinity();
  double radius_sum_sq_ = std::numeric_limits<double>::infinity();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

 } // end namespace

 #endif // EDGE_HUMAN_ROBOT_TTCplus_H_
