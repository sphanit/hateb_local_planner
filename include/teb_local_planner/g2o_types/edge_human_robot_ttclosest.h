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
 * Author: MIchele Imparato (Laas Stagiaire (R.Alami))
 */

 #ifndef EDGE_HUMAN_ROBOT_TTClosest_H_
#define EDGE_HUMAN_ROBOT_TTClosest_H_

 #include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>
#include <teb_local_planner/g2o_types/base_teb_edges.h>

 // #include "g2o/core/base_multi_edge.h"

 namespace teb_local_planner {

 class EdgeHumanRobotTTClosest : public BaseTebMultiEdge<1, double> {
public:
  EdgeHumanRobotTTClosest() {
    this->resize(6);
    // this->setMeasurement(0.);
    // _vertices[0] = _vertices[1] = _vertices[2] = _vertices[3] = _vertices[4] =
    //     _vertices[5] = NULL;
  }

   // virtual ~EdgeHumanRobotTTClosest() {
   //  for (unsigned int i = 0; i < 6; i++) {
   //    if (_vertices[i])
   //      _vertices[i]->edges().erase(this);
   //
   //   }

   void computeError() {
    ROS_ASSERT_MSG(cfg_ && (radius_sum_ < std::numeric_limits<double>::infinity()),
                   "You must call setParameters() on EdgeHumanRobotTTClosest()");
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

     Eigen::Vector2d diff_robot =
        robot_bandpt_nxt->position() - robot_bandpt->position();
    Eigen::Vector2d robot_vel = diff_robot / dt_robot->dt();
    Eigen::Vector2d diff_human =
        human_bandpt_nxt->position() - human_bandpt->position();
    Eigen::Vector2d human_vel = diff_human / dt_human->dt();

     Eigen::Vector2d C = human_bandpt->position() - robot_bandpt->position();

     double ttcl = std::numeric_limits<double>::infinity();
    double C_sq = C.dot(C);
    if (C_sq <= radius_sum_sq_) {
      ttcl = 0.0;
    } else {
      Eigen::Vector2d V = human_vel - robot_vel;
      double C_dot_V = C.dot(V);
     if (C_dot_V > 0) { // otherwise ttcl is infinite
        double V_sq = V.dot(V);
        //double f =  (C_dot_V * C_dot_V) - (V_sq * (C_sq - (C.dot(human_vel))*(C.dot(human_vel))/  (robot_vel.dot(robot_vel) + human_vel.dot(human_vel)  )));
        //if (f > 0) { // otherwise ttcl is infinite
          ttcl = /*(C_dot_V - std::sqrt(f)) / V_sq;*/  std::sqrt(C_sq/V_sq );     /* C_sq * (V#.dot(human_vel))/(robot_vel.dot(robot_vel) + human_vel.dot(human_vel) ) ) ;*/  C.dot(V)/ V_sq ;
        }
      //}
    }

     if (ttcl < std::numeric_limits<double>::infinity()) {
      // if (ttcl > 0) {
      //   // valid ttcl
      //   _error[0] = penaltyBoundFromBelow(ttcl, cfg_->human.ttcl_threshold,
      //                                 cfg_->optim.penalty_epsilon);
      // } else {
      //   // already in collision
      //   _error[0] = cfg_->optim.max_ttcl_penalty;
      // }

       _error[0] = penaltyBoundFromBelow(ttcl, cfg_->hateb.ttclosest_threshold,
                                        cfg_->optim.penalty_epsilon);
      if (cfg_->hateb.scale_human_robot_ttc_c) {
        _error[0] = _error[0] * cfg_->optim.human_robot_ttc_scale_alpha / C_sq;
      }

     } else {
      // no collsion possible
      _error[0] = 0.0;
    }
    ROS_DEBUG_THROTTLE(0.5, "ttcl value : %f", ttcl);

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

 #endif // EDGE_HUMAN_ROBOT_TTClosest_H_
