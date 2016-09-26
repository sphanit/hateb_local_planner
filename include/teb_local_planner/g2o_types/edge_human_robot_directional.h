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
 * Author: Harmish Khambhaita (harmish@laas.fr)
 */

#ifndef EDGE_HUMAN_ROBOT_DIR_H_
#define EDGE_HUMAN_ROBOT_DIR_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/vertex_timediff.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_multi_edge.h"

namespace teb_local_planner {

class EdgeHumanRobotDirectional : public g2o::BaseMultiEdge<1, double> {
public:
  EdgeHumanRobotDirectional() {
    this->resize(6);
    // this->setMeasurement(0.);
    _vertices[0] = _vertices[1] = _vertices[2] = _vertices[3] = _vertices[4] =
        _vertices[5] = NULL;
  }

  virtual ~EdgeHumanRobotDirectional() {
    for (unsigned int i = 0; i < 6; i++) {
      if (_vertices[i])
        _vertices[i]->edges().erase(this);
    }
  }

  void computeError() {
    ROS_ASSERT_MSG(
        cfg_ && (radius_sum_ < std::numeric_limits<double>::infinity()),
        "You must call setTebConfig() on EdgeHumanRobotDirectional()");
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

    Eigen::Vector2d d_rtoh =
        human_bandpt->position() - robot_bandpt->position();
    Eigen::Vector2d d_htor =
        robot_bandpt->position() - human_bandpt->position();

    double dir_cost = (std::max(robot_vel.dot(d_rtoh), 0.0) +
                       std::max(human_vel.dot(d_htor), 0.0)) /
                      d_rtoh.dot(d_rtoh);
    ROS_INFO_THROTTLE(0.5, "dir_cost value : %f", dir_cost);

    _error[0] = penaltyBoundFromBelow(dir_cost, cfg_->human.dir_cost_threshold,
                                      cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeHumanRobot::computeError() _error[0]=%f\n", _error[0]);
  }

  ErrorVector &getError() {
    computeError();
    return _error;
  }

  virtual bool read(std::istream &is) {
    // is >> _measurement[0];
    return true;
  }

  virtual bool write(std::ostream &os) const {
    // os << information()(0,0) << " Error: " << _error[0] << ", Measurement:"
    //    << _measurement[0];
    return os.good();
  }

  void setTebConfig(const TebConfig &cfg) { cfg_ = &cfg; }

protected:
  const TebConfig *cfg_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#endif // EDGE_HUMAN_ROBOT_DIR_H_
