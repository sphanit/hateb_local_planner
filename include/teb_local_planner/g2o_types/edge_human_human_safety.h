/*/
 * Copyright (c) 2017 LAAS/CNRS
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

#ifndef EDGE_HUMAN_HUMAN_SAFETY_H_
#define EDGE_HUMAN_HUMAN_SAFETY_H_

#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_unary_edge.h"

namespace teb_local_planner {

class EdgeHumanHumanSafety
    : public g2o::BaseBinaryEdge<1, double, VertexPose, VertexPose> {
public:
  EdgeHumanHumanSafety() {
    this->setMeasurement(0.);
    _vertices[0] = _vertices[1] = NULL;
  }

  virtual ~EdgeHumanHumanSafety() {
    for (unsigned int i = 0; i < 2; i++) {
      if (_vertices[i])
        _vertices[i]->edges().erase(this);
    }
  }

  void computeError() {
    ROS_ASSERT_MSG(cfg_ &&
                       human_radius_ < std::numeric_limits<double>::infinity(),
                   "You must call setParameters() on EdgeHumanHumanSafety()");
    const VertexPose *human1_bandpt =
        static_cast<const VertexPose *>(_vertices[0]);
    const VertexPose *human2_bandpt =
        static_cast<const VertexPose *>(_vertices[1]);

    double dist = std::hypot(human1_bandpt->x() - human2_bandpt->x(),
                             human1_bandpt->y() - human2_bandpt->y()) -
                  (2 * human_radius_);

    ROS_DEBUG_THROTTLE(0.5, "human human external dist = %f", dist);
    _error[0] = penaltyBoundFromBelow(dist, cfg_->human.min_human_human_dist,
                                      cfg_->optim.penalty_epsilon);

    ROS_ASSERT_MSG(std::isfinite(_error[0]),
                   "EdgeHumanHumanSafety::computeError() _error[0]=%f\n",
                   _error[0]);
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

  void setHumanRadius(const double human_radius) {
    human_radius_ = human_radius;
  }

  void setTebConfig(const TebConfig &cfg) { cfg_ = &cfg; }

  void setParameters(const TebConfig &cfg, const double human_radius) {
    cfg_ = &cfg;
    human_radius_ = human_radius;
  }

protected:
  const TebConfig *cfg_;
  double human_radius_ = std::numeric_limits<double>::infinity();
  ;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // end namespace

#endif // EDGE_HUMAN_HUMAN_SAFETY_H_
