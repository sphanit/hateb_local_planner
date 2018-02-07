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
 * Author: Guilhem Buisan (buisan@laas.fr)
 */

#ifndef EDGE_HUMAN_ROBOT_VISIBILITY_H_
#define EDGE_HUMAN_ROBOT_VISIBILITY_H_

#include <teb_local_planner/g2o_types/vertex_pose.h>
#include <teb_local_planner/g2o_types/penalties.h>
#include <teb_local_planner/teb_config.h>

#include "g2o/core/base_unary_edge.h"

namespace teb_local_planner {

    class EdgeHumanRobotVisibility
            : public g2o::BaseBinaryEdge<1, double, VertexPose, VertexPose> {
    public:
        EdgeHumanRobotVisibility() {
            this->setMeasurement(0.);
            _vertices[0] = _vertices[1] = NULL;
        }

        virtual ~EdgeHumanRobotVisibility() {
            for (unsigned int i = 0; i < 2; i++) {
                if (_vertices[i])
                    _vertices[i]->edges().erase(this);
            }
        }

        void computeError() {
            ROS_ASSERT_MSG(cfg_,
                           "You must call setParameters() on EdgeHumanRobotVisibility()");
            const VertexPose *robot_bandpt =
                    static_cast<const VertexPose *>(_vertices[0]);
            const VertexPose *human_bandpt =
                    static_cast<const VertexPose *>(_vertices[1]);
            Eigen::Vector2d d_rtoh =
                    human_bandpt->position() - robot_bandpt->position();
            Eigen::Vector2d d_htor = robot_bandpt->position() - human_bandpt->position();
            Eigen::Vector2d humanLookAt = {cos(human_bandpt->theta()), sin(human_bandpt->theta())};
            double deltaPsi = fabs(acos(humanLookAt.dot(d_htor) / (humanLookAt.norm() + d_htor.norm())));
            //ROS_DEBUG_THROTTLE(0.5, "robot_human_angle deg : %f", deltaPsi * 180 / M_PI);
            double c_visibility;
            if (deltaPsi >= cfg_->human.fov * M_PI / 180){
                c_visibility = deltaPsi * ((cos(d_rtoh.x()) + 1) * (cos(d_rtoh.y()) + 1));
            }else{
                c_visibility = 0.;
            }
//            double robot_human_angle = center_radians(atan2(d_rtoh.y(), d_rtoh.x()) - human_bandpt->theta());
//
//            double c_visibility = fmaxf(0.,
//                                           (pow(robot_human_angle, 2) - cfg_->human.fov / 2)
//                                           / d_rtoh.dot(d_rtoh));

            //ROS_DEBUG("c_visibility = %f", c_visibility);
            _error[0] = penaltyBoundFromAbove(c_visibility, cfg_->human.visibility_cost_threshold,
                                              cfg_->optim.penalty_epsilon);

            ROS_ASSERT_MSG(std::isfinite(_error[0]),
                           "EdgeHumanRobotVisibility::computeError() _error[0]=%f\n",
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

        void setTebConfig(const TebConfig &cfg) { cfg_ = &cfg; }

        void setParameters(const TebConfig &cfg) {
            cfg_ = &cfg;
        }

    protected:
        const TebConfig *cfg_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // end namespace

#endif