#include "kinematics_time_warper.h"
#include <utility>
#include "boost/numeric/conversion/cast.hpp"
#include "math_utils.h"

namespace kinematics {

// public func.

TimeWarper::TimeWarper()
    :original_motion_sequence_(new math::SpatialTemporalVector6d_t),
    hard_constraint_coll_(new TimeWarpHardConstraintColl_t),
    time_step_(double{0.0}),
    min_time_step_(double{0.0}),
    max_time_step_(double{0.0})
{
}

TimeWarper::~TimeWarper()
{
}

double TimeWarper::time_step() const
{
    return time_step_;
}

double TimeWarper::min_time_step() const
{
    return min_time_step_;
}

double TimeWarper::max_time_step() const
{
    return max_time_step_;
}

void TimeWarper::Configure(
        const math::SpatialTemporalVector6d_t &original_motion_sequence,
        const double time_step,
        const double min_time_step,
        const double max_time_step
        )
{
    *original_motion_sequence_ = original_motion_sequence;
    time_step_ = time_step;
    min_time_step_ = min_time_step;
    max_time_step_ = max_time_step;
}

math::SpatialTemporalVector6d_t TimeWarper::ComputeWarpedMotion(
    const TimeWarpHardConstraintColl_t& hard_constraint_coll
)
{
    // TO DO
    *hard_constraint_coll_ = hard_constraint_coll;

    math::SpatialTemporalVector6d_t New_motion_sequence;
    New_motion_sequence.Resize(original_motion_sequence_->spatial_size(), original_motion_sequence_->temporal_size());
    for (int i = 0; i < original_motion_sequence_->spatial_size(); i++) {
        math::Vector6d_t start;
        math::Vector6d_t middle;
        math::Vector6d_t end;
        start.set_angular_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(0).frame_idx].angular_vector());
        start.set_linear_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(0).frame_idx].linear_vector());
        middle.set_angular_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(1).frame_idx].angular_vector());
        middle.set_linear_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(1).frame_idx].linear_vector());
        end.set_angular_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(2).frame_idx].angular_vector());
        end.set_linear_vector(original_motion_sequence_->spatial_elements(i)[hard_constraint_coll_->at(2).frame_idx].linear_vector());
        New_motion_sequence.set_element(i, hard_constraint_coll_->at(0).frame_idx, start);
        New_motion_sequence.set_element(i, hard_constraint_coll_->at(1).frame_idx, middle);
        New_motion_sequence.set_element(i, hard_constraint_coll_->at(2).frame_idx, end);
    }
    for (int i = 1; i < hard_constraint_coll_->size(); i++) {
        TimeWarpHardConstraint_t Current = hard_constraint_coll_->at(i);
        TimeWarpHardConstraint_t Before = hard_constraint_coll_->at(i-1);
        double step = (Current.play_second - Before.play_second) / (Current.frame_idx - Before.frame_idx);
        for (int frame = 1; frame < Current.frame_idx; frame++) {
            for (int num = 0; num < original_motion_sequence_->spatial_size(); num++) {
                
            }
        }

    }
    
    //return New_motion_sequence;
        return *original_motion_sequence_;
}

// protected func.

// private func.

} // namespace kinematics {
