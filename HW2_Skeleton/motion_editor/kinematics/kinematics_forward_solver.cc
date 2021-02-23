#include "kinematics_forward_solver.h"
#include <algorithm>
#include "math_utils.h"
#include "acclaim_skeleton.h"
#include "acclaim_motion.h"
#include "helper_forward_kinematics.h"
#include "kinematics_artic_idx.h"
#include "kinematics_pose.h"
//using namespace Eigen;

namespace kinematics {

// public func.

ForwardSolver::ForwardSolver()
    :skeleton_(nullptr),
    motion_(nullptr),
    artic_path_(new ArticIdxColl_t),
    helper_fk_(new helper::ForwardKinematics)
    
{
}

ForwardSolver::~ForwardSolver()
{
}

std::shared_ptr<acclaim::Skeleton> ForwardSolver::skeleton() const
{
    return skeleton_;
}

std::shared_ptr<acclaim::Motion> ForwardSolver::motion() const
{
    return motion_;
}

void ForwardSolver::set_skeleton(const std::shared_ptr<acclaim::Skeleton> &skeleton)
{
    skeleton_ = skeleton;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::set_motion(const std::shared_ptr<acclaim::Motion> &motion)
{
    motion_ = motion;
    helper_fk_->set_skeleton(skeleton_);
}

void ForwardSolver::ConstructArticPath()
{
    //artic_path_ = ;
    helper_fk_->ConstructArticPath();
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const int32_t frame_idx)
{
    //std::cout << frame_idx;
    return this->ComputeSkeletonPose(motion_->joint_spatial_pos(frame_idx));
}

PoseColl_t ForwardSolver::ComputeSkeletonPose(const math::Vector6dColl_t &joint_spatial_pos)
{
    // TO DO
    PoseColl_t AfterPose;
    Pose NewPose;
    math::Vector6dColl_t DegreetoRadian = math::ToRadian(joint_spatial_pos); //because ComputeRotMatXyz() need use radianX , radianY , radianZ
    math::RotMat3d_t RotationAmc = math::ComputeRotMatXyz(DegreetoRadian[0].x(), DegreetoRadian[0].y(), DegreetoRadian[0].z());
    NewPose.set_start_pos( math::Vector3d_t(joint_spatial_pos[0].tail(3)) ); //linear vector
    math::RotMat3d_t RotationAsf;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            RotationAsf(i, j) = skeleton_->root_bone()->rot_parent_current[i][j];
        }
    }
    math::RotMat3d_t TransformRot = RotationAsf * RotationAmc;
    math::Vector3d_t joint = skeleton_->bone_local_dir(0) * skeleton_->bone_length(0);
    joint = TransformRot * joint;
    NewPose.set_rotation(TransformRot);
    NewPose.set_end_pos( NewPose.start_pos() + joint );
    AfterPose.push_back(NewPose);

    for (int index = 1; index < joint_spatial_pos.size(); index++) {
        RotationAmc = math::ComputeRotMatXyz(DegreetoRadian[index].x(), DegreetoRadian[index].y(), DegreetoRadian[index].z());
        NewPose.set_start_pos(AfterPose[skeleton_->bone_ptr(index)->parent->idx].end_pos());
        math::RotMat3d_t Before = AfterPose[skeleton_->bone_ptr(index)->parent->idx].rotation();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                RotationAsf(i, j) = skeleton_->bone_ptr(index)->rot_parent_current[i][j];
            }
            
        }
        RotationAsf = Before * RotationAsf.transpose(); 
        TransformRot = RotationAsf * RotationAmc;
        joint = skeleton_->bone_local_dir(index) * skeleton_->bone_length(index);
        joint = TransformRot * joint;
        NewPose.set_rotation(TransformRot);
        NewPose.set_end_pos(NewPose.start_pos() + joint);
        AfterPose.push_back(NewPose);
    }
    return AfterPose;
    //return helper_fk_->ComputeSkeletonPose(joint_spatial_pos);
    
}

// protected func.

// private func.

} // namespace kinematics {
