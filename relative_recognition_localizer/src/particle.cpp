#include "relative_recognition_localizer/mcl.h"

using namespace relative_recognition_localizer;

MCL::Particle::Particle(MCL* mcl) : mcl_(mcl)
{
    mcl_->init_pose(pose_,0.0,0.0,0.0);
    weight_ = 1.0/(double)mcl_->PARTICLES_NUM_;
}

void MCL::Particle::set_pose(double x,double y,double yaw,double x_var,double y_var,double yaw_var)
{
    double tmp_x = mcl_->get_gaussian(x,x_var);
    double tmp_y = mcl_->get_gaussian(y,y_var);
    double tmp_yaw = mcl_->get_gaussian(yaw,yaw_var);
    mcl_->set_pose(pose_,tmp_x,tmp_y,tmp_yaw);
}

void MCL::Particle::move(double dx,double dy,double dyaw)
{
    double yaw = tf2::getYaw(pose_.pose.orientation);

    double delta_trans = std::sqrt(dx*dx + dy*dy);
    double delta_rot1;
    if(delta_trans < 1e-2) delta_rot1 = 0.0;
    else delta_rot1 = dyaw;
    double delta_rot2 = mcl_->get_angle_diff(dyaw,delta_rot1);

    double delta_rot1_noise = std::min(std::fabs(mcl_->get_angle_diff(delta_rot1,0.0)),
                                       std::fabs(mcl_->get_angle_diff(delta_rot1,M_PI)));
    double delta_rot2_noise = std::min(std::fabs(mcl_->get_angle_diff(delta_rot2,0.0)),
                                       std::fabs(mcl_->get_angle_diff(delta_rot2,M_PI)));

    double rot1_sigma = mcl_->ALPHA_1_*delta_rot1_noise*delta_rot1_noise -
                        mcl_->ALPHA_2_*delta_trans*delta_trans;
    double rot2_sigma = mcl_->ALPHA_1_*delta_rot2_noise*delta_rot2_noise -
                        mcl_->ALPHA_2_*delta_trans*delta_trans;
    double trans_sigma = mcl_->ALPHA_3_*delta_trans*delta_trans +
                         mcl_->ALPHA_4_*delta_rot1_noise*delta_rot1_noise +
                         mcl_->ALPHA_4_*delta_rot2_noise*delta_rot2_noise;

    double delta_rot1_hat = mcl_->get_angle_diff(delta_rot1,mcl_->get_gaussian(0.0,rot1_sigma));
    double delta_rot2_hat = mcl_->get_angle_diff(delta_rot2,mcl_->get_gaussian(0.0,rot2_sigma));
    double delta_trans_hat = delta_trans - mcl_->get_gaussian(0.0,trans_sigma);

    double tmp_x = pose_.pose.position.x + delta_trans_hat*std::cos(yaw + delta_rot1_hat);
    double tmp_y = pose_.pose.position.y + delta_trans_hat*std::sin(yaw + delta_rot1_hat);
    double tmp_yaw = yaw + delta_rot1_hat + delta_rot2_hat;
    mcl_->set_pose(pose_,tmp_x,tmp_y,tmp_yaw);
}