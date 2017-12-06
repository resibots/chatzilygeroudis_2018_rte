#ifndef HEXAPOD_DART_DESCRIPTORS_HPP
#define HEXAPOD_DART_DESCRIPTORS_HPP

#include <algorithm>
#include <map>
#include <vector>
#include <numeric>

#include <Eigen/Core>

#include <hexapod_dart/hexapod.hpp>

namespace hexapod_dart {

    namespace descriptors {

        struct DescriptorBase {
        public:
            using robot_t = std::shared_ptr<Hexapod>;

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                assert(false);
            }

            template <typename T>
            void get(T& results)
            {
                assert(false);
            }
        };

        struct DutyCycle : public DescriptorBase {
        public:
            DutyCycle()
            {
                for (size_t i = 0; i < 6; i++)
                    _contacts[i] = std::vector<size_t>();
            }

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                const dart::collision::CollisionResult& col_res = simu.world()->getLastCollisionResult();
                for (size_t i = 0; i < 6; ++i) {
                    std::string leg_name = "leg_" + std::to_string(i) + "_3";
                    dart::dynamics::BodyNodePtr body_to_check = rob->skeleton()->getBodyNode(leg_name);

                    if (rob->is_broken(i)) {
                        _contacts[i].push_back(0);
                    }
                    else {
                        _contacts[i].push_back(col_res.inCollision(body_to_check));
                    }
                }
            }

            void get(std::vector<double>& results)
            {
                for (size_t i = 0; i < 6; i++) {
                    results.push_back(std::round(std::accumulate(_contacts[i].begin(), _contacts[i].end(), 0.0) / double(_contacts[i].size()) * 100.0) / 100.0);
                }
            }

        protected:
            std::map<size_t, std::vector<size_t>> _contacts;
        };

        struct PositionTraj : public DescriptorBase {
        public:
            PositionTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                Eigen::Vector6d pose = rob->pose();
                Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
                Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                Eigen::MatrixXd init_homogeneous(4, 4);
                init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
                Eigen::MatrixXd final_homogeneous(4, 4);
                final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
                Eigen::Vector4d pos = {init_trans[3], init_trans[4], init_trans[5], 1.0};
                pos = init_homogeneous.inverse() * final_homogeneous * pos;

                _pos_traj.push_back({pos[0], pos[1], pos[2]});
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _pos_traj;
            }

        protected:
            std::vector<Eigen::Vector3d> _pos_traj;
        };

        struct RotationTraj : public DescriptorBase {
        public:
            RotationTraj() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                // roll-pitch-yaw
                Eigen::Matrix3d rot = dart::math::expMapRot(rob->rot());
                Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                auto rpy = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

                _rotation_traj.push_back(rpy);
            }

            void get(std::vector<Eigen::Vector3d>& results)
            {
                results = _rotation_traj;
            }

        protected:
            std::vector<Eigen::Vector3d> _rotation_traj;
        };

        struct BodyOrientation : public DescriptorBase {
        public:
            BodyOrientation() {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                // roll-pitch-yaw
                Eigen::Matrix3d rr = dart::math::expMapRot(rob->rot());
                Eigen::Matrix3d ro = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                auto rpy = dart::math::matrixToEulerXYZ(ro.inverse() * rr);

                _roll_vec.push_back(rpy(0));
                _pitch_vec.push_back(rpy(1));
                _yaw_vec.push_back(rpy(2));
            }

            void get(std::vector<double>& results)
            {
                double threshold = (_perc_threshold / 100.0) * dart::math::constants<double>::pi();
                results.clear();
                results.push_back(std::round(std::count_if(_roll_vec.begin(), _roll_vec.end(), [&threshold](double i) {return i>threshold; }) / double(_roll_vec.size()) * 100.0) / 100.0);
                results.push_back(std::round(std::count_if(_roll_vec.begin(), _roll_vec.end(), [&threshold](double i) {return i<-threshold; }) / double(_roll_vec.size()) * 100.0) / 100.0);
                results.push_back(std::round(std::count_if(_pitch_vec.begin(), _pitch_vec.end(), [&threshold](double i) {return i>threshold; }) / double(_pitch_vec.size()) * 100.0) / 100.0);
                results.push_back(std::round(std::count_if(_pitch_vec.begin(), _pitch_vec.end(), [&threshold](double i) {return i<-threshold; }) / double(_pitch_vec.size()) * 100.0) / 100.0);
                results.push_back(std::round(std::count_if(_yaw_vec.begin(), _yaw_vec.end(), [&threshold](double i) {return i>threshold; }) / double(_yaw_vec.size()) * 100.0) / 100.0);
                results.push_back(std::round(std::count_if(_yaw_vec.begin(), _yaw_vec.end(), [&threshold](double i) {return i<-threshold; }) / double(_yaw_vec.size()) * 100.0) / 100.0);
            }

        protected:
            // We count the time the robot's root orientation has exceeded a threshold angle in every direction.
            // This threshold angle is _perc_threshold*pi (empirically chosen).
            const double _perc_threshold = 0.5;

            std::vector<double> _roll_vec, _pitch_vec, _yaw_vec;
        };
    }
}

#endif
