#ifndef HEXAPOD_DART_SAFETY_MEASURES_HPP
#define HEXAPOD_DART_SAFETY_MEASURES_HPP

namespace hexapod_dart {

    namespace safety_measures {

        struct BodyColliding {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                const dart::collision::CollisionResult& col_res = simu.world()->getLastCollisionResult();
                auto body = rob->skeleton()->getRootBodyNode();
                if (col_res.inCollision(body))
                    simu.stop_sim();
            }
        };

        struct MaxHeight {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                auto COM = rob->skeleton()->getCOM();
                if (std::abs(COM(2)) > 0.3)
                    simu.stop_sim();
            }

        protected:
            double _max_height;
        };

        struct TurnOver {
        public:
            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                auto rot_mat = dart::math::expMapRot(rob->rot());
                Eigen::Vector3d z_axis = {0.0, 0.0, 1.0};
                Eigen::Vector3d robot_z_axis = rot_mat * z_axis;
                double z_angle = std::atan2((z_axis.cross(robot_z_axis)).norm(), z_axis.dot(robot_z_axis));
                if (std::abs(z_angle) >= dart::math::constants<double>::half_pi())
                    simu.stop_sim();
            }

        protected:
            double _max_angle;
        };
    }
}

#endif
