#ifndef HEXAPOD_DART_VISUALIZATIONS_HPP
#define HEXAPOD_DART_VISUALIZATIONS_HPP

#include <Eigen/Core>

namespace hexapod_dart {
    namespace visualizations {

        struct HeadingArrow {
        public:
            HeadingArrow() : init(false) {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                if (!init) {
                    dart::dynamics::ArrowShape::Properties arrow_properties;
                    arrow_properties.mRadius = 0.01;
                    _arrow = std::shared_ptr<dart::dynamics::ArrowShape>(new dart::dynamics::ArrowShape(
                        rob->pos(),
                        rob->pos(),
                        arrow_properties, dart::Color::Orange(1.0)));
                    auto bn = rob->skeleton()->getBodyNode("base_link");
                    bn->template createShapeNodeWith<dart::dynamics::VisualAspect>(_arrow);
                    init = true;
                }

                // roll-pitch-yaw
                Eigen::Matrix3d rot = dart::math::expMapRot(rob->rot());
                Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
                auto rpy = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);
                double angle = rpy(2);

                _arrow->setPositions(
                    Eigen::Vector3d(0, 0, 0),
                    Eigen::Vector3d(std::cos(angle) * 0.5, std::sin(angle) * 0.5, 0));
            }

        protected:
            std::shared_ptr<dart::dynamics::ArrowShape> _arrow;
            bool init;
        };

        template <typename Params>
        struct PointingArrow {
        public:
            PointingArrow() : init(false) {}

            template <typename Simu, typename robot>
            void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            {
                if (!init) {
                    dart::dynamics::ArrowShape::Properties arrow_properties;
                    arrow_properties.mRadius = Params::radius();
                    _arrow = std::shared_ptr<dart::dynamics::ArrowShape>(new dart::dynamics::ArrowShape(
                        Params::head(),
                        Params::tail(),
                        arrow_properties, Params::color()));
                    auto bn = simu.world()->getSkeleton(Params::skel_name())->getBodyNode(Params::body_name());
                    bn->template createShapeNodeWith<dart::dynamics::VisualAspect>(_arrow);
                    init = true;
                }

                _arrow->setPositions(Params::head(), Params::tail());
            }

        protected:
            std::shared_ptr<dart::dynamics::ArrowShape> _arrow;
            bool init;
        };
    }
}

#endif
