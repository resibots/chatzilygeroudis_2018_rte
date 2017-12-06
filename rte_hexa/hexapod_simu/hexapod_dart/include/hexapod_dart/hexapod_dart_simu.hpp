#ifndef HEXAPOD_DART_SIMU_HPP
#define HEXAPOD_DART_SIMU_HPP

#include <boost/parameter.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/include/accumulate.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/fusion/include/find.hpp>

#include <dart/dart.hpp>
#include <dart/collision/dart/DARTCollisionDetector.hpp>
#include <Eigen/Core>
#include <hexapod_dart/hexapod.hpp>
#include <hexapod_dart/hexapod_control.hpp>
#include <hexapod_dart/safety_measures.hpp>
#include <hexapod_dart/descriptors.hpp>
#include <hexapod_dart/visualizations.hpp>

#ifdef GRAPHIC
#include <dart/gui/osg/osg.hpp>
#endif

namespace hexapod_dart {

    BOOST_PARAMETER_TEMPLATE_KEYWORD(hexapod_control)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(safety)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(desc)
    BOOST_PARAMETER_TEMPLATE_KEYWORD(viz)

    typedef boost::parameter::parameters<boost::parameter::optional<tag::hexapod_control>,
        boost::parameter::optional<tag::safety>,
        boost::parameter::optional<tag::desc>,
        boost::parameter::optional<tag::viz>> class_signature;

    template <typename Simu, typename robot>
    struct Refresh {
        Refresh(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
            : _simu(simu), _robot(rob), _init_trans(init_trans) {}

        Simu& _simu;
        std::shared_ptr<robot> _robot;
        Eigen::Vector6d _init_trans;

        template <typename T>
        void operator()(T& x) const { x(_simu, _robot, _init_trans); }
    };

    template <class A1 = boost::parameter::void_, class A2 = boost::parameter::void_, class A3 = boost::parameter::void_, class A4 = boost::parameter::void_>
    class HexapodDARTSimu {
    public:
        using robot_t = std::shared_ptr<Hexapod>;
        // defaults
        struct defaults {
            using hexapod_control_t = HexapodControl;
            using safety_measures_t = boost::fusion::vector<safety_measures::MaxHeight>;
            using descriptors_t = boost::fusion::vector<descriptors::DutyCycle>;
            using viz_t = boost::fusion::vector<visualizations::HeadingArrow>;
        };

        // extract the types
        using args = typename class_signature::bind<A1, A2, A3, A4>::type;
        using hexapod_control_t = typename boost::parameter::binding<args, tag::hexapod_control, typename defaults::hexapod_control_t>::type;
        using SafetyMeasures = typename boost::parameter::binding<args, tag::safety, typename defaults::safety_measures_t>::type;
        using Descriptors = typename boost::parameter::binding<args, tag::desc, typename defaults::descriptors_t>::type;
        using Visualizations = typename boost::parameter::binding<args, tag::viz, typename defaults::viz_t>::type;
        using safety_measures_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<SafetyMeasures>, SafetyMeasures, boost::fusion::vector<SafetyMeasures>>::type;
        using descriptors_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Descriptors>, Descriptors, boost::fusion::vector<Descriptors>>::type;
        using viz_t = typename boost::mpl::if_<boost::fusion::traits::is_sequence<Visualizations>, Visualizations, boost::fusion::vector<Visualizations>>::type;

        HexapodDARTSimu(const std::vector<double>& ctrl, robot_t robot) : _covered_distance(0.0),
                                                                          _energy(0.0),
                                                                          _world(std::make_shared<dart::simulation::World>()),
                                                                          _controller(ctrl, robot),
                                                                          _old_index(0),
                                                                          _desc_period(2),
                                                                          _break(false)
        {
            _world->getConstraintSolver()->setCollisionDetector(dart::collision::DARTCollisionDetector::create());
            _robot = robot;
            // set position of hexapod
            _robot->skeleton()->setPosition(5, 0.2);
            _add_floor();
            _world->addSkeleton(_robot->skeleton());
            _world->setTimeStep(0.015);

            std::vector<double> c_tmp(36, 0.0);
            _controller.set_parameters(c_tmp);
            _stabilize_robot(true);
            _world->setTime(0.0);
            _controller.set_parameters(ctrl);

#ifdef GRAPHIC
            _fixed_camera = false;
            _osg_world_node = new dart::gui::osg::WorldNode(_world);
            _osg_viewer.addWorldNode(_osg_world_node);
            _osg_viewer.setUpViewInWindow(0, 0, 640, 480);
// full-screen
// _osg_viewer.setUpViewOnSingleScreen();
#endif
        }

        ~HexapodDARTSimu() {}

        void run(double duration = 5.0, bool continuous = false, bool chain = false)
        {
            _break = false;
            robot_t rob = this->robot();
            double old_t = _world->getTime();
            size_t index = _old_index;

            // TO-DO: maybe wee need better solution for this/reset them?
            static Eigen::Vector6d init_trans = rob->pose();

#ifdef GRAPHIC
            while ((_world->getTime() - old_t) < duration && !_osg_viewer.done())
#else
            while ((_world->getTime() - old_t) < duration)
#endif
            {
                _controller.update(chain ? (_world->getTime() - old_t) : _world->getTime());

                _world->step(false);

                // integrate Torque (force) over time
                Eigen::VectorXd state = rob->skeleton()->getForces().array().abs() * _world->getTimeStep();
                _energy += state.sum();

                // update safety measures
                boost::fusion::for_each(_safety_measures, Refresh<HexapodDARTSimu, Hexapod>(*this, rob, init_trans));
                // update visualizations
                boost::fusion::for_each(_visualizations, Refresh<HexapodDARTSimu, Hexapod>(*this, rob, init_trans));

                if (_break) {
                    _covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }

#ifdef GRAPHIC
                if (!_fixed_camera) {
                    auto COM = rob->skeleton()->getCOM();
                    // set camera to follow hexapod
                    _osg_viewer.getCameraManipulator()->setHomePosition(
                        osg::Vec3d(-0.5, 3, 1), osg::Vec3d(COM(0), COM(1), COM(2)), osg::Vec3d(0, 0, 1));
                    _osg_viewer.home();
                }
                // process next frame
                _osg_viewer.frame();
#endif

                if (index % _desc_period == 0) {
                    // update descriptors
                    boost::fusion::for_each(_descriptors, Refresh<HexapodDARTSimu, Hexapod>(*this, rob, init_trans));
                }

                ++index;
            }
            _old_index = index;

            if (!continuous) {
                if (!_stabilize_robot()) {
                    _covered_distance = -10002.0;
                    _arrival_angle = -10002.0;
                    _energy = -10002.0;
                    return;
                }
            }

            // Position computation
            Eigen::Vector6d pose = rob->pose();
            Eigen::Matrix3d rot = dart::math::expMapRot({pose[0], pose[1], pose[2]});
            Eigen::Matrix3d init_rot = dart::math::expMapRot({init_trans[0], init_trans[1], init_trans[2]});
            Eigen::MatrixXd init_homogeneous(4, 4);
            init_homogeneous << init_rot(0, 0), init_rot(0, 1), init_rot(0, 2), init_trans[3], init_rot(1, 0), init_rot(1, 1), init_rot(1, 2), init_trans[4], init_rot(2, 0), init_rot(2, 1), init_rot(2, 2), init_trans[5], 0, 0, 0, 1;
            Eigen::MatrixXd final_homogeneous(4, 4);
            final_homogeneous << rot(0, 0), rot(0, 1), rot(0, 2), pose[3], rot(1, 0), rot(1, 1), rot(1, 2), pose[4], rot(2, 0), rot(2, 1), rot(2, 2), pose[5], 0, 0, 0, 1;
            Eigen::Vector4d pos = {init_trans[3], init_trans[4], init_trans[5], 1.0};
            pos = init_homogeneous.inverse() * final_homogeneous * pos;

            _final_pos = Eigen::Vector3d(pos(0), pos(1), pos(2));

            _covered_distance = std::round(_final_pos(0) * 100) / 100.0;

            // Angle computation
            _final_rot = dart::math::matrixToEulerXYZ(init_rot.inverse() * rot);

            // roll-pitch-yaw
            _arrival_angle = std::round(_final_rot(2) * 100) / 100.0;
        }

        robot_t robot()
        {
            return _robot;
        }

        dart::simulation::WorldPtr world()
        {
            return _world;
        }

#ifdef GRAPHIC
        void fixed_camera(const Eigen::Vector3d& camera_pos, const Eigen::Vector3d& look_at = Eigen::Vector3d(0, 0, 0), const Eigen::Vector3d& up = Eigen::Vector3d(0, 0, 1))
        {
            _fixed_camera = true;
            _camera_pos = camera_pos;
            _look_at = look_at;
            _camera_up = up;

            // set camera position
            _osg_viewer.getCameraManipulator()->setHomePosition(
                osg::Vec3d(_camera_pos(0), _camera_pos(1), _camera_pos(2)), osg::Vec3d(_look_at(0), _look_at(1), _look_at(2)), osg::Vec3d(_camera_up(0), _camera_up(1), _camera_up(2)));
            _osg_viewer.home();
        }

        void follow_hexapod()
        {
            _fixed_camera = false;
        }
#endif

        template <typename Desc, typename T>
        void get_descriptor(T& result)
        {
            auto d = boost::fusion::find<Desc>(_descriptors);
            (*d).get(result);
        }

        double covered_distance() const
        {
            return _covered_distance;
        }

        double energy() const
        {
            return _energy;
        }

        double arrival_angle() const
        {
            return _arrival_angle;
        }

        const Eigen::Vector3d& final_pos() const
        {
            return _final_pos;
        }

        const Eigen::Vector3d& final_rot() const
        {
            return _final_rot;
        }

        double step() const
        {
            assert(_world != nullptr);
            return _world->getTimeStep();
        }

        void set_step(double step)
        {
            assert(_world != nullptr);
            _world->setTimeStep(step);
        }

        size_t desc_dump() const
        {
            return _desc_period;
        }

        void set_desc_dump(size_t desc_dump)
        {
            _desc_period = desc_dump;
        }

        void stop_sim(bool disable = true)
        {
            _break = disable;
        }

        hexapod_control_t& controller()
        {
            return _controller;
        }

        // pose: Orientation-Position, dims: XYZ
        void add_box(const Eigen::Vector6d& pose, const Eigen::Vector3d& dims, std::string type = "free", double mass = 1.0, const Eigen::Vector4d& color = dart::Color::Red(1.0), const std::string& box_name = "box")
        {
            std::string name = box_name;
            // We do not want boxes with the same names!
            while (_world->getSkeleton(name) != nullptr) {
                if (name[name.size() - 2] == '_') {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else {
                    name = name + "_1";
                }
            }

            dart::dynamics::SkeletonPtr box_skel = dart::dynamics::Skeleton::create(name);

            // Give the box a body
            dart::dynamics::BodyNodePtr body;
            if (type == "free")
                body = box_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
            else
                body = box_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            body->setMass(mass);
            body->setName(name);

            // Give the body a shape
            auto box = std::make_shared<dart::dynamics::BoxShape>(dims);
            auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
            box_node->getVisualAspect()->setColor(color);

            // Put the body into position
            if (type == "free") // free floating
                box_skel->setPositions(pose);
            else // fixed
                body->getParentJoint()->setTransformFromParentBodyNode(dart::math::expMap(pose));

            _world->addSkeleton(box_skel);
            _objects.push_back(box_skel);
        }

        // pose: Orientation-Position, dims: XYZ
        void add_ellipsoid(const Eigen::Vector6d& pose, const Eigen::Vector3d& dims, std::string type = "free", double mass = 1.0, const Eigen::Vector4d& color = dart::Color::Red(1.0), const std::string& ellipsoid_name = "sphere")
        {
            std::string name = ellipsoid_name;
            // We do not want ellipsoids with the same names!
            while (_world->getSkeleton(name) != nullptr) {
                if (name[name.size() - 2] == '_') {
                    int i = name.back() - '0';
                    i++;
                    name.pop_back();
                    name = name + std::to_string(i);
                }
                else {
                    name = name + "_1";
                }
            }

            dart::dynamics::SkeletonPtr ellipsoid_skel = dart::dynamics::Skeleton::create(name);

            // Give the ellipsoid a body
            dart::dynamics::BodyNodePtr body;
            if (type == "free")
                body = ellipsoid_skel->createJointAndBodyNodePair<dart::dynamics::FreeJoint>(nullptr).second;
            else
                body = ellipsoid_skel->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
            body->setMass(mass);
            body->setName(name);

            // Give the body a shape
            auto ellipsoid = std::make_shared<dart::dynamics::EllipsoidShape>(dims);
            auto ellipsoid_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(ellipsoid);
            ellipsoid_node->getVisualAspect()->setColor(color);

            // Put the body into position
            if (type == "free") // free floating
                ellipsoid_skel->setPositions(pose);
            else // fixed
                body->getParentJoint()->setTransformFromParentBodyNode(dart::math::expMap(pose));

            _world->addSkeleton(ellipsoid_skel);
            _objects.push_back(ellipsoid_skel);
        }

        void clear_objects()
        {
            for (auto obj : _objects) {
                _world->removeSkeleton(obj);
            }
            _objects.clear();
        }

    protected:
        bool _stabilize_robot(bool update_ctrl = false)
        {
            robot_t rob = this->robot();

            bool stabilized = false;
            int stab = 0;

            if (update_ctrl)
                _world->setTimeStep(0.001);

            for (size_t s = 0; s < 1000 && !stabilized; ++s) {
                Eigen::Vector6d prev_pose = rob->pose();

                if (update_ctrl)
                    _controller.update(_world->getTime());
                else
                    _controller.set_commands();
                _world->step();

                if ((rob->pose() - prev_pose).norm() < 1e-4)
                    stab++;
                else
                    stab = 0;
                if (stab > 30)
                    stabilized = true;
            }

            if (update_ctrl)
                _world->setTimeStep(0.015);

            return stabilized;
        }

        void _add_floor()
        {
            // We do not want 2 floors!
            if (_world->getSkeleton("floor") != nullptr)
                return;

            dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");

            // Give the floor a body
            dart::dynamics::BodyNodePtr body = floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

            // Give the body a shape
            double floor_width = 10.0;
            double floor_height = 0.1;
            auto box = std::make_shared<dart::dynamics::BoxShape>(Eigen::Vector3d(floor_width, floor_width, floor_height));
            auto box_node = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
            box_node->getVisualAspect()->setColor(dart::Color::Gray());

            // Put the body into position
            Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
            tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
            body->getParentJoint()->setTransformFromParentBodyNode(tf);

            _world->addSkeleton(floor);
        }

        robot_t _robot;
        Eigen::Vector3d _final_pos;
        Eigen::Vector3d _final_rot;
        double _arrival_angle;
        double _covered_distance;
        double _energy;
        dart::simulation::WorldPtr _world;
        hexapod_control_t _controller;
        size_t _old_index;
        size_t _desc_period;
        bool _break;
        safety_measures_t _safety_measures;
        descriptors_t _descriptors;
        viz_t _visualizations;
        std::vector<dart::dynamics::SkeletonPtr> _objects;
#ifdef GRAPHIC
        bool _fixed_camera;
        Eigen::Vector3d _look_at;
        Eigen::Vector3d _camera_pos;
        Eigen::Vector3d _camera_up;
        osg::ref_ptr<dart::gui::osg::WorldNode> _osg_world_node;
        dart::gui::osg::Viewer _osg_viewer;
#endif
    };
}

#endif
