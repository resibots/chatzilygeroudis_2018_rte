#include <limbo/limbo.hpp>
#include <map_elites/binary_map.hpp>
#include <hexapod_dart/hexapod_dart_simu.hpp>
#include <mcts/uct.hpp>
#include <svg/simple_svg.hpp>
#include <boost/program_options.hpp>
#include <astar/a_star.hpp>
#include <algorithm>
#include <vector>
#include <chrono>

#ifdef ROBOT
#include <ros/ros.h>
#include <hexapod_driver/hexapod.hpp>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <thread>
#endif

#ifdef LOW_DIM
#define ARCHIVE_SIZE 2
#else
#define ARCHIVE_SIZE 8
#endif

using namespace limbo;

template <typename T>
inline T gaussian_rand(T m = 0.0, T v = 1.0)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::normal_distribution<T> gaussian(m, v);

    return gaussian(gen);
}

// b-a
double angle_dist(double a, double b)
{
    double theta = b - a;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    return theta;
}

svg::Color get_color(double v, double vmin = 0.0, double vmax = 1.0)
{
    double r = 1.0, g = 1.0, b = 1.0;
    double dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        r = 0;
        g = 4 * (v - vmin) / dv;
    }
    else if (v < (vmin + 0.5 * dv)) {
        r = 0;
        b = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
    }
    else if (v < (vmin + 0.75 * dv)) {
        r = 4 * (v - vmin - 0.5 * dv) / dv;
        b = 0;
    }
    else {
        g = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
        b = 0;
    }

    return svg::Color(r * 255, g * 255, b * 255);
}

struct VizParams {
    static constexpr double radius() { return 0.05; }

    BO_DYN_PARAM(Eigen::Vector3d, head);

    BO_DYN_PARAM(Eigen::Vector3d, tail);

    static Eigen::Vector4d color() { return Eigen::Vector4d(0, 1, 0, 1); }

    static std::string skel_name() { return "floor"; }

    static std::string body_name() { return "BodyNode"; }
};

struct Params {
    struct uct {
        MCTS_DYN_PARAM(double, c);
    };

    struct spw {
        MCTS_DYN_PARAM(double, a);
    };

    struct cont_outcome {
        MCTS_DYN_PARAM(double, b);
    };

    struct mcts_node {
        MCTS_DYN_PARAM(size_t, parallel_roots);
    };

    struct active_learning {
        MCTS_DYN_PARAM(double, k);
        MCTS_DYN_PARAM(double, scaling);
    };

    MCTS_DYN_PARAM(double, goal_x);
    MCTS_DYN_PARAM(double, goal_y);
    MCTS_DYN_PARAM(double, goal_theta);
    MCTS_DYN_PARAM(double, iterations);
    MCTS_DYN_PARAM(bool, learning);
    MCTS_DYN_PARAM(size_t, collisions);
    MCTS_PARAM(double, threshold, 1e-2);
#ifndef ROBOT
    MCTS_PARAM(double, cell_size, 0.5);
    MCTS_PARAM(double, robot_radius, 0.125);
#else
    MCTS_PARAM(double, cell_size, 0.4);
    MCTS_PARAM(double, robot_radius, 0.125);
#endif

    struct kernel_exp {
        BO_DYN_PARAM(double, l);
        BO_DYN_PARAM(double, sigma_sq);
    };

    struct archiveparams {
        struct elem_archive {
            double x, y, cos_theta, sin_theta;
            std::vector<double> controller;
        };

        struct classcomp {
            bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const
            {
                assert(lhs.size() == ARCHIVE_SIZE && rhs.size() == ARCHIVE_SIZE);
                int i = 0;
                while (i < (ARCHIVE_SIZE - 1) && std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000)) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 1000) < std::round(rhs[i] * 1000); //lhs[i]<rhs[i];
            }
        };

        struct classcompequal {
            bool operator()(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs) const
            {
                assert(lhs.size() == ARCHIVE_SIZE && rhs.size() == ARCHIVE_SIZE);
                int i = 0;
                while (i < (ARCHIVE_SIZE - 1) && std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000)) //lhs[i]==rhs[i])
                    i++;
                return std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000); //lhs[i]<rhs[i];
            }
        };

        using archive_t = std::map<std::vector<double>, elem_archive, classcomp>;
        static archive_t archive;
    };
};

template <typename Params>
struct MeanArchive {
    MeanArchive(size_t dim_out = 4)
    {
    }
    template <typename GP>
    Eigen::VectorXd operator()(const Eigen::VectorXd& v, const GP&) const
    {
        Eigen::VectorXd r(4);
        std::vector<double> vv(v.size(), 0.0);
        Eigen::VectorXd::Map(&vv[0], v.size()) = v;
        typename Params::archiveparams::elem_archive elem = Params::archiveparams::archive[vv];
        r << elem.x, elem.y, elem.cos_theta, elem.sin_theta;
        return r;
    }
};

struct SimpleObstacle {
    double _x, _y, _radius, _radius_sq;

    SimpleObstacle(double x, double y, double r = 0.1) : _x(x), _y(y), _radius(r), _radius_sq(r * r) {}
};

using kernel_t = kernel::Exp<Params>;
using mean_t = MeanArchive<Params>;
using GP_t = model::GP<Params, kernel_t, mean_t>;

bool collides(double x, double y, double r = Params::robot_radius());

struct HexaColliding {
public:
    template <typename Simu, typename robot>
    void operator()(Simu& simu, std::shared_ptr<robot> rob, const Eigen::Vector6d& init_trans)
    {
        const dart::collision::CollisionResult& col_res = simu.world()->getLastCollisionResult();
        std::vector<dart::collision::Contact> contacts = col_res.getContacts();
        if (contacts.size() > 0) {
            for (auto cont : contacts) {
                // Check body collision
                std::string bd_name = "base_link";

                // Set colliding bodies
                auto bd_shapeFrame1 = const_cast<dart::dynamics::ShapeFrame*>(
                    cont.collisionObject1->getShapeFrame());
                auto bd_shapeFrame2 = const_cast<dart::dynamics::ShapeFrame*>(
                    cont.collisionObject2->getShapeFrame());
                // std::cout << shapeFrame1->getName() << " vs " << shapeFrame2->getName() << std::endl;

                if (bd_shapeFrame1->getName().find(bd_name) != std::string::npos && bd_shapeFrame2->getName().find("sphere") != std::string::npos) {
                    Params::set_collisions(Params::collisions() + 1);
                }

                if (bd_shapeFrame2->getName().find(bd_name) != std::string::npos && bd_shapeFrame1->getName().find("sphere") != std::string::npos) {
                    Params::set_collisions(Params::collisions() + 1);
                }

                // Check legs collision
                for (size_t i = 0; i < 6; ++i) {
                    if (rob->is_broken(i))
                        continue;
                    for (size_t j = 0; j < 3; ++j) {
                        std::string leg_name = "leg_" + std::to_string(i + 1) + "_" + std::to_string(j + 1);

                        // Set colliding bodies
                        auto shapeFrame1 = const_cast<dart::dynamics::ShapeFrame*>(
                            cont.collisionObject1->getShapeFrame());
                        auto shapeFrame2 = const_cast<dart::dynamics::ShapeFrame*>(
                            cont.collisionObject2->getShapeFrame());
                        // std::cout << shapeFrame1->getName() << " vs " << shapeFrame2->getName() << std::endl;

                        if (shapeFrame1->getName().find(leg_name) != std::string::npos && shapeFrame2->getName().find("sphere") != std::string::npos) {
                            Params::set_collisions(Params::collisions() + 1);
                        }

                        if (shapeFrame2->getName().find(leg_name) != std::string::npos && shapeFrame1->getName().find("sphere") != std::string::npos) {
                            Params::set_collisions(Params::collisions() + 1);
                        }
                    }
                }
            }
        }
    }
};

namespace global {
    GP_t gp_model(ARCHIVE_SIZE, 4);

    std::shared_ptr<hexapod_dart::Hexapod> global_robot, simulated_robot;
    using safe_t = boost::fusion::vector</*hexapod_dart::safety_measures::BodyColliding, hexapod_dart::safety_measures::MaxHeight,*/ hexapod_dart::safety_measures::TurnOver, HexaColliding>;
    // using desc_t = boost::fusion::vector<hexapod_dart::descriptors::PositionTraj, hexapod_dart::descriptors::RotationTraj, hexapod_dart::descriptors::BodyOrientation>;
    using viz_t = boost::fusion::vector<hexapod_dart::visualizations::HeadingArrow, hexapod_dart::visualizations::PointingArrow<VizParams>>;
    using hexapod_simu_t = hexapod_dart::HexapodDARTSimu<hexapod_dart::safety<safe_t>, /*hexapod_dart::desc<desc_t>,*/ hexapod_dart::viz<viz_t>>;
    std::shared_ptr<hexapod_simu_t> simu;
    std::vector<hexapod_dart::HexapodDamage> damages;

    // Robot pose (x,y,theta)
    Eigen::Vector3d robot_pose;

    std::vector<SimpleObstacle> obstacles;
    size_t height, width, entity_size, map_size, map_size_x, map_size_y;
    svg::Dimensions dimensions;
    std::shared_ptr<svg::Document> doc;
    size_t target_num;

#ifdef ROBOT
    std::shared_ptr<hexapod_ros::Hexapod> hexa;
    ros::Subscriber sub;
    bool first = false;
    Eigen::MatrixXd transform;
    double orig_theta;
#endif

    // statistics
    std::ofstream robot_file, ctrl_file, iter_file, misc_file;
}

// Stat GP
void write_gp(std::string filename)
{
    std::ofstream ofs;
    ofs.open(filename);
    typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;
    for (archive_it_t it = Params::archiveparams::archive.begin(); it != Params::archiveparams::archive.end(); it++) {
        Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
        Eigen::VectorXd mu;
        double sigma;
        std::tie(mu, sigma) = global::gp_model.query(desc);
        ofs << desc.transpose() << " "
            << mu.transpose() << " "
            << sigma << std::endl;
    }
    ofs.close();
}

#ifdef ROBOT
Eigen::Vector3d get_tf(std::string from, std::string to)
{
    tf::TransformListener listener;
    tf::StampedTransform pos;
    Eigen::Vector3d ret;
    ros::Time start_t = ros::Time::now();
    bool err = false;
    while (ros::ok()) {
        try {
            listener.lookupTransform(from, to, ros::Time(0), pos);
            break;
        }
        catch (tf::TransformException ex) {
            ROS_DEBUG_STREAM("Failed to get transfromation from '" << from << "' to '" << to << "': " << ex.what());
        }
        ros::Duration(0.001).sleep();
        if ((ros::Time::now() - start_t) > ros::Duration(1.0)) {
            ROS_ERROR_STREAM("Timeout error: Failed to get transfromation from '" << from << "' to '" << to << "'");
            err = true;
            break;
        }
    }
    if (!err) {
        double th, tt, tt2;
        auto q = pos.getRotation();
        tf::Matrix3x3(q).getRPY(tt, tt2, th);
        ret << pos.getOrigin()[0], pos.getOrigin()[1], th;
    }

    return ret;
}
#endif

bool collides(double x, double y, double r)
{
    for (size_t i = 0; i < global::obstacles.size(); i++) {
        double dx = x - global::obstacles[i]._x;
        double dy = y - global::obstacles[i]._y;
        if (std::sqrt(dx * dx + dy * dy) <= global::obstacles[i]._radius + r) {
            return true;
        }
    }
    return false;
}

bool collides_segment(const Eigen::Vector2d& start, const Eigen::Vector2d& end)
{
    const double epsilon = 1e-6;
    double Dx = end(0) - start(0);
    double Dy = end(1) - start(1);

    for (size_t i = 0; i < global::obstacles.size(); i++) {
        double Fx = start(0) - global::obstacles[i]._x;
        double Fy = start(1) - global::obstacles[i]._y;

        double a = Dx * Dx + Dy * Dy;
        double b = 2.0 * (Fx * Dx + Fy * Dy);
        double c = (Fx * Fx + Fy * Fy) - global::obstacles[i]._radius_sq;

        double discriminant = b * b - 4.0 * a * c;
        if (discriminant > epsilon) {
            discriminant = std::sqrt(discriminant);

            double t1 = (-b - discriminant) / (2.0 * a);
            double t2 = (-b + discriminant) / (2.0 * a);

            if (t1 > epsilon && (t1 - 1.0) < epsilon) {
                return true;
            }

            if (t2 > epsilon && (t2 - 1.0) < epsilon) {
                return true;
            }
        }
    }
    return false;
}

bool collides(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double r = Params::robot_radius())
{
    Eigen::Vector2d dir = end - start;
    Eigen::Vector2d perp = Eigen::Vector2d(-dir(1), dir(0));
    Eigen::Vector2d A = start + perp * r;
    Eigen::Vector2d B = end + perp * r;
    Eigen::Vector2d C = end - perp * r;
    Eigen::Vector2d D = start - perp * r;
    return (collides(start(0), start(1), r) || collides(end(0), end(1), r) || collides_segment(A, B) || collides_segment(B, C) || collides_segment(C, D) || collides_segment(D, A));
}

void init_simu(std::string robot_file, std::vector<hexapod_dart::HexapodDamage> damages = std::vector<hexapod_dart::HexapodDamage>())
{
    global::global_robot = std::make_shared<hexapod_dart::Hexapod>(robot_file, damages);
#ifndef ROBOT
    global::simulated_robot = global::global_robot->clone();
    global::simu = std::make_shared<global::hexapod_simu_t>(std::vector<double>(36, 0.0), global::simulated_robot);
#endif
#ifdef GRAPHIC
    double x = global::map_size * Params::cell_size() / 2.0;
    global::simu->fixed_camera(Eigen::Vector3d(x, x, global::map_size), Eigen::Vector3d(x, x, 0));
#endif
    global::robot_pose << 0.0, 0.0, 0.0;
}

void replay(const std::vector<double>& ctrl, double t = 3.0)
{
    // launching the simulation
    auto robot = global::global_robot->clone();
    using safe_t = boost::fusion::vector<hexapod_dart::safety_measures::BodyColliding, hexapod_dart::safety_measures::MaxHeight, hexapod_dart::safety_measures::TurnOver>;
    using desc_t = boost::fusion::vector<hexapod_dart::descriptors::PositionTraj, hexapod_dart::descriptors::RotationTraj, hexapod_dart::descriptors::BodyOrientation>;
    hexapod_dart::HexapodDARTSimu<hexapod_dart::safety<safe_t>, hexapod_dart::desc<desc_t>> simu(ctrl, robot);
    simu.set_desc_dump(1);
    simu.run(t);

    double angle = simu.arrival_angle();

    std::vector<Eigen::Vector3d> pos_traj;
    simu.get_descriptor<hexapod_dart::descriptors::PositionTraj>(pos_traj);
    // Performance

    //Angle Difference
    double x, y;
    x = pos_traj.back()(0);
    y = pos_traj.back()(1);

    double B = std::sqrt((x / 2.0) * (x / 2.0) + (y / 2.0) * (y / 2.0));
    double alpha = std::atan2(y, x);
    double A = B / std::cos(alpha);

    double beta = std::atan2(y, x - A);

    if (x < 0)
        beta = beta - M_PI;
    while (beta < -M_PI)
        beta += 2 * M_PI;
    while (beta > M_PI)
        beta -= 2 * M_PI;

    double angle_diff = std::abs(beta - angle);

    // Speed/Displacement
    double speed = std::round(std::sqrt(x * x + y * y) * 100.0);

    // std::cout << "speed: " << speed << " angle: " << angle_diff << " -> ";

    double value = speed + (1.0 - angle_diff / M_PI);

    Eigen::Vector2d pos, pos_new;
    pos << pos_traj.back()(0), pos_traj.back()(1);
    pos_new = pos + 0.1 * Eigen::Vector2d(std::cos(beta), std::sin(beta));
    pos = pos_new - pos;
    pos.normalize();
    Eigen::Vector2d perp;
    perp << -pos(1), pos(0);
    Eigen::Vector2d baseline;
    baseline << 0.0, 1.0;

    std::cout << value << std::endl;
    std::cout << pos_traj.back().transpose() << std::endl;
    std::cout << beta << std::endl;
}

bool astar_collides(int x, int y, int x_new, int y_new)
{
    Eigen::Vector2d s(x * Params::cell_size(), y * Params::cell_size());
    Eigen::Vector2d t(x_new * Params::cell_size(), y_new * Params::cell_size());
    return collides(s, t, Params::robot_radius() * 1.5);
}

template <typename State, typename Action>
struct DefaultPolicy {
    // Action operator()(const std::shared_ptr<State>& state)
    Action operator()(const State* state, bool draw = false)
    {
        size_t N = 100;
        double dx = state->_x - Params::goal_x();
        double dy = state->_y - Params::goal_y();
        double d = dx * dx + dy * dy;
        if (d <= Params::cell_size() * Params::cell_size()) {
            Action best_action;
            double best_value = std::numeric_limits<double>::max();
            for (size_t i = 0; i < N; i++) {
                Action act = state->random_action();
                auto final = state->move(act, true);
                double dx = final._x - Params::goal_x();
                double dy = final._y - Params::goal_y();
                double val = dx * dx + dy * dy;
                if (collides(final._x, final._y))
                    val = std::numeric_limits<double>::max();
                if (val < best_value) {
                    best_value = val;
                    best_action = act;
                }
            }

            return best_action;
        }
        astar::AStar<> a_star;
        astar::Node ss(std::round(state->_x / Params::cell_size()), std::round(state->_y / Params::cell_size()), global::map_size_x, global::map_size_y);
        if (collides(ss._x * Params::cell_size(), ss._y * Params::cell_size()) || ss._x <= 0 || ss._x >= int(global::map_size_x) || ss._y <= 0 || ss._y >= int(global::map_size_y)) {
            astar::Node best_root = ss;
            bool changed = false;
            double val = std::numeric_limits<double>::max();
            for (int i = -1; i <= 1; i++) {
                for (int j = -1; j <= 1; j++) {
                    if (i == 0 && j == 0)
                        continue;
                    int x_new = ss._x + i;
                    int y_new = ss._y + j;
                    double dx = x_new * Params::cell_size() - state->_x;
                    double dy = y_new * Params::cell_size() - state->_y;
                    double v = dx * dx + dy * dy;
                    if (!collides(x_new * Params::cell_size(), y_new * Params::cell_size()) && x_new > 0 && x_new < int(global::map_size_x) && y_new > 0 && y_new < int(global::map_size_y) && v < val) {
                        best_root._x = x_new;
                        best_root._y = y_new;
                        val = v;
                        changed = true;
                    }
                }
            }

            if (!changed)
                state->random_action();

            ss = best_root;
        }
        astar::Node ee(std::round(Params::goal_x() / Params::cell_size()), std::round(Params::goal_y() / Params::cell_size()), global::map_size_x, global::map_size_y);
        auto path = a_star.search(ss, ee, astar_collides, global::map_size_x, global::map_size_y);
        if (path.size() < 2) {
            // std::cout << "Error: A* path size less than 2: " << path.size() << ". Returning random action!" << std::endl;
            return state->random_action();
        }

        astar::Node best = path[1];

        // Find best action
        Eigen::Vector2d best_pos(best._x * Params::cell_size(), best._y * Params::cell_size());
        Eigen::Vector2d init_vec = Eigen::Vector2d(state->_x, state->_y);
        // First check if we have clean path
        if (path.size() >= 3 && (best_pos - init_vec).norm() < Params::cell_size()) {
            Eigen::Vector2d new_pos(path[2]._x * Params::cell_size(), path[2]._y * Params::cell_size());
            if (!collides(init_vec, new_pos, 1.5 * Params::robot_radius())) {
                best = path[2];
                best_pos = new_pos;
            }
        }

        // We are going to just use the best from sampled actions

        // Put best position better in space (to avoid collisions) if we are not in a safer region
        if (collides(best_pos(0), best_pos(1), 2.0 * Params::robot_radius())) {
            SimpleObstacle closest_obs(0, 0, 0);
            double closet_dist = std::numeric_limits<double>::max();
            for (auto obs : global::obstacles) {
                double dx = best_pos(0) - obs._x;
                double dy = best_pos(1) - obs._y;
                double v = dx * dx + dy * dy;
                if (v < closet_dist) {
                    closet_dist = v;
                    closest_obs = obs;
                }
            }
            Eigen::Vector2d dir_to_obs = best_pos - Eigen::Vector2d(closest_obs._x, closest_obs._y);
            Eigen::Vector2d new_best = best_pos;
            double step = Params::cell_size();
            size_t n = 0;
            bool b = false;
            do {
                new_best = best_pos.array() + (n + 1) * step * dir_to_obs.array();
                n++;
                b = collides(new_best(0), new_best(1), 2.0 * Params::robot_radius());
                // if (b) {
                //     svg::Circle circle_target(svg::Point(new_best(1) * global::entity_size, new_best(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Red));
                //     (*global::doc) << circle_target;
                // }
                // else {
                //     svg::Circle circle_target(svg::Point(new_best(1) * global::entity_size, new_best(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Blue));
                //     (*global::doc) << circle_target;
                // }
            } while (b && n < 10);

            // svg::Polyline polyline_clean(svg::Stroke(1.0, svg::Color::Black));
            // polyline_clean << svg::Point(closest_obs._y * global::entity_size, closest_obs._x * global::entity_size);
            // polyline_clean << svg::Point(best_pos(1) * global::entity_size, best_pos(0) * global::entity_size);
            // (*global::doc) << polyline_clean;
            //
            // std::cout << "n: " << n << std::endl;

            if (n < 10)
                best_pos = new_best;
        }

        // if (draw) {
        //     svg::Circle circle_target(svg::Point(best_pos(1) * global::entity_size, best_pos(0) * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Green));
        //     (*global::doc) << circle_target;
        // }

        Action best_action;
        double best_value = std::numeric_limits<double>::max();
        for (size_t i = 0; i < N; i++) {
            Action act = state->random_action();
            auto final = state->move(act, true);
            double dx = final._x - best_pos(0);
            double dy = final._y - best_pos(1);
            double val = dx * dx + dy * dy;
            if (collides(final._x, final._y))
                val = std::numeric_limits<double>::max();
            if (val < best_value) {
                best_value = val;
                best_action = act;
            }
        }

        return best_action;
    }
};

template <typename Params>
struct HexaAction {
    Eigen::VectorXd _desc;

    HexaAction() {}
    HexaAction(Eigen::VectorXd desc) : _desc(desc) {}

    bool operator==(const HexaAction& other) const
    {
        return (typename Params::archiveparams::classcompequal()(_desc, other._desc));
    }
};

template <typename Params>
struct HexaState {
    double _x, _y, _theta;
    static constexpr double _epsilon = 1e-3;

    HexaState()
    {
        _x = _y = _theta = 0;
    }

    HexaState(double x, double y, double theta)
    {
        _x = x;
        _y = y;
        _theta = theta;
    }

    bool valid(const HexaAction<Params>& act) const
    {
        // HexaState<Params> tmp = move(act);
        // // Check if state is outside of bounds
        // if (tmp._x < 0.0 || tmp._x > global::map_size * Params::cell_size() || tmp._y < 0.0 || tmp._y > global::map_size * Params::cell_size())
        //     return false;
        //
        // if (collides(tmp._x, tmp._y) || collides(Eigen::Vector2d(_x, _y), Eigen::Vector2d(tmp._x, tmp._y)))
        //     return false;
        return true;
    }

    HexaAction<Params> next_action() const
    {
        return DefaultPolicy<HexaState<Params>, HexaAction<Params>>()(this);
    }

    HexaAction<Params> random_action() const
    {
        static tools::rgen_int_t rgen(0, Params::archiveparams::archive.size() - 1);
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

        HexaAction<Params> act;
        do {
            archive_it_t it = Params::archiveparams::archive.begin();
            std::advance(it, rgen.rand());
            Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
            act._desc = desc;
        } while (!valid(act));
        return act;
    }

    HexaState move(const HexaAction<Params>& action, bool no_noise = false) const
    {
        double x_new, y_new, theta_new;
        Eigen::VectorXd mu;
        double sigma;
        std::tie(mu, sigma) = global::gp_model.query(action._desc);
        if (!no_noise) {
            // std::cout << mu.transpose() << std::endl;
            mu(0) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(0), sigma)));
            mu(1) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(1), sigma)));
            mu(2) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(2), sigma)));
            mu(3) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(3), sigma)));
            // std::cout << "To: " << mu.transpose() << std::endl;
        }
        double theta = std::atan2(mu(3), mu(2));
        double c = std::cos(_theta), s = std::sin(_theta);
        Eigen::MatrixXd tr(3, 3);
        tr << c, -s, _x, s, c, _y, 0, 0, 1;
        Eigen::Vector3d tr_pos;
        tr_pos << mu(0), mu(1), 1.0;
        tr_pos = tr * tr_pos;
        x_new = tr_pos(0);
        y_new = tr_pos(1);
        theta_new = _theta + theta;

        while (theta_new < -M_PI)
            theta_new += 2 * M_PI;
        while (theta_new > M_PI)
            theta_new -= 2 * M_PI;

        // std::cout << "(" << _x << "," << _y << "," << _theta << ") with (" << mu(0) << "," << mu(1) << "," << mu(2) << ") -> (" << x_new << "," << y_new << "," << theta_new << ")" << std::endl;
        return HexaState(x_new, y_new, theta_new);
    }

    bool terminal() const
    {
        // Check if state is outside of bounds
        if (_x < 0.0 || _x >= global::map_size_x * Params::cell_size() || _y < 0.0 || _y >= global::map_size_y * Params::cell_size())
            return true;
        // Check if state is goal
        if (goal())
            return true;
        // Check if state is colliding
        if (collides(_x, _y))
            return true;
        return false;
    }

    bool goal() const
    {
        double dx = _x - Params::goal_x();
        double dy = _y - Params::goal_y();
        double threshold_xy = Params::cell_size() * Params::cell_size() / 4.0;
        if ((dx * dx + dy * dy) < threshold_xy)
            return true;
        return false;
    }

    bool operator==(const HexaState& other) const
    {
        double dx = _x - other._x;
        double dy = _y - other._y;
        double dth = std::abs(angle_dist(other._theta, _theta));
        return ((dx * dx + dy * dy) < (Params::cell_size() * Params::cell_size() / 4.0) && dth < 0.3);
    }
};

struct RewardFunction {
    template <typename State>
    double operator()(std::shared_ptr<State> from_state, HexaAction<Params> action, std::shared_ptr<State> to_state)
    {
        // Check if state is outside of bounds
        if (to_state->_x < 0.0 || to_state->_x >= global::map_size_x * Params::cell_size() || to_state->_y < 0.0 || to_state->_y >= global::map_size_y * Params::cell_size())
            return -1000.0;

        // Return values
        if (collides(to_state->_x, to_state->_y) || collides(Eigen::Vector2d(from_state->_x, from_state->_y), Eigen::Vector2d(to_state->_x, to_state->_y)))
            return -1000.0;
        if (to_state->goal())
            return 100.0;
        return 0.0;
    }
};

bool load_archive(const std::string& filename)
{
    Params::archiveparams::archive.clear();
    binary_map::BinaryMap b_map = binary_map::load(filename);

    for (auto v : b_map.elems) {
        std::vector<double> desc(v.pos.size(), 0.0);
        std::copy(v.pos.begin(), v.pos.end(), desc.begin());
        for (size_t i = 0; i < desc.size(); i++) {
            desc[i] /= (double)(b_map.dims[i]);
        }

        std::vector<double> ctrl(v.phen.size(), 0.0);
        std::copy(v.phen.begin(), v.phen.end(), ctrl.begin());
        assert(ctrl.size() == 36);

        Params::archiveparams::elem_archive elem;
        elem.controller = ctrl;
        elem.x = -2.0 + desc[ARCHIVE_SIZE - 2] * 4.0;
        elem.y = -2.0 + desc[ARCHIVE_SIZE - 1] * 4.0;
        elem.cos_theta = std::cos(v.extra);
        elem.sin_theta = std::sin(v.extra);
        Params::archiveparams::archive[desc] = elem;
    }

    std::cout << "Loaded " << Params::archiveparams::archive.size() << " elements!" << std::endl;

    return true;
}

#ifdef ROBOT
void check_collision(double t)
{
    ros::Time start_time = ros::Time::now();
    ros::Duration timeout(t);
    while (ros::Time::now() - start_time < timeout) {
        Eigen::Vector3d robot = get_tf("odom", "/base_link");
        Eigen::Vector3d pos;
        pos << robot(0), robot(1), 1.0;
        pos = global::transform * pos;
        if (collides(pos(0), pos(1), 1.5 * Params::robot_radius()) && (ros::Time::now() - start_time) > ros::Duration(2.0 * t / 3.0)) {
            global::hexa->zero();
        }
    }
}
#endif

void execute(const Eigen::VectorXd& desc, double t, bool stat = true)
{
    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;
    std::vector<double> ctrl = Params::archiveparams::archive[d].controller;

    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }

#ifndef ROBOT
    // Resetting forces for stability
    global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
    global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
    global::simulated_robot->skeleton()->clearExternalForces();
    global::simulated_robot->skeleton()->clearInternalForces();

    // Run the controller
    global::simu->controller().set_parameters(ctrl);
    global::simu->run(t, true, true);

    // If the robot tries to fall or collides
    size_t n = 0;
    while (global::simu->covered_distance() < -10000.0 && n < 10) {
        // try to stabilize the robot
        global::simulated_robot->skeleton()->setPosition(0, 0.0);
        global::simulated_robot->skeleton()->setPosition(1, 0.0);
        global::simulated_robot->skeleton()->setPosition(5, 0.2);
        global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
        global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
        global::simulated_robot->skeleton()->clearExternalForces();
        global::simulated_robot->skeleton()->clearInternalForces();

        global::simu->controller().set_parameters(std::vector<double>(36, 0.0));
        global::simu->run(1.0, true, false);
        n++;
    }
    // reset to zero configuration for stability
    global::simu->controller().set_parameters(std::vector<double>(36, 0.0));
    global::simu->run(1.0, true, false);
#else
    // global::hexa->move(ctrl, t, !global::first);
    std::thread t1 = std::thread(&hexapod_ros::Hexapod::move, global::hexa, ctrl, t, !global::first);
    std::thread t2 = std::thread(check_collision, t);
    t1.join();
    t2.join();
    global::first = true;

    Eigen::Vector3d robot = get_tf("odom", "/base_link");
    Eigen::Vector3d pos;
    pos << robot(0), robot(1), 1.0;
    pos = global::transform * pos;
#endif

#ifndef ROBOT
    double theta = global::simulated_robot->pose()(2);
#else
    double theta = robot(2) + global::orig_theta;
#endif
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
#ifndef ROBOT
    global::robot_pose << global::simulated_robot->pos()(0), global::simulated_robot->pos()(1), theta;
#else
    global::robot_pose << pos(0), pos(1), theta;
#endif
}

// radius, speed
std::tuple<double, double, double> get_x_y_theta(const Eigen::Vector3d& prev_pose, const Eigen::Vector3d& curr_pose)
{
    double c = std::cos(prev_pose(2)), s = std::sin(prev_pose(2));
    Eigen::MatrixXd r(2, 2);
    r << c, -s, s, c;
    r.transposeInPlace();
    Eigen::VectorXd d(2);
    d << prev_pose(0), prev_pose(1);
    d = r * d;
    Eigen::MatrixXd tr(3, 3);
    tr << r(0, 0), r(0, 1), -d(0), r(1, 0), r(1, 1), -d(1), 0, 0, 1;
    Eigen::MatrixXd tr2(3, 3);
    c = std::cos(curr_pose(2));
    s = std::sin(curr_pose(2));
    tr2 << c, -s, curr_pose(0), s, c, curr_pose(1), 0, 0, 1;
    Eigen::Vector3d tr_pos;
    tr_pos << 0.0, 0.0, 1.0;
    tr_pos = tr * tr2 * tr_pos;
    // std::cout << prev_pose.transpose() << " -> " << curr_pose.transpose() << " leads to: " << tr_pos.transpose() << std::endl;
    // Eigen::MatrixXd rr(2, 2);
    // c = std::cos(curr_pose(2));
    // s = std::sin(curr_pose(2));
    // rr << c, -s, s, c;
    // rr = r * rr;
    // double th = std::atan2(rr(1, 0), rr(0, 0));
    // // It's the same!
    // std::cout << th << " vs " << angle_dist(prev_pose(2), curr_pose(2)) << std::endl;

    return std::make_tuple(tr_pos(0), tr_pos(1), angle_dist(prev_pose(2), curr_pose(2)));
}

template <typename Params>
struct ActiveLearningValue {
    const double _epsilon = 1e-6;

    template <typename MCTSAction>
    double operator()(const std::shared_ptr<MCTSAction>& action)
    {
        return action->value() / (double(action->visits()) + _epsilon) + Params::active_learning::k() * Params::active_learning::scaling() * global::gp_model.sigma(action->action()._desc);
    }
};

void draw_obs_svg(svg::Document& doc)
{
    for (auto ob : global::obstacles) {
        svg::Circle circle(svg::Point(ob._y * global::entity_size, ob._x * global::entity_size), ob._radius * 2.0 * global::entity_size, svg::Fill(svg::Color::Red), svg::Stroke(1, svg::Color::Red));
        doc << circle;
    }
}

void draw_target_svg(svg::Document& doc)
{
    svg::Circle circle_target(svg::Point(Params::goal_y() * global::entity_size, Params::goal_x() * global::entity_size), Params::cell_size() * global::entity_size, svg::Fill(), svg::Stroke(2, svg::Color::Green));
    doc << circle_target;
}

void draw_robot_svg(svg::Document& doc, const Eigen::Vector3d& pose)
{
    // Draw robot
    svg::Circle circle_init(svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size), Params::cell_size() / 2.0 * global::entity_size, svg::Fill(svg::Color::Blue), svg::Stroke(1, svg::Color::Black));
    doc << circle_init;

    svg::Circle circle_small(svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size), Params::cell_size() / 4.0 * global::entity_size, svg::Fill(svg::Color::Fuchsia), svg::Stroke(1, svg::Color::Black));
    doc << circle_small;

    // Draw direction
    Eigen::Vector2d dir(pose(0) + std::cos(pose(2)) * Params::cell_size() / 2.0, pose(1) + std::sin(pose(2)) * Params::cell_size() / 2.0);
    svg::Polyline polyline_clean(svg::Stroke(2.0, svg::Color::Fuchsia));
    polyline_clean << svg::Point(pose(1) * global::entity_size, pose(0) * global::entity_size);
    polyline_clean << svg::Point(dir(1) * global::entity_size, dir(0) * global::entity_size);
    doc << polyline_clean;
}

void draw_line_svg(svg::Document& doc, const Eigen::Vector2d& from, const Eigen::Vector2d& to)
{
    svg::Polyline polyline_clean(svg::Stroke(1.0, svg::Color::Black));
    polyline_clean << svg::Point(from(1) * global::entity_size, from(0) * global::entity_size);
    polyline_clean << svg::Point(to(1) * global::entity_size, to(0) * global::entity_size);
    doc << polyline_clean;
}

std::tuple<bool, size_t, size_t> reach_target(const Eigen::Vector3d& goal_state, size_t max_iter = std::numeric_limits<size_t>::max())
{
    // using Choose = mcts::GreedyValue;
    using Choose = ActiveLearningValue<Params>;

    Params::set_goal_x(goal_state(0));
    Params::set_goal_y(goal_state(1));
    Params::set_goal_theta(goal_state(2));

#ifndef ROBOT
    // TO-DO: Fix visualization
    // Add target to simulation (arrow)
    VizParams::set_head(Eigen::Vector3d(goal_state(0), goal_state(1), 0.0));
    VizParams::set_tail(Eigen::Vector3d(goal_state(0), goal_state(1), 0.25));

    // Run dummy time to have something displayed
    global::simu->run(0.25);
#endif

    global::target_num++;
    global::dimensions = svg::Dimensions(global::width, global::height);
    global::doc = std::make_shared<svg::Document>("plan_" + std::to_string(global::target_num) + ".svg", svg::Layout(global::dimensions, svg::Layout::TopLeft));

    // Initialize statistics
    global::robot_file.open("robot_" + std::to_string(global::target_num) + ".dat");
    global::ctrl_file.open("ctrl_" + std::to_string(global::target_num) + ".dat");
    global::iter_file.open("iter_" + std::to_string(global::target_num) + ".dat");
    global::misc_file.open("misc_" + std::to_string(global::target_num) + ".dat");

    // Draw obstacles
    draw_obs_svg(*global::doc);
    // Draw target
    draw_target_svg(*global::doc);
    // Draw robot
    draw_robot_svg(*global::doc, global::robot_pose);
    // Save doc
    global::doc->save();

    RewardFunction world;

    size_t n = 0;

    // statistics
    global::robot_file << "-1 " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;

    bool collided = false;
    bool terminal = false;
    Params::set_collisions(0);

    while (!terminal && !collided && (n < max_iter)) {
        auto t1 = std::chrono::steady_clock::now();
        // Get last post from hexapod simulation/real robot
        HexaState<Params> init = HexaState<Params>(global::robot_pose(0), global::robot_pose(1), global::robot_pose(2));
        // DefaultPolicy<HexaState<Params>, HexaAction<Params>>()(&init, true);

        // Run MCTS
        auto tree = std::make_shared<mcts::MCTSNode<Params, HexaState<Params>, mcts::SimpleStateInit<HexaState<Params>>, mcts::SimpleValueInit, mcts::UCTValue<Params>, mcts::UniformRandomPolicy<HexaState<Params>, HexaAction<Params>>, HexaAction<Params>, mcts::SPWSelectPolicy<Params>, mcts::ContinuousOutcomeSelect<Params>>>(init, 20);

        tree->compute(world, Params::iterations());

        auto time_running = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t1).count();
        // std::cout << "Time in sec: " << time_running / 1000.0 << std::endl;

        // std::cout << "Children: " << tree->children().size() << std::endl;
        double sum = 0;
        double max = -std::numeric_limits<double>::max();
        double min = std::numeric_limits<double>::max();
        for (auto child : tree->children()) {
            double v = child->value() / double(child->visits());
            sum += v;
            if (v > max)
                max = v;
            if (v < min)
                min = v;
        }
        Params::active_learning::set_scaling((max - min) / Params::kernel_exp::sigma_sq());
        // std::cout << "Active learning scaling: " << Params::active_learning::scaling() << std::endl;

        // Get best action/behavior
        auto best = tree->best_action<Choose>();
        // std::cout << "val: " << best->value() / double(best->visits()) << std::endl;
        auto other_best = tree->best_action<mcts::GreedyValue>();
        // std::cout << "val without AL: " << other_best->value() / double(other_best->visits()) << std::endl;
        // std::cout << "avg: " << (sum / double(tree->children().size())) << std::endl;
        auto tmp = init.move(best->action(), true);
        // std::cout << tmp._x << " " << tmp._y << " -> " << tmp._theta << std::endl;
        global::iter_file << n << " " << time_running / 1000.0 << " " << best->value() / double(best->visits()) << " " << other_best->value() / double(other_best->visits()) << " " << (sum / double(tree->children().size())) << " " << tmp._x << " " << tmp._y << " " << tmp._theta << std::endl;

        // Execute in simulation/real robot
        Eigen::Vector3d prev_pose = global::robot_pose;
        execute(best->action()._desc, 3.0);

        // Draw robot
        draw_robot_svg(*global::doc, global::robot_pose);
        // Draw line connecting steps
        draw_line_svg(*global::doc, Eigen::Vector2d(prev_pose(0), prev_pose(1)), Eigen::Vector2d(global::robot_pose(0), global::robot_pose(1)));
        global::doc->save();

        // std::cout << "Robot: " << global::robot_pose.transpose() << std::endl;
        global::robot_file << n << " " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;

        global::misc_file << n << " ";
        if (Params::learning()) {
            // Update GP (add_sample)
            Eigen::VectorXd observation(3);
            std::tie(observation(0), observation(1), observation(2)) = get_x_y_theta(prev_pose, global::robot_pose);
            Eigen::VectorXd data(4);
            data(0) = observation(0);
            data(1) = observation(1);
            data(2) = std::cos(observation(2));
            data(3) = std::sin(observation(2));
            // Eigen::VectorXd test;
            // test = global::gp_model.mu(best->action()._desc);
            // std::cout << test(0) << " " << test(1) << " " << test(2) << " " << test(3) << std::endl;
            global::gp_model.add_sample(best->action()._desc, data, 0.01);
            global::misc_file << data(0) << " " << data(1) << " " << data(2) << " " << data(3) << " ";
            // std::cout << observation(0) << " " << observation(1) << " " << std::cos(observation(2)) << " " << std::sin(observation(2)) << std::endl;
            // std::cout << observation(2) << std::endl;
            // std::vector<double> d;
            // for (size_t i = 0; i < best->action()._desc.size(); i++) {
            //     d.push_back(best->action()._desc[i]);
            // }
            // auto vvv = Params::archiveparams::archive[d];
            // std::cout << vvv.x << " " << vvv.y << " " << vvv.cos_theta << " " << vvv.sin_theta << " -> " << std::atan2(vvv.sin_theta, vvv.cos_theta) << std::endl;
            // std::cout << "----------------------------" << std::endl;
            // std::cout << observation(0) << " " << observation(1) << " " << observation(2) << std::endl;
            write_gp("gp_" + std::to_string((global::gp_model.samples().empty()) ? 0 : global::gp_model.nb_samples()) + ".dat");
        }
        global::misc_file << "3.0" << std::endl;

        // Check collisions/termination
        if (collides(global::robot_pose(0), global::robot_pose(1))) {
            collided = true;
            std::cout << "Collision!" << std::endl;
        }

        double dx = global::robot_pose(0) - Params::goal_x();
        double dy = global::robot_pose(1) - Params::goal_y();
        double threshold_xy = Params::cell_size() * Params::cell_size() / 4.0;
        if ((dx * dx + dy * dy) < threshold_xy) {
            terminal = true;
        }
        n++;
    }

    // Close statistics
    global::robot_file.close();
    global::ctrl_file.close();
    global::iter_file.close();
    global::misc_file.close();

    if (n < max_iter && !collided)
        return std::make_tuple(true, n, Params::collisions());
    return std::make_tuple(false, n, Params::collisions());
}

std::vector<Eigen::Vector3d> generate_targets(const Eigen::Vector2d& start, const Eigen::Vector2d& map_size, double dist, size_t N = 50)
{
    static tools::rgen_double_t rgen_x(Params::cell_size(), map_size(0));
    static tools::rgen_double_t rgen_y(Params::cell_size(), map_size(1));

    Eigen::Vector2d s = start;
    std::vector<Eigen::Vector3d> targets;
    for (size_t i = 0; i < N; i++) {
        Eigen::Vector2d t(rgen_x.rand(), rgen_y.rand());
        while (std::abs((s - t).norm() - dist) > Params::cell_size() / 5.0 || collides(t(0), t(1), 2.0 * Params::robot_radius())) {
            t << rgen_x.rand(), rgen_y.rand();
        }

        targets.push_back(Eigen::Vector3d(t(0), t(1), 0.0));
        s = t;
    }

    return targets;
}

std::tuple<std::vector<Eigen::Vector3d>, Eigen::Vector3d> init_map(const std::string& map_string)
{
    // Init obstacles
    double path_width = Params::cell_size();
    double i_x = 0, i_y = 0, i_th = 0;
    std::map<int, Eigen::Vector3d> goals;
    bool goal_in_map = false;
    bool init_in_map = false;
    double max_x = 0, max_y = 0;
    int r = 0, c = 0;
    for (size_t i = 0; i < map_string.size(); i++) {
        if (map_string[i] == '\n') {
            r++;
            if (i < map_string.size() - 1) {
                c = 0;
            }
            continue;
        }

        double x = path_width * r;
        double y = path_width * c;

        if (x > max_x)
            max_x = x;
        if (y > max_y)
            max_y = y;

        if (map_string[i] == '*') {
            global::obstacles.push_back(SimpleObstacle(x, y, path_width / 2.0));
        }
        else if (map_string[i] == '^') {
            i_x = x;
            i_y = y;
            i_th = dart::math::constants<double>::pi();
            init_in_map = true;
        }
        else if (map_string[i] == 'v') {
            i_x = x;
            i_y = y;
            i_th = 0;
            init_in_map = true;
        }
        else if (map_string[i] == '<') {
            i_x = x;
            i_y = y;
            i_th = -dart::math::constants<double>::half_pi();
            init_in_map = true;
        }
        else if (map_string[i] == '>') {
            i_x = x;
            i_y = y;
            i_th = dart::math::constants<double>::half_pi();
            init_in_map = true;
        }
        else if (map_string[i] != ' ') {
            int t = map_string[i] - '0';
            goals[t] = Eigen::Vector3d(x, y, 0);
            goal_in_map = true;
        }
        c++;
    }

#ifdef ROBOT
    double ww = 0.05;
    for (auto obs : global::obstacles) {
        if (obs._x > Params::cell_size() && obs._x < max_x - Params::cell_size() && obs._y > Params::cell_size() && obs._y < max_y - Params::cell_size()) {
            global::obstacles.push_back(SimpleObstacle(obs._x + ww, obs._y, path_width / 2.0));
            global::obstacles.push_back(SimpleObstacle(obs._x - ww, obs._y, path_width / 2.0));
            if (collides(obs._x, obs._y - Params::cell_size(), 0.8 * Params::robot_radius())) {
                global::obstacles.push_back(SimpleObstacle(obs._x + ww, obs._y - path_width / 4.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x + ww, obs._y - path_width / 2.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x - ww, obs._y - path_width / 2.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x - ww, obs._y - path_width / 4.0, path_width / 2.0));
            }
            if (collides(obs._x, obs._y + Params::cell_size(), 0.8 * Params::robot_radius())) {
                global::obstacles.push_back(SimpleObstacle(obs._x + ww, obs._y + path_width / 4.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x + ww, obs._y + path_width / 2.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x - ww, obs._y + path_width / 2.0, path_width / 2.0));
                global::obstacles.push_back(SimpleObstacle(obs._x - ww, obs._y + path_width / 4.0, path_width / 2.0));
            }
            // else {
            //     global::obstacles.push_back(SimpleObstacle(obs._x + path_width / 4.0, obs._y + path_width / 4.5, path_width / 3.0));
            //     global::obstacles.push_back(SimpleObstacle(obs._x - path_width / 4.0, obs._y + path_width / 4.5, path_width / 3.0));
            // }
        }
    }
#endif

    global::height = 768;
    global::width = static_cast<size_t>(double(c) / double(r) * 768);
    global::map_size = (r > c) ? r - 1 : c - 1;
    global::map_size_x = r;
    global::map_size_y = c;
    global::entity_size = static_cast<size_t>(((global::height > global::width) ? global::width : global::height) / double(global::map_size * Params::cell_size()));

    if (!goal_in_map || !init_in_map) {
        std::cerr << "No goal or robot in the map." << std::endl;
        exit(1);
    }

    size_t N = 50;
#ifdef ROBOT
    N = 10;
#endif
    std::vector<Eigen::Vector3d> g = generate_targets(Eigen::Vector2d(i_x, i_y), Eigen::Vector2d((r - 1) * Params::cell_size(), (c - 1) * Params::cell_size()), (Eigen::Vector2d(i_x, i_y) - Eigen::Vector2d(goals[1](0), goals[1](1))).norm(), N);

    return std::make_tuple(g, Eigen::Vector3d(i_x, i_y, i_th));
}

void init_simu_world(const Eigen::Vector3d& start)
{
#ifndef ROBOT
    // Clear previous maps
    global::simu->clear_objects();

    // Add obstacles to simulation
    for (auto obs : global::obstacles) {
        Eigen::Vector6d obs_pos;
        obs_pos << 0, 0, 0, obs._x, obs._y, 0.0;
        global::simu->add_ellipsoid(obs_pos, Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05), "fixed");
        obs_pos << 0, 0, 0, obs._x, obs._y, obs._radius / 2.0;
        global::simu->add_ellipsoid(obs_pos, Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05), "fixed");
        obs_pos << 0, 0, 0, obs._x, obs._y, obs._radius;
        global::simu->add_ellipsoid(obs_pos, Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05), "fixed");
        obs_pos << 0, 0, 0, obs._x, obs._y, 3.0 * obs._radius / 2.0;
        global::simu->add_ellipsoid(obs_pos, Eigen::Vector3d(obs._radius * 2.05, obs._radius * 2.05, obs._radius * 2.05), "fixed");
        // global::simu->add_box(obs_pos, Eigen::Vector3d(obs._radius * 2.0, obs._radius * 2.0, obs._radius * 2.0), "fixed");
    }

    // Change robot position/orientation
    global::simulated_robot->skeleton()->setPosition(2, start(2));
    global::simulated_robot->skeleton()->setPosition(3, start(0));
    global::simulated_robot->skeleton()->setPosition(4, start(1));
#endif

    global::robot_pose = start;

#ifdef ROBOT
    Eigen::Vector3d robot_pose = get_tf("/odom", "/base_link");
    std::cout << "Robot: " << robot_pose.transpose() << std::endl;

    double c = std::cos(robot_pose(2)), s = std::sin(robot_pose(2));
    Eigen::MatrixXd r(2, 2);
    r << c, -s, s, c;
    r.transposeInPlace();
    Eigen::VectorXd d(2);
    d << robot_pose(0), robot_pose(1);
    d = r * d;
    Eigen::MatrixXd Twr(3, 3);
    Twr << r(0, 0), r(0, 1), -d(0), r(1, 0), r(1, 1), -d(1), 0, 0, 1;
    c = std::cos(start(2));
    s = std::sin(start(2));
    Eigen::MatrixXd Tmr(3, 3);
    Tmr << c, -s, start(0), s, c, start(1), 0, 0, 1;
    Eigen::MatrixXd tr = Tmr * Twr;
    global::transform = tr;
    global::orig_theta = std::atan2(-tr(0, 1), tr(0, 0));
#endif

    global::target_num = 0;
}

void hexa_init(size_t N = 15)
{
    auto robot = global::global_robot->clone();

    global::hexapod_simu_t simu(std::vector<double>(36, 0.0), robot);

    for (size_t i = 0; i < N; i++) {
        // Execute random action
        // static double times[3] = {1.0, 2.0, 3.0};
        static tools::rgen_int_t rgen(0, Params::archiveparams::archive.size() - 1);
        // static tools::rgen_int_t rgen_time(0, 2);
        // static tools::rgen_double_t rgen_time(1.0, 3.0);
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

        archive_it_t it = Params::archiveparams::archive.begin();
        std::advance(it, rgen.rand());
        std::vector<double> ctrl = it->second.controller;
        double c_time = 3.0; // times[rgen_time.rand()];
        // replay(ctrl, c_time);

        // Resetting forces for stability
        robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(robot->skeleton()->getVelocities().size()));
        robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(robot->skeleton()->getAccelerations().size()));
        robot->skeleton()->clearExternalForces();
        robot->skeleton()->clearInternalForces();

        // robot->skeleton()->setPosition(3, 0.0);
        // robot->skeleton()->setPosition(4, 0.0);

        Eigen::Vector3d prev_pose;
        prev_pose << robot->pos()(0), robot->pos()(1), robot->pose()(2);

        // Run the controller
        simu.controller().set_parameters(ctrl);
        simu.run(c_time, false, true);

        // If the robot tries to fall
        size_t n = 0;
        while (simu.covered_distance() < -10000.0 && n < 10) {
            // try to stabilize the robot
            robot->skeleton()->setPosition(0, 0.0);
            robot->skeleton()->setPosition(1, 0.0);
            robot->skeleton()->setPosition(5, 0.2);
            robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(robot->skeleton()->getVelocities().size()));
            robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(robot->skeleton()->getAccelerations().size()));
            robot->skeleton()->clearExternalForces();
            robot->skeleton()->clearInternalForces();

            simu.controller().set_parameters(std::vector<double>(36, 0.0));
            simu.run(1.0, false, true);
            n++;
        }
        // reset to zero configuration for stability
        simu.controller().set_parameters(std::vector<double>(36, 0.0));
        simu.run(1.0, false, true);

        double theta = robot->pose()(2);
        while (theta < -M_PI)
            theta += 2 * M_PI;
        while (theta > M_PI)
            theta -= 2 * M_PI;
        Eigen::Vector3d robot_pose;
        robot_pose << robot->pos()(0), robot->pos()(1), theta;

        // Update GP (add_sample)
        Eigen::VectorXd observation(3);
        std::tie(observation(0), observation(1), observation(2)) = get_x_y_theta(prev_pose, robot_pose);
        // Eigen::VectorXd mu;
        // double sigma;
        // std::tie(mu, sigma) = global::gp_model.query(Eigen::VectorXd::Map(it->first.data(), it->first.size()));
        // std::cout << mu.transpose() << " vs " << observation.transpose() << std::endl;
        global::gp_model.add_sample(Eigen::VectorXd::Map(it->first.data(), it->first.size()), observation, 0.01);
    }
}

MCTS_DECLARE_DYN_PARAM(double, Params::uct, c);
MCTS_DECLARE_DYN_PARAM(double, Params::spw, a);
MCTS_DECLARE_DYN_PARAM(double, Params::cont_outcome, b);
MCTS_DECLARE_DYN_PARAM(double, Params::active_learning, scaling);
MCTS_DECLARE_DYN_PARAM(double, Params::active_learning, k);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_x);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_y);
MCTS_DECLARE_DYN_PARAM(double, Params, goal_theta);
MCTS_DECLARE_DYN_PARAM(double, Params, iterations);
MCTS_DECLARE_DYN_PARAM(bool, Params, learning);
MCTS_DECLARE_DYN_PARAM(size_t, Params, collisions);
MCTS_DECLARE_DYN_PARAM(size_t, Params::mcts_node, parallel_roots);
BO_DECLARE_DYN_PARAM(double, Params::kernel_exp, sigma_sq);
BO_DECLARE_DYN_PARAM(double, Params::kernel_exp, l);
BO_DECLARE_DYN_PARAM(Eigen::Vector3d, VizParams, head);
BO_DECLARE_DYN_PARAM(Eigen::Vector3d, VizParams, tail);

Params::archiveparams::archive_t Params::archiveparams::archive;

int main(int argc, char** argv)
{
    mcts::par::init();

    std::string map_string = "";
    std::string map_file = "";
    std::string archive_file = "";
    std::string exp_folder = "";
    std::vector<int> removed_legs, shortened_legs;
    bool no_learning = false;

    namespace po = boost::program_options;
    po::options_description desc("Command line arguments");
    desc.add_options()("help,h", "Prints this help message")("archive,m", po::value<std::string>()->required(), "Archive file")("load,l", po::value<std::string>()->required(), "Load map from file")("uct,c", po::value<double>(), "UCT c value (in range (0,+00))")("spw,a", po::value<double>(), "SPW a value (in range (0,1))")("dpw,b", po::value<double>(), "DPW b value (in range (0,1))")("iter,i", po::value<size_t>(), "Number of iteartions to run MCTS")("remove_legs,r", po::value<std::vector<int>>()->multitoken(), "Specify which legs to remove")("shorten_legs,s", po::value<std::vector<int>>()->multitoken(), "Specify which legs to shorten")("parallel_roots,p", po::value<size_t>(), "Number of parallel trees in MCTS")("active_learning,k", po::value<double>(), "Active Learning k parameter")("signal_variance,v", po::value<double>(), "Initial signal variance in kernel (squared)")("kernel_scale,d", po::value<double>(), "Characteristic length scale in kernel")("replay_exp,e", po::value<std::string>(), "Folder of experiment to replay")("no_learning,n", po::bool_switch(&no_learning), "Do not learn anything");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        else if (vm.count("replay_exp")) {
            exp_folder = vm["replay_exp"].as<std::string>();
        }
        if (exp_folder.empty()) {
            po::notify(vm);
        }
        if (vm.count("archive")) {
            archive_file = vm["archive"].as<std::string>();
        }
        if (vm.count("load")) {
            map_file = vm["load"].as<std::string>();
            try {
                std::ifstream t(map_file);
                if (!t.is_open() || !t.good()) {
                    std::cerr << "Exception while reading the map file: " << map_file << std::endl;
                    return 1;
                }
                map_string = std::string((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());
            }
            catch (...) {
                std::cerr << "Exception while reading the map file: " << map_file << std::endl;
                return 1;
            }
        }
        if (vm.count("remove_legs")) {
            removed_legs = vm["remove_legs"].as<std::vector<int>>();
            hexapod_dart::HexapodDamage dmg;
            dmg.type = "leg_removal";
            dmg.data = "";
            for (size_t i = 0; i < removed_legs.size(); i++) {
                dmg.data = dmg.data + std::to_string(removed_legs[i]);
            }
            global::damages.push_back(dmg);
        }
        if (vm.count("shorten_legs")) {
            shortened_legs = vm["shorten_legs"].as<std::vector<int>>();
            for (size_t i = 0; i < shortened_legs.size(); i++) {
                hexapod_dart::HexapodDamage dmg;
                dmg.type = "leg_shortening";
                dmg.data = std::to_string(shortened_legs[i]);
                global::damages.push_back(dmg);
            }
        }
        if (vm.count("uct")) {
            double c = vm["uct"].as<double>();
            if (c < 0.0)
                c = 0.0;
            Params::uct::set_c(c);
        }
        else {
            Params::uct::set_c(50.0);
        }
        if (vm.count("spw")) {
            double a = vm["spw"].as<double>();
            if (a < 1e-6 || (1.0 - a) < 1e-6) {
                std::cerr << "SPW a value has to be in range (0,1)! Zero and One are not included!" << std::endl;
                return 1;
            }
            Params::spw::set_a(a);
        }
        else {
            Params::spw::set_a(0.5);
        }
        if (vm.count("dpw")) {
            double b = vm["dpw"].as<double>();
            if (b < 1e-6 || (1.0 - b) < 1e-6) {
                std::cerr << "DPW b value has to be in range (0,1)! Zero and One are not included!" << std::endl;
                return 1;
            }
            Params::cont_outcome::set_b(b);
        }
        else {
            Params::cont_outcome::set_b(0.6);
        }
        if (vm.count("iter")) {
            Params::set_iterations(vm["iter"].as<size_t>());
        }
        else {
            Params::set_iterations(1000);
        }
        if (vm.count("parallel_roots")) {
            Params::mcts_node::set_parallel_roots(vm["parallel_roots"].as<size_t>());
        }
        else {
            Params::mcts_node::set_parallel_roots(4);
        }
        if (vm.count("active_learning")) {
            double c = vm["active_learning"].as<double>();
            if (c < 0.0)
                c = 0.0;
            Params::active_learning::set_k(c);
        }
        else {
            Params::active_learning::set_k(1.0);
        }
        if (vm.count("signal_variance")) {
            double c = vm["signal_variance"].as<double>();
            if (c < 0.0)
                c = 0.0;
            Params::kernel_exp::set_sigma_sq(c);
        }
        else {
            Params::kernel_exp::set_sigma_sq(0.5);
        }
        if (vm.count("kernel_scale")) {
            double c = vm["kernel_scale"].as<double>();
            if (c < 0.0)
                c = 0.0;
            Params::kernel_exp::set_l(c);
        }
        else {
            Params::kernel_exp::set_l(0.5);
        }
    }
    catch (po::error& e) {
        std::cerr << "[Exception caught while parsing command line arguments]: " << e.what() << std::endl;
        return 1;
    }

    if (no_learning) {
        Params::set_learning(false);
        // Set variance to very small value if we do not learn
        Params::kernel_exp::set_sigma_sq(1e-20);
    }
    else {
        Params::set_learning(true);
    }

    if (!archive_file.empty() && exp_folder.empty()) {
        // Loading archive
        std::cout << "Loading archive..." << std::endl;
        load_archive(archive_file);
    }

    if (!exp_folder.empty()) {
        std::cout << "Replaying experiment: " << exp_folder << std::endl;

        std::ifstream exp_file(exp_folder + "/exp.dat");
        std::string str, arch_file, m_file;
        int n_removed = 0, n_shorten = 0;
        int i = 0;
        while (std::getline(exp_file, str)) {
            if (i == 0)
                arch_file = str;
            else if (i == 1)
                m_file = str;
            else if (i == 2) {
                std::stringstream in(str);
                int temp;
                std::vector<int> a;
                while (in >> temp) {
                    a.push_back(temp);
                }
                n_removed = a[0];
                n_shorten = a[1];
            }
            else if (i == 3 && n_removed > 0) {
                std::stringstream in(str);
                int temp;
                while (in >> temp) {
                    removed_legs.push_back(temp);
                }
            }
            else if ((i == 3 || (i == 4 && n_removed > 0)) && n_shorten > 0) {
                std::stringstream in(str);
                int temp;
                while (in >> temp) {
                    shortened_legs.push_back(temp);
                }
            }
            i++;
        }

        std::cout << "Loading archive..." << std::endl;
        load_archive(arch_file);

        std::cout << "Set damages..." << std::endl;
        global::damages.clear();

        if (shortened_legs.size() > 0) {
            for (size_t i = 0; i < shortened_legs.size(); i++) {
                hexapod_dart::HexapodDamage dmg;
                dmg.type = "leg_shortening";
                dmg.data = std::to_string(shortened_legs[i]);
                global::damages.push_back(dmg);
            }
        }

        if (removed_legs.size() > 0) {
            hexapod_dart::HexapodDamage dmg;
            dmg.type = "leg_removal";
            dmg.data = "";
            for (size_t i = 0; i < removed_legs.size(); i++) {
                dmg.data = dmg.data + std::to_string(removed_legs[i]);
            }
            global::damages.push_back(dmg);
        }

        std::cout << "Reading map file..." << std::endl;
        std::string map_dirs = "./exp/mcts-hexa/test_maps";
        try {
            std::ifstream t(map_dirs + "/" + m_file);
            if (!t.is_open() || !t.good()) {
                std::cerr << "Exception while reading the map file: " << m_file << std::endl;
                return 1;
            }
            map_string = std::string((std::istreambuf_iterator<char>(t)),
                std::istreambuf_iterator<char>());
        }
        catch (...) {
            std::cerr << "Exception while reading the map file: " << map_file << std::endl;
            return 1;
        }
    }

    // Intialize map
    Eigen::Vector3d robot_state;
    std::vector<Eigen::Vector3d> goal_states;
    std::tie(goal_states, robot_state) = init_map(map_string);

#ifndef ROBOT
    std::cout << "Initializing simulation" << std::endl;
    // initilisation of the simulation and the simulated robot
    const char* env_p = std::getenv("RESIBOTS_DIR");
    if (env_p) //if the environment variable exists
        init_simu(std::string(env_p) + "/share/hexapod_models/URDF/pexod.urdf", global::damages);
    else //if it does not exist, we might be running this on the cluster
        init_simu("/nfs/hal01/kchatzil/Workspaces/ResiBots/share/hexapod_models/URDF/pexod.urdf", global::damages);
#else
    // TO-DO: Make multiple targets for real hexapod
    // Init ROS
    ros::init(argc, argv, "hexapod_mcts");
    ros::NodeHandle n;

    global::hexa = std::make_shared<hexapod_ros::Hexapod>(n);

    ros::AsyncSpinner spinner(4);
    spinner.start();

// goal_states.clear();
// goal_states.push_back(get_tf("/odom", "/target_frame"));
#endif

    init_simu_world(robot_state);
#ifdef ROBOT
    // std::cout << "Target: " << goal_states[0].transpose() << std::endl;
    std::cout << "Transform: " << global::transform << std::endl;
// Eigen::Vector3d pos;
// pos << goal_states[0](0), goal_states[0](1), 1.0;
// pos = global::transform * pos;
// goal_states[0] << pos(0), pos(1), goal_states[0](2) + global::orig_theta;
#endif
    std::cout << "Robot starting: " << robot_state.transpose() << "\nGoals: ";
    for (auto g : goal_states)
        std::cout << g.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;

    if (exp_folder.empty()) {
        std::ofstream exp_file("exp.dat");
        exp_file << archive_file.substr(archive_file.find_last_of("/") + 1) << std::endl
                 << map_file.substr(map_file.find_last_of("/") + 1) << std::endl
                 << removed_legs.size() << " " << shortened_legs.size() << std::endl;
        for (size_t i = 0; i < removed_legs.size(); i++)
            exp_file << removed_legs[i] << " ";
        if (removed_legs.size() > 0)
            exp_file << std::endl;
        for (size_t i = 0; i < shortened_legs.size(); i++)
            exp_file << shortened_legs[i] << " ";
        if (shortened_legs.size() > 0)
            exp_file << std::endl;
        exp_file << robot_state(0) << " " << robot_state(1) << " " << robot_state(2) << std::endl;
        for (auto g : goal_states)
            exp_file << g(0) << " " << g(1) << " " << g(2) << std::endl;

        // hexa_init();
        if (Params::learning()) {
            write_gp("gp_0.dat");
        }

        std::ofstream results_file("results.dat");
        results_file << goal_states.size() << std::endl;
        // Max trials are 100
        bool found = true;
        size_t n_iter, n_cols;
        for (size_t i = 0; i < goal_states.size(); i++) {
            if (!found) {
#ifdef ROBOT
                // results_file << "0 100 1000" << std::endl;
                // continue;
                global::robot_pose << goal_states[i - 1](0), goal_states[i - 1](1), robot_state(2);
                std::cout << "Reset robot and press any key..." << std::endl;
                std::cin.get();
#else
                Eigen::Vector2d state;
                // Just as a safety, i will always be bigger than 0
                if (i > 0)
                    state << goal_states[i - 1](0), goal_states[i - 1](1);
                else
                    state << robot_state(0), robot_state(1);
                // #ifndef ROBOT
                global::simulated_robot->skeleton()->setPosition(0, 0.0);
                global::simulated_robot->skeleton()->setPosition(1, 0.0);
                global::simulated_robot->skeleton()->setPosition(2, robot_state(2));
                global::simulated_robot->skeleton()->setPosition(3, state(0));
                global::simulated_robot->skeleton()->setPosition(4, state(1));
                global::simulated_robot->skeleton()->setPosition(5, 0.2);
                global::simulated_robot->skeleton()->setVelocities(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getVelocities().size()));
                global::simulated_robot->skeleton()->setAccelerations(Eigen::VectorXd::Zero(global::simulated_robot->skeleton()->getAccelerations().size()));
                global::simulated_robot->skeleton()->clearExternalForces();
                global::simulated_robot->skeleton()->clearInternalForces();

                global::simu->controller().set_parameters(std::vector<double>(36, 0.0));
                global::simu->run(1.0, true, false);
                // #endif

                global::robot_pose << state(0), state(1), robot_state(2);

// #ifdef ROBOT
//                 std::cout << "Press enter to reset...." << std::endl;
//                 std::cin.get();
//                 Eigen::Vector3d robot_pose = get_tf("/odom", "/base_link");
//                 double c = std::cos(robot_pose(2)), s = std::sin(robot_pose(2));
//                 Eigen::MatrixXd r(2, 2);
//                 r << c, -s, s, c;
//                 r.transposeInPlace();
//                 Eigen::VectorXd d(2);
//                 d << robot_pose(0), robot_pose(1);
//                 d = r * d;
//                 Eigen::MatrixXd Twr(3, 3);
//                 Twr << r(0, 0), r(0, 1), -d(0), r(1, 0), r(1, 1), -d(1), 0, 0, 1;
//                 c = std::cos(global::robot_pose(2));
//                 s = std::sin(global::robot_pose(2));
//                 Eigen::MatrixXd Tmr(3, 3);
//                 Tmr << c, -s, global::robot_pose(0), s, c, global::robot_pose(1), 0, 0, 1;
//                 Eigen::MatrixXd tr = Tmr * Twr; //.inverse();
//                 global::transform = tr;
//                 global::orig_theta = std::atan2(-tr(0, 1), tr(0, 0));
// #endif

#endif
            }
            std::tie(found, n_iter, n_cols) = reach_target(goal_states[i], 100);
            results_file << found << " " << n_iter << " " << n_cols << std::endl;
        }
        results_file.close();
    }
    else {
        std::cout << "Reading controllers..." << std::endl;
        std::vector<std::vector<double>> controllers;
        std::ifstream ctrl_file(exp_folder + "/ctrl_1.dat");
        std::string str;
        while (std::getline(ctrl_file, str)) {
            std::vector<double> a;
            std::stringstream in(str);
            double temp;
            while (in >> temp) {
                a.push_back(temp);
            }
            controllers.push_back(a);
        }

        std::cout << "Reading times..." << std::endl;
        std::vector<double> times;
        std::ifstream time_file(exp_folder + "/misc_1.dat");
        while (std::getline(time_file, str)) {
            std::vector<double> a;
            std::stringstream in(str);
            double temp;
            while (in >> temp) {
                a.push_back(temp);
            }
            times.push_back(a.back());
        }

        std::cout << "-1 " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;
        for (size_t i = 0; i < controllers.size(); i++) {
            execute(Eigen::VectorXd::Map(controllers[i].data(), controllers[i].size()), times[i], false);
            std::cout << i << " " << global::robot_pose(0) << " " << global::robot_pose(1) << " " << global::robot_pose(2) << std::endl;
        }
    }

#ifdef ROBOT
    global::hexa.reset();
#endif
    global::simulated_robot.reset();
    global::global_robot.reset();

    return 0;
}
