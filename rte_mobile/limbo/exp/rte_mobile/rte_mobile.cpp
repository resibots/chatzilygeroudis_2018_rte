#include <limbo/limbo.hpp>
#include <libfastsim/fastsim.hpp>
#include <map_elites/binary_map.hpp>
#include <mcts/uct.hpp>
#include <boost/program_options.hpp>
#include <astar/a_star.hpp>
#include <algorithm>
#include <vector>
#include <chrono>

#define ARCHIVE_SIZE 2

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

    MCTS_PARAM(double, cell_size, 40.0);
    MCTS_PARAM(double, robot_radius, 19.0);
    MCTS_PARAM(int, time_steps, 100);

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

#ifndef TEXPLORE
template <typename Params>
struct MeanArchive {
    MeanArchive(size_t dim_out = 4) {}

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
#else
template <typename Params>
struct MeanArchive : public mean::NullFunction<Params> {
    MeanArchive(size_t dim_out = 4) : mean::NullFunction<Params>(dim_out) {}
};
#endif

struct SimpleObstacle {
    double _x, _y, _radius, _radius_sq;

    SimpleObstacle(double x, double y, double r = 0.1) : _x(x), _y(y), _radius(r), _radius_sq(r * r) {}
};

using kernel_t = kernel::Exp<Params>;
using mean_t = MeanArchive<Params>;
using GP_t = model::GP<Params, kernel_t, mean_t>;

namespace global {
    GP_t gp_model(ARCHIVE_SIZE, 4);

    // fastsim things
    boost::shared_ptr<fastsim::Map> map;
    std::shared_ptr<fastsim::Robot> robot;
    std::shared_ptr<fastsim::Display> system;

    // Robot pose (x,y,theta)
    Eigen::Vector3d robot_pose;

    std::vector<SimpleObstacle> obstacles;
    size_t map_size, map_size_x, map_size_y;
    size_t target_num;

    // statistics
    std::ofstream robot_file, ctrl_file, iter_file, misc_file;
}

bool collides(double x, double y, double r = Params::robot_radius())
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
    return (collides(start(0), start(1), r) || collides(end(0), end(1), r) || collides_segment(A, B) || collides_segment(C, D)); //collides_segment(A, B) || collides_segment(B, C) || collides_segment(C, D) || collides_segment(D, A));
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

bool astar_collides(int x, int y, int x_new, int y_new)
{
    Eigen::Vector2d s(x * Params::cell_size(), y * Params::cell_size());
    Eigen::Vector2d t(x_new * Params::cell_size(), y_new * Params::cell_size());
    return collides(s, t, Params::robot_radius()); // * 1.5);
}

template <typename State, typename Action>
struct DefaultPolicy {
    // Action operator()(const std::shared_ptr<State>& state)
    Action operator()(const State* state)
    {
        size_t N = 100;
        double dx = state->_x - Params::goal_x();
        double dy = state->_y - Params::goal_y();
        double d = dx * dx + dy * dy;
        if (d <= Params::cell_size() * Params::cell_size()) {
            Action best_action = state->random_action();
            double best_value = std::numeric_limits<double>::max();
            for (size_t i = 0; i < N; i++) {
                Action act = state->random_action();
                auto final = state->move(act, true);
                double dx = final._x - Params::goal_x();
                double dy = final._y - Params::goal_y();
                double val = std::sqrt(dx * dx + dy * dy);
                if (collides(final._x, final._y))
                    val = std::numeric_limits<double>::max();
                // else {
                //     if (Params::archiveparams::archive_record.find(act._desc) != Params::archiveparams::archive_record.end()) {
                //         auto rec = Params::archiveparams::archive_record[act._desc];
                //         Eigen::VectorXd mu;
                //         double sigma;
                //         std::tie(mu, sigma) = global::gp_model.query(act._desc);
                //         double d1, d2, d3, d4;
                //         d1 = mu(0) - rec.x;
                //         d2 = mu(1) - rec.y;
                //         d3 = mu(2) - rec.cos_theta;
                //         d4 = mu(3) - rec.sin_theta;
                //         val -= std::sqrt(d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4) / std::sqrt(0.01 + sigma);
                //     }
                //     else {
                //         val -= 100;
                //     }
                // }
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
            } while (b && n < 10);

            if (n < 10)
                best_pos = new_best;
        }

        Action best_action = state->random_action();
        double best_value = std::numeric_limits<double>::max();
        for (size_t i = 0; i < N; i++) {
            Action act = state->random_action();
            auto final = state->move(act, true);
            double dx = final._x - best_pos(0);
            double dy = final._y - best_pos(1);
            double val = std::sqrt(dx * dx + dy * dy);
            if (collides(final._x, final._y))
                val = std::numeric_limits<double>::max();
            // else {
            //     if (Params::archiveparams::archive_record.find(act._desc) != Params::archiveparams::archive_record.end()) {
            //         auto rec = Params::archiveparams::archive_record[act._desc];
            //         Eigen::VectorXd mu;
            //         double sigma;
            //         std::tie(mu, sigma) = global::gp_model.query(act._desc);
            //         double d1, d2, d3, d4;
            //         d1 = mu(0) - rec.x;
            //         d2 = mu(1) - rec.y;
            //         d3 = mu(2) - rec.cos_theta;
            //         d4 = mu(3) - rec.sin_theta;
            //         val -= std::sqrt(d1 * d1 + d2 * d2 + d3 * d3 + d4 * d4) / std::sqrt(0.01 + sigma);
            //     }
            //     else {
            //         val -= 100;
            //     }
            // }
            if (val < best_value) {
                best_value = val;
                best_action = act;
            }
        }

        return best_action;
    }
};

template <typename Params>
struct MobileAction {
    Eigen::VectorXd _desc;

    MobileAction() {}
    MobileAction(const Eigen::VectorXd& desc) : _desc(desc) {}

    MobileAction(const MobileAction& other)
    {
        _desc = other._desc;
    }

    bool operator==(const MobileAction& other) const
    {
#ifndef TEXPLORE
        return (typename Params::archiveparams::classcompequal()(_desc, other._desc));
#else
        return (_desc - other._desc).norm() < 1e-8;
#endif
    }
};

template <typename Params>
struct MobileState {
    double _x, _y, _theta;
    static constexpr double _epsilon = 1e-3;

    MobileState()
    {
        _x = _y = _theta = 0;
    }

    MobileState(double x, double y, double theta)
    {
        _x = x;
        _y = y;
        _theta = theta;
    }

    bool valid(const MobileAction<Params>& act) const
    {
        // MobileState<Params> tmp = move(act);
        // // Check if state is outside of bounds
        // if (tmp._x < 0.0 || tmp._x > global::map_size * Params::cell_size() || tmp._y < 0.0 || tmp._y > global::map_size * Params::cell_size())
        //     return false;
        //
        // if (collides(tmp._x, tmp._y) || collides(Eigen::Vector2d(_x, _y), Eigen::Vector2d(tmp._x, tmp._y)))
        //     return false;
        return true;
    }

    MobileAction<Params> next_action() const
    {
        // #ifndef TEXPLORE
        return DefaultPolicy<MobileState<Params>, MobileAction<Params>>()(this);
        // #else
        //         return random_action();
        // #endif
    }

    MobileAction<Params> random_action() const
    {
#ifndef TEXPLORE
        static tools::rgen_int_t rgen(0, Params::archiveparams::archive.size() - 1);
        typedef typename Params::archiveparams::archive_t::const_iterator archive_it_t;

        MobileAction<Params> act;
        do {
            archive_it_t it = Params::archiveparams::archive.begin();
            std::advance(it, rgen.rand());
            Eigen::VectorXd desc = Eigen::VectorXd::Map(it->first.data(), it->first.size());
            act._desc = desc;
        } while (!valid(act));
#else
        static tools::rgen_double_t rgen(-1.0, 1.0);

        MobileAction<Params> act;
        do {
            act._desc = tools::random_vector(2).array() * 2.0 - 1.0;
            for (int i = 0; i < act._desc.size(); i++) {
                act._desc[i] = std::round(10.0 * act._desc[i]) / 10.0;
            }
        } while (!valid(act));
#endif
        return act;
    }

    MobileState move(const MobileAction<Params>& action, bool no_noise = false) const
    {
        double x_new, y_new, theta_new;
        Eigen::VectorXd mu;
        double sigma;
        std::tie(mu, sigma) = global::gp_model.query(action._desc);
#ifndef TEXPLORE
        if (!no_noise) {
            // std::cout << mu.transpose() << std::endl;
            // mu(0) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(0), sigma)));
            // mu(1) = std::max(-1.5, std::min(1.5, gaussian_rand(mu(1), sigma)));
            // mu(2) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(2), sigma)));
            // mu(3) = std::max(-1.0, std::min(1.0, gaussian_rand(mu(3), sigma)));
            for (int i = 0; i < mu.size(); i++)
                mu(i) = gaussian_rand(mu(i), std::sqrt(sigma));
            mu(2) = std::max(-1.0, std::min(1.0, mu(2)));
            mu(3) = std::max(-1.0, std::min(1.0, mu(3)));
            // std::cout << "To: " << mu.transpose() << std::endl;
        }
#endif
        double theta = std::atan2(mu(3), mu(2));
        // double theta = (mu(3) > 0) ? std::acos(mu(2)) : -std::acos(mu(2));
        // std::cout << theta << " vs " << angleRadian << std::endl;
        double c = std::cos(_theta), s = std::sin(_theta);
        Eigen::MatrixXd tr(3, 3);
        tr << c, -s, _x, s, c, _y, 0, 0, 1;

        // double c2 = std::cos(theta), s2 = std::sin(theta);
        // Eigen::MatrixXd tr2(3, 3);
        // tr2 << c2, -s2, mu(0), s2, c2, mu(1), 0, 0, 1;

        // Eigen::MatrixXd TR = tr; //tr2.inverse() * tr;
        Eigen::Vector3d tr_pos, tmp;
        tr_pos << mu(0), mu(1), 1.0;
        // tr_pos << 0.0, 0.0, 1.0;
        tmp = tr * tr_pos;
        x_new = tmp(0);
        y_new = tmp(1);
        // theta_new = std::atan2(TR(1, 0), TR(0, 0));
        theta_new = _theta + theta;

        while (theta_new < -M_PI)
            theta_new += 2 * M_PI;
        while (theta_new > M_PI)
            theta_new -= 2 * M_PI;

        // std::cout << "(" << _x << "," << _y << "," << _theta << ") with (" << mu(0) << "," << mu(1) << "," << mu(2) << ") -> (" << x_new << "," << y_new << "," << theta_new << ")" << std::endl;
        return MobileState(x_new, y_new, theta_new);
    }

    bool terminal() const
    {
        // Check if state is outside of bounds
        if (_x < 0.0 || _x >= global::map_size_x * Params::cell_size() || _y < 0.0 || _y >= global::map_size_y * Params::cell_size()) {
            // std::cout << "Out of bounds" << std::endl;
            return true;
        }
        // Check if state is goal
        if (goal()) {
            // std::cout << "Goal" << std::endl;
            return true;
        }
        // Check if state is colliding
        if (collides(_x, _y)) {
            // std::cout << "Collision" << std::endl;
            return true;
        }
        return false;
    }

    bool goal() const
    {
        double dx = _x - Params::goal_x();
        double dy = _y - Params::goal_y();
        double threshold_xy = Params::cell_size(); // * Params::cell_size(); // / 4.0;
        if (std::sqrt(dx * dx + dy * dy) < threshold_xy)
            return true;
        return false;
    }

    bool operator==(const MobileState& other) const
    {
        double dx = _x - other._x;
        double dy = _y - other._y;
        double dth = std::abs(angle_dist(other._theta, _theta));
        return (std::sqrt(dx * dx + dy * dy) < Params::cell_size() && dth < 0.2);
    }
};

struct RewardFunction {
    template <typename State>
    double operator()(std::shared_ptr<State> from_state, MobileAction<Params> action, std::shared_ptr<State> to_state)
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
        // double dx = to_state->_x - Params::goal_x();
        // double dy = to_state->_y - Params::goal_y();
        // return -(dx * dx + dy * dy);
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
        assert(ctrl.size() == 2);
        for (auto& c : ctrl)
            c = c * 2.0 - 1.0;

        Params::archiveparams::elem_archive elem;
        elem.controller = ctrl;
        elem.x = v.extra[0];
        elem.y = v.extra[1];
        elem.cos_theta = std::cos(v.extra[2]);
        elem.sin_theta = std::sin(v.extra[2]);
        // std::cout << elem.x << " " << elem.y << " " << v.extra[2] << std::endl;
        Params::archiveparams::archive[desc] = elem;
    }

    std::cout << "Loaded " << Params::archiveparams::archive.size() << " elements!" << std::endl;

    return true;
}

void execute(const Eigen::VectorXd& desc, int t, bool stat = true)
{
    std::vector<double> d(desc.size(), 0.0);
    Eigen::VectorXd::Map(d.data(), d.size()) = desc;
#ifndef TEXPLORE
    std::vector<double> ctrl = Params::archiveparams::archive[d].controller;
#else
    std::vector<double> ctrl = d;
#endif

    if (stat) {
        // statistics - descriptor
        for (int i = 0; i < desc.size(); i++)
            global::ctrl_file << desc(i) << " ";
        global::ctrl_file << std::endl;
    }

    // This is the damage
    std::cout << ctrl[0] << " " << ctrl[1] / 2.0 << std::endl;

    // Run simulation with damage
    for (int i = 0; i < t; ++i) {
        global::system->update();
        global::robot->reinit();
        global::robot->move(ctrl[0], ctrl[1] / 2.0, global::map, false); // non-sticky walls
    }

    // Get outcome
    fastsim::Posture pos = global::robot->get_pos();

    double x = pos.x();
    double y = pos.y();
    double theta = pos.theta();

    // while (theta < -M_PI)
    //     theta += 2 * M_PI;
    // while (theta > M_PI)
    //     theta -= 2 * M_PI;

    global::robot_pose << x, y, theta;
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

std::tuple<bool, size_t, size_t> reach_target(const Eigen::Vector3d& goal_state, size_t max_iter = std::numeric_limits<size_t>::max())
{
    using Choose = mcts::GreedyValue;

    Params::set_goal_x(goal_state(0));
    Params::set_goal_y(goal_state(1));
    Params::set_goal_theta(goal_state(2));

    global::target_num++;

    // Initialize statistics
    global::robot_file.open("robot_" + std::to_string(global::target_num) + ".dat");
    global::ctrl_file.open("ctrl_" + std::to_string(global::target_num) + ".dat");
    global::iter_file.open("iter_" + std::to_string(global::target_num) + ".dat");
    global::misc_file.open("misc_" + std::to_string(global::target_num) + ".dat");

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
        MobileState<Params> init = MobileState<Params>(global::robot_pose(0), global::robot_pose(1), global::robot_pose(2));
        // DefaultPolicy<MobileState<Params>, MobileAction<Params>>()(&init, true);

        // Run MCTS
        auto tree = std::make_shared<mcts::MCTSNode<Params, MobileState<Params>, mcts::SimpleStateInit<MobileState<Params>>, mcts::SimpleValueInit, mcts::UCTValue<Params>, mcts::UniformRandomPolicy<MobileState<Params>, MobileAction<Params>>, MobileAction<Params>, mcts::SPWSelectPolicy<Params>, mcts::ContinuousOutcomeSelect<Params>>>(init, 1000);

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

        // Get best action/behavior
        auto best = tree->best_action<Choose>();
        std::cout << "val: " << best->value() / double(best->visits()) << std::endl;
        auto other_best = tree->best_action<mcts::GreedyValue>();
        // std::cout << "val without AL: " << other_best->value() / double(other_best->visits()) << std::endl;
        std::cout << "avg: " << (sum / double(tree->children().size())) << std::endl;
        auto tmp = init.move(best->action(), true);
        std::cout << tmp._x << " " << tmp._y << " -> " << tmp._theta << std::endl;
        global::iter_file << n << " " << time_running / 1000.0 << " " << best->value() / double(best->visits()) << " " << other_best->value() / double(other_best->visits()) << " " << (sum / double(tree->children().size())) << " " << tmp._x << " " << tmp._y << " " << tmp._theta << std::endl;

        // Execute in simulation/real robot
        Eigen::Vector3d prev_pose = global::robot_pose;
        execute(best->action()._desc, Params::time_steps());

        std::cout << "Robot " << n << ": " << global::robot_pose.transpose() << std::endl;
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
        global::misc_file << "100" << std::endl;

        // Check collisions/termination
        if (collides(global::robot_pose(0), global::robot_pose(1))) {
            collided = true;
            std::cout << "Collision!" << std::endl;
        }

        double dx = global::robot_pose(0) - Params::goal_x();
        double dy = global::robot_pose(1) - Params::goal_y();
        double threshold_xy = Params::cell_size(); // * Params::cell_size(); // / 4.0;
        if (std::sqrt(dx * dx + dy * dy) < threshold_xy) {
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

        double x = path_width * c + path_width / 2.0;
        double y = path_width * r + path_width / 2.0;

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
            i_th = -M_PI / 2.0; //M_PI;
            init_in_map = true;
        }
        else if (map_string[i] == 'v') {
            i_x = x;
            i_y = y;
            i_th = M_PI / 2.0; //0;
            init_in_map = true;
        }
        else if (map_string[i] == '<') {
            i_x = x;
            i_y = y;
            i_th = M_PI; //-M_PI / 2.0;
            init_in_map = true;
        }
        else if (map_string[i] == '>') {
            i_x = x;
            i_y = y;
            i_th = 0.0; //M_PI / 2.0;
            init_in_map = true;
        }
        else if (map_string[i] != ' ') {
            int t = map_string[i] - '0';
            goals[t] = Eigen::Vector3d(x, y, 0);
            goal_in_map = true;
        }
        c++;
    }

    global::map_size = (r > c) ? r + 1 : c + 1;
    global::map_size_x = c;
    global::map_size_y = r;

    if (!goal_in_map || !init_in_map) {
        std::cerr << "No goal or robot in the map." << std::endl;
        exit(1);
    }

    size_t N = 50;
    std::vector<Eigen::Vector3d> g = generate_targets(Eigen::Vector2d(i_x, i_y), Eigen::Vector2d((c - 1) * Params::cell_size(), (r - 1) * Params::cell_size()), (Eigen::Vector2d(i_x, i_y) - Eigen::Vector2d(goals[1](0), goals[1](1))).norm(), N);
    // g.push_back(goals[1]);

    return std::make_tuple(g, Eigen::Vector3d(i_x, i_y, i_th));
}

void init_simu(const std::string& map_file, const Eigen::Vector3d& robot_state)
{
    fastsim::Posture init_pos(robot_state(0), robot_state(1), robot_state(2));
    // TO-DO: maybe 400 needs to be changed
    global::map = boost::shared_ptr<fastsim::Map>(new fastsim::Map(map_file.c_str(), 800));
    // global::robot = std::make_shared<fastsim::Robot>(Params::robot_radius() * 2.0, init_pos);
    global::robot = std::make_shared<fastsim::Robot>(20.0, init_pos);

    global::system = std::make_shared<fastsim::Display>(global::map, *global::robot);
    global::system->update();

    global::robot_pose = robot_state;
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

Params::archiveparams::archive_t Params::archiveparams::archive;

int main(int argc, char** argv)
{
    mcts::par::init();

    std::string map_string = "";
    std::string map_file = "";
    std::string archive_file = "";
    std::vector<int> removed_legs, shortened_legs;
    bool no_learning = false;

    namespace po = boost::program_options;
    po::options_description desc("Command line arguments");
    desc.add_options()("help,h", "Prints this help message")("archive,m", po::value<std::string>()->required(), "Archive file")("load,l", po::value<std::string>()->required(), "Load map from file")("uct,c", po::value<double>(), "UCT c value (in range (0,+00))")("spw,a", po::value<double>(), "SPW a value (in range (0,1))")("dpw,b", po::value<double>(), "DPW b value (in range (0,1))")("iter,i", po::value<size_t>(), "Number of iteartions to run MCTS")("parallel_roots,p", po::value<size_t>(), "Number of parallel trees in MCTS")("signal_variance,v", po::value<double>(), "Initial signal variance in kernel (squared)")("kernel_scale,d", po::value<double>(), "Characteristic length scale in kernel")("no_learning,n", po::bool_switch(&no_learning), "Do not learn anything");

    try {
        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);
        if (vm.count("help")) {
            std::cout << desc << std::endl;
            return 0;
        }
        po::notify(vm);

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

#ifndef TEXPLORE
    if (!archive_file.empty()) {
        // Loading archive
        std::cout << "Loading archive..." << std::endl;
        load_archive(archive_file);
    }
#endif

    // Intialize map
    Eigen::Vector3d robot_state;
    std::vector<Eigen::Vector3d> goal_states;
    std::tie(goal_states, robot_state) = init_map(map_string);

    std::cout << "Initializing simulation" << std::endl;
    // initilisation of the simulation and the simulated robot
    const char* env_p = std::getenv("RESIBOTS_DIR");
    std::string map_filename;
    // TO-DO: Fix path for cluster
    std::size_t i_found = map_file.find_last_of("/\\");
    std::string name = map_file.substr(i_found + 1);
    name = name.substr(0, name.find_last_of("."));
    if (!env_p) //if it does not exist, we might be running this on the cluster
        map_filename = "/nfs/hal01/kchatzil/Workspaces/ResiBots/source/medrops_uncertain/limbo/exp/rte_mobile/" + name + ".pbm";
    else
        map_filename = "./exp/rte_mobile/" + name + ".pbm";

    init_simu(map_filename, robot_state);

    std::cout << "Robot starting: " << robot_state.transpose() << "\nGoals: ";
    for (auto g : goal_states)
        std::cout << g.transpose() << std::endl;
    std::cout << "--------------------------" << std::endl;

    // hexa_init();
    if (Params::learning()) {
        write_gp("gp_0.dat");
    }

    // // TESTING
    // for (size_t i = 0; i < 50; i++) {
    //     // Eigen::VectorXd pose = tools::random_vector(3);
    //     // pose(0) = pose(0) * global::map_size_x * Params::cell_size();
    //     // pose(1) = pose(1) * global::map_size_y * Params::cell_size();
    //     // pose(2) = pose(2) * 2.0 * M_PI - M_PI;
    //     MobileState<Params> init = MobileState<Params>(robot_state(0), robot_state(1), robot_state(2));
    //     MobileAction<Params> act = init.random_action();
    //     std::vector<double> d(act._desc.size(), 0.0);
    //     Eigen::VectorXd::Map(d.data(), d.size()) = act._desc;
    //     std::vector<double> ctrl = Params::archiveparams::archive[d].controller;
    //     // if (init.terminal())
    //     //     std::cout << pose.transpose() << " -> terminal" << std::endl;
    //     global::robot->set_pos(fastsim::Posture(robot_state(0), robot_state(1), robot_state(2)));
    //     for (int j = 0; j < 100; j++) {
    //         global::system->update();
    //         global::robot->move(ctrl[0], ctrl[1], global::map);
    //     }
    //     // std::cout << global::robot->get_pos().x() << " " << global::robot->get_pos().y() << " " << std::cos(global::robot->get_pos().theta()) << " " << std::sin(global::robot->get_pos().theta()) << std::endl;
    //     // std::cout << global::gp_model.mu(act._desc).transpose() << std::endl;
    //     std::cout << "real: " << global::robot->get_pos().x() << " " << global::robot->get_pos().y() << " " << global::robot->get_pos().theta() << std::endl;
    //     std::cout << "diff: " << angle_dist(robot_state(2), global::robot->get_pos().theta()) << std::endl;
    //     auto final = init.move(act, true);
    //     std::cout << "sim: " << final._x << " " << final._y << " " << final._theta << std::endl;
    //     sleep(2);
    // }

    // // TESTING
    // for (size_t i = 0; i < 50; i++) {
    //     Eigen::VectorXd pose = tools::random_vector(3);
    //     pose(0) = pose(0) * global::map_size_x * Params::cell_size();
    //     pose(1) = pose(1) * global::map_size_y * Params::cell_size();
    //     pose(2) = pose(2) * 2.0 * M_PI - M_PI;
    //     MobileState<Params> init = MobileState<Params>(pose(0), pose(1), pose(2));
    //     if (init.terminal())
    //         std::cout << pose.transpose() << " -> terminal" << std::endl;
    //     global::robot->set_pos(fastsim::Posture(pose(0), pose(1), pose(2)));
    //     global::system->update();
    //     sleep(2);
    // }

    std::ofstream results_file("results.dat");
    results_file << goal_states.size() << std::endl;
    // Max trials are 100
    bool found = true;
    size_t n_iter, n_cols;
    for (size_t i = 0; i < goal_states.size(); i++) {
        if (!found) {
            Eigen::Vector2d state;
            // Just as a safety, i will always be bigger than 0
            if (i > 0)
                state << goal_states[i - 1](0), goal_states[i - 1](1);
            else
                state << robot_state(0), robot_state(1);

            global::robot->set_pos(fastsim::Posture(state(0), state(1), robot_state(2)));
            global::robot_pose << state(0), state(1), robot_state(2);
        }

        std::tie(found, n_iter, n_cols) = reach_target(goal_states[i], 100);
        results_file << found << " " << n_iter << " " << n_cols << std::endl;
    }
    results_file.close();

    return 0;
}
