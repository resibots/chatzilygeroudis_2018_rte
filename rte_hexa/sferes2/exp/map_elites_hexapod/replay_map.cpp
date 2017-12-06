#include <iostream>
#include <tuple>
#include <modules/map_elites/binary_map.hpp>
#include <hexapod_dart/hexapod_dart_simu.hpp>

namespace global {
    std::shared_ptr<hexapod_dart::Hexapod> global_robot;
    std::vector<hexapod_dart::HexapodDamage> damages;
};

// <parallel, point of intersection>
std::tuple<bool, Eigen::Vector2d> line_intersection(Eigen::Vector2d p0, Eigen::Vector2d p1, Eigen::Vector2d q0, Eigen::Vector2d q1)
{
    double epsilon = 1e-2;
    Eigen::Vector2d u = (p1 - p0).normalized();
    Eigen::Vector2d v = (q1 - q0).normalized();
    Eigen::Vector2d w = p0 - q0;
    double uTv = u(0) * v(1) - u(1) * v(0);
    if (std::abs(uTv) < epsilon)
        return std::make_tuple(true, Eigen::Vector2d(0.0, 0.0));
    double si = (v(1) * w(0) - v(0) * w(1)) / (v(0) * u(1) - v(1) * u(0));
    return std::make_tuple(false, p0 + si * u);
}

void init_simu(std::string robot_file, std::vector<hexapod_dart::HexapodDamage> damages = std::vector<hexapod_dart::HexapodDamage>())
{
    global::global_robot = std::make_shared<hexapod_dart::Hexapod>(robot_file, damages);
}

void replay(const std::vector<double>& ctrl)
{
    // launching the simulation
    auto robot = global::global_robot->clone();
    using safe_t = boost::fusion::vector<hexapod_dart::safety_measures::BodyColliding, hexapod_dart::safety_measures::MaxHeight, hexapod_dart::safety_measures::TurnOver>;
    using desc_t = boost::fusion::vector<hexapod_dart::descriptors::PositionTraj, hexapod_dart::descriptors::RotationTraj, hexapod_dart::descriptors::BodyOrientation>;
    hexapod_dart::HexapodDARTSimu<hexapod_dart::safety<safe_t>, hexapod_dart::desc<desc_t>> simu(ctrl, robot);
    simu.set_desc_dump(1);
    simu.run(3);

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
    bool fwd = (pos_traj.back()(0) > 0.0);

    bool parallel;
    Eigen::Vector2d inter;
    std::tie(parallel, inter) = line_intersection(Eigen::Vector2d(pos_traj.back()(0), pos_traj.back()(1)), Eigen::Vector2d(pos_traj.back()(0), pos_traj.back()(1)) + perp, Eigen::Vector2d(0.0, 0.0), baseline);
    if (!parallel)
        std::cout << "Radius: " << inter(1) << std::endl;
    else if (fwd)
        std::cout << "Radius: MAX" << std::endl;
    else
        std::cout << "Radius: MIN" << std::endl;

    std::cout << value << std::endl;
    std::cout << pos_traj.back().transpose() << std::endl;
    std::cout << angle << std::endl;
}

int main(int argc, char** argv)
{
    std::cout << "init SIMU" << std::endl;
    // initilisation of the simulation and the simulated robot
    init_simu(std::string(std::getenv("RESIBOTS_DIR")) + "/share/hexapod_models/URDF/pexod.urdf", global::damages);

    if (argc < 3) {
        std::cout << "Usage: map pos" << std::endl;
        return 0;
    }

    binary_map::BinaryMap m = binary_map::load(argv[1]);
    std::vector<binary_map::Elem> v = m.elems;
    int pos = std::stoi(argv[2]);
    if (pos >= 0 && pos < v.size()) {
        auto x = v[pos];
        std::vector<double> ctrl(x.phen.size(), 0.0);
        std::copy(x.pos.begin(), x.pos.end(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << x.fit << " " << x.extra << std::endl;
        std::copy(x.phen.begin(), x.phen.end(), ctrl.begin());
        replay(ctrl);
    }

    return 0;
}
