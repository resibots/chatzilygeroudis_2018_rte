#include <iostream>
#include <tuple>
#include <binary_map.hpp>
#include <libfastsim/fastsim.hpp>

namespace global {
    std::string filename = "./exp/map_elites_mobile/map.pbm";
}

double angle_dist(double a, double b)
{
    double theta = b - a;
    while (theta < -M_PI)
        theta += 2 * M_PI;
    while (theta > M_PI)
        theta -= 2 * M_PI;
    return theta;
}

void replay(const std::vector<double>& ctrl)
{
    std::vector<double> vel = ctrl;
    for (auto& v : vel)
        v = v * 2.0 - 1.0;
    // launching the simulation
    fastsim::Posture init_pos(200, 200, 0);
    boost::shared_ptr<fastsim::Map> m = boost::shared_ptr<fastsim::Map>(new fastsim::Map(global::filename.c_str(), 400));
    fastsim::Robot robot(20.0f, init_pos);

    fastsim::Display system(m, robot);

    // TO-DO: Define max steps
    int steps = 100;
    for (int i = 0; i < steps; ++i) {
        system.update();
        robot.move(vel[0], vel[1], m);
    }

    double x, y;
    fastsim::Posture pos = robot.get_pos();
    x = pos.x() - init_pos.x();
    y = pos.y() - init_pos.y();

    double angle = angle_dist(init_pos.theta(), pos.theta());

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

    double angle_diff = std::abs(angle_dist(beta, angle));

    double value = -angle_diff;

    std::cout << value << std::endl;
    std::cout << x << " " << y << std::endl;
    std::cout << angle << std::endl;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: map pos" << std::endl;
        return 0;
    }

    const char* env_p = std::getenv("RESIBOTS_DIR");
    if (!env_p) //if it does not exist, we might be running this on the cluster
        global::filename = "/nfs/hal01/kchatzil/Workspaces/ResiBots/source/sferes2/exp/map_elites_mobile/map.pbm";

    binary_map::BinaryMap m = binary_map::load(argv[1]);
    std::vector<binary_map::Elem> v = m.elems;
    int pos = std::stoi(argv[2]);
    if (pos >= 0 && pos < v.size()) {
        auto x = v[pos];
        // for (size_t i = 0; i < x.pos.size(); i++) {
        //     std::cout << (x.pos[i] / double(m.dims[i])) * 200.0 - 100.0 << " ";
        // }
        std::vector<double> ctrl(x.phen.size(), 0.0);
        // std::copy(x.pos.begin(), x.pos.end(), std::ostream_iterator<double>(std::cout, " "));
        std::cout << x.fit << std::endl
                  << x.extra[0] << " " << x.extra[1] << std::endl
                  << x.extra[2] << std::endl;
        std::copy(x.phen.begin(), x.phen.end(), ctrl.begin());
        replay(ctrl);
    }

    return 0;
}
