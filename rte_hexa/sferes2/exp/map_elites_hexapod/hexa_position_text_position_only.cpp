// THIS IS A GENERATED FILE - DO NOT EDIT
#define TEXT
#line 1 "/home/kchatzil/ownCloud/PhD/Papers/Conferences/2017-ICRA/code/sferes2/exp/map_elites_hexapod/hexa_position.cpp"
#define POSITION_ONLY
#line 1 "/home/kchatzil/ownCloud/PhD/Papers/Conferences/2017-ICRA/code/sferes2/exp/map_elites_hexapod/hexa_position.cpp"
#include <iostream>
#include <tuple>

#include <sferes/phen/parameters.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/gen/sampled.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <stat_map_binary_new.hpp>
#include <modules/map_elites/stat_progress.hpp>
#include <hexapod_dart/hexapod_dart_simu.hpp>

#ifdef GRAPHIC
#define NO_PARALLEL
#endif

#define NO_MPI

#ifndef NO_PARALLEL
#include <sferes/eval/parallel.hpp>
#ifndef NO_MPI
#include <sferes/eval/mpi.hpp>
#endif
#else
#include <sferes/eval/eval.hpp>
#endif

using namespace sferes;
using namespace sferes::gen::evo_float;

struct Params {
    struct surrogate {
        SFERES_CONST int nb_transf_max = 10;
        SFERES_CONST float tau_div = 0.05f;
    };

    struct ea {
#ifndef POSITION_ONLY
        SFERES_CONST size_t behav_dim = 8;
        SFERES_ARRAY(size_t, behav_shape, 5, 5, 5, 5, 5, 5, 20, 20);
#else
        SFERES_CONST size_t behav_dim = 2;
        SFERES_ARRAY(size_t, behav_shape, 60, 60);
#endif
        SFERES_CONST float epsilon = 0.0;
        SFERES_CONST float max = 2.0f;
        SFERES_CONST float min = -2.0f;
    };

    struct sampled {
        SFERES_ARRAY(float, values, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35,
            0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85,
            0.90, 0.95, 1);
        SFERES_CONST float mutation_rate = 0.05f;
        SFERES_CONST float cross_rate = 0.00f;
        SFERES_CONST bool ordered = false;
    };
    struct evo_float {
        SFERES_CONST float cross_rate = 0.0f;
        SFERES_CONST float mutation_rate = 1.0f / 36.0f;
        SFERES_CONST float eta_m = 10.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
    struct pop {
        SFERES_CONST unsigned size = 200;
        SFERES_CONST unsigned init_size = 200;
        SFERES_CONST unsigned nb_gen = 100001;
        SFERES_CONST int dump_period = 500;
        SFERES_CONST int initial_aleat = 1;
    };
    struct parameters {
        SFERES_CONST float min = 0.0f;
        SFERES_CONST float max = 1.0f;
    };
};

namespace global {
    std::shared_ptr<hexapod_dart::Hexapod> global_robot;
    std::vector<hexapod_dart::HexapodDamage> damages;
};

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

void init_simu(std::string robot_file, std::vector<hexapod_dart::HexapodDamage> damages = std::vector<hexapod_dart::HexapodDamage>())
{
    global::global_robot = std::make_shared<hexapod_dart::Hexapod>(robot_file, damages);
}

FIT_MAP(FitAdapt)
{
public:
    template <typename Indiv>
    void eval(Indiv & indiv)
    {

        this->_objs.resize(2);
        std::fill(this->_objs.begin(), this->_objs.end(), 0);
        this->_dead = false;
        _eval(indiv);
    }

    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        dbg::trace trace("fit", DBG_HERE);

        ar& boost::serialization::make_nvp("_value", this->_value);
        ar& boost::serialization::make_nvp("_objs", this->_objs);
    }

    bool dead() { return _dead; }
    std::vector<double> ctrl() { return _ctrl; }
    float angle() { return _angle; }

protected:
    bool _dead;
    std::vector<double> _ctrl;
    float _angle;

    template <typename Indiv>
    void _eval(Indiv & indiv)
    {
        // copy of controler's parameters
        _ctrl.clear();
        for (size_t i = 0; i < 36; i++)
            _ctrl.push_back(indiv.data(i));
        // launching the simulation
        auto robot = global::global_robot->clone();
        using safe_t = boost::fusion::vector<hexapod_dart::safety_measures::BodyColliding, hexapod_dart::safety_measures::MaxHeight, hexapod_dart::safety_measures::TurnOver>;
        using desc_t = boost::fusion::vector<hexapod_dart::descriptors::PositionTraj, hexapod_dart::descriptors::RotationTraj, hexapod_dart::descriptors::BodyOrientation>;
        hexapod_dart::HexapodDARTSimu<hexapod_dart::safety<safe_t>, hexapod_dart::desc<desc_t>> simu(_ctrl, robot);
        simu.set_desc_dump(1);
        simu.run(3);

        double angle = simu.arrival_angle();
        _angle = angle;

        std::vector<float> desc;

#ifndef POSITION_ONLY
        int desc_size = 8;
#else
        int desc_size = 2;
#endif

        if (simu.covered_distance() < -1000) {
            this->_dead = true;
            desc.resize(desc_size, 0.0);
            this->_value = -1000;
        }
        else {
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

            double angle_diff = std::abs(angle_dist(beta, angle));

            this->_value = -angle_diff;

            // Descriptor
            desc.resize(desc_size, 0.0);
#ifndef POSITION_ONLY
            std::vector<double> bd_orient;
            simu.get_descriptor<hexapod_dart::descriptors::BodyOrientation>(bd_orient);
            for (size_t i = 0; i < bd_orient.size(); i++)
                desc[i] = bd_orient[i];
#endif

            double d = Params::ea::max - Params::ea::min;
            x = (x - Params::ea::min) / d;
            y = (y - Params::ea::min) / d;

            desc[desc_size - 2] = x;
            desc[desc_size - 1] = y;
        }
        // for (auto f : desc)
        //     std::cout << f << " ";
        // std::cout << std::endl;

        this->set_desc(desc);
    }
};

int main(int argc, char** argv)
{
#ifndef NO_PARALLEL
#ifndef NO_MPI
    typedef eval::Mpi<Params> eval_t;
#else
    typedef eval::Parallel<Params> eval_t;
#endif
#else
    typedef eval::Eval<Params> eval_t;
#endif

    typedef gen::Sampled<36, Params> gen_t;
    typedef FitAdapt<Params> fit_t;
    typedef phen::Parameters<gen_t, fit_t, Params> phen_t;

    typedef boost::fusion::vector<sferes::stat::MapBinaryNew<phen_t, Params>, sferes::stat::MapProgress<phen_t, Params>> stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

    ea_t ea;

    std::cout << "init SIMU" << std::endl;
    // initilisation of the simulation and the simulated robot
    const char* env_p = std::getenv("RESIBOTS_DIR");

    if (env_p) //if the environment variable exists
        init_simu(std::string(env_p) + "/share/hexapod_models/URDF/pexod.urdf", global::damages);
    else //if it does not exist, we might be running this on the cluster
        init_simu("/nfs/hal01/kchatzil/Workspaces/ResiBots/share/hexapod_models/URDF/pexod.urdf", global::damages);

    std::cout << "debut run" << std::endl;

    run_ea(argc, argv, ea);
    std::cout << "fin run" << std::endl;

    std::cout << "fin" << std::endl;
    return 0;
}
