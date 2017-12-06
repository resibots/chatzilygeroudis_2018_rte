#include <iostream>

#include <sferes/phen/parameters.hpp>
#include <sferes/gen/evo_float.hpp>
#include <sferes/gen/sampled.hpp>
#include <sferes/stat/pareto_front.hpp>
#include <sferes/modif/dummy.hpp>
#include <sferes/run.hpp>
#include <modules/map_elites/map_elites.hpp>
#include <modules/map_elites/fit_map.hpp>
#include <stat_map_binary.hpp>
#include <modules/map_elites/stat_progress.hpp>
#include <libfastsim/fastsim.hpp>

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
        SFERES_CONST size_t behav_dim = 2;
        SFERES_ARRAY(size_t, behav_shape, 60, 60);
        SFERES_CONST float epsilon = 0.0;
        SFERES_CONST float min = -100.0f;
        SFERES_CONST float max = 100.0f;
    };

    struct sampled {
        SFERES_ARRAY(float, values, 0.00, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30, 0.35,
            0.40, 0.45, 0.50, 0.55, 0.60, 0.65, 0.70, 0.75, 0.80, 0.85,
            0.90, 0.95, 1);
        SFERES_CONST float mutation_rate = 0.05f;
        SFERES_CONST float cross_rate = 0.00f;
        SFERES_CONST bool ordered = false;
        SFERES_CONST float min = -1.0f;
        SFERES_CONST float max = 1.0f;
    };
    struct evo_float {
        SFERES_CONST float cross_rate = 0.0f;
        SFERES_CONST float mutation_rate = 1.0f / 2.0f;
        SFERES_CONST float eta_m = 10.0f;
        SFERES_CONST float eta_c = 10.0f;
        SFERES_CONST mutation_t mutation_type = polynomial;
        SFERES_CONST cross_over_t cross_over_type = sbx;
    };
    struct pop {
        SFERES_CONST unsigned size = 200;
        SFERES_CONST unsigned init_size = 200;
        SFERES_CONST unsigned nb_gen = 100001;
        SFERES_CONST int dump_period = 1000;
        SFERES_CONST int initial_aleat = 1;
    };
    struct parameters {
        SFERES_CONST float min = 0.0f;
        SFERES_CONST float max = 1.0f;
    };
};

namespace global {
    std::string filename = "./exp/map_elites_mobile/map.pbm";
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
    std::vector<float> extra() { return _extra; }

protected:
    bool _dead;
    std::vector<double> _ctrl;
    float _angle;
    std::vector<float> _extra;

    template <typename Indiv>
    void _eval(Indiv & indiv)
    {
        // copy of controler's parameters
        _ctrl.clear();
        for (size_t i = 0; i < 2; i++)
            _ctrl.push_back(indiv.data(i) * (Params::sampled::max - Params::sampled::min) + Params::sampled::min);

        // launching the simulation
        fastsim::Posture init_pos(200, 200, 0);
        boost::shared_ptr<fastsim::Map> m = boost::shared_ptr<fastsim::Map>(new fastsim::Map(global::filename.c_str(), 400));
        fastsim::Robot robot(20.0f, init_pos);

        fastsim::Display system(m, robot);

        // TO-DO: Define max steps
        int steps = 100;
        for (int i = 0; i < steps; ++i) {
            system.update();
            robot.move(_ctrl[0], _ctrl[1], m);
        }

        double x, y;
        fastsim::Posture pos = robot.get_pos();
        x = pos.x() - init_pos.x();
        y = pos.y() - init_pos.y();

        _angle = angle_dist(init_pos.theta(), pos.theta());
        // std::cout << x << " " << y << " " << _angle << std::endl;

        _extra.resize(3);
        _extra[0] = x;
        _extra[1] = y;
        _extra[2] = _angle;

        std::vector<float> desc;

        int desc_size = 2;

        // if (simu.covered_distance() < -1000) {
        //     this->_dead = true;
        //     desc.resize(desc_size, 0.0);
        //     this->_value = -1000;
        // }
        // else {

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

        double angle_diff = std::abs(angle_dist(beta, _angle));

        this->_value = -angle_diff;

        // Descriptor
        desc.resize(desc_size, 0.0);

        double d = Params::ea::max - Params::ea::min;
        x = (x - Params::ea::min) / d;
        y = (y - Params::ea::min) / d;

        desc[desc_size - 2] = x;
        desc[desc_size - 1] = y;
        // }

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

    // typedef gen::Sampled<2, Params> gen_t;
    typedef gen::EvoFloat<2, Params> gen_t;
    typedef FitAdapt<Params> fit_t;
    typedef phen::Parameters<gen_t, fit_t, Params> phen_t;

    typedef boost::fusion::vector<sferes::stat::MapBinaryNew<phen_t, Params>, sferes::stat::MapProgress<phen_t, Params>> stat_t;
    typedef modif::Dummy<> modifier_t;
    typedef ea::MapElites<phen_t, eval_t, stat_t, modifier_t, Params> ea_t;

    ea_t ea;

    const char* env_p = std::getenv("RESIBOTS_DIR");
    if (!env_p) //if it does not exist, we might be running this on the cluster
        global::filename = "/nfs/hal01/kchatzil/Workspaces/ResiBots/source/sferes2/exp/map_elites_mobile/map.pbm";

    std::cout << "start run" << std::endl;
    run_ea(argc, argv, ea);
    std::cout << "end of run" << std::endl;
    return 0;
}
