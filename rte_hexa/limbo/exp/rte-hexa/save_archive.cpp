#include <map_elites/binary_map.hpp>
#include <algorithm>
#include <vector>
#include <fstream>
// #include <map>
//
// struct Params {
//     struct archiveparams {
//         struct elem_archive {
//             double x, y, theta;
//         };
//
//         struct classcomp {
//             bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const
//             {
//                 assert(lhs.size() == 2 && rhs.size() == 2);
//                 int i = 0;
//                 while (i < 1 && std::round(lhs[i] * 1000) == std::round(rhs[i] * 1000)) //lhs[i]==rhs[i])
//                     i++;
//                 return std::round(lhs[i] * 1000) < std::round(rhs[i] * 1000); //lhs[i]<rhs[i];
//             }
//         };
//
//         using archive_t = std::map<std::vector<double>, elem_archive, classcomp>;
//         static archive_t archive;
//     };
// };

bool load_and_save_archive(const std::string& filename, const std::string& to_save)
{
    // Params::archiveparams::archive.clear();
    binary_map::BinaryMap b_map = binary_map::load(filename);
    std::ofstream out_file(to_save);
    size_t j = 0;

    for (auto v : b_map.elems) {
        std::vector<double> desc(v.pos.size(), 0.0);
        std::copy(v.pos.begin(), v.pos.end(), desc.begin());
        for (size_t i = 0; i < desc.size(); i++) {
            desc[i] /= (double)(b_map.dims[i]);
        }

        // Params::archiveparams::elem_archive elem;
        // elem.x = -2.0 + desc[0] * 4.0;
        // elem.y = -2.0 + desc[1] * 4.0;
        // elem.theta = v.extra;
        // Params::archiveparams::archive[desc] = elem;
        double x = -2.0 + desc[0] * 4.0;
        double y = -2.0 + desc[1] * 4.0;
        double th = v.extra;
        double fit = v.fit;
        out_file << x << " " << y << " " << th << " " << fit << std::endl;
        j++;
    }
    out_file.close();

    // std::cout << "Loaded " << Params::archiveparams::archive.size() << " elements!" << std::endl;
    std::cout << "Loaded and saved " << j << " elements!" << std::endl;

    return true;
}

// Params::archiveparams::archive_t Params::archiveparams::archive;

int main(int argc, char** argv)
{
    if (argc != 3) {
        std::cerr << "Usage: save_archive archive_file new_file" << std::endl;
        return 1;
    }

    load_and_save_archive(argv[1], argv[2]);

    return 0;
}
