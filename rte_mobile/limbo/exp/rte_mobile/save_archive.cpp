#include <map_elites/binary_map.hpp>
#include <algorithm>
#include <vector>
#include <fstream>

bool load_and_save_archive(const std::string& filename, const std::string& to_save)
{
    binary_map::BinaryMap b_map = binary_map::load(filename);
    std::ofstream out_file(to_save);
    size_t j = 0;

    for (auto v : b_map.elems) {
        double x = v.extra[0];
        double y = v.extra[1];
        double th = v.extra[2];
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
