#ifndef FILESYSTEM_H
#define FILESYSTEM_H


#include <vector>
#include <string>

namespace filesystem_extra {

    std::vector<std::string> get_dirs_list(std::string path);
    std::vector<std::string> get_files_list(std::string path);

}

#endif // FILESYSTEM_H
