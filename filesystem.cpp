#include "filesystem.h"
#include <stdexcept>
#include <filesystem>


namespace filesystem_extra {

    std::vector<std::string> get_dirs_list(std::filesystem::path path){
        std::vector<std::string> result;
        if (!std::filesystem::is_directory(path))
            throw std::runtime_error("Input directory does not exist!");

        for(auto const& entry : std::filesystem::directory_iterator(path))
            if(entry.is_directory())
                result.push_back(entry.path().filename());


        return result;
    }

    std::vector<std::string> get_files_list(std::string path){
        std::vector<std::string> result;
        if (!std::filesystem::is_directory(path))
            throw std::runtime_error("Input directory does not exist!");

        for(auto const& entry : std::filesystem::directory_iterator(path))
            if(entry.is_regular_file())
                result.push_back(entry.path().filename());

        return result;
    }

}


