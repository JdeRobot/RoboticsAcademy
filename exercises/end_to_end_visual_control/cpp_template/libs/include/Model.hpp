#ifndef MODEL_HPP_
#define MODEL_HPP_

#include <string>
#include <filesystem>

namespace models {

inline const std::string model_dir_path = "/workspace/code";

inline std::string model_path_func(const std::string& name) {
    std::filesystem::path dir(model_dir_path);
    std::filesystem::path file(name);
    return (dir / file).string();
}

}

#endif