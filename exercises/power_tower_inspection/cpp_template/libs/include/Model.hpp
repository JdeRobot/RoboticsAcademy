#ifndef INCLUDE_MODEL_HPP_
#define INCLUDE_MODEL_HPP_

#include <string>
#include <filesystem>

namespace models {

inline const std::string model_dir_path = "/workspace/code";

inline std::string model_path_func(const std::string& name) {
    return (std::filesystem::path(model_dir_path) / name).string();
}

}  // namespace models

#endif
