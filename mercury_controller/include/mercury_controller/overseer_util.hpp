#pragma once

#include <yaml-cpp/yaml.h>
#include <eigen3/Eigen/Dense>

using v3d = Eigen::Vector3d;
  

/*
    Helper function to make a yaml node into a vector, used for thrusters
*/
template<typename T>
T getYamlNodeAs(const YAML::Node& n, const std::vector<std::string>& keywords)
{
    if(keywords.empty())
    {
        throw std::runtime_error("getYamlNodeAs() requires at least one keyword.");
    }

    YAML::Node node = YAML::Clone(n);
    
    try
    {
        for(std::string s : keywords)
        {
            node = node[s];
        }

        return node.as<T>();
    } catch(YAML::Exception& e)
    {
        std::string msg = "Failed to parse value at tag " + keywords[0];
        for(size_t i = 1; i < keywords.size(); i++)
        {
            msg +=  " -> " + keywords[i];
        }
        
        msg += ": " + std::string(e.what());
        throw std::runtime_error(msg);
    }
}
/*
    Helper function to quickly go to Eigen 3d vector object
*/
Eigen::Vector3d std2v3d(std::vector<double> stdVect);
