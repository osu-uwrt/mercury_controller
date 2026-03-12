#include <overseer_util.hpp>
using v3d = Eigen::Vector3d;
  

/*
    Helper function to quickly go to Eigen 3d vector object
*/
Eigen::Vector3d std2v3d(std::vector<double> stdVect)
{
    return v3d(stdVect[0], stdVect[1], stdVect[2]);
}
