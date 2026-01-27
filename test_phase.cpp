#include "src/ymbot_amp_devel/include/ymbot_amp_devel/phase_generator.hpp"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "Testing PhaseGenerator..." << std::endl;
    
    PhaseGenerator pg;
    
    // 测试基本功能
    Eigen::Vector3d base_lin_vel(1.0, 0.0, 0.0);
    Eigen::Vector3d cmd(1.5, 0.0, 0.2);
    
    Eigen::VectorXd phase_obs = pg.generatePhase(base_lin_vel, cmd, 0.02);
    
    std::cout << "Phase observations: [";
    for (int i = 0; i < phase_obs.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << phase_obs(i);
        if (i < phase_obs.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
    
    std::cout << "PhaseGenerator test completed successfully!" << std::endl;
    return 0;
}