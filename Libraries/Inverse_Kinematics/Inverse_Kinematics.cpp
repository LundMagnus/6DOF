#include <iostream>
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>

double get_actual_angle(double angle) {

    double degrees = angle * 180.0 / M_PI;
    if(degrees < 0) {
        return 360 - degrees;
    } else {
        return degrees;
    } 
}

std::vector<double> IK_solver() 
{
    using namespace KDL;


    struct Link {
        double length;
        Joint::JointType joint;
    };

    std::vector<Link> robot = {
        {0.045, Joint::RotZ},
        {0.113132, Joint::RotY},
        {0.102184, Joint::RotY},
        {0.054058, Joint::RotX},
        {0.061168, Joint::RotY},
        //{0.04, Joint::RotX}
    };

    Chain chain;

    for(const auto& link : robot)
    {
        chain.addSegment(
            Segment(
                Joint(link.joint),
                Frame(Vector(link.length,0,0))
            )
        );
    }

    // Forward kinematics solver
    ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse kinematics solver (position-priority: orientation almost ignored)
    Eigen::Matrix<double, 6, 1> lma_weights;
    lma_weights << 1.0, 1.0, 1.0, 1e-6, 1e-6, 1e-6;
    ChainIkSolverPos_LMA ik_solver(chain, lma_weights);

    // Desired end-effector pose
    KDL::Frame target(Frame::Identity());
    target.p = KDL::Vector(0.150, 0, 0.150);

    // Initial joint guess
    JntArray q_init(chain.getNrOfJoints());
    for(int i = 0; i < chain.getNrOfJoints(); i++) {
        q_init(i) = 0;
    }
    


    // Output solution
    JntArray q_out(chain.getNrOfJoints());

    int ret = ik_solver.CartToJnt(q_init, target, q_out);

    


    if(ret >= 0)
    {
        std::cout << "Solution found:\n";
        for (int i = 0; i < chain.getNrOfJoints(); i++) {
            std::cout << "q" << i << " rad=" << q_out(i) << " deg=" << q_out(i) * 180.0 / M_PI << "\n";
        }
        return std::vector<double>{get_actual_angle(q_out(0)), get_actual_angle(q_out(1)), get_actual_angle(q_out(2)), get_actual_angle(q_out(3))};

    }
    else
    {
        std::cout << "IK failed:\n";
        std::cout << ik_solver.strError(ret) << std::endl;
    }
    return std::vector<double>{-1, -1, -1, -1};
}
