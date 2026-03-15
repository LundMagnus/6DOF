#include <iostream>
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#define E_GRADIENT_JOINTS_TOO_SMALL -100
#define E_INCREMENT_JOINTS_TOO_SMALL -101

void IK_solver() 
{
    using namespace KDL;

    // Create robot chain
    Chain chain;

    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));

    // Forward kinematics solver
    ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse kinematics solver (position-priority: orientation almost ignored)
    Eigen::Matrix<double, 6, 1> lma_weights;
    lma_weights << 1.0, 1.0, 1.0, 1e-6, 1e-6, 1e-6;
    ChainIkSolverPos_LMA ik_solver(chain, lma_weights);

    // Desired end-effector pose
    Frame target(Frame::Identity());
    target.p = Vector(10, 13, 0);

    // Initial joint guess
    JntArray q_init(chain.getNrOfJoints());
    q_init(0) = 0;
    q_init(1) = 0;
    q_init(2) = 0;
    q_init(3) = 0;
    q_init(4) = 0;
    q_init(5) = 0;

    // Output solution
    JntArray q_out(chain.getNrOfJoints());

    int ret = ik_solver.CartToJnt(q_init, target, q_out);

    if (ret == E_GRADIENT_JOINTS_TOO_SMALL) {
        std::cout << "Gradient joints too small" << std::endl;
    } else if (ret == E_INCREMENT_JOINTS_TOO_SMALL) {
        std::cout << "Increment joints too small" << std::endl;
    }


    if(ret >= 0)
    {
        std::cout << "Solution found:\n";
        std::cout << "q1 = " << q_out(0) << "\n";
        std::cout << "q2 = " << q_out(1) << "\n";
    }
    else
    {
        std::cout << "IK failed\n";
    }
    return;
}
