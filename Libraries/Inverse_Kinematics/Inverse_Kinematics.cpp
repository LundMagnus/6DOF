#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

void IK_solver() 
{
    using namespace KDL;

    // Create robot chain
    Chain chain;

    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));
    chain.addSegment(Segment(Joint(Joint::RotZ), Frame(Vector(10,0,0))));

    // Forward kinematics solver
    ChainFkSolverPos_recursive fk_solver(chain);

    // Inverse kinematics solver
    ChainIkSolverPos_LMA ik_solver(chain);

    // Desired end-effector pose
    Frame target(Frame::Identity());
    target.p = Vector(10, 13, 0);

    // Initial joint guess
    JntArray q_init(chain.getNrOfJoints());
    q_init(0) = 0;
    q_init(1) = 0;

    // Output solution
    JntArray q_out(chain.getNrOfJoints());

    auto ret = ik_solver.CartToJnt(q_init, target, q_out);
    std::cout << ret << std::endl;

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
