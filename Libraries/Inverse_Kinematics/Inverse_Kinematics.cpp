#include <iostream>
#include <Eigen/Core>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>

double get_actual_angle(double angle) {
    double degrees = angle * 180.0 / M_PI;
    if(degrees < 0) return 360.0 + degrees;  // was: 360 - degrees
    return degrees;
}



std::vector<double> IK_solver(float x, float y, float z) 
{
    using namespace KDL;


    struct Link {
        Joint::JointType joint;
        double a;
        double d;
        double alpha;
    };

    //std::vector<Link> robot = {
    //    // joint,       a         d          alpha
    //    {Joint::RotZ,  0.023491,  0.043682,  M_PI/2},  // J1: base yaw, X offset + Z rise to J2
    //    {Joint::RotZ,  0.11312,   0.0,       M_PI},       // J2: upper arm, nearly pure Z
    //    {Joint::RotZ,  0.097049,  0.015182,  -M_PI/2},       // J3: forearm
    //    {Joint::RotZ,  0.017141,  0.049753,  -M_PI/2},    // J4: wrist roll, twist to J5
    //    {Joint::RotZ,  0.041431,  0.045000,  0.0},       // J5: wrist pitch (end)
    //};

    std::vector<Link> robot = {
        // joint        a         d          alpha
        {Joint::RotZ,  0.022816, 0.043826,  M_PI/2},   // J1
        {Joint::RotZ,  0.113124, 0.0,       M_PI},      // J2
        {Joint::RotZ,  0.101050, 0.0,      -M_PI/2},    // J3
        {Joint::RotZ,  0.049753, 0.0,       M_PI/2},    // J4
        {Joint::RotZ,  0.0,      0.0,       0.0},       // J5
    };

    Chain chain;
    for(const auto& link : robot) {
        chain.addSegment(
            Segment(
                Joint(link.joint),
                Frame(
                    Rotation::RotX(link.alpha),   // twist between axes
                    Vector(link.a, 0.0, link.d)   // a along X, d along Z
                )
            )
        );
    }

    std::cout << "Chain has " << chain.getNrOfJoints() << " joints and " 
          << chain.getNrOfSegments() << " segments" << std::endl;


    // Initial joint guess
    //JntArray q_init(chain.getNrOfJoints());
    //q_init(0) = (135.0 - 135.0) * M_PI / 180.0;  // 0
    //q_init(1) = (142.0 - 90.0)  * M_PI / 180.0;  // +52°
    //q_init(2) = (60.0  - 90.0)  * M_PI / 180.0;  // -30°
    //q_init(3) = (90.0  - 90.0)  * M_PI / 180.0;  // 0
    //q_init(4) = (90.0  - 90.0)  * M_PI / 180.0;  // 0
    //for(int i = 0; i < chain.getNrOfJoints(); i++) {
    //    q_init(i) = 0;
    //}
    

    ChainFkSolverPos_recursive fk_solver(chain);
    JntArray q_home(chain.getNrOfJoints());
    q_home(0) = (135.0 - 135.0) * M_PI/180.0;                          // J1: 135° - 135° = 0
    q_home(1) = (141.0 - 135.0) * M_PI/180.0; // J2: +52°
    q_home(2) = (60.0  - 90.0)  * M_PI/180.0; // J3: -30°
    q_home(3) = (90.0 - 90.0)   * M_PI/180.0;                         // J4: 90° - 90° = 0
    q_home(4) = (90.0 - 90.0)   * M_PI/180.0;                         // J5: 90° - 90° = 0

    //q_home(0) = 0.0;
    //q_home(1) = 0.0;
    //q_home(2) = 0.0;
    //q_home(3) = 0.0;
    //q_home(4) = 0.0;

    KDL::Frame fk_home;
    fk_solver.JntToCart(q_home, fk_home);
    std::cout << "FK home pose: "
              << fk_home.p.x() << " "
              << fk_home.p.y() << " "
              << fk_home.p.z() << std::endl;

    // Inverse kinematics solver (position-priority: orientation almost ignored)
    Eigen::Matrix<double, 6, 1> lma_weights;
    lma_weights << 1.0, 1.0, 1.0, 1e-6, 1e-6, 1e-6;
    ChainIkSolverPos_LMA ik_solver(chain, lma_weights);

    // Desired end-effector pose
    KDL::Frame target(Frame::Identity());
    target.p = KDL::Vector(x, y, z);

    JntArray q_init = q_home;


    // Output solution
    JntArray q_out(chain.getNrOfJoints());
    int ret = ik_solver.CartToJnt(q_init, target, q_out);

    if(ret >= 0) {
        return std::vector<double>{
            get_actual_angle(q_out(0)),
            get_actual_angle(q_out(1)),
            get_actual_angle(q_out(2)),
            get_actual_angle(q_out(3)),
            get_actual_angle(q_out(4))
        };
    }

    std::cout << "IK failed: " << ik_solver.strError(ret) << std::endl;
    return std::vector<double>{-1};
}

