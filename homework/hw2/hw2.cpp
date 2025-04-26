// some standard library includes
#include <math.h>

#include <iostream>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>

// sai main libraries includes
#include "SaiModel.h"

// sai utilities from sai-common
#include "timer/LoopTimer.h"
#include "redis/RedisClient.h"

// redis keys
#include "redis_keys.h"

// for handling ctrl+c and interruptions properly
#include <signal.h>
bool runloop = true;
void sighandler(int) { runloop = false; }

// namespaces for compactness of code
using namespace std;
using namespace Eigen;

// config file names and object names
const string robot_file = "${CS225A_URDF_FOLDER}/panda/panda_arm.urdf";

int main(int argc, char** argv) {
    SaiModel::URDF_FOLDERS["CS225A_URDF_FOLDER"] = string(CS225A_URDF_FOLDER);

    // check for command line arguments
    if (argc != 2) {
        cout << "Incorrect number of command line arguments" << endl;
        cout << "Expected usage: ./{HW_NUMBER} {QUESTION_NUMBER}" << endl;
        return 1;
    }
    // convert char to int, check for correct controller number input
    string arg = argv[1];
    int controller_number;
    try {
        size_t pos;
        controller_number = stoi(arg, &pos);
        if (pos < arg.size()) {
            cerr << "Trailing characters after number: " << arg << '\n';
            return 1;
        }
        else if (controller_number < 1 || controller_number > 4) {
            cout << "Incorrect controller number" << endl;
            return 1;
        }
    } catch (invalid_argument const &ex) {
        cerr << "Invalid number: " << arg << '\n';
        return 1;
    } catch (out_of_range const &ex) {
        cerr << "Number out of range: " << arg << '\n';
        return 1;
    }

    // set up signal handler
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT, &sighandler);

    // load robots
    auto robot = new SaiModel::SaiModel(robot_file);

    // prepare controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.10);
	VectorXd control_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	Jv = robot->Jv(link_name, pos_in_link);
	Lambda = robot->taskInertiaMatrix(Jv);
	J_bar = robot->dynConsistentInverseJacobian(Jv);
	N = robot->nullspaceMatrix(Jv);

    // flag for enabling gravity compensation
    bool gravity_comp_enabled = false;

    // start redis client
    auto redis_client = SaiCommon::RedisClient();
    redis_client.connect();

    // setup send and receive groups
    VectorXd robot_q = redis_client.getEigen(JOINT_ANGLES_KEY);
    VectorXd robot_dq = redis_client.getEigen(JOINT_VELOCITIES_KEY);
    redis_client.addToReceiveGroup(JOINT_ANGLES_KEY, robot_q);
    redis_client.addToReceiveGroup(JOINT_VELOCITIES_KEY, robot_dq);

    redis_client.addToSendGroup(JOINT_TORQUES_COMMANDED_KEY, control_torques);
    redis_client.addToSendGroup(GRAVITY_COMP_ENABLED_KEY, gravity_comp_enabled);

    redis_client.receiveAllFromGroup();
    redis_client.sendAllFromGroup();

    // update robot model from simulation configuration
    robot->setQ(robot_q);
    robot->setDq(robot_dq);
    robot->updateModel();

    // record initial configuration
    VectorXd initial_q = robot->q();
    VectorXd q_desired = initial_q;
    // create a loop timer
    const double control_freq = 1000;
    SaiCommon::LoopTimer timer(control_freq);

    //for writing out
    ofstream file_2;
	file_2.open("../../homework/hw2/q2.txt");
    while (runloop) {
        // wait for next scheduled loop
        timer.waitForNextLoop();
        double time = timer.elapsedTime();

        // read robot state from redis
        redis_client.receiveAllFromGroup();
        robot->setQ(robot_q);
        robot->setDq(robot_dq);
        robot->updateModel();

        // **********************
        // WRITE YOUR CODE AFTER
        // **********************

        // ---------------------------  question 1 ---------------------------------------
        if(controller_number == 1) {
            const double kpi = 400.0;
            const double kvi = 50.0;
            
            const double kp7 = 50.0;
            const double kv7 = -0.29;

            q_desired << initial_q;
            q_desired(6) = 0.1;
            
            //set torque for all other joints
            control_torques = -kpi*(robot->q() - q_desired) -kvi*robot->dq() + robot->coriolisForce() + robot->jointGravityVector();  

            //modify torque for joint 7
            control_torques(6) = -kp7*(robot->q()(6)-0.1) - kv7*robot->dq()(6) + robot->coriolisForce()(6) + robot->jointGravityVector()(6);
        }

        // ---------------------------  question 2 ---------------------------------------
        else if(controller_number == 2) {
            const double kp = 200.0;
            const double kv = 28.0;
            const double kvj = 10.0;
            q_desired << initial_q;

            Vector3d x = robot->positionInWorld(link_name, pos_in_link);
            Vector3d x_desired(0.3, 0.1, 0.5);
            Vector3d x_dot = robot->linearVelocity(link_name, pos_in_link);
            VectorXd q = robot->q();
            VectorXd q_dot = robot->dq();
            control_torques = robot->Jv(link_name, pos_in_link).transpose() * (robot->taskInertiaMatrix(Jv)* (-kp*(x - x_desired) - kv*x_dot)) + robot->jointGravityVector() + robot->nullspaceMatrix(robot->Jv(link_name, pos_in_link)).transpose() * robot->M() * (-1.0 * kvj*q_dot);
            
            file_2 << time << "\t" << x(0) << "\t" << x(1) << "\t" << x(2) << "\t" << q(0) << "\t" << q(1) << "\t" << q(2) << "\t" << q(3) << "\t" << q(4) << "\t" << q(5) << "\t" << q(6) << "\n"; 
        }

        // ---------------------------  question 3 ---------------------------------------
        else if(controller_number == 3) {
            const double kp = 200.0;
            const double kv = 28.0;
            const double kvj = 10.0;

            q_desired << initial_q;

            Vector3d x = robot->positionInWorld(link_name, pos_in_link);
            Vector3d x_desired(0.3, 0.1, 0.5);
            Vector3d x_dot = robot->linearVelocity(link_name, pos_in_link);
            VectorXd q = robot->q();
            VectorXd q_dot = robot->dq();
            control_torques = robot->Jv(link_name, pos_in_link).transpose() * (robot->taskInertiaMatrix(Jv)* (-kp*(x - x_desired) - kv*x_dot) - robot->dynConsistentInverseJacobian(robot->Jv(link_name, pos_in_link)) * robot->jointGravityVector()) + robot->nullspaceMatrix(robot->Jv(link_name, pos_in_link)).transpose() * robot->M() * (kvj*q_dot);
            
            file_2 << time << "\t" << x(0) << "\t" << x(1) << "\t" << x(2) << "\t" << q(0) << "\t" << q(1) << "\t" << q(2) << "\t" << q(3) << "\t" << q(4) << "\t" << q(5) << "\t" << q(6) << "\n"; 
        
        }

        // ---------------------------  question 4 ---------------------------------------
        else if(controller_number == 4) {

            control_torques.setZero();
        }

        // **********************
        // WRITE YOUR CODE BEFORE
        // **********************

        // send to redis
        redis_client.sendAllFromGroup();
    }

    control_torques.setZero();
    gravity_comp_enabled = true;
    redis_client.sendAllFromGroup();

    file_2.close();
    timer.stop();
    cout << "\nControl loop timer stats:\n";
    timer.printInfoPostRun();

    return 0;
}
