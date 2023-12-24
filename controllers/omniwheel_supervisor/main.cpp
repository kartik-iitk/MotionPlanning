#include "Motion.hpp"
#include "PathPlanner.hpp"
#include "Robot.hpp"
#include "Visualize.hpp"
#include "iostream"
#include "webots/GPS.hpp"
#include "webots/InertialUnit.hpp"
#include "webots/Motor.hpp"
#include "webots/PositionSensor.hpp"
#include "webots/Robot.hpp"
#include "webots/Supervisor.hpp"

using namespace webots;
const int maxNumberCord = 10000;
const int refresh_factor = 2.0;

Motor *motor[4];
PositionSensor *enc[4];
InertialUnit *imu_device;
std::vector<double> prevPulse{0, 0, 0, 0};
std::vector<double> encData{0, 0, 0, 0};

using namespace robot;
Motion *mot = new Motion();
Point2D nowPos(0, 0, 0);
int count = 0;

Supervisor *robotSup = new Supervisor();
Node *my_root = robotSup->getRoot();
Visualize *window = new Visualize(2400);

void get_Trajectory(std::vector<Point2D> &path, Point2D &outputPID,
                    Point2D &nowPos, wheelAngularVel &outInvers, double yaw) {
    window->visualizeGame(path, nowPos, count, yaw);
    double errorX = path[count].x - nowPos.x;
    double errorY = path[count].y - nowPos.y;

    double dist = sqrt(errorX * errorX + errorY * errorY);
    double errTheta = path[count].theta - (nowPos.theta * 180 / M_PI);

    // Limit errTheta to -180 to +180 degrees
    if (errTheta > 180) errTheta -= 360;
    if (errTheta < -180) errTheta += 360;

    // IF using IMU as orientation
    mot->positionAngularControl(errorX, errorY, errTheta, yaw, outputPID);

    // If using Odometry orientation
    // mot->PositionAngularControl(errorX, errorY, errTheta, nowPos.theta,
    //                             outputPID);

    RobotKinematic::getInstance()->inverseKinematics(
        outInvers, outputPID.x, outputPID.y, outputPID.theta);
    if (dist < 0.07 && fabs(errTheta) < 3) {
        mot->position_pid->reset();
        mot->yaw_pid->reset();
        count++;
        IC(count);
    }
    if (count > path.size() - 1) {
        count = 0;
    }
}

void setVel(wheelAngularVel &outMotor) {
    motor[0]->setVelocity(outMotor.w1);
    motor[1]->setVelocity(outMotor.w2);
    motor[2]->setVelocity(outMotor.w3);
    motor[3]->setVelocity(outMotor.w4);
}

static void create_line() {
    Node *existing_line = robotSup->getFromDef("TRACK");
    // Node *myRobot = super->getFromDef("OMNI_WHEELS");
    // Field *translation_field = myRobot->getField("translation");

    if (existing_line) existing_line->remove();

    int i;
    std::string track_string =
        "";  // Initialize a big string which will contain the TRAIL node.
    // Create the TRAIL Shape.
    track_string += "DEF TRACK Shape {\n";
    track_string += "  appearance Appearance {\n";
    track_string += "    material Material {\n";
    track_string += "      diffuseColor 0 0 0\n";
    track_string += "      emissiveColor 0 0 0\n";
    track_string += "    }\n";
    track_string += "  }\n";
    track_string += "  geometry DEF TRACK_LINE_SET IndexedLineSet {\n";
    track_string += "    coord Coordinate {\n";
    track_string += "      point [\n";
    for (i = 0; i < maxNumberCord; ++i) track_string += "      0 0 0\n";
    track_string += "      ]\n";
    track_string += "    }\n";
    track_string += "    coordIndex [\n";
    for (i = 0; i < maxNumberCord; ++i) track_string += "      0 0 -1\n";
    track_string += "    ]\n";
    track_string += "  }\n";
    track_string += "}\n";

    // Import TRAIL and append it as the world root nodes.

    Field *field_children = my_root->getField("children");

    field_children->importMFNodeFromString(-1, track_string);
}

int main(int argc, char **argv) {
    // OMPL Code
    double runTime = 1.0;
    optimalPlanner plannerType = PLANNER_RRTSTAR;
    planningObjective objectiveType = OBJECTIVE_PATHLENGTH;
    std::string outputFile = "output.txt";

    plan(runTime, plannerType, objectiveType, outputFile);
    // OMPL Code Ends

    wheelAngularVel outInvers;
    double dt = 0.0;
    Point2D outputPID(0, 0, 0.654);
    int stepTime = robotSup->getBasicTimeStep();
    stepTime *= refresh_factor;

    imu_device = robotSup->getInertialUnit("IMU");
    imu_device->enable(stepTime);

    GPS *gps_dev = robotSup->getGPS("gps");
    gps_dev->enable(stepTime);

    // Motor Initial
    char motorNames[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};

    Node *target_line = robotSup->getFromDef("OMNI_WHEELS_4");
    // Node *target_robot = robotSup->getFromDef("field");

    // SET the first Position our Robot
    RobotKinematic::getInstance()->setInitialPosition(
        0, 0, 0);  // Set Position x = 0, y = 0, theta = 0

    // Create Line in webots environment to see actual path traced
    create_line();

    Node *trail_line_set = robotSup->getFromDef("TRACK_LINE_SET");
    Field *coord_field = trail_line_set->getField("coord");

    Node *coordinate_node = coord_field->getSFNode();
    Field *pointField = coordinate_node->getField("point");
    Field *coord_index_field = trail_line_set->getField("coordIndex");

    int index = 0;
    bool first_step = true;
    // Track line end-----------

    // set Motor to webots
    for (int i = 0; i < 4; i++) {
        motor[i] = robotSup->getMotor(motorNames[i]);
        motor[i]->setPosition(INFINITY);
        motor[i]->setVelocity(0);
    }

    // Encoder Initial
    char encNames[4][8] = {"pw1", "pw2", "pw3", "pw4"};

    for (int i = 0; i < 4; i++) {
        enc[i] = robotSup->getPositionSensor(encNames[i]);
        enc[i]->enable(stepTime);
    }

    std::vector<Point2D> circVec;
    for (int i = 0; i < 360; i += 10) {
        float x_ = 1 * cos((float)i * M_PI / 180);
        float y_ = 1 * sin((float)i * M_PI / 180);
        circVec.push_back(Point2D(x_, y_, 0));
    }

    std::vector<PointPair> points = readPointsFromFile();

    std::vector<Point2D> targetPos;
    for (auto &ptr : points) {
        targetPos.push_back(Point2D(ptr.first, ptr.second, 0));
    }

    // std::vector<Point2D> targetPos = {Point2D(-10, 0, 0), Point2D(2, -1, 0),
    //                                   Point2D(0, -2, 0), Point2D(0, 2, 0)};
    Point2D gps_pos{0, 0, 0};

    while (robotSup->step(stepTime) != -1) {
        //     std::chrono::high_resolution_clock::time_point start_time =
        //         std::chrono::high_resolution_clock::now();
        //     // Get Current Translation
        //     double rbtime = robotSup->getTime();

        const double *robotTranslations = target_line->getPosition();
        const double *orientation = imu_device->getRollPitchYaw();
        const double yaw = orientation[2];

        // add New Position
        pointField->setMFVec3f(index, robotTranslations);

        // update line track
        if (index > 0) {
            coord_index_field->setMFInt32(3 * (index - 1), index - 1);
            coord_index_field->setMFInt32(3 * (index - 1) + 1, index);
        } else if (index == 0 && first_step == false) {
            coord_index_field->setMFInt32(3 * (maxNumberCord - 1), 0);
            coord_index_field->setMFInt32(3 * (maxNumberCord - 1) + 1,
                                          (maxNumberCord - 1));
        }

        // GPS for testing
        const double *gps_raw = gps_dev->getValues();
        gps_pos.x = gps_raw[0];
        gps_pos.y = gps_raw[1];
        gps_pos.theta = yaw;

        nowPos = robot::RobotKinematic::getInstance()->getPos();
        IC(nowPos.x, nowPos.y, nowPos.theta);

        get_Trajectory(targetPos, outputPID, nowPos, outInvers, yaw);

        setVel(outInvers);

        // Assign Enc val
        for (int i = 0; i < 4; i++) {
            RobotKinematic::getInstance()->encData[i] = enc[i]->getValue();
        }
        RobotKinematic::getInstance()->calculateOdometry(yaw);

        // unset next indices
        coord_index_field->setMFInt32(3 * index, index);
        coord_index_field->setMFInt32(3 * index + 1, index);

        if (robotSup->step(robotSup->getBasicTimeStep()) == -1) break;
        first_step = false;
        index++;
        index = index % maxNumberCord;
    }

    robotSup->simulationQuit(EXIT_SUCCESS);
    delete robotSup;

    return 0;
}