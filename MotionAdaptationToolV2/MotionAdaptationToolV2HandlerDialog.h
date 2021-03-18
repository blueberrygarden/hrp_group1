/*
This file is part of MMM.

MMM is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

MMM is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with MMM.  If not, see <http://www.gnu.org/licenses/>.
*
* @package    MMM
* @author     Andre Meixner
* @copyright  2021 High Performance Humanoid Technologies (H2T), Karlsruhe, Germany
*
*/

#pragma once

#include <QDialog>
#include <QSettings>
#include <Inventor/nodes/SoSeparator.h>

#ifndef Q_MOC_RUN
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <MMM/Motion/Motion.h>
#endif

namespace Ui {
class MotionAdaptationToolV2HandlerDialog;
}

struct Transformation {
    int xOffset = 0; // in mm
    int yOffset = 0; // in mm
    int zOffset = 0; // in mm
    bool dynamicOffset = false;

    Eigen::Matrix4f getTransformation(float timestep);
};

typedef std::shared_ptr<Transformation> TransformationPtr;

class MotionAdaptationToolV2HandlerDialog : public QDialog
{
    Q_OBJECT

public:
    explicit MotionAdaptationToolV2HandlerDialog(QWidget* parent, MMM::MotionRecordingPtr motions);
    ~MotionAdaptationToolV2HandlerDialog();

    void jumpTo(float timestep);

    void open(MMM::MotionRecordingPtr motions);

    //! Linear Interpolation of float values
    static float linearInterpolation(float value1, float v1_timestep, float value2, float v2_timestep, float timestep) {
        return value1 + (value2 - value1) / (v2_timestep - v1_timestep) * (timestep - v1_timestep);
    }

    //! Linear Interpolation of Vector3f
    static Eigen::Vector3f linearInterpolation(const Eigen::Vector3f &value1, float v1_timestep, const Eigen::Vector3f &value2, float v2_timestep, float timestep)  {
        Eigen::Vector3f interpolatedVector;
        for (int i = 0; i < 3; i++) {
            interpolatedVector(i) = linearInterpolation(value1(i), v1_timestep, value2(i), v2_timestep, timestep);
        }
        return interpolatedVector;
    }
   
private slots:
    void setCurrentMotion(const QString &name);
    void storeMotion();
    void setTransformation();
    void randomInit();
    void test();
signals:
    void addVisualisation(SoSeparator* sep);

private:
    void loadMotions();
    Eigen::Vector3f getPosition(const std::map<std::string, float> &m, const std::string &nodeName, float maxDistance = 1000);

    SoSeparator* createPointCloudVisualization(std::shared_ptr<VirtualRobot::CoinVisualization> visualization, int samples = 30000, float pointSize = 1.0f);
    std::vector<Eigen::Vector3f> createPointCloud(std::shared_ptr<VirtualRobot::CoinVisualization> visualization, VirtualRobot::RobotPtr robot, int n = 30000);

    Ui::MotionAdaptationToolV2HandlerDialog* ui;
    // stores the current in MMMViewer opened motions
    MMM::MotionRecordingPtr motions;
    // stores the motion corresponding to an object selected in the gui
    MMM::MotionPtr currentObject;
    // stores the mmm reference model which is visualized
    VirtualRobot::RobotPtr mmm_visualized;
    // stores an mmm reference model which is just for helping with the ik solving
    VirtualRobot::RobotPtr mmm;
    // stores a map of all objects models using the motion name as key - this is just for calculations
    std::map<std::string, VirtualRobot::RobotPtr> objects;
    // stores a map of all objects models using the motion name as key - this is the robot which is visualized
    std::map<std::string, VirtualRobot::RobotPtr> objects_visualized;
    // stores a map of all objects transformations set in the graphical user interface using the motion name as key
    std::map<std::string, TransformationPtr> objectTransformation;
    // stores a map of all local positions of sampled points on the surface of the object using the motion name as key
    std::map<std::string, std::vector<Eigen::Vector3f>> pointCloud;
    // stores a map of all relevant joint node names (key 1) for the ik solving, set in the vector robotnodes (see loadMotion in .cpp), to a map, which maps timesteps (key 2) of the opened motion to a map,
    //     which maps environment object names (key 3)  to the distance calculated between the corresponding object and joint node
    // MAP[ROBOT_NODE_NAME, MAP[TIMESTEP, MAP[OBJECT_NAME, DISTANCE(ROBOT_NODE, OBJECT)]]]
    std::map<std::string, std::map<float, std::map<std::string, float>>> distancesByNode;
    // name of the motion for the mmm reference model
    std::string mmm_name;
    float currentTimestep = -1.0f;
    QSettings settings;
};
