#include "MotionAdaptationToolV2HandlerDialog.h"
#include "ui_MotionAdaptationToolV2HandlerDialog.h"

#include <MMM/Exceptions.h>
#include <MMM/Motion/MotionRecording.h>
#include <MMM/Model/ModelReaderXML.h>
#include <QFileDialog>
#include <QCheckBox>
#include <QMessageBox>
#include <VirtualRobot/Robot.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensor.h>
#include <MMM/Motion/Plugin/ModelPosePlugin/ModelPoseSensorMeasurement.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensorMeasurement.h>
#include <MMM/Motion/Plugin/KinematicPlugin/KinematicSensor.h>
#include <SimoxUtility/math/convert.h>
#include <VirtualRobot/IK/DiffIK/CompositeDiffIK.h>
#include <VirtualRobot/Visualization/VisualizationNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h>
#include <VirtualRobot/Visualization/TriMeshUtils.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Inventor/nodes/SoPointSet.h>
#include <Inventor/SbVec3f.h>
#include <QPushButton>
#include <QFileDialog>
#include <set>
#include <QDoubleSpinBox>
#include <math.h>


MotionAdaptationToolV2HandlerDialog::MotionAdaptationToolV2HandlerDialog(QWidget* parent, MMM::MotionRecordingPtr /*motions*/) :
    QDialog(parent),
    ui(new Ui::MotionAdaptationToolV2HandlerDialog),
    motions(nullptr),
    currentObject(nullptr)
{
    ui->setupUi(this);

    // map changes to gui elements (in *.ui) call methods in this class - Take a look at QSignal https://doc.qt.io/qt-5/signalsandslots.html
    connect(ui->ChooseMotionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(setCurrentMotion(const QString&)));
    connect(ui->storeButton, &QPushButton::clicked, this, &MotionAdaptationToolV2HandlerDialog::storeMotion);
    connect(ui->xOffset, SIGNAL(valueChanged(int)), this, SLOT(setTransformation()));
    connect(ui->yOffset, SIGNAL(valueChanged(int)), this, SLOT(setTransformation()));
    connect(ui->zOffset, SIGNAL(valueChanged(int)), this, SLOT(setTransformation()));
    connect(ui->randomButton, SIGNAL(clicked()), this, SLOT(randomInit()));
    connect(ui->checkBox, SIGNAL(stateChanged(int)), this, SLOT(setTransformation()));
    connect(ui->dButton, SIGNAL(clicked()), this, SLOT(test()));
}

MotionAdaptationToolV2HandlerDialog::~MotionAdaptationToolV2HandlerDialog() {
    delete ui;
}

void MotionAdaptationToolV2HandlerDialog::jumpTo(float timestep) {
    for (auto object : objects) {
        // Set and visualize object motion
        MMM::MotionPtr motion = motions->getMotion(object.first); // retrieve the motion by name
        motion->initializeModel(object.second, timestep);
        // Retrieve objct root pose from recording for given timestep
        MMM::ModelPoseSensorPtr modelPoseSensor = motion->getSensorByType<MMM::ModelPoseSensor>(); // model pose sensor stores all root poses over time
        MMM::ModelPoseSensorMeasurementPtr modelPoseSensorMeasurement = modelPoseSensor->getDerivedMeasurement(timestep); // retrieve the measurement at a certain timestep
        if (modelPoseSensorMeasurement) {
            // normally interpolated, but if timestep is less than start or bigger than end the measurement is a nullptr
            Eigen::Matrix4f pose = modelPoseSensorMeasurement->getRootPose(); // retrieves the root pose from the measurement

            // ################ TODO ###################
            // Task 1: Use objectTransformation to add offset to environment object
            // Add offset to pose
            // ################ TODO ###################

            Eigen::Matrix4f transformation = objectTransformation[object.first]->getTransformation(timestep);

            pose = transformation*pose;

            objects_visualized[object.first]->setGlobalPose(pose);
        }
    }
    MMM::MotionPtr motion = motions->getReferenceModelMotion(); // gets the motion of the MMM reference model
    if (motion) {
        motion->initializeModel(mmm, timestep);

        auto robotNodeSet = mmm_visualized->getRobotNodeSet("WholeBodyNew"); // retrieves the robot node set containing joint nodes of both arms and both legs as set in the mmm.xml as well as virtual joints to change the rootpose
        VirtualRobot::CompositeDiffIK cik(robotNodeSet);
        mmm_visualized->setGlobalPose(mmm->getGlobalPose()); // sets the global pose of the MMM robot model which is visualized

        for (const std::pair<std::string, std::map<float, std::map<std::string, float>>> &nodeDistances : distancesByNode) {
            std::string nodeName = nodeDistances.first; // joint node name
            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();


            // The following lines are used for interpolation as the timestep of the method does not have to match. Not as important to understand!
            {
                auto it2 = nodeDistances.second.upper_bound(timestep);
                if (it2 == nodeDistances.second.end())
                    return;
                auto it1 = std::prev(it2);
                float timestep1 = it1->first;
                for (auto object : objects) {
                    MMM::MotionPtr objectMotion = motions->getMotion(object.first); // retrieve the motion by name
                    objectMotion->initializeModel(object.second, timestep1);
                }
                Eigen::Vector3f pos1 = getPosition(it1->second, nodeName);
                float timestep2 = it2->first;
                for (auto object : objects) {
                    MMM::MotionPtr objectMotion = motions->getMotion(object.first); // retrieve the motion by name
                    objectMotion->initializeModel(object.second, timestep2);
                }
                Eigen::Vector3f pos2 = getPosition(it2->second, nodeName);
                Eigen::Vector3f position = linearInterpolation(pos1, timestep1, pos2, timestep2, timestep);
                pose.block(0, 3, 3, 1) = position;
            }
            // The lines above are used for interpolation as the timestep of the method does not have to match. Not as important to understand!


            // Add end effector target
            cik.addTarget(mmm_visualized->getRobotNode(nodeName),
                          mmm_visualized->toLocalCoordinateSystem(pose), VirtualRobot::IKSolver::CartesianSelection::Position);
        }


        // The following IK Settings are not important
        {
            // Joint Limit Avoidance in Nullspace - Adapt joint angles in nullspace to try to prevent values near joint angles
            VirtualRobot::CompositeDiffIK::NullspaceJointLimitAvoidancePtr nsjla(new VirtualRobot::CompositeDiffIK::NullspaceJointLimitAvoidance(robotNodeSet));
            nsjla->kP = 0.1;
            cik.addNullspaceGradient(nsjla);

            VirtualRobot::CompositeDiffIK::Parameters cp;
            for (auto joint : robotNodeSet->getAllRobotNodes()) {
                if (joint->isTranslationalJoint()) {
                    nsjla->setWeight(joint->getName(), 0);;
                }
            }
            cp.jointRegularizationTranslation = 1000;
            cp.resetRnsValues = false; // not required to understand
            cp.steps = 0;
            cp.maxJointAngleStep = 0.05;
            VirtualRobot::CompositeDiffIK::SolveState s;
            VirtualRobot::CompositeDiffIK::Result result = cik.solve(cp, s); // solve the ik
            int index = 0;
            while (!result.reached) {
                cik.step(cp, s, index);
                index++;
                if (index > 200) {
                    MMM_WARNING << "IK Solver unsuccessfull" << std::endl;
                    break;
                }
            }
        }
        // The ik settings above are not important


        this->currentTimestep = timestep;
    }
}

void MotionAdaptationToolV2HandlerDialog::open(MMM::MotionRecordingPtr motions) {
    this->motions = motions;
    loadMotions();
}

void MotionAdaptationToolV2HandlerDialog::loadMotions() {
    // reset everything
    emit addVisualisation(0);
    currentTimestep = -1.0f;
    mmm = nullptr;
    mmm_visualized = nullptr;
    mmm_name = "";
    objects.clear();
    objects_visualized.clear();
    objectTransformation.clear();
    ui->ChooseMotionComboBox->clear();

     // So* stuff is for visualization
    SoSeparator* visualization = new SoSeparator();

    // Set the visualization stuff to milimeters
    SoUnits *u = new SoUnits();
    u->units = SoUnits::MILLIMETERS;
    visualization->addChild(u);

    // set a transformation matrix for the visualization
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation(1,3) = 2000; // move visualization 2000mm/2m in y-Direction
    SoMatrixTransform* mt = new SoMatrixTransform();
    SbMatrix m_(reinterpret_cast<SbMat*>(transformation.data()));
    mt->matrix.setValue(m_);
    visualization->addChild(mt);

    // Iterate over all opened motions. A motion can either be the motion of an object and contain its model and root pose over time
    // or the MMM reference and contain the mmm model, root pose and joint angles over time
    for (const std::string &name : motions->getMotionNames()) {
        MMM::MotionPtr motion = motions->getMotion(name); // retrieve the motion from the container by name
        VirtualRobot::RobotPtr robot =  motion->getModel()->cloneScaling(); // create a clone of the model
        if (robot) {
            if (motion->isReferenceModelMotion()) { // check if motion corresponds to mmm reference model or to environment/objects
                mmm_visualized = robot; // robot model for visualization
                mmm = motion->getModel()->cloneScaling(); // create a second robot model for ik solving only
                mmm_name = motion->getName(); // store mmm motion name
            }
            else {
                objects_visualized[motion->getName()] = robot; // store all models of the objects by the motion name
                objects[motion->getName()] = motion->getModel()->cloneScaling(); // create a second robot model for ik solving only
                objectTransformation[motion->getName()] = std::make_shared<Transformation>(); // adds transformation;
                ui->ChooseMotionComboBox->addItem(QString::fromStdString(motion->getName())); // add object names to combo box gui to choose from
                if (!currentObject) currentObject = motion;
            }
            robot->reloadVisualizationFromXML(); // Not important for you
            robot->setupVisualization(true, true); // Not important for you
            // Creates a visualization of the full robot
            std::shared_ptr<VirtualRobot::CoinVisualization> robot_vis = robot->getVisualization<VirtualRobot::CoinVisualization>(VirtualRobot::SceneObject::Full);

            if (!motion->isReferenceModelMotion()) { // check if motion corresponds to mmm reference model or to environment/objects
                // If Object e.g. create point cloud from robot_vis
                pointCloud[motion->getName()] = createPointCloud(robot_vis, objects[motion->getName()], 1000);

                bool visualizePointCloud = false; // change if you want to wish to visualize the point cloud. This set only once if objects are moving this does not work, this would have to be called everytime if an object changes
                if (visualizePointCloud) {
                    visualization->addChild(createPointCloudVisualization(robot_vis)); // adds the point cloud to the visualization
                }
                else {
                    visualization->addChild(robot_vis->getCoinVisualization()); // adds the object to the visualization
                }
            }
            else {
                visualization->addChild(robot_vis->getCoinVisualization()); // adds the mmm reference model to the visualization
            }


        }
    };

    // TASK 2: Calculate Distance between joint nodes and the environment objects including the floor

    std::vector<std::string> robotNodes;
    // ################ TODO ###################
    // TASK 2
    // Add relevant robot nodes to vector robotNodes
    // ################ TODO ###################

//    robotNodes.push_back("RightFootHeight_joint");
//    robotNodes.push_back("LeftFootHeight_joint");
//    robotNodes.push_back("RMrot_joint");
//    robotNodes.push_back("LMrot_joint");
    robotNodes.push_back("RFx_joint");
    robotNodes.push_back("LFx_joint");
//    robotNodes.push_back("RAx_joint");
//    robotNodes.push_back("LAx_joint");
//    robotNodes.push_back("RAy_joint");
//    robotNodes.push_back("LAy_joint");
//    robotNodes.push_back("RAz_joint");
//    robotNodes.push_back("LAz_joint");
//    robotNodes.push_back("RAsegment_joint");
//    robotNodes.push_back("LAsegment_joint");

    MMM::MotionPtr mmm_motion = motions->getMotion(mmm_name);
    if (mmm_motion) {
        for (const std::string &nodeName : robotNodes) {
            std::map<float, std::map<std::string, float>> distances;
            for (float timestep : mmm_motion->getTimesteps()) {
                mmm_motion->initializeModel(mmm, timestep); // sets the mmm model to respective pose in motion
                Eigen::Vector3f globalJointNodePosition = mmm->getRobotNode(nodeName)->getGlobalPosition(); // retrieve pose

                for (auto object : objects) {
                    MMM::MotionPtr motion = motions->getMotion(object.first); // retrieve the motion by name
                    motion->initializeModel(object.second, timestep); // sets the object model to respective pose in motion
                    Eigen::Matrix4f globalObjectPose = object.second->getGlobalPose(); // retrieve pose
                    Eigen::Matrix3f globalObjectOrientation = globalObjectPose.block(0, 0, 3, 3);
                    Eigen::Vector3f globalObjectTranslation = globalObjectPose.block(0, 3, 3, 1);

                    // ################ TODO ###################
                    // TASK 2
                    // Calculate distance of joint node to object
                    float distance = -1.0f;
                    float minimum = 10000.0f;
                    for (const Eigen::Vector3f &localPositionInObjectFrame : pointCloud[object.first]) {

                        Eigen::Vector3f globalObjectPosition = globalObjectOrientation*localPositionInObjectFrame + globalObjectTranslation;
                        distance = (globalJointNodePosition-globalObjectPosition).norm();
                        if (distance < minimum){
                            minimum = distance;
                        }

                    }
                    // ################ TODO ###################

                    distances[timestep][object.first] = minimum;
                }

                // ################ TODO ###################
                // TASK 2
                // Calculate distance of joint node to floor

                double floorDistance = -1.0f;
                floorDistance = globalJointNodePosition(2);

                // ################ TODO ###################

                distances[timestep]["Floor"] = floorDistance;
            }
            this->distancesByNode[nodeName] = distances;
        }
    }

    emit addVisualisation(visualization); // emit a qsignal to add your visualization after motion is changed
}

/*!
 * \brief MotionAdaptationToolV2HandlerDialog::getPosition return an average position between all closest environment objects based in the initial distance to object.
 * Objects closer to the joint node are more important for calculating the position
 * \param m HashMap mapping object names to distance to node
 * \param nodeName The corresponding node name
 * \param maxDistance The maximum distance when objects in the environment have to be considered
 * \return
 */
Eigen::Vector3f MotionAdaptationToolV2HandlerDialog::getPosition(const std::map<std::string, float> &m, const std::string &nodeName, float maxDistance) {
    // Task 3: Calculate new position

    Eigen::Vector3f combinedPosition = Eigen::Vector3f::Zero();
    Eigen::Vector3f z;
    z << 0, 0, 20.0f;
    float sum = 0.0f;

    for (const std::pair<std::string, float> &d : m) {
        if (d.second < maxDistance) {
            if (d.first == "Floor") {
                sum += pow((maxDistance - d.second), 4) * 0.1;
            }
            else {
                sum += pow((maxDistance - d.second), 4);
            }
        }
    }
    for (const std::pair<std::string, float> &d : m) {
        if (d.second < maxDistance) {
            Eigen::Vector3f position = Eigen::Vector3f::Zero();
            double factor = 0.0f;

            // ################ TODO ###################
            // Task 3
            // Compute a factor to combine multiple global and local trajectories based on the distance between the joint node and the object
            // Smaller distance means that the pose is more important for calculating the new pose
            // ################ TODO ###################

            if (d.first == "Floor") {
                // ################ TODO ###################
                // Task 3
                // Set position to global trajectory of joint node
                // ################ TODO ###################

                position = mmm->getRobotNode(nodeName)->getGlobalPosition();

                factor = (pow(maxDistance-d.second, 4)*0.1)/sum;
                std::cout << nodeName << " " << factor << " " << d.first << std::endl;
            }
            else {
                // ################ TODO ###################
                // Task 3
                // Transform global position of joint node to local position in object
                // Transform local position back to global position
                // objects contains the original model pose
                // objects_visualized contains the adapted model pose
                // ################ TODO ###################

                position = mmm->getRobotNode(nodeName)->getGlobalPosition();

                Eigen::Matrix4f globalObjectAdapted = objects_visualized[d.first]->getGlobalPose();
                Eigen::Matrix4f globalObjectOriginal = objects[d.first]->getGlobalPose();


                Eigen::Matrix3f orientationOriginal = globalObjectOriginal.block(0, 0, 3, 3);
                Eigen::Vector3f translationOriginal = globalObjectOriginal.block(0, 3, 3, 1);
                Eigen::Vector3f newposition = (orientationOriginal.transpose()) * (position - translationOriginal);

                Eigen::Matrix3f orientationAdapted = globalObjectAdapted.block(0, 0, 3, 3);
                Eigen::Vector3f translationAdapted = globalObjectAdapted.block(0, 3, 3, 1);
                position = orientationAdapted * newposition + translationAdapted + z;

                factor = pow(maxDistance-d.second, 4)/sum;
                std::cout << nodeName << " " << factor << " " << d.first << std::endl;
            }
            combinedPosition += factor * position;

        }
    }


    // If no objects are nearby, take the global position
    if (combinedPosition.isZero()) {
        combinedPosition += mmm->getRobotNode(nodeName)->getGlobalPosition();
    }

    return combinedPosition;
}

void MotionAdaptationToolV2HandlerDialog::setCurrentMotion(const QString &name) {
    currentObject = motions->getMotion(name.toStdString()); // sets the current object from gui
    if (currentObject && objectTransformation.find(currentObject->getName()) != objectTransformation.end()) {
        TransformationPtr transformation = objectTransformation.at(currentObject->getName());
        ui->xOffset->setValue(transformation->xOffset);
        ui->yOffset->setValue(transformation->yOffset);
        ui->zOffset->setValue(transformation->zOffset);
    }
}

void MotionAdaptationToolV2HandlerDialog::storeMotion() {
    std::filesystem::path defaultFilePath = std::string(motions->getOriginFilePath().parent_path()) + std::string(motions->getOriginFilePath().stem()) + "_adapted.xml";
    std::filesystem::path motionFilePath = QFileDialog::getSaveFileName(this, tr("Save motions"), QString::fromStdString(defaultFilePath), tr("XML files (*.xml)")).toStdString();
    if (!motionFilePath.empty()) {
        MMM::MotionRecordingPtr motions = MMM::MotionRecording::EmptyRecording();

        // Task 5: Store new motion in xml-based MMM dataformat

        for (std::pair<std::string, VirtualRobot::RobotPtr> object : objects) {
            // Set and visualize object motion
            MMM::MotionPtr motion = motions->getMotion(object.first)->clone(); // retrieve the motion by name
            // Retrieve objct root pose from recording for given timestep
            MMM::ModelPoseSensorPtr modelPoseSensor = motion->getSensorByType<MMM::ModelPoseSensor>(); // model pose sensor stores all root poses over time
            for (float timestep : modelPoseSensor->getTimesteps()) {
                MMM::ModelPoseSensorMeasurementPtr modelPoseSensorMeasurement = modelPoseSensor->getDerivedMeasurement(timestep); // retrieve the measurement at a certain timestep
                if (modelPoseSensorMeasurement) {
                    Eigen::Matrix4f pose = modelPoseSensorMeasurement->getRootPose(); // retrieves the root pose from the measurement

                    // ################ TODO ###################
                    // Task 5
                    // Add offset to pose (see jumpTo)
                    // ################ TODO ###################

                    modelPoseSensorMeasurement->setRootPose(pose); // update root pose
                }
            }
        }

        // ################ TODO ###################
        // Task 5
        // Store mmm reference model motion - Currently stores the original motion. You have to adapt it to store the adapted motion (see jumpTo)
        // ################ TODO ###################

        {
            // Example for mmm. Do this for all objects, too
            std::string motionName = mmm_name;

            MMM::MotionPtr oldMotion = motions->getMotion(motionName);

            MMM::MotionPtr motion(new MMM::Motion(motionName, oldMotion->getModel(false), oldMotion->getModelProcessor())); // just copy this line

            MMM::ModelPoseSensorPtr modelPoseSensor(new MMM::ModelPoseSensor()); // create a model pose sensor
            // use kinematic sensor only for mmm reference model motion
            std::vector<std::string> jointNames = {}; // add the joint names you actuated here, e.g. part of the robotNodeset you have chosen
            MMM::KinematicSensorPtr kinematicSensor(new MMM::KinematicSensor(jointNames)); // create a kinematic sensor

            for (float timestep : oldMotion->getTimesteps()) {
                Eigen::Matrix4f rootPose = oldMotion->getRootPose(timestep); // retrieve the old root pose. Maybe you want to do it differently
                MMM::ModelPoseSensorMeasurementPtr measurement(new MMM::ModelPoseSensorMeasurement(timestep, rootPose));
                modelPoseSensor->addSensorMeasurement(measurement);

                // TODO run you IK solution or something to do this
                Eigen::VectorXf jointValues; // TODO add the joint values you created with your new motion description
                MMM::KinematicSensorMeasurementPtr kMeasurement(new MMM::KinematicSensorMeasurement(timestep, jointValues));
                kinematicSensor->addSensorMeasurement(kMeasurement);
            }

            motions->addMotion(motion);
        }

        motions->saveXML(motionFilePath, true); // store at selected file path - You can try open it again in MMMViewer to see if it works
    }
}

void MotionAdaptationToolV2HandlerDialog::setTransformation() {
    if (currentObject && objectTransformation.find(currentObject->getName()) != objectTransformation.end()) {
        TransformationPtr transformation = objectTransformation.at(currentObject->getName()); // objectTransformation["NAME"]
        if (ui->checkBox->isChecked())
            transformation->xOffset = ui->xOffset->value();
        else
            transformation->xOffset = 0.0f;
        transformation->yOffset = ui->yOffset->value();
        transformation->zOffset = ui->zOffset->value();
    }
}

#include <random>

void MotionAdaptationToolV2HandlerDialog::randomInit() {
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    int max = 250;
    std::uniform_int_distribution<> distr(0, max);
    for (const auto &t : objectTransformation) {
        t.second->xOffset = distr(gen);
        t.second->yOffset = distr(gen);
        t.second->zOffset = distr(gen);
    }
    setCurrentMotion(ui->ChooseMotionComboBox->currentText());
}

void MotionAdaptationToolV2HandlerDialog::test() {
    if (currentObject && objectTransformation.find(currentObject->getName()) != objectTransformation.end()) {
        TransformationPtr transformation = objectTransformation.at(currentObject->getName()); // objectTransformation["NAME"]
        transformation->dynamicOffset = !transformation->dynamicOffset;
    }
}

SoSeparator* MotionAdaptationToolV2HandlerDialog::createPointCloudVisualization(std::shared_ptr<VirtualRobot::CoinVisualization> visualization, int samples, float pointSize) {
    SoSeparator* allSep = new SoSeparator();
    // Insert color information into scene graph
    for (VirtualRobot::VisualizationNodePtr node : visualization->getVisualizationNodes()) {
        if (!node) continue;
        VirtualRobot::TriMeshModelPtr trimeshModel = node->getTriMeshModel();

        // following method can be used to sample a point cloud in local coordinates
        // the other part is just for visualization
        std::vector<Eigen::Vector3f> vertices = VirtualRobot::TriMeshUtils::uniform_sampling(trimeshModel, samples);

        // Add point coordinates
        SoSeparator* sep = new SoSeparator();
        SoCoordinate3* coordinates = new SoCoordinate3();
        std::vector<SbVec3f> pointData;
        pointData.reserve(pointSize);
        for (int i = 0; i < pointSize; i++)
        {
            SbVec3f pointContainer;
            pointContainer[0] = vertices[i](0);
            pointContainer[1] = vertices[i](1);
            pointContainer[2] = vertices[i](2);

            pointData.push_back(pointContainer);
        }
        coordinates->point.setValues(0, pointData.size(), pointData.data());
        sep->addChild(coordinates);

        sep->addChild(VirtualRobot::CoinVisualizationFactory::getMatrixTransform(node->getGlobalPose()));

        // Set point size
        SoDrawStyle* sopointSize = new SoDrawStyle();
        sopointSize->pointSize = pointSize;
        sep->addChild(sopointSize);

        // Draw a point set out of all that data
        SoPointSet* pointSet = new SoPointSet();
        sep->addChild(pointSet);
        allSep->addChild(sep);
    }

    return allSep;
}

// Creates a vector of points corresponding to the surface of the object model
std::vector<Eigen::Vector3f> MotionAdaptationToolV2HandlerDialog::createPointCloud(std::shared_ptr<VirtualRobot::CoinVisualization> visualization, VirtualRobot::RobotPtr robot, int n) {
    std::vector<Eigen::Vector3f> pointCloud;
    int samples = n / visualization->getVisualizationNodes().size();
    // a model can have multiple visualization nodes e.g. the mmm reference model, however in your case all objects should just contain 1
    for (VirtualRobot::VisualizationNodePtr node : visualization->getVisualizationNodes()) {
        if (!node) continue;
        VirtualRobot::TriMeshModelPtr trimeshModel = node->getTriMeshModel(); // retrieves the TriMeshModel
        std::vector<Eigen::Vector3f> vertices = VirtualRobot::TriMeshUtils::uniform_sampling(trimeshModel, samples); // Method for uniformly sampling points on the surface
        Eigen::Matrix4f globalPose = node->getGlobalPose(); // Each visualization node can have its own global pose sometimes different from the robots root pose
        Eigen::Matrix3f rotation = simox::math::mat4f_to_mat3f(globalPose);
        Eigen::Vector3f translation = simox::math::mat4f_to_pos(globalPose);
        // transfer from visualization node to local robot coordinate system
        for (const Eigen::Vector3f vertex : vertices) {
            // transfer to global pose
            Eigen::Vector3f vertex_global = rotation * vertex + translation;
            // transfer to local robot coordinate system
            pointCloud.push_back(robot->toGlobalCoordinateSystemVec(vertex_global));
        }
    }
    return pointCloud;
}

Eigen::Matrix4f Transformation::getTransformation(float timestep) {
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    float dynamicOffsetValue = sin(timestep);


    if (dynamicOffset) {
        transformation(0, 3) = xOffset * dynamicOffsetValue;
        transformation(1, 3) = yOffset * dynamicOffsetValue;
        transformation(2, 3) = zOffset * abs(dynamicOffsetValue);
    }
    else {
        transformation(0, 3) = xOffset;
        transformation(1, 3) = yOffset;
        transformation(2, 3) = zOffset;
    }

    // ################ TODO ###################
    // Task 4
    // Add dynamic offset based on timestep here - Objects should be moving
    // ################ TODO ###################

    return transformation;
}
