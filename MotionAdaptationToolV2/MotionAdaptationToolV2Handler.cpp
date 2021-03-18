#include "MotionAdaptationToolV2Handler.h"
#include <QFileDialog>
#include <QMessageBox>
#include <MMM/Motion/MotionRecording.h>

using namespace MMM;

MotionAdaptationToolV2Handler::MotionAdaptationToolV2Handler(QWidget* widget) :
    MotionHandler(MotionHandlerType::GENERAL, "Motion Adaptation Tool v2"),
    searchPath(""),
    widget(widget),
    dialog(nullptr)
{
}

void MotionAdaptationToolV2Handler::handleMotion(MotionRecordingPtr motions, std::map<std::string, VirtualRobot::RobotPtr> /*currentRobots*/) {
    if (motions && !motions->empty()) {
        if (!dialog) {
            dialog = new MotionAdaptationToolV2HandlerDialog(widget, motions);
            connect(dialog, &MotionAdaptationToolV2HandlerDialog::addVisualisation, this, &MotionAdaptationToolV2Handler::addVisualisation);
        }
        if (!dialog->isVisible()) {
            dialog->show();
        }
        dialog->open(motions);
    }
    else MMM_ERROR << "Cannot open motion adaptation tool, because no motions are present!" << std::endl;
}

std::string MotionAdaptationToolV2Handler::getName() {
    return NAME;
}
