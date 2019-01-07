/**
 * @file RobotControlHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

// iDynTree
#include <iDynTree/Core/Utils.h>

#include <RobotControlHelper.hpp>
#include <Utils.hpp>

bool RobotControlHelper::configure(const yarp::os::Searchable& config, const std::string& name,
                                   bool isMandatory)
{
    m_isMandatory = isMandatory;

    // robot name: used to connect to the robot
    std::string robot;
    robot = config.check("robot", yarp::os::Value("icubSim")).asString();

    // get all controlled icub parts from the resource finder
    std::vector<std::string> iCubParts;
    yarp::os::Value* iCubPartsYarp;
    if (!config.check("remote_control_boards", iCubPartsYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find remote_control_boards into "
                    "config file.";
        return false;
    }
    if (!YarpHelper::yarpListToStringVector(iCubPartsYarp, iCubParts))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a vector of "
                    "strings.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    yarp::os::Value* axesListYarp;
    if (!config.check("joints_list", axesListYarp))
    {
        yError() << "[RobotControlHelper::configure] Unable to find joints_list into config file.";
        return false;
    }

    if (!YarpHelper::yarpListToStringVector(axesListYarp, m_axesList))
    {
        yError() << "[RobotControlHelper::configure] Unable to convert yarp list into a "
                    "vector of strings.";
        return false;
    }

    options.put("device", "remotecontrolboardremapper");
    YarpHelper::addVectorOfStringToProperty(options, "axesNames", m_axesList);

    // prepare the remotecontrolboards
    yarp::os::Bottle remoteControlBoards;
    remoteControlBoards.clear();
    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (auto iCubPart : iCubParts)
        remoteControlBoardsList.addString("/" + robot + "/" + iCubPart);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    m_actuatedDOFs = m_axesList.size();

    // open the device
    if (!m_robotDevice.open(options) && m_isMandatory)
    {
        yError() << "[RobotControlHelper::configure] Could not open remotecontrolboardremapper "
                    "object.";
        return false;
    }

    if (!m_robotDevice.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IEncoders interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionInterface) || !m_positionInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionControl interface";
        return false;
    }

    if (!m_robotDevice.view(m_positionDirectInterface) || !m_positionDirectInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_limitsInterface) || !m_limitsInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IPositionDirect interface";
        return false;
    }

    if (!m_robotDevice.view(m_controlModeInterface) || !m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain IControlMode interface";
        return false;
    }

    if (!m_robotDevice.view(m_timedInterface) || !m_timedInterface)
    {
        yError() << "[RobotControlHelper::configure] Cannot obtain iTimed interface";
        return false;
    }

    m_desiredPositionInDegrees.resize(m_actuatedDOFs);
    m_positionFeedbackInDegrees.resize(m_actuatedDOFs);
    m_positionFeedbackInRadians.resize(m_actuatedDOFs);

    // check if the robot is alive
    bool okPosition = false;
    for (int i = 0; i < 10 && !okPosition; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

        if (!okPosition)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << "[RobotControlHelper::close] Unable to read encoders (position).";
        return false;
    }

    return true;
}

bool RobotControlHelper::switchToControlMode(const int& controlMode)
{
    // check if the control interface is ready
    if (!m_controlModeInterface)
    {
        yError() << "[RobotControlHelper::switchToControlMode] ControlMode I/F not ready.";
        return false;
    }

    // set the control interface
    std::vector<int> controlModes(m_actuatedDOFs, controlMode);
    if (!m_controlModeInterface->setControlModes(controlModes.data())
        && m_isMandatory)
    {
        yError()
            << "[RobotControlHelper::switchToControlMode] Error while setting the controlMode.";
        return false;
    }
    return true;
}

bool RobotControlHelper::setDirectPositionReferences(const yarp::sig::Vector& desiredPosition)
{
    if (m_positionDirectInterface == nullptr)
    {
        yError()
            << "[RobotControlHelper::setDirectPositionReferences] PositionDirect I/F not ready.";
        return false;
    }

    if (desiredPosition.size() != m_actuatedDOFs)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Dimension mismatch between "
                    "desired position vector and the number of controlled joints.";
        return false;
    }

    // set desired position
    if (!m_positionDirectInterface->setPositions(m_desiredPositionInDegrees.data())
        && m_isMandatory)
    {
        yError() << "[RobotControlHelper::setDirectPositionReferences] Error while setting the "
                    "desired position.";
        return false;
    }

    return true;
}

void RobotControlHelper::updateTimeStamp()
{
    if (m_timedInterface)
        m_timeStamp = m_timedInterface->getLastInputStamp();
    else
        m_timeStamp.update();
}

bool RobotControlHelper::getFeedback()
{
    if (!m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data()) &&
        m_isMandatory)
    {
        yError() << "[RobotControlHelper::getFeedbacks] Unable to get joint position";
        return false;
    }

    for (unsigned j = 0; j < m_actuatedDOFs; ++j)
        m_positionFeedbackInRadians(j) = iDynTree::deg2rad(m_positionFeedbackInDegrees(j));

    return true;
}

const yarp::os::Stamp& RobotControlHelper::timeStamp() const
{
    return m_timeStamp;
}

yarp::os::Stamp& RobotControlHelper::timeStamp()
{
    return m_timeStamp;
}

const yarp::sig::Vector& RobotControlHelper::jointEncoders() const
{
    return m_positionFeedbackInRadians;
}

void RobotControlHelper::close()
{
    if (!switchToControlMode(VOCAB_CM_POSITION))
        yError() << "[RobotControlHelper::close] Unable to switch in position control.";

    if (!m_robotDevice.close())
        yError() << "[RobotControlHelper::close] Unable to close the device.";
}

int RobotControlHelper::getDoFs()
{
    return m_actuatedDOFs;
}

bool RobotControlHelper::getLimits(yarp::sig::Matrix& limits)
{
    bool okPosition = false;
    for (int i = 0; i < 10 && !okPosition; i++)
    {
        okPosition = m_encodersInterface->getEncoders(m_positionFeedbackInDegrees.data());

        if (!okPosition)
            yarp::os::Time::delay(0.1);
    }
    if (!okPosition)
    {
        yError() << "[RobotControlHelper::getLimits] Unable to read encoders (position).";
        return false;
    }

    // resize matrix
    limits.resize(m_actuatedDOFs, 2);

    for (int i = 0; i < m_actuatedDOFs; i++)
        // get position limits
        if (m_limitsInterface->getLimits(i, &limits(i, 0), &limits(i, 1)))
        {
            if(m_isMandatory)
            {
                yError() << "[RobotControlHelper::getLimits] Unable get " << m_axesList[i]
                         << " joint limits.";
                return false;
            }
            else
            {
                limits(i,0) = m_positionFeedbackInDegrees(i);
                limits(i,1) = m_positionFeedbackInDegrees(i);
                yWarning() << "[RobotControlHelper::getLimits] Unable get " << m_axesList[i]
                           << " joint limits. The current joint value is used as lower and upper limits.";
            }

        }
    return true;
}
