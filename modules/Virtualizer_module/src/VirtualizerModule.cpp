/**
 * @file VirtualizerModule.cpp
 * @authors  Mohamed Babiker Mohamed Elobaid <mohamed.elobaid@iit.it>
 *           Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2018 iCub Facility - Istituto Italiano di Tecnologia
 *            Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 * @date 2018
 */

#define _USE_MATH_DEFINES
#include <cmath>

// YARP
#include <yarp/os/LogStream.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>

#include "Utils.hpp"
#include "VirtualizerModule.hpp"

bool VirtualizerModule::configureVirtualizer()
{
    // try to connect to the virtualizer
    int maxAttempt = 5;
    for (int i = 0; i < maxAttempt; i++)
    {
        m_cvirtDeviceID = CVirt::FindDevice();
        if (m_cvirtDeviceID != nullptr)
        {
            if (!m_cvirtDeviceID->Open())
            {
                yError() << "[configureVirtualizer] Unable to open the device";
                return false;
            }

            return true;
        }
        // wait one millisecond
        yarp::os::Time::delay(0.001);
    }

    yError() << "[configureVirtualizer] I'm not able to configure the virtualizer";
    return false;
}

bool VirtualizerModule::configure(yarp::os::ResourceFinder& rf)
{
    yarp::os::Value* value;
    // check if the configuration file is empty
    if (rf.isNull())
    {
        yError() << "[configure] Empty configuration for the force torque sensors.";
        return false;
    }

    // get the period
    m_dT = rf.check("period", yarp::os::Value(0.1)).asDouble();

    // set the module name
    std::string name;
    if (!YarpHelper::getStringFromSearchable(rf, "name", name))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    setName(name.c_str());

    // set deadzone
    if (!YarpHelper::getDoubleFromSearchable(rf, "deadzone", m_deadzone))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    if (!YarpHelper::getDoubleFromSearchable(rf, "velocityScaling", m_velocityScaling))
    {
        yError() << "[configure] Unable to get a double from a searchable";
        return false;
    }

    // open ports
    std::string portName;
    if (!YarpHelper::getStringFromSearchable(rf, "playerOrientationPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }

    if (!m_playerOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "robotOrientationPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_robotOrientationPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    if (!YarpHelper::getStringFromSearchable(rf, "rpcWalkingPort_name", portName))
    {
        yError() << "[configure] Unable to get a string from a searchable";
        return false;
    }
    if (!m_rpcPort.open("/" + getName() + portName))
    {
        yError() << "[configure] " << portName << " port already open.";
        return false;
    }

    // open RPC port for external command
    std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcServerPort);
    if(!m_rpcServerPort.open(rpcPortName))
    {
        yError() << "[configure] Could not open" << rpcPortName << "RPC port.";
        return false;
    }

    if (!configureVirtualizer())
    {
        yError() << "[configure] Unable to configure the virtualizer";
        return false;
    }

    // remove me!!!
    // this is because the virtualizer is not ready
    yarp::os::Time::delay(0.5);

	// reset player orientation
    m_cvirtDeviceID->ResetPlayerOrientation();

    // reset some quanties
    m_robotYaw = 0;
    m_oldPlayerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());
    m_oldPlayerYaw *= 360.0f;
    m_oldPlayerYaw = m_oldPlayerYaw * M_PI / 180;
    m_oldPlayerYaw = Angles::normalizeAngle(m_oldPlayerYaw);

    yarp::sig::Vector buff(2, 0.0);
    m_userPositionIntegrator = std::make_unique<iCub::ctrl::Integrator>(m_dT, buffer);

    double cutFrequency;
    if(!YarpHelper::getDoubleFromSearchable(rf, "omega_cut_frequency", cutFrequency))
    {
        yError() << "[configure] Unable get double from searchable.";
        return false;
    }

    yarp::sig::Vector buffOmega(1, 0.0);
    m_angularVelocityFilter = std::make_unique<iCub::ctrl::FirstOrderLowPassFilter>(cutFrequency, m_dT);
    m_positionFilter->init(buffOmega);

    return true;
}

double VirtualizerModule::getPeriod()
{
    return m_dT;
}

bool VirtualizerModule::close()
{
    // close the ports
    m_rpcPort.close();
    m_robotOrientationPort.close();
    m_playerOrientationPort.close();
    m_rpcServerPort.close();

    // deallocate memory
    delete m_cvirtDeviceID;

    return true;
}

bool VirtualizerModule::updateModule()
{
    std::lock_guard<std::mutex> guard(m_mutex);

    // get data from virtualizer
    double playerYaw;
    playerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());

    playerYaw *= 360.0f;
    playerYaw = playerYaw * M_PI / 180;
    playerYaw = Angles::normalizeAngle(playerYaw);

    yInfo() << "Current player yaw: " << playerYaw;
    // get the robot orientation
    yarp::sig::Vector* tmp = m_robotOrientationPort.read(false);
    if (tmp != NULL)
    {
        auto vector = *tmp;
        m_robotYaw = -Angles::normalizeAngle(vector[0]);
    }
    if (std::fabs(Angles::shortestAngularDistance(playerYaw, m_oldPlayerYaw)) > 0.15)
    {
        yError() << "Virtualizer misscalibrated or disconnected";
        return false;
    }

    double omega = (Angles::shortestAngularDistance(playerYaw, m_oldPlayerYaw))/m_dT;
    yarp::sig::Vector omegaVector(1);
    omegaVector = m_angularVelocityFilter->filt(yarp::sig::Vector(1, omega));

    m_oldPlayerYaw = playerYaw;

    double playerYawReverted = -playerYaw;
    // error between the robot orientation and the player orientation
    double angulareError = threshold(Angles::shortestAngularDistance(m_robotYaw, playerYaw));

    // get the player speed
    double speedData = (double)(m_cvirtDeviceID->GetMovementSpeed());

    double x = speedData * cos(playerYawReverted) * m_velocityScaling;
    double y = speedData * sin(playerYawReverted) * m_velocityScaling;

    iDynTree::MatrixFixSize<2,2> skewMatrix, rotationMatrix;
    skewMatrix.zero();
    skewMatrix(0,1) = -1;
    skewMatrix(1,0) = 1;
    rotationMatrix(0,0) = cos(playerYawReverted);
    rotationMatrix(0,1) = -sin(playerYawReverted);
    rotationMatrix(1,0) = sin(playerYawReverted);
    rotationMatrix(0,1) = cos(playerYawReverted);

    iDynTree::VectorFixSize<2> angularAdjustment;
    iDynTree::toEigen(angularAdjustment) = treshold(omegaVector(0)) * iDynTree::toEigen(rotationMatrix) *
        iDynTree::toEigen(rotationMatrix) * 0.1;

    // send data to the walking module
    yarp::os::Bottle cmd, outcome;
    cmd.addString("setGoal");
    cmd.addDouble(x + angularAdjustment(0));
    cmd.addDouble(y + angularAdjustment(1));
    m_rpcPort.write(cmd, outcome);

    // send the orientation of the player
    yarp::sig::Vector& playerOrientationVector = m_playerOrientationPort.prepare();
    playerOrientationVector.clear();
    playerOrientationVector.push_back(playerYaw);
    m_playerOrientationPort.write();

    return true;
}

void VirtualizerModule::resetPlayerOrientation()
{
    std::lock_guard<std::mutex> guard(m_mutex);
    m_cvirtDeviceID->ResetPlayerOrientation();

    m_oldPlayerYaw = (double)(m_cvirtDeviceID->GetPlayerOrientation());
    m_oldPlayerYaw *= 360.0f;
    m_oldPlayerYaw = m_oldPlayerYaw * M_PI / 180;
    m_oldPlayerYaw = Angles::normalizeAngle(m_oldPlayerYaw);
    return;
}

double VirtualizerModule::threshold(const double& input)
{
    if (input >= 0)
    {
        if (input > m_deadzone)
            return input;
        else
            return 0.0;
    } else
    {
        if (input < -m_deadzone)
            return input;
        else
            return 0.0;
    }
}
