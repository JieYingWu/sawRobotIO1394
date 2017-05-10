/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2012-07-31

  (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights Reserved.

 --- begin cisst license - do not edit ---

 This software is provided "as is" under an open source license, with
 no warranty.  The complete license can be found in license.txt and
 http://www.cisst.org/cisst/license.txt.

 --- end cisst license ---
 */

#include <iostream>

#include <cisstCommon/cmnXMLPath.h>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnUnits.h>

#include <cisstOSAbstraction/osaCPUAffinity.h>

#include <cisstMultiTask/mtsInterfaceProvided.h>

#include <sawRobotIO1394/mtsRobotIO1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaXML1394.h>

#include "mtsRobot1394.h"
#include "mtsDigitalInput1394.h"
#include "mtsDigitalOutput1394.h"


CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsRobotIO1394, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg)

using namespace sawRobotIO1394;

mtsRobotIO1394::mtsRobotIO1394(const std::string & name, const double periodInSeconds, const int portNumber):
    mtsTaskPeriodic(name, periodInSeconds)
{
    Init(portNumber);
}

mtsRobotIO1394::mtsRobotIO1394(const mtsTaskPeriodicConstructorArg & arg):
    mtsTaskPeriodic(arg)
{
    Init(0);
}

mtsRobotIO1394::~mtsRobotIO1394()
{
    // delete port and message stream
    delete mPort;
    delete MessageStream;
}

void mtsRobotIO1394::SetProtocol(const sawRobotIO1394::ProtocolType & protocol) 
{
    mPort->SetProtocol(protocol);
}

void mtsRobotIO1394::Init(const int portNumber)
{
    // construct port
    MessageStream = new std::ostream(this->GetLogMultiplexer());
    try {
        mPort = new sawRobotIO1394::osaPort1394(portNumber, *MessageStream);
    } catch (std::runtime_error &err) {
        CMN_LOG_CLASS_INIT_ERROR << err.what();
        abort();
    }

    mtsInterfaceProvided * mainInterface = AddInterfaceProvided("MainInterface");
    if (mainInterface) {
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBoards, this, "GetNumberOfBoards");
        mainInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this, "GetNumberOfRobots");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Init: failed to create provided interface \"MainInterface\", method Init should be called only once."
                                 << std::endl;
    }

    //////////////////////////////////////////////////////////////////
    ////////// RobotIO1394QtManager Configure Connection//////////////
    //////////////////////////////////////////////////////////////////

    // At this stage, the robot interfaces and the digital input interfaces should be ready.
    // Add on Configuration provided interface with functionWrite with vector of strings.
    // Provide names of robot, names of digital inputs, and name of this member.

    // All previous interfaces are ready. Good start. Let's make a new provided interface.
    mtsInterfaceProvided * configurationInterface   = this->AddInterfaceProvided("Configuration");
    if (configurationInterface) {
        configurationInterface->AddCommandRead(&osaPort1394::GetRobotNames, mPort,
                                               "GetRobotNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfActuatorsPerRobot, this,
                                               "GetNumActuators");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfBrakesPerRobot, this,
                                               "GetNumBrakes");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfRobots, this,
                                               "GetNumRobots");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalInputs, this,
                                               "GetNumDigitalInputs");
        configurationInterface->AddCommandRead(&osaPort1394::GetDigitalInputNames, mPort,
                                               "GetDigitalInputNames");
        configurationInterface->AddCommandRead(&mtsRobotIO1394::GetNumberOfDigitalOutputs, this,
                                               "GetNumDigitalOutputs");
        configurationInterface->AddCommandRead(&osaPort1394::GetDigitalOutputNames, mPort,
                                               "GetDigitalOutputNames");
        configurationInterface->AddCommandRead<mtsComponent>(&mtsComponent::GetName, this,
                                                             "GetName");
    } else {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to create configuration interface." << std::endl;
    }
}

void mtsRobotIO1394::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: configuring from " << filename << std::endl;

    osaPort1394Configuration config;
    osaXML1394ConfigurePort(filename, config);

    // Add all the robots
    for (std::vector<osaRobot1394Configuration>::const_iterator it = config.Robots.begin();
         it != config.Robots.end();
         ++it) {
        // Create a new robot
        mtsRobot1394 * robot = new mtsRobot1394(*this, *it);
        // Check the configuration
        if (!robot->CheckConfiguration()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: error in configuration file \""
                                     << filename << "\" for robot \""
                                     << robot->Name() << "\"" << std::endl;
            abort();
        }
        // Set up the cisstMultiTask interfaces
        if (!this->SetupRobot(robot)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: unable to setup interface(s) for robot \""
                                     << robot->Name() << "\"" << std::endl;
            delete robot;
        } else {
            mRobots.push_back(robot);
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: added robot \""
                                       << robot->Name() << "\"" << std::endl;

        }
    }

    // Add all the digital inputs
    for (std::vector<osaDigitalInput1394Configuration>::const_iterator it = config.DigitalInputs.begin();
         it != config.DigitalInputs.end();
         ++it) {
        // Create a new digital input
        mtsDigitalInput1394 * digitalInput = new mtsDigitalInput1394(*this, *it);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalInput(digitalInput)) {
            delete digitalInput;
        } else {
            mDigitalInputs.push_back(digitalInput);
        }
    }

    // Add all the digital outputs
    for (std::vector<osaDigitalOutput1394Configuration>::const_iterator it = config.DigitalOutputs.begin();
         it != config.DigitalOutputs.end();
         ++it) {
        // Create a new digital input
        mtsDigitalOutput1394 * digitalOutput = new mtsDigitalOutput1394(*this, *it);
        // Set up the cisstMultiTask interfaces
        if (!this->SetupDigitalOutput(digitalOutput)) {
            delete digitalOutput;
        } else {
            mDigitalOutputs.push_back(digitalOutput);
        }
    }
}

bool mtsRobotIO1394::SetupRobot(mtsRobot1394 * robot)
{
    mtsStateTable * stateTableRead;
    mtsStateTable * stateTableWrite;

    // Configure StateTable for this Robot
    if (!robot->SetupStateTables(2000, // hard coded number of elements in state tables
                                 stateTableRead, stateTableWrite)) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: unable to setup state tables" << std::endl;
        return false;
    }

    this->AddStateTable(stateTableRead);
    this->AddStateTable(stateTableWrite);

    // Add new InterfaceProvided for this Robot with Name.
    // Ensure all names from XML Config file are UNIQUE!
    mtsInterfaceProvided * robotInterface = this->AddInterfaceProvided(robot->Name());
    if (!robotInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot interface \""
                                 << robot->Name() << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }

    // Create actuator interface
    std::string actuatorInterfaceName = robot->Name();
    actuatorInterfaceName.append("Actuators");
    mtsInterfaceProvided * actuatorInterface = this->AddInterfaceProvided(actuatorInterfaceName);
    if (!actuatorInterface) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: failed to create robot actuator interface \""
                                 << actuatorInterfaceName << "\", do we have multiple robots with the same name?" << std::endl;
        return false;
    }

    // Setup the MTS interfaces
    robot->SetupInterfaces(robotInterface, actuatorInterface);

    // Add the robot to the port
    try {
        mPort->AddRobot(robot);
    } catch (osaRuntimeError1394 & err) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupRobot: unable to add the robot to the port: " << err.what() << std::endl;
        return false;
    }
    return true;
}

bool mtsRobotIO1394::SetupDigitalInput(mtsDigitalInput1394 * digitalInput)
{
    // Configure pressed active direction and edge detection
    digitalInput->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * digitalInInterface = this->AddInterfaceProvided(digitalInput->Name());

    digitalInput->SetupProvidedInterface(digitalInInterface, this->StateTable);

    // Add the digital input to the port
    try {
        mPort->AddDigitalInput(digitalInput);
    } catch (osaRuntimeError1394 & err) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupDigitalInput: unable to add the digital input to the port: " << err.what() << std::endl;
        return false;
    }
    return true;
}

bool mtsRobotIO1394::SetupDigitalOutput(mtsDigitalOutput1394 * digitalOutput)
{
    // Configure pressed active direction and edge detection
    digitalOutput->SetupStateTable(this->StateTable);

    mtsInterfaceProvided * digitalOutInterface = this->AddInterfaceProvided(digitalOutput->Name());

    digitalOutput->SetupProvidedInterface(digitalOutInterface, this->StateTable);

    // Add the digital input to the port
    try {
        mPort->AddDigitalOutput(digitalOutput);
    } catch (osaRuntimeError1394 & err) {
        CMN_LOG_CLASS_INIT_ERROR << "SetupDigitalOutput: unable to add the digital output to the port: " << err.what() << std::endl;
        return false;
    }
    return true;
}

void mtsRobotIO1394::Startup(void)
{
    // osaCPUSetAffinity(OSA_CPU4);
    const robots_iterator robotsEnd = mRobots.end();
    for (robots_iterator robot = mRobots.begin();
         robot != robotsEnd;
         ++robot) {
        (*robot)->SetEncoderPosition(vctDoubleVec((*robot)->NumberOfActuators(), 0.0));
    }
}

void mtsRobotIO1394::PreRead(void)
{
    const robots_iterator robotsEnd = mRobots.end();
    for (robots_iterator robot = mRobots.begin();
         robot != robotsEnd;
         ++robot) {
        (*robot)->StartReadStateTable();
    }
}

void mtsRobotIO1394::PostRead(void)
{
    // Trigger robot events
    const robots_iterator robotsEnd = mRobots.end();
    for (robots_iterator robot = mRobots.begin();
         robot != robotsEnd;
         ++robot) {
        try {
            (*robot)->CheckState();
        } catch (std::exception & stdException) {
            CMN_LOG_CLASS_RUN_ERROR << "PostRead: " << (*robot)->Name() << ": standard exception \"" << stdException.what() << "\"" << std::endl;
            (*robot)->mInterface->SendError("IO exception: " + (*robot)->Name() + ", " + stdException.what());
        } catch (...) {
            CMN_LOG_CLASS_RUN_ERROR << "PostRead: " << (*robot)->Name() << ": unknown exception" << std::endl;
            (*robot)->mInterface->SendError("IO unknown exception: " + (*robot)->Name());
        }
        (*robot)->AdvanceReadStateTable();
    }
    // Trigger digital input events
    const digital_inputs_iterator digital_inputs_end = mDigitalInputs.end();
    for (digital_inputs_iterator digital_input = mDigitalInputs.begin();
         digital_input != digital_inputs_end;
         ++digital_input) {
        (*digital_input)->CheckState();
    }
}

void mtsRobotIO1394::PreWrite(void)
{
    const robots_iterator robotsEnd = mRobots.end();
    for (robots_iterator robot = mRobots.begin();
         robot != robotsEnd;
         ++robot) {
        (*robot)->StartWriteStateTable();
    }
}

void mtsRobotIO1394::PostWrite(void)
{
    // Trigger robot events
    const robots_iterator robotsEnd = mRobots.end();
    for (robots_iterator robot = mRobots.begin();
         robot != robotsEnd;
         ++robot) {
        (*robot)->AdvanceWriteStateTable();
    }
}

void mtsRobotIO1394::Run(void)
{
    // Read from all boards
    bool gotException = false;
    std::string message;

    this->PreRead();
    try {
        mPort->Read();
    } catch (sawRobotIO1394::osaRuntimeError1394 & sawException) {
        gotException = true;
        message = this->Name + ": sawRobotIO1394 exception \"" + sawException.what() + "\"";
    } catch (std::exception & stdException) {
        gotException = true;
        message = this->Name + ": standard exception \"" + stdException.what() + "\"";
    } catch (...) {
        gotException = true;
        message = this->Name + ": unknown exception";
    }
    if (gotException) {
        CMN_LOG_CLASS_RUN_ERROR << "Run: port read, " << message << std::endl;
        // Trigger robot events
        const robots_iterator robotsEnd = mRobots.end();
        for (robots_iterator robot = mRobots.begin();
             robot != robotsEnd;
             ++robot) {
            (*robot)->mInterface->SendError(message);
        }
    }
    this->PostRead(); // this performs all state conversions and checks

    // Invoke connected components (if any)
    this->RunEvent();

    // Process queued commands (e.g., to set motor current)
    this->ProcessQueuedCommands();

    // Write to all boards
    this->PreWrite();
    mPort->Write();
    this->PostWrite();
}

void mtsRobotIO1394::Cleanup(void)
{
    for (size_t i = 0; i < mRobots.size(); i++) {
        if (mRobots[i]->Valid()) {
            mRobots[i]->DisablePower();
        }
    }
    // Write to all boards
    mPort->Write();
}

void mtsRobotIO1394::GetNumberOfDigitalInputs(int & placeHolder) const
{
    placeHolder = mPort->NumberOfDigitalInputs();
}

void mtsRobotIO1394::GetNumberOfDigitalOutputs(int & placeHolder) const
{
    placeHolder = mPort->NumberOfDigitalOutputs();
}

void mtsRobotIO1394::GetNumberOfBoards(int & placeHolder) const
{
    placeHolder = mPort->NumberOfBoards();
}

void mtsRobotIO1394::GetNumberOfRobots(int & placeHolder) const
{
    placeHolder = mPort->NumberOfRobots();
}

void mtsRobotIO1394::GetNumberOfActuatorsPerRobot(vctIntVec & placeHolder) const
{
    size_t numRobots = mPort->NumberOfRobots();
    placeHolder.resize(numRobots);

    for (size_t i = 0; i < numRobots; i++) {
        placeHolder[i] = mPort->Robot(i)->NumberOfActuators();
    }
}

void mtsRobotIO1394::GetNumberOfBrakesPerRobot(vctIntVec & placeHolder) const
{
    size_t numRobots = mPort->NumberOfRobots();
    placeHolder.resize(numRobots);

    for (size_t i = 0; i < numRobots; i++) {
        placeHolder[i] = mPort->Robot(i)->NumberOfBrakes();
    }
}
