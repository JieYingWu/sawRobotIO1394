/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Anton Deguet
  Created on: 2014-01-09

  (C) Copyright 2014-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---

*/

// system
#include <iostream>
// cisst/saw
#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <sawRobotIO1394/osaConfiguration1394.h>
#include <sawRobotIO1394/osaXML1394.h>
#include <sawRobotIO1394/osaPort1394.h>
#include <sawRobotIO1394/osaRobot1394.h>


int main(int argc, char * argv[])
{
    cmnCommandLineOptions options;
    int portNumber = 0;
    size_t actuatorIndex = 0;
    size_t numberOfIterations;
    double sleepBetweenReads = 0.3 * cmn_ms;
    std::string configFile;
    options.AddOptionOneValue("c", "config",
                              "configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &configFile);
    options.AddOptionOneValue("a", "actuator",
                              "actuator index",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &actuatorIndex);
    options.AddOptionOneValue("p", "port",
                              "firewire port number(s)",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &portNumber);
    options.AddOptionOneValue("n", "number-iterations",
                              "number of iterations",
                              cmnCommandLineOptions::REQUIRED_OPTION, &numberOfIterations);
    options.AddOptionOneValue("s", "sleep-between-reads",
                              "sleep between reads",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &sleepBetweenReads);

    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }

    if (!cmnPath::Exists(configFile)) {
        std::cerr << "Can't find file \"" << configFile << "\"." << std::endl;
        return -1;
    }
    std::cout << "Configuration file: " << configFile << std::endl
              << "Port: " << portNumber << std::endl;

    // add arrays here for all data collected....
    std::cout << "Allocation memory for " << numberOfIterations << " samples." << std::endl;
    size_t * allIterations = new size_t[numberOfIterations];
    double * allActuatorTimeStamps = new double[numberOfIterations];
    double * allCPUTimes = new double[numberOfIterations];
    double * allTimesDiff = new double[numberOfIterations];
    double * allPositions = new double[numberOfIterations];
    double * allFpgaVelocities = new double[numberOfIterations];
    double * allFpgaVelocitiesLowRes = new double[numberOfIterations];
    double * allSoftwareVelocities = new double[numberOfIterations];
    double * allSoftwareDxDtFPGA = new double[numberOfIterations];
    double * allSoftwareDxDtCPU = new double[numberOfIterations];
    bool * allLatched = new bool[numberOfIterations];
    uint * allFpgaVelocitiesRaw = new uint[numberOfIterations];
    uint * allFpgaVelocitiesLowResRaw = new uint[numberOfIterations];
    
    std::cout << "Loading config file ..." << std::endl;
    sawRobotIO1394::osaPort1394Configuration config;
    sawRobotIO1394::osaXML1394ConfigurePort(configFile, config);

    std::cout << "Creating robot ..." << std::endl;
    if (config.Robots.size() == 0) {
        std::cerr << "Error: the config file doesn't define a robot." << std::endl;
        return -1;
    }
    if (config.Robots.size() != 1) {
        std::cerr << "Error: the config file defines more than one robot." << std::endl;
        return -1;
    }
    sawRobotIO1394::osaRobot1394 * robot = new sawRobotIO1394::osaRobot1394(config.Robots[0]);

    std::cout << "Creating port ..." << std::endl;
    sawRobotIO1394::osaPort1394 * port = new sawRobotIO1394::osaPort1394(portNumber);
    port->AddRobot(robot);

    // make sure we have at least one set of pots values
    try {
        port->Read();
    } catch (const std::runtime_error & e) {
        std::cerr << "Caught exception: " << e.what() << std::endl;
    }
    // preload encoders
    robot->CalibrateEncoderOffsetsFromPots();

    std::cout << "Starting data collection." << std::endl;

    size_t percent = numberOfIterations / 100;
    size_t progress = 0;
    double oldPos = 0.0;
    double fpgaTime = 0.0;
    double oldCPUTime = 0.0;
    
    // main loop
    for (size_t iter = 0;
         iter < numberOfIterations;
         ++iter) {
        port->Read();

        // save index
        allIterations[iter] = iter;

        // CPU time
        const double currentCPUTime =
            mtsManagerLocal::GetInstance()->GetTimeServer().GetRelativeTime(); 
        allCPUTimes[iter] = currentCPUTime; 

        // get time from FPGA
        const double currentActuatorTimeStamp =
            robot->ActuatorTimeStamp()[actuatorIndex];
        allActuatorTimeStamps[iter] = currentActuatorTimeStamp;

        // Check for offset between CPU and FPGA time
        fpgaTime = fpgaTime + currentActuatorTimeStamp;
        allTimesDiff[iter] = currentCPUTime - fpgaTime;
        
        // Get positions, see osaRobot1394.cpp, line ~1000
        const double currentEncoderPosition =
            robot->EncoderPosition()[actuatorIndex];
        allPositions[iter] = currentEncoderPosition;

        // Get velocities
        allFpgaVelocities[iter] =
            robot->EncoderVelocity()[actuatorIndex];

        // Get low resolution FPGA velocity
        allFpgaVelocitiesLowRes[iter] =
            robot->EncoderVelocityLowRes()[actuatorIndex];

        // Get original software velocity
        allSoftwareVelocities[iter] =
            robot->EncoderVelocitySoftware()[actuatorIndex];
        
        // Software velocity Dx/Dt
        allSoftwareDxDtFPGA[iter] = ((currentEncoderPosition - oldPos)
                                     / currentActuatorTimeStamp) + 0.01;

        allSoftwareDxDtCPU[iter] = (currentEncoderPosition - oldPos)
            / (currentCPUTime - oldCPUTime);

        // Get whether velocity is latched or free running
        allLatched[iter] =
            robot->EncoderVelocityLatched()[actuatorIndex];

        // Get raw velocity quadlets
        allFpgaVelocitiesRaw[iter] =
            robot->EncoderVelocityRaw()[actuatorIndex];
        
        // Get raw low res velocity quadlets
        allFpgaVelocitiesLowResRaw[iter] =
            robot->EncoderVelocityLowResRaw()[actuatorIndex];
        
        // maintain old variables
        oldPos = currentEncoderPosition;
        oldCPUTime = currentCPUTime;
        
        // display progress
        progress++;
        if (progress == percent) {
            std::cout << "." << std::flush;
            progress = 0;
        }

        // finally sleep as requested
        osaSleep(sleepBetweenReads);
    }
    std::cout << std::endl;

    // save to csv file
    std::string currentDateTime;
    osaGetDateTimeString(currentDateTime);
    std::string fileName = "data-" + currentDateTime + ".csv";

    std::cout << "Saving to file: " << fileName << std::endl;
    std::ofstream output;
    output.open(fileName.c_str());

    // header
    output << "iteration,"
           << "cpu-time,"
           << "fpga-time,"
           << "fpga-dtime,"
           << "encoder-pos,"
           << "fpga-velocities,"
           << "fpga-velocities-low-res,"
           << "software-velocities,"
           << "dx/dt-fpga,"
           << "dx/dt-cpu,"
           << "latched,"
           << "fpga-raw,"
           << "fpga-low-res-raw"
           << std::endl;
    
    output << std::setprecision(17);
    // start at 2000 since first data might have some garbage
    for (size_t iter = 2000;
         iter < numberOfIterations;
         ++iter) {
        output << allIterations[iter] << ","
               << allCPUTimes[iter] << ","
               << allTimesDiff[iter] << ","
               << allActuatorTimeStamps[iter] << ","
               << allPositions[iter] << ","
               << allFpgaVelocities[iter] << ","
               << allFpgaVelocitiesLowRes[iter] << ","
               << allSoftwareVelocities[iter] << ","
               << allSoftwareDxDtFPGA[iter] << ","
               << allSoftwareDxDtCPU[iter] << ","
               << allLatched[iter] << ","
               << allFpgaVelocitiesRaw[iter] << ","
               << allFpgaVelocitiesLowResRaw[iter]
               << std::endl;
    }

    output.close();

    // cleanup
    delete allIterations;
    delete allActuatorTimeStamps;

    delete port;
    return 0;
}
