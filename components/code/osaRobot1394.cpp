/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides, Jonathan Bohren, Anton Deguet, Jie Ying Wu
  Created on: 2011-06-10

  (C) Copyright 2011-2015 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cmath>

#include <sawRobotIO1394/osaRobot1394.h>

#ifndef SAW_ROBOT_IO_1394_WO_CISST
#include <cisstCommon/cmnUnits.h>
#include <cisstOSAbstraction/osaSleep.h>
#endif

#include <Amp1394/AmpIORevision.h>
#if ((Amp1394_VERSION_MAJOR < 1) || ((Amp1394_VERSION_MAJOR == 1) && (Amp1394_VERSION_MINOR < 1)))
#error "Version 1.1 or higher of libAmpIO is required (change to signed encoder positions)"
#endif

#include <AmpIO.h>

using namespace sawRobotIO1394;

osaRobot1394::osaRobot1394(const osaRobot1394Configuration & config,
                           const size_t maxConsecutiveCurrentSafetyViolations,
                           const size_t maxConsecutivePotsToEncodersViolations):
    // IO Structures
    mActuatorInfo(),
    mUniqueBoards(),
    // State Initialization
    mValid(false),
    mPowerStatus(false),
    mPreviousPowerStatus(false),
    mWatchdogStatus(false),
    mPreviousWatchdogStatus(false),
    mSafetyRelay(false),
    mCurrentSafetyViolationsCounter(0),
    mCurrentSafetyViolationsMaximum(maxConsecutiveCurrentSafetyViolations),
    mPotsToEncodersViolationsCounter(0),
    mPotsToEncodersViolationsMaximum(maxConsecutivePotsToEncodersViolations)
{
    this->Configure(config);
}

void osaRobot1394::Configure(const osaRobot1394Configuration & config)
{
    // Store the configuration
    mConfiguration = config;

    //  info
    mName = config.Name;
    mNumberOfActuators = config.NumberOfActuators;
    mNumberOfJoints = config.NumberOfJoints;
    mSerialNumber = config.SerialNumber;
    mPotType = config.PotLocation;

    // Low-level API
    mActuatorInfo.resize(mNumberOfActuators);

    // Initialize state vectors to the appropriate sizes
    mActuatorPowerStatus.SetSize(mNumberOfActuators);
    mActuatorPowerEnabled.SetSize(mNumberOfActuators);
    mDigitalInputs.SetSize(mNumberOfActuators);
    mEncoderChannelsA.SetSize(mNumberOfActuators);
    mPotBits.SetSize(mNumberOfActuators);
    mEncoderOverflow.SetSize(mNumberOfActuators);
    mPreviousEncoderOverflow.SetSize(mNumberOfActuators);
    mPreviousEncoderOverflow.SetAll(false);
    mEncoderPositionBits.SetSize(mNumberOfActuators);
    mEncoderPositionBitsPrev.SetSize(mNumberOfActuators);
    mEncoderPositionBitLowResPrev.SetSize(mNumberOfActuators);
    mEncoderVelocityBits.SetSize(mNumberOfActuators);
    mEncoderVelocityLowResBits.SetSize(mNumberOfActuators);
    mActuatorCurrentBitsCommand.SetSize(mNumberOfActuators);
    mActuatorCurrentBitsFeedback.SetSize(mNumberOfActuators);

    mActuatorTimestamp.SetSize(mNumberOfActuators);
    mActuatorTimestampChange.SetSize(mNumberOfActuators);
    mActuatorTimestampChange.SetAll(0.0);
    mActuatorTimestampChangeLowRes.SetSize(mNumberOfActuators);
    mActuatorTimestampChangeLowRes.SetAll(0.0);
    mVelocitySlopeToZero.SetSize(mNumberOfActuators);
    mVelocitySlopeToZero.SetAll(0.0);
    mPotVoltage.SetSize(mNumberOfActuators);
    mPotPosition.SetSize(mNumberOfActuators);
    mEncoderPosition.SetSize(mNumberOfActuators);
    mEncoderPositionPrev.SetSize(mNumberOfActuators);
    mEncoderVelocity.SetSize(mNumberOfActuators);
    mEncoderVelocityLowRes.SetSize(mNumberOfActuators);
    mEncoderVelocitySoftwareLowRes.SetSize(mNumberOfActuators);
    mEncoderVelocityDxDt.SetSize(mNumberOfActuators);
    mEncoderVelocitySoftware.SetSize(mNumberOfActuators);
    mJointPosition.SetSize(mNumberOfJoints);
    mJointVelocity.SetSize(mNumberOfJoints);
    mJointVelocityLowRes.SetSize(mNumberOfJoints);
    mJointTorque.SetSize(mNumberOfJoints);
    mActuatorCurrentCommand.SetSize(mNumberOfActuators);
    mActuatorEffortCommand.SetSize(mNumberOfActuators);
    mActuatorCurrentFeedback.SetSize(mNumberOfActuators);
    mActuatorEffortFeedback.SetSize(mNumberOfActuators);

    // Initialize property vectors to the appropriate sizes
    mJointType.SetSize(mNumberOfJoints);

    mEffortToCurrentScales.SetSize(mNumberOfActuators);
    mActuatorCurrentToBitsScales.SetSize(mNumberOfActuators);
    mActuatorCurrentToBitsOffsets.SetSize(mNumberOfActuators);
    mActuatorBitsToCurrentScales.SetSize(mNumberOfActuators);
    mActuatorBitsToCurrentOffsets.SetSize(mNumberOfActuators);
    mActuatorEffortCommandLimits.SetSize(mNumberOfActuators);
    mJointEffortCommandLimits.SetSize(mNumberOfJoints);
    mActuatorCurrentCommandLimits.SetSize(mNumberOfActuators);
    mActuatorCurrentFeedbackLimits.SetSize(mNumberOfActuators);
    if (mPotType == POTENTIOMETER_ON_ACTUATORS) {
        mPotsToEncodersTolerance.SetSize(mNumberOfActuators);
        mPotsToEncodersError.SetSize(mNumberOfActuators);
        mPotsToEncodersErrorFlag.SetSize(mNumberOfActuators);
    } else if (mPotType == POTENTIOMETER_ON_JOINTS) {
        mPotsToEncodersTolerance.SetSize(mNumberOfJoints);
        mPotsToEncodersError.SetSize(mNumberOfJoints);
        mPotsToEncodersErrorFlag.SetSize(mNumberOfJoints);
    }
    mPotsToEncodersTolerance.SetAll(0.0);
    mPotsToEncodersErrorFlag.SetAll(true);
    mUsePotsForSafetyCheck = false;

    mBitsToPositionScales.SetSize(mNumberOfActuators);
    mBitsToDPositionScales.SetSize(mNumberOfActuators);
    mBitsToDPositionOffsets.SetSize(mNumberOfActuators);
    mBitsToDTimeScales.SetSize(mNumberOfActuators);
    mBitsToDTimeOffsets.SetSize(mNumberOfActuators);
    mBitsToVecocityScales.SetSize(mNumberOfActuators);
    mBitsToVelocityOffsets.SetSize(mNumberOfActuators);

    mBitsToVoltageScales.SetSize(mNumberOfActuators);
    mBitsToVoltageOffsets.SetSize(mNumberOfActuators);
    mVoltageToPositionScales.SetSize(mNumberOfActuators);
    mVoltageToPositionOffsets.SetSize(mNumberOfActuators);
    mCountsPerTurn.SetSize(mNumberOfActuators);

    mActuatorTemperature.SetSize(mNumberOfActuators);

    mNumberOfBrakes = 0;
    mBrakeReleasing = false;

    // Construct property vectors
    for (size_t i = 0; i < mNumberOfActuators; i++) {

        // Local references to the config properties
        const osaActuator1394Configuration & actuator = config.Actuators[i];
        const osaDrive1394Configuration & drive = actuator.Drive;
        const osaEncoder1394Configuration & encoder = actuator.Encoder;
        const osaPot1394Configuration & pot = actuator.Pot;

        mJointType[i] = actuator.JointType;

        mEffortToCurrentScales[i]         = drive.EffortToCurrentScale;
        mActuatorCurrentToBitsScales[i]   = drive.CurrentToBitsScale;
        mActuatorCurrentToBitsOffsets[i]  = drive.CurrentToBitsOffset;
        mActuatorBitsToCurrentScales[i]   = drive.BitsToCurrentScale;
        mActuatorBitsToCurrentOffsets[i]  = drive.BitsToCurrentOffset;
        mActuatorEffortCommandLimits[i]   = drive.EffortCommandLimit;
        mActuatorCurrentCommandLimits[i]  = drive.CurrentCommandLimit;
        // 120% of command current is in the acceptable range
        // Add 50 mA for non motorized actuators due to a2d noise
        mActuatorCurrentFeedbackLimits[i] = 1.2 * mActuatorCurrentCommandLimits[i] + (50.0 / 1000.0);

        mBitsToPositionScales[i]   = encoder.BitsToPositionScale;
        mBitsToDPositionScales[i]  = encoder.BitsToDPositionScale;
        mBitsToDPositionOffsets[i] = encoder.BitsToDPositionOffset;
        mBitsToDTimeScales[i]      = encoder.BitsToDTimeScale;
        mBitsToDTimeOffsets[i]     = encoder.BitsToDTimeOffset;
        mBitsToVecocityScales[i]   = encoder.BitsToVelocityScale;
        mBitsToVelocityOffsets[i]  = encoder.BitsToVelocityOffset;
        mCountsPerTurn[i]          = encoder.CountsPerTurn;

        mBitsToVoltageScales[i]      = pot.BitsToVoltageScale;
        mBitsToVoltageOffsets[i]     = pot.BitsToVoltageOffset;
        mVoltageToPositionScales[i]  = pot.VoltageToPositionScale;
        mVoltageToPositionOffsets[i] = pot.VoltageToPositionOffset;

        // Initialize state vectors
        mEncoderPosition[i] = 0.0;
        mEncoderPositionPrev[i] = 0.0;
        mActuatorCurrentCommand[i] = 0.0;
        mActuatorCurrentFeedback[i] = 0.0;

        // Count number of brakes
        if (actuator.Brake) {
            mNumberOfBrakes++;
        }
    }

    // Update brake data
    mBrakeInfo.resize(mNumberOfBrakes);
    mBrakeReleasingTimer.resize(mNumberOfBrakes);

    mBrakeCurrentToBitsScales.SetSize(mNumberOfBrakes);
    mBrakeCurrentToBitsOffsets.SetSize(mNumberOfBrakes);
    mBrakeBitsToCurrentScales.SetSize(mNumberOfBrakes);
    mBrakeBitsToCurrentOffsets.SetSize(mNumberOfBrakes);
    mBrakeCurrentCommandLimits.SetSize(mNumberOfBrakes);
    mBrakeCurrentFeedbackLimits.SetSize(mNumberOfBrakes);
    mBrakePowerStatus.SetSize(mNumberOfBrakes);
    mBrakePowerEnabled.SetSize(mNumberOfBrakes);
    mBrakeCurrentBitsCommand.SetSize(mNumberOfBrakes);
    mBrakeCurrentBitsFeedback.SetSize(mNumberOfBrakes);
    mBrakeTimestamp.SetSize(mNumberOfBrakes);
    mBrakeCurrentCommand.SetSize(mNumberOfBrakes);
    mBrakeCurrentFeedback.SetSize(mNumberOfBrakes);
    mBrakeTemperature.SetSize(mNumberOfBrakes);
    mBrakeReleaseCurrent.SetSize(mNumberOfBrakes);
    mBrakeReleaseTime.SetSize(mNumberOfBrakes);
    mBrakeReleasedCurrent.SetSize(mNumberOfBrakes);
    mBrakeEngagedCurrent.SetSize(mNumberOfBrakes);

    // Construct property vectors for brakes
    size_t currentBrake = 0;
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        const osaActuator1394Configuration & actuator = config.Actuators[i];

        // Count number of brakes
        if (actuator.Brake) {
            const osaAnalogBrake1394Configuration * brake = actuator.Brake;
            const osaDrive1394Configuration & drive = brake->Drive;
            mBrakeCurrentToBitsScales[currentBrake]   = drive.CurrentToBitsScale;
            mBrakeCurrentToBitsOffsets[currentBrake]  = drive.CurrentToBitsOffset;
            mBrakeBitsToCurrentScales[currentBrake]   = drive.BitsToCurrentScale;
            mBrakeBitsToCurrentOffsets[currentBrake]  = drive.BitsToCurrentOffset;
            mBrakeCurrentCommandLimits[currentBrake]  = drive.CurrentCommandLimit;
            // 120% of command current is in the acceptable range
            // Add 50 mA for a2d noise around 0
            mBrakeCurrentFeedbackLimits[currentBrake] = 1.2 * mBrakeCurrentCommandLimits[currentBrake] + (50.0 / 1000.0);

            mBrakeReleaseCurrent[currentBrake]  = brake->ReleaseCurrent;
            mBrakeReleaseTime[currentBrake]     = brake->ReleaseTime;
            mBrakeReleasedCurrent[currentBrake] = brake->ReleasedCurrent;
            mBrakeEngagedCurrent[currentBrake]  = brake->EngagedCurrent;

            // Initialize defaults
            mBrakeCurrentCommand[currentBrake] = 0.0;
            mBrakeCurrentFeedback[currentBrake] = 0.0;

            currentBrake++;
        }
    }

    // Compute effort command limits
    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointEffortCommandLimits.ProductOf(mConfiguration.Coupling.ActuatorToJointEffort(),
                                            mActuatorEffortCommandLimits);
    } else {
        mJointEffortCommandLimits.Assign(mActuatorEffortCommandLimits);
    }
}

void osaRobot1394::SetBoards(const std::vector<osaActuatorMapping> & actuatorBoards,
                             const std::vector<osaBrakeMapping> & brakeBoards)
{
    if (actuatorBoards.size() != mNumberOfActuators) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of actuator boards different than the number of actuators."));
    }

    if (brakeBoards.size() != mNumberOfBrakes) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of brake boards different than the number of brakes."));
    }

    for (size_t i = 0; i < mNumberOfActuators; i++) {
        // Store this board
        mActuatorInfo[i].Board = actuatorBoards[i].Board;
        mActuatorInfo[i].Axis = actuatorBoards[i].Axis;
        // Construct a list of unique boards
        mUniqueBoards[actuatorBoards[i].Board->GetBoardId()] = actuatorBoards[i].Board;
    }

    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        // Store this board
        mBrakeInfo[i].Board = brakeBoards[i].Board;
        mBrakeInfo[i].Axis = brakeBoards[i].Axis;
        // Construct a list of unique boards
        mUniqueBoards[brakeBoards[i].Board->GetBoardId()] = brakeBoards[i].Board;
    }

    mIsAllBoardsFirmWareFour = true;
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO_UInt32 fversion = board->second->GetFirmwareVersion();
        if (fversion < 4) {
            mIsAllBoardsFirmWareFour = false;
            break;
        }
    }
}

void osaRobot1394::PollValidity(void)
{
    // Make sure the boards have been configured
    if (mNumberOfActuators != mActuatorInfo.size()) {
        cmnThrow(osaRuntimeError1394(this->Name() + ": number of boards different than the number of actuators."));
    }

    // Store previous state
    mPreviousPowerStatus = mPowerStatus;
    mPreviousWatchdogStatus = mWatchdogStatus;

    // Initialize flags
    mValid = true;
    mPowerStatus = true;
    mSafetyRelay = true;
    mWatchdogStatus = true;

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        mValid &= board->second->ValidRead();
        mPowerStatus &= board->second->GetPowerStatus();
        mSafetyRelay &= board->second->GetSafetyRelayStatus();
        mWatchdogStatus &= board->second->GetWatchdogTimeoutStatus();
    }

    if (!mValid) {
        std::stringstream message;
        message << this->Name() << ": read error on board(s) ";
        for (unique_board_iterator board = mUniqueBoards.begin();
             board != mUniqueBoards.end();
             ++board) {
            if (!board->second->ValidRead()) {
                message << static_cast<int>(board->second->GetBoardId()) << " ";
            }
        }
        cmnThrow(osaRuntimeError1394(message.str()));
    }
}

void osaRobot1394::PollState(void)
{
    // Poll data
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        AmpIO * board = mActuatorInfo[i].Board;
        int axis = mActuatorInfo[i].Axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mActuatorTimestamp[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        mDigitalInputs[i] = board->GetDigitalInput();

        // vectors of bits
        mEncoderOverflow[i] = board->GetEncoderOverflow(axis);
        mEncoderChannelsA[i] = board->GetEncoderChannelA(axis);

        // convert from 24 bits signed stored in 32 unsigned to 32 signed
        mEncoderPositionBits[i] = board->GetEncoderPosition(axis);
        mEncoderVelocityBits[i] = board->GetEncoderVelocity(axis,0);
        mEncoderVelocityLowResBits[i] = floor(board->GetEncoderVelocity(axis,1)/64);

        mPotBits[i] = board->GetAnalogInput(axis);

        mActuatorCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mActuatorPowerEnabled[i] = board->GetAmpEnable(axis);
        mActuatorPowerStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 actuators, second to last 2
        // board reports temperature in celsius * 2
        mActuatorTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        AmpIO * board = mBrakeInfo[i].Board;
        int axis = mBrakeInfo[i].Axis;

        if (!board || (axis < 0)) continue; // We probably don't need this check any more

        mBrakeTimestamp[i] = board->GetTimestamp() * 1.0 / 49125000.0;
        mBrakeCurrentBitsFeedback[i] = board->GetMotorCurrent(axis);
        mBrakePowerEnabled[i] = board->GetAmpEnable(axis);
        mBrakePowerStatus[i] = board->GetAmpStatus(axis);

        // first temperature corresponds to first 2 brakes, second to last 2
        // board reports temperature in celsius * 2
        mBrakeTemperature[i] = (board->GetAmpTemperature(axis / 2)) / 2.0;
    }

}

void osaRobot1394::ConvertState(void)
{
    // Perform read conversions
    EncoderBitsToPosition(mEncoderPositionBits, mEncoderPosition);
    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointPosition.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                                 mEncoderPosition);
    } else {
        mJointPosition.Assign(mEncoderPosition);
    }

    // Velocity computation
    // compute both
    EncoderBitsToVelocity(mEncoderVelocityBits, mEncoderVelocity);   // 1/dt
    EncoderBitsToVelocity(mEncoderVelocityLowResBits, mEncoderVelocityLowRes);   // 1/dt

    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointVelocity.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                                 mEncoderVelocity);
    } else {
        mJointVelocity.Assign(mEncoderVelocity);
    }

    mActuatorTemperature.Assign(mJointVelocity);

    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointVelocityLowRes.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                                 mEncoderVelocityLowRes);
    } else {
        mJointVelocityLowRes.Assign(mEncoderVelocityLowRes);
    }


    // using iterator for efficiency and going over all actuators
    const double timeToZeroVelocity = 1.0 * cmn_s;

        const vctIntVec::const_iterator end = mEncoderPositionBits.end();
        vctIntVec::const_iterator currentEncoder, previousEncoder;
        vctDoubleVec::const_iterator currentTimestamp, bitsToPos;
        vctDoubleVec::iterator lastChangeTimestamp, slope, velocity;
        for (currentEncoder = mEncoderPositionBits.begin(),
                 previousEncoder = mEncoderPositionBitsPrev.begin(),
                 currentTimestamp = mActuatorTimestamp.begin(),
                 bitsToPos = mBitsToPositionScales.begin(),
                 lastChangeTimestamp = mActuatorTimestampChange.begin(),
                 slope = mVelocitySlopeToZero.begin(),
                 velocity = mEncoderVelocitySoftware.begin();
             // end
             currentEncoder != end;
             // increment
             ++currentEncoder,
                 ++previousEncoder,
                 ++currentTimestamp,
                 ++bitsToPos,
                 ++lastChangeTimestamp,
                 ++slope,
                 ++velocity) {
            // first see if there has been any change
            const int difference = (*currentEncoder) - (*previousEncoder);
            if (difference == 0) {
                if (*lastChangeTimestamp < timeToZeroVelocity) {
                    *velocity -= (*slope) * (*currentTimestamp);
                } else {
                    *velocity = 0.0;
                }
                *lastChangeTimestamp += (*currentTimestamp);
            } else {
                {
                    *lastChangeTimestamp += (*currentTimestamp);
                // if we only have one bit change compute velocity since last change
                if ((difference == 1) || (difference == -1)) {
                    *velocity = (difference / (*lastChangeTimestamp))
                        * (*bitsToPos);
                } else if (difference > 1) {
                    // we know all but 1 bit difference happened in last Dt, other bit change happened between now and last change
                    *velocity = ((difference - 1.0) / (*currentTimestamp) + 1.0 / (*lastChangeTimestamp))
                        * (*bitsToPos);
                } else {
                    *velocity = ((difference + 1.0) / (*currentTimestamp) - 1.0 / (*lastChangeTimestamp))
                        * (*bitsToPos);
                }
                // keep record of this change
                *lastChangeTimestamp = 0.0;
                *slope = (*velocity) / (timeToZeroVelocity);
                }
            }
        }

        // finally save previous encoder bits position
        mEncoderPositionBitsPrev.Assign(mEncoderPositionBits);
        mEncoderVelocity.Assign(mEncoderVelocitySoftware);

        // This needs to be reviewed and we need to provide a better mechanism to choose one way or
        // another to compute the joint velocities (used by PID and displayed in Qt widget).

        // mJointVelocity.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderVelocity);
        if (mConfiguration.HasActuatorToJointCoupling) {
            mJointVelocity.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
                                     mEncoderVelocitySoftware);
        } else {
            mJointVelocity.Assign(mEncoderVelocitySoftware);
        }




//     {
//         const vctIntVec::const_iterator end = mEncoderVelocityLowResBits.end(); // last 2 are positions
//         vctIntVec::const_iterator currentEncoder, previousEncoder;
//         vctDoubleVec::const_iterator currentTimestamp, bitsToPos;
//         vctDoubleVec::iterator lastChangeTimestamp, slope, velocity;
//         for (currentEncoder = mEncoderVelocityLowResBits.begin(),
//                  previousEncoder = mEncoderPositionBitLowResPrev.begin(),
//                  currentTimestamp = mActuatorTimestamp.begin(),
//                  bitsToPos = mBitsToPositionScales.begin(),
//                  lastChangeTimestamp = mActuatorTimestampChangeLowRes.begin(),
//                  slope = mVelocitySlopeToZero.begin(),
//                  velocity = mEncoderVelocitySoftwareLowRes.begin();
//              // end
//              currentEncoder != end;
//              // increment
//              ++currentEncoder,
//                  ++previousEncoder,
//                  ++currentTimestamp,
//                  ++bitsToPos,
//                  ++lastChangeTimestamp,
//                  ++slope,
//                  ++velocity) {
//             // first see if there has been any change
//             const int difference = (*currentEncoder) - (*previousEncoder);
//             if (difference == 0) {
//                 if (*lastChangeTimestamp < timeToZeroVelocity) {
//                     *velocity -= (*slope) * (*currentTimestamp);
//                 } else {
//                     *velocity = 0.0;
//                 }
//                 *lastChangeTimestamp += (*currentTimestamp);
//             } else {
//                 *lastChangeTimestamp += (*currentTimestamp);
//                 // if we only have one bit change compute velocity since last change
//                 if ((difference == 1) || (difference == -1)) {
//                     *velocity = (difference / (*lastChangeTimestamp))
//                         * (*bitsToPos);
//                 } else if (difference > 1) {
//                     // we know all but 1 bit difference happened in last Dt, other bit change happened between now and last change
//                     *velocity = ((difference - 1.0) / (*currentTimestamp) + 1.0 / (*lastChangeTimestamp))
//                         * (*bitsToPos);
//                 } else {
//                     *velocity = ((difference + 1.0) / (*currentTimestamp) - 1.0 / (*lastChangeTimestamp))
//                         * (*bitsToPos);
//                 }
//                 // keep record of this change
//                 *lastChangeTimestamp = 0.0;
//                 *slope = (*velocity) / (timeToZeroVelocity);
//             }
//         }

//         // finally save previous encoder bits position
//         mEncoderPositionBitLowResPrev.Assign(mEncoderVelocityLowResBits);
// #if 0




//         // this has been commented because in testing branch we assume coupling is identity




//         // mEncoderVelocity.Assign(mEncoderVelocitySoftwareLowRes);

//         // This needs to be reviewed and we need to provide a better mechanism to choose one way or
//         // another to compute the joint velocities (used by PID and displayed in Qt widget).

//         // mJointVelocity.ProductOf(mConfiguration.ActuatorToJointPosition, mEncoderVelocity);
//         if (mConfiguration.HasActuatorToJointCoupling) {
//             mJointVelocity.ProductOf(mConfiguration.Coupling.ActuatorToJointPosition(),
//                                      mEncoderVelocitySoftware);
//         } else {
//             mJointVelocity.Assign(mEncoderVelocitySoftware);
//         }
// #endif
//    }





    // Effort computation
    ActuatorBitsToCurrent(mActuatorCurrentBitsFeedback, mActuatorCurrentFeedback);
    ActuatorCurrentToEffort(mActuatorCurrentFeedback, mActuatorEffortFeedback);
    if (mConfiguration.HasActuatorToJointCoupling) {
        mJointTorque.ProductOf(mConfiguration.Coupling.ActuatorToJointEffort(),
                               mActuatorEffortFeedback);
    } else {
        mJointTorque.Assign(mActuatorEffortFeedback);
    }

    BrakeBitsToCurrent(mBrakeCurrentBitsFeedback, mBrakeCurrentFeedback);

    PotBitsToVoltage(mPotBits, mPotVoltage);
    PotVoltageToPosition(mPotVoltage, mPotPosition);
}

void osaRobot1394::CheckState(void)
{
    // Save EncoderPositionPrev
    mEncoderPositionPrev.Assign(mEncoderPosition);

    // Perform safety checks
    bool currentSafetyViolation = false;
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        if (fabs(mActuatorCurrentFeedback[i]) >= mActuatorCurrentFeedbackLimits[i]) {
            CMN_LOG_RUN_WARNING << "CheckState: " << this->mName << ", actuator " << i
                                << " power: " << mActuatorCurrentFeedback[i]
                                << " > limit: " << mActuatorCurrentFeedbackLimits[i] << std::endl;
            currentSafetyViolation = true;
        }
    }

    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        if (fabs(mBrakeCurrentFeedback[i]) >= mBrakeCurrentFeedbackLimits[i]) {
            CMN_LOG_RUN_WARNING << "CheckState: " << this->mName << ", brake " << i
                                << " power: " << mBrakeCurrentFeedback[i]
                                << " > limit: " << mBrakeCurrentFeedbackLimits[i] << std::endl;
            currentSafetyViolation = true;
        }
    }

    if (currentSafetyViolation) {
        mCurrentSafetyViolationsCounter++;
    } else {
        mCurrentSafetyViolationsCounter = 0;
    }

    if (mCurrentSafetyViolationsCounter > mCurrentSafetyViolationsMaximum) {
        this->DisablePower();
        cmnThrow(osaRuntimeError1394(this->Name() + ": too many consecutive current safety violations.  Power has been disabled."));
    }

    // check safety amp disable
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO_UInt32 safetyAmpDisable = board->second->GetSafetyAmpDisable();
        if (safetyAmpDisable) {
            cmnThrow(osaRuntimeError1394(this->Name() + ": hardware current safety amp disable tripped." + mActuatorTimestamp.ToString()));
        }
    }

    // Check if brakes are releasing/released
    if (mBrakeReleasing) {
        bool allReleased = true;
        // check how much time per brake, set all to high (release current)
        mBrakeCurrentCommand.Assign(mBrakeReleaseCurrent);
        mBrakeReleasingTimer.Add(mBrakeTimestamp);
        for (size_t index = 0; index < mNumberOfBrakes; ++index) {
            if (mBrakeReleasingTimer[index] > mBrakeReleaseTime[index]) {
                // lower to releaseD current
                mBrakeCurrentCommand[index] = mBrakeReleasedCurrent[index];
            } else {
                allReleased = false;
            }
        }
        SetBrakeCurrent(mBrakeCurrentCommand);
        mBrakeReleasing = !allReleased;
    }

    // Check if encoders and potentiometers agree
    if (mUsePotsForSafetyCheck) {
        bool errorFound = false;
        switch (mPotType) {
        case POTENTIOMETER_UNDEFINED:
            break;
        case POTENTIOMETER_ON_ACTUATORS:
            mPotsToEncodersError.DifferenceOf(mEncoderPosition, mPotPosition).AbsSelf();
            if (!mPotsToEncodersError.LesserOrEqual(mPotsToEncodersTolerance)) {
                errorFound = true;
            }
            break;
        case POTENTIOMETER_ON_JOINTS:
            mPotsToEncodersError.DifferenceOf(mJointPosition, mPotPosition).AbsSelf();
            if (!mPotsToEncodersError.LesserOrEqual(mPotsToEncodersTolerance)) {
                errorFound = true;
            }
            break;
        default:
            break;
        }

        if (errorFound) {
            mPotsToEncodersViolationsCounter++;
        } else {
            mPotsToEncodersViolationsCounter = 0;
        }

        if (mPotsToEncodersViolationsCounter > mPotsToEncodersViolationsMaximum) {
            this->DisablePower();
            vctBoolVec newErrors(mPotsToEncodersError.ElementwiseLesserOrEqual(mPotsToEncodersTolerance));
            if (newErrors.NotEqual(mPotsToEncodersErrorFlag)) {
                mPotsToEncodersErrorFlag.Assign(newErrors);
                std::string errorMessage = "IO: " + this->Name() + ": inconsistency between encoders and potentiometers, \n errors: ";
                errorMessage.append(mPotsToEncodersError.ToString());
                errorMessage.append(", \n mask (0 is error): ");
                errorMessage.append(mPotsToEncodersErrorFlag.ToString());
                cmnThrow(osaRuntimeError1394(errorMessage));
            }
        }
    }

    // Check for encoder overflow
    if (mEncoderOverflow.Any()) {
        this->DisablePower();
        if (mEncoderOverflow.NotEqual(mPreviousEncoderOverflow)) {
            mPreviousEncoderOverflow.Assign(mEncoderOverflow);
            std::string errorMessage = this->Name() + ": detected encoder overflow: ";
            errorMessage.append(mEncoderOverflow.ToString());
            cmnThrow(osaRuntimeError1394(errorMessage));
        }
    }
}

void osaRobot1394::SetCoupling(const prmActuatorJointCoupling & coupling)
{
    // check sizes
    if ((coupling.ActuatorToJointPosition().rows() != mNumberOfJoints) ||
        (coupling.ActuatorToJointPosition().cols() != mNumberOfActuators)) {
        cmnThrow("SetCoupling: invalid size for ActuatorToJointPosition");
    }

    if ((coupling.JointToActuatorPosition().rows() != mNumberOfActuators) ||
        (coupling.JointToActuatorPosition().cols() != mNumberOfJoints)) {
        cmnThrow("SetCoupling: invalid size for JointToActuatorPosition");
    }

    if ((coupling.ActuatorToJointEffort().rows() != mNumberOfJoints) ||
        (coupling.ActuatorToJointEffort().cols() != mNumberOfActuators)) {
        cmnThrow("SetCoupling: invalid size for ActuatorToJointEffort");
    }

    if ((coupling.JointToActuatorEffort().rows() != mNumberOfActuators) ||
        (coupling.JointToActuatorEffort().cols() != mNumberOfJoints)) {
        cmnThrow("SetCoupling: invalid size for JointToActuatorEffort");
    }

    // check for identity using inverse
    const vctDoubleMat identity = vctDoubleMat::Eye(mNumberOfActuators);
    vctDoubleMat product;
    product.SetSize(mNumberOfActuators, mNumberOfActuators);
    product.ProductOf(coupling.ActuatorToJointPosition(),
                      coupling.JointToActuatorPosition());
    if (!product.AlmostEqual(identity, 0.001)) {
        cmnThrow("SetCoupling: product of position coupling matrices not identity");
    }
    product.ProductOf(coupling.ActuatorToJointEffort(),
                      coupling.JointToActuatorEffort());
    if (!product.AlmostEqual(identity, 0.001)) {
        cmnThrow("ConfigureCoupling: product of torque coupling matrices not identity");
    }

    // finally assign values
    mConfiguration.Coupling.ActuatorToJointPosition().ForceAssign(coupling.ActuatorToJointPosition());
    mConfiguration.Coupling.JointToActuatorPosition().ForceAssign(coupling.JointToActuatorPosition());
    mConfiguration.Coupling.ActuatorToJointEffort().ForceAssign(coupling.ActuatorToJointEffort());
    mConfiguration.Coupling.JointToActuatorEffort().ForceAssign(coupling.JointToActuatorEffort());
}

void osaRobot1394::EnablePower(void)
{
    this->EnableBoardsPower();
    osaSleep(50.0 * cmn_ms);
    this->SetActuatorPower(true);
    this->SetBrakePower(true);
}

void osaRobot1394::EnableBoardsPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteSafetyRelay(true);
        board->second->WritePowerEnable(true);
    }
}

void osaRobot1394::DisablePower(void)
{
    // write to boards directly
    // disable all axes
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteAmpEnable(0x0f, 0x00);
    }

    // disable all boards
    this->DisableBoardPower();

    mPreviousPowerStatus = false;
}

void osaRobot1394::DisableBoardPower(void)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WritePowerEnable(false);
        board->second->WriteSafetyRelay(false);
    }
}

void osaRobot1394::WriteSafetyRelay(const bool & enabled)
{
    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        AmpIO * boardPointer = board->second;
        boardPointer->WriteSafetyRelay(enabled);
    }
}

void osaRobot1394::SetWatchdogPeriod(const double & periodInSeconds)
{
    uint32_t periodCounts;
    if (periodInSeconds == 0.0) {
        // Disable watchdog
        periodCounts = 0;
    } else {
        // Use at least one tick just to make sure we don't accidentaly disable
        // the truth is that the count will be so low that watchdog will
        // continuously trigger.
        periodCounts = (periodInSeconds * 1000.0) * WATCHDOG_MS_TO_COUNT;
        periodCounts = std::max(periodCounts, static_cast<uint32_t>(1));
    }

    for (unique_board_iterator board = mUniqueBoards.begin();
         board != mUniqueBoards.end();
         ++board) {
        board->second->WriteWatchdogPeriod(periodCounts);
    }
}

void osaRobot1394::SetActuatorPower(const bool & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetAmpEnable(mActuatorInfo[i].Axis, enabled);
    }
}

void osaRobot1394::SetActuatorPower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetAmpEnable(mActuatorInfo[i].Axis, enabled[i]);
    }
}

void osaRobot1394::SetBrakePower(const bool & enabled)
{
    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetAmpEnable(mBrakeInfo[i].Axis, enabled);
    }
}

void osaRobot1394::SetBrakePower(const vctBoolVec & enabled)
{
    for (size_t i = 0; i < mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetAmpEnable(mBrakeInfo[i].Axis, enabled[i]);
    }
}

void osaRobot1394::SetEncoderPosition(const vctDoubleVec & pos)
{
    vctIntVec bits(mNumberOfActuators);
    this->EncoderPositionToBits(pos, bits);
    this->SetEncoderPositionBits(bits);
}

void osaRobot1394::SetEncoderPositionBits(const vctIntVec & bits)
{
    for (size_t i = 0; i < mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->WriteEncoderPreload(mActuatorInfo[i].Axis, bits[i]);
    }
    // initialize previous bits value
    mEncoderPositionBitsPrev.Assign(bits);
    mActuatorTimestampChange.SetAll(0.0);
    mVelocitySlopeToZero.SetAll(0.0);
}

void osaRobot1394::SetSingleEncoderPosition(const int index, const double pos)
{
    SetSingleEncoderPositionBits(index, pos / mBitsToPositionScales[index]);
}

void osaRobot1394::SetSingleEncoderPositionBits(const int index, const int bits)
{
    mActuatorInfo[index].Board->WriteEncoderPreload(mActuatorInfo[index].Axis, bits);

    // initialize previous bits value
    mEncoderPositionBitsPrev.Element(index) = bits;
    mActuatorTimestampChange.Element(index) = 0.0;
    mVelocitySlopeToZero.Element(index) = 0.0;
}

void osaRobot1394::ClipActuatorEffort(vctDoubleVec & efforts)
{
    efforts.ElementwiseClipIn(mActuatorEffortCommandLimits);
}

void osaRobot1394::ClipActuatorCurrent(vctDoubleVec & currents)
{
    currents.ElementwiseClipIn(mActuatorCurrentCommandLimits);
}

void osaRobot1394::ClipBrakeCurrent(vctDoubleVec & currents)
{
    currents.ElementwiseClipIn(mBrakeCurrentCommandLimits);
}

void osaRobot1394::SetJointEffort(const vctDoubleVec & efforts)
{
    vctDoubleVec actuatorEfforts(mNumberOfActuators);
    if (mConfiguration.HasActuatorToJointCoupling) {
        actuatorEfforts.ProductOf(mConfiguration.Coupling.JointToActuatorEffort(), efforts);
    } else {
        actuatorEfforts.Assign(efforts);
    }
    this->SetActuatorEffort(actuatorEfforts);
}

void osaRobot1394::SetActuatorEffort(const vctDoubleVec & efforts)
{
    // Convert efforts to bits and set the command
    vctDoubleVec clipped_efforts = efforts;
    vctDoubleVec currents(mNumberOfActuators);

    // this->clip_actuator_efforts(clipped_efforts);

    this->ActuatorEffortToCurrent(clipped_efforts, currents);
    this->SetActuatorCurrent(currents);
}

void osaRobot1394::SetActuatorCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(mNumberOfActuators);

    this->ClipActuatorCurrent(clipped_amps);
    this->ActuatorCurrentToBits(clipped_amps, bits);
    this->SetActuatorCurrentBits(bits);

    // Store commanded amps
    mActuatorCurrentCommand = clipped_amps;
}

void osaRobot1394::SetActuatorCurrentBits(const vctIntVec & bits)
{
    for (size_t i=0; i<mNumberOfActuators; i++) {
        mActuatorInfo[i].Board->SetMotorCurrent(mActuatorInfo[i].Axis, bits[i]);
    }

    // Store commanded bits
    mActuatorCurrentBitsCommand = bits;
}

void osaRobot1394::UsePotsForSafetyCheck(const bool & usePotsForSafetyCheck)
{
    mUsePotsForSafetyCheck = usePotsForSafetyCheck;
}

void osaRobot1394::SetPotsToEncodersTolerance(const vctDoubleVec & tolerances)
{
    mPotsToEncodersTolerance = tolerances;
}

void osaRobot1394::SetBrakeCurrent(const vctDoubleVec & currents)
{
    // Convert amps to bits and set the command
    vctDoubleVec clipped_amps = currents;
    vctIntVec bits(mNumberOfBrakes);

    this->ClipBrakeCurrent(clipped_amps);
    this->BrakeCurrentToBits(clipped_amps, bits);
    this->SetBrakeCurrentBits(bits);

    // Store commanded amps
    mBrakeCurrentCommand = clipped_amps;
}

void osaRobot1394::SetBrakeCurrentBits(const vctIntVec & bits)
{
    for (size_t i=0; i<mNumberOfBrakes; i++) {
        mBrakeInfo[i].Board->SetMotorCurrent(mBrakeInfo[i].Axis, bits[i]);
    }

    // Store commanded bits
    mBrakeCurrentBitsCommand = bits;
}

void osaRobot1394::BrakeRelease(void)
{
    if (mNumberOfBrakes != 0) {
        mBrakeReleasing = true;
        mBrakeReleasingTimer.SetAll(0.0);
        SetBrakeCurrent(mBrakeReleaseCurrent);
    }
}

void osaRobot1394::BrakeEngage(void)
{
    if (mNumberOfBrakes != 0) {
        mBrakeReleasing = false;
        SetBrakeCurrent(mBrakeEngagedCurrent);
    }
}

void osaRobot1394::CalibrateEncoderOffsetsFromPots(void)
{
    vctDoubleVec actuatorPosition(mNumberOfActuators);

    switch(mPotType) {

    case POTENTIOMETER_UNDEFINED:
        cmnThrow("osaRobot1394::CalibrateEncoderOffsetsFromPots: can't set encoder offset, potentiometer's position undefined");
        break;

    case POTENTIOMETER_ON_JOINTS:
        if (mConfiguration.HasActuatorToJointCoupling) {
            actuatorPosition.ProductOf(mConfiguration.Coupling.JointToActuatorPosition(),
                                       mPotPosition);
        } else {
            actuatorPosition.Assign(mPotPosition);
        }
        SetEncoderPosition(actuatorPosition);
        break;

    case POTENTIOMETER_ON_ACTUATORS:
        SetEncoderPosition(mPotPosition);
        break;
    };
}

bool osaRobot1394::Valid(void) const {
    return mValid;
}

bool osaRobot1394::PowerStatus(void) const {
    return mPowerStatus;
}

bool osaRobot1394::SafetyRelay(void) const {
    return mSafetyRelay;
}

bool osaRobot1394::WatchdogStatus(void) const {
    return mWatchdogStatus;
}

const vctBoolVec & osaRobot1394::ActuatorPowerStatus(void) const {
    return mActuatorPowerStatus;
}

const vctBoolVec & osaRobot1394::BrakePowerStatus(void) const {
    return mBrakePowerStatus;
}

const vctDoubleVec & osaRobot1394::ActuatorCurrentFeedback(void) const {
    return mActuatorCurrentFeedback;
}

const vctDoubleVec & osaRobot1394::BrakeCurrentFeedback(void) const {
    return mBrakeCurrentFeedback;
}

const vctDoubleVec & osaRobot1394::PotPosition(void) const {
    return mPotPosition;
}

const vctDoubleVec & osaRobot1394::ActuatorTimeStamp(void) const {
    return mActuatorTimestamp;
}

const vctDoubleVec & osaRobot1394::BrakeTimeStamp(void) const {
    return mBrakeTimestamp;
}

const vctDoubleVec & osaRobot1394::EncoderPosition(void) const {
    return mEncoderPosition;
}

const vctDoubleVec & osaRobot1394::EncoderVelocity(void) const {
    return mEncoderVelocity;
}

const vctDoubleVec & osaRobot1394::EncoderVelocitySoftware(void) const {
    return mEncoderVelocitySoftware;
}

osaRobot1394Configuration osaRobot1394::GetConfiguration(void) const {
    return mConfiguration;
}

std::string osaRobot1394::Name(void) const {
    return mName;
}

size_t osaRobot1394::NumberOfJoints(void) const {
    return mNumberOfJoints;
}

size_t osaRobot1394::NumberOfActuators(void) const {
    return mNumberOfActuators;
}

size_t osaRobot1394::SerialNumber(void) const {
    return mSerialNumber;
}

size_t osaRobot1394::NumberOfBrakes(void) const {
    return mNumberOfBrakes;
}

void osaRobot1394::GetJointTypes(prmJointTypeVec & joint_types) const
{
    joint_types.SetSize(mNumberOfJoints);
    for (size_t i = 0; i < mNumberOfJoints; i++) {
        joint_types[i] = mJointType[i];
    }
}
void osaRobot1394::GetJointEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mJointEffortCommandLimits;
}

void osaRobot1394::GetActuatorEffortCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorEffortCommandLimits;
}

void osaRobot1394::GetActuatorCurrentCommandLimits(vctDoubleVec & limits) const
{
    limits = mActuatorCurrentCommandLimits;
}

void osaRobot1394::EncoderPositionToBits(const vctDoubleVec & pos, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        bits[i] = static_cast<long>(pos[i] / mBitsToPositionScales[i]);
    }
}

void osaRobot1394::EncoderBitsToPosition(const vctIntVec & bits, vctDoubleVec & pos) const {
    for (size_t i = 0; i < bits.size() && i < pos.size(); i++) {
        pos[i] = static_cast<double>(bits[i]) * mBitsToPositionScales[i];
    }
}

void osaRobot1394::EncoderBitsToDPosition(const vctIntVec & bits, vctDoubleVec & dpos) const {
    for (size_t i = 0; i < bits.size() && i < dpos.size(); i++) {
        dpos[i] = static_cast<double>(bits[i]) * mBitsToDPositionScales[i] + mBitsToDPositionOffsets[i];
    }
}

void osaRobot1394::EncoderBitsToDTime(const vctIntVec & bits, vctDoubleVec & dt) const {
    for (size_t i = 0; i < bits.size() && i < dt.size(); i++) {
        dt[i] = static_cast<double>(bits[i]) * mBitsToDTimeScales[i] + mBitsToDTimeOffsets[i];
    }
}

void osaRobot1394::EncoderBitsToVelocity(const vctIntVec & bits, vctDoubleVec & vel) const
{
    double period = 1.0/3072000.0; //Clock period defined in firmware - different than system clock
    for (size_t i = 0; i < bits.size() && i < vel.size(); i++)
    {
        int counter = bits[i];

        if (mIsAllBoardsFirmWareFour) {
            if (counter == 4194303 || counter == -4194303) { //overflow value +/- 0x3fffff, sign set separately

                vel[i] = 0.0;
            }
            else {
                vel[i] = mBitsToPositionScales[i] / ((double) counter*period) * 4;
            }
        } else {
            vel[i] = mBitsToPositionScales[i] / ((double) bits[i]);
        }
    }
}

void osaRobot1394::ActuatorEffortToCurrent(const vctDoubleVec & efforts, vctDoubleVec & currents) const {
    currents.ElementwiseProductOf(efforts, mEffortToCurrentScales);
}

void osaRobot1394::ActuatorCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        bits[i] = static_cast<long>(currents[i] * mActuatorCurrentToBitsScales[i] + mActuatorCurrentToBitsOffsets[i]);
    }
}

void osaRobot1394::ActuatorBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        currents[i] = static_cast<double>(bits[i]) * mActuatorBitsToCurrentScales[i] + mActuatorBitsToCurrentOffsets[i];
    }
}

void osaRobot1394::ActuatorCurrentToEffort(const vctDoubleVec & currents, vctDoubleVec & efforts) const {
    efforts.ElementwiseRatioOf(currents, mEffortToCurrentScales);
}

void osaRobot1394::BrakeCurrentToBits(const vctDoubleVec & currents, vctIntVec & bits) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        bits[i] = static_cast<long>(currents[i] * mBrakeCurrentToBitsScales[i] + mBrakeCurrentToBitsOffsets[i]);
    }
}

void osaRobot1394::BrakeBitsToCurrent(const vctIntVec & bits, vctDoubleVec & currents) const {
    for (size_t i = 0; i < bits.size() && i < currents.size(); i++) {
        currents[i] = static_cast<double>(bits[i]) * mBrakeBitsToCurrentScales[i] + mBrakeBitsToCurrentOffsets[i];
    }
}

void osaRobot1394::PotBitsToVoltage(const vctIntVec & bits, vctDoubleVec & voltages) const {
    for (size_t i = 0; i < bits.size() && i < voltages.size(); i++) {
        voltages[i] = static_cast<double>(bits[i]) * mBitsToVoltageScales[i] + mBitsToVoltageOffsets[i];
    }
}

void osaRobot1394::PotVoltageToPosition(const vctDoubleVec & voltages, vctDoubleVec & pos) const {
    pos.ElementwiseProductOf(voltages, mVoltageToPositionScales);
    pos.SumOf(pos, mVoltageToPositionOffsets);
}
