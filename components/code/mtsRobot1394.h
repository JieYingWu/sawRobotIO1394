/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Peter Kazanzides
  Created on: 2011-06-10

  (C) Copyright 2011-2017 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsRobot1394_h
#define _mtsRobot1394_h

#include <cisstParameterTypes/prmJointType.h>
#include <cisstParameterTypes/prmPositionJointGet.h>
#include <cisstParameterTypes/prmVelocityJointGet.h>
#include <cisstParameterTypes/prmForceTorqueJointSet.h>

#include <sawRobotIO1394/osaRobot1394.h>

namespace sawRobotIO1394 {

    class mtsRobot1394: public osaRobot1394 {
    public:
        /*! Pointer on existing services.  This allows to use the class
          name and level of detail of another class, e.g. the class that
          owns this map.  To set the "Owner", use the method SetOwner
          after the cmnNamedMap is constructed. */
        const cmnClassServicesBase * OwnerServices;

        /*! Method used to emulate the cmnGenericObject interface used by
          CMN_LOG_CLASS macros. */
        //@{
        inline const cmnClassServicesBase * Services(void) const {
            return this->OwnerServices;
        }

        inline cmnLogger::StreamBufType * GetLogMultiplexer(void) const {
            return cmnLogger::GetMultiplexer();
        }
        //@}

        mtsRobot1394(const cmnGenericObject & owner,
                     const osaRobot1394Configuration & config);
        ~mtsRobot1394();

        bool SetupStateTables(const size_t stateTableSize,
                              mtsStateTable * & stateTableRead,
                              mtsStateTable * & stateTableWrite);
        void SetupInterfaces(mtsInterfaceProvided * robotInterface,
                             mtsInterfaceProvided * actuatorInterface);

        void StartReadStateTable(void);
        void AdvanceReadStateTable(void);
        void StartWriteStateTable(void);
        void AdvanceWriteStateTable(void);
        bool CheckConfiguration(void);
        void CheckState(void);

        // Wrapper of osa methods to match command signatures
        void GetNumberOfActuators(int & num_actuators) const;
        void GetNumberOfJoints(int & num_joints) const;
        void GetSerialNumber(int & serialNumber) const;
        void SetTorqueJoint(const prmForceTorqueJointSet & jointTorques);
        void ResetSingleEncoder(const int & index);
        void SetCoupling(const prmActuatorJointCoupling & coupling);

        /*! \name Bias Calibration */
        void CalibrateEncoderOffsetsFromPots(const int & numberOfSamples);

    protected:
        mtsStateTable * mStateTableRead;
        mtsStateTable * mStateTableWrite;
        bool mFirstWatchdog;

        prmForceTorqueJointSet mTorqueJoint;
        prmPositionJointGet mPositionJointGet;
        prmPositionJointGet mPositionActuatorGet;
        prmVelocityJointGet mVelocityJointGet;

        // Functions for events
        struct {
            mtsFunctionWrite PowerStatus;
            mtsFunctionWrite WatchdogStatus;
            mtsFunctionWrite Coupling;
            mtsFunctionWrite BiasEncoder;
        } EventTriggers;

        int mSamplesForCalibrateEncoderOffsetsFromPots;
        int mSamplesForCalibrateEncoderOffsetsFromPotsRequested;
        mtsStateTable::Accessor<vctDoubleVec> * mPotPositionAccessor;
        mtsStateTable::Accessor<prmPositionJointGet> * mPositionActuatorGetAccessor;

    public:
        mtsInterfaceProvided * mInterface;
    };

} // namespace sawRobotIO1394

#endif // _mtsRobot1394_h
