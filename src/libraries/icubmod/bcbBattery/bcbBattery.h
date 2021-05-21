// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#ifndef __BCBBATTERY_H__
#define __BCBBATTERY_H__

#include <mutex>
#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/IBattery.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/SerialInterfaces.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/sig/Vector.h>

using namespace yarp::os;
using namespace yarp::dev;

class batteryReaderThread : public PeriodicThread
{
    public:
    //configuration options
    bool               verboseEnable = false;
    bool               screenEnable = true;
    bool               silenceSyncWarnings = false;

    //the buffer
    bool               closeRequested = false;
    double             timeStamp;
    char               serial_buff[10];
    int                output[10];

    ISerialDevice*     pSerial = nullptr;
    double             battery_charge = 0;
    double             battery_voltage = 0;
    double             battery_current = 0;
    std::string        battery_info = "icub battery system v1.0";
    unsigned char      backpack_status = 0;

    batteryReaderThread (ISerialDevice *_pSerial, double period) :
    PeriodicThread((double)period),
    pSerial(_pSerial)
    {}

    void startTransmission();
    void stopTransmission();
    virtual bool threadInit() override;
    virtual void threadRelease() override;
    virtual void run() override;
};

class BcbBattery: public yarp::dev::IBattery, public DeviceDriver
{
protected:
    batteryReaderThread* batteryReader =nullptr;

    ResourceFinder      rf;
    PolyDriver          driver;
    ISerialDevice       *pSerial = nullptr;

public:
    BcbBattery()  {}
    ~BcbBattery()  {}

    virtual bool open(yarp::os::Searchable& config);
    virtual bool close();

    virtual bool getBatteryVoltage     (double &voltage) override;
    virtual bool getBatteryCurrent     (double &current) override;
    virtual bool getBatteryCharge      (double &charge) override;
    virtual bool getBatteryStatus      (Battery_status &status) override;
    virtual bool getBatteryInfo        (std::string &info) override;
    virtual bool getBatteryTemperature (double &temperature) override;
};


#endif
