/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>

#include "robotDriver.h"
#include "action.h"

#ifndef BROADCASTING_THREAD
#define BROADCASTING_THREAD

// ******************** THE THREAD
class BroadcastingThread: public yarp::os::PeriodicThread
{
    size_t njoints=0;
    double* encs=nullptr;
    double* outs=nullptr;
    double* errs=nullptr;
    double* opts=nullptr;

private:
    action_class          *actions=nullptr;
    robotDriver           *driver=nullptr;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_data_out;

public:
    BroadcastingThread(double period = 0.001);
    ~BroadcastingThread();
    void attachRobotDriver(robotDriver *p);
    void attachActions(action_class *a);
    bool threadInit() override;
    void run() override;
};

#endif
