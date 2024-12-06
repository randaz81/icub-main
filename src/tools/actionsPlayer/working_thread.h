/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <string>
#include <cmath>
#include <mutex>

#include "robotDriver.h"
#include "action.h"

#ifndef WORKING_THREAD
#define WORKING_THREAD

class WorkingThread: public yarp::os::PeriodicThread
{
    std::mutex            mtx;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_out;
    yarp::os::BufferedPort<yarp::os::Bottle>  port_command_joints;
    double                start_time = 0.0;

    action_class          *current_action = nullptr;
    robotDriver           *current_driver=nullptr;
    action_status_enum    status = ACTION_IDLE;
    double                home_position_tolerance = 2.0;
    size_t                home_position_timeout = 100;
    bool                  home_position_strict_check_enabled = false;

   public:
    bool                  enable_execute_joint_command=false;

    action_status_enum getStatus();
    bool action_stop();
    bool action_print();
    bool action_reset();
    bool action_forever();
    bool action_start();
    bool action_change(action_class *action, robotDriver *driver);
    bool action_getname(std::string& name);

    WorkingThread(double period = 0.005);
    ~WorkingThread();
    bool threadInit() override;
    void run() override;

    bool execute_joint_command(int frame_id);
    void compute_and_send_command(int frame_id);
};

#endif
