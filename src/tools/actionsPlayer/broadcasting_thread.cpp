/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/dev/IPidcontrol.h>

#include "robotDriver.h"
#include "action.h"
#include "broadcasting_thread.h"


BroadcastingThread::BroadcastingThread(double period): PeriodicThread(period)
{
    port_data_out.open("/trajectoryPlayer/all_joints_data_out:o");
}

BroadcastingThread::~BroadcastingThread()
{
    port_data_out.interrupt();
    port_data_out.close();
}

void BroadcastingThread::attachRobotDriver(robotDriver *p)
{
    if (p)
    {
        driver = p;
        njoints = p->getNJoints();
    }
    if (encs) { delete[] encs; encs = nullptr; }
    if (outs) { delete[] outs; outs = nullptr; }
    if (errs) { delete[] errs; errs = nullptr; }
    if (opts) { delete[] opts; opts = nullptr; }
    encs = new double[njoints];
    outs = new double[njoints];
    errs = new double[njoints];
    opts = new double[njoints];
}

void BroadcastingThread::attachActions(action_class *a)
{
    if (a)
    {
        actions = a;
    }
}

bool BroadcastingThread::threadInit()
{
    if (!driver)
    {
        return false;
    }
    return true;
}

void BroadcastingThread::run()
{
    //reads the current position
    if (driver && driver->ienc_ll)
    {
        driver->ienc_ll->getEncoders(encs);
    }
    else
    {
        //invalid driver
    }

    //reads the pid output
    if (driver && driver->ipid_ll)
    {
        driver->ipid_ll->getPidOutputs(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION,outs);
    }
    else
    {
        //invalid driver
    }

    //reads the pid error
    if (driver && driver->ipid_ll)
    {
        driver->ipid_ll->getPidErrors(yarp::dev::PidControlTypeEnum::VOCAB_PIDTYPE_POSITION,errs);
    }
    else
    {
        //invalid driver
    }

    //reads the optical encoders
    if (driver && driver->imotenc_ll)
    {
        driver->imotenc_ll->getMotorEncoders(opts);
    }
    else
    {
        //invalid driver
    }

    size_t j = actions->current_frame;

    yarp::os::Bottle& bot2 = this->port_data_out.prepare();
    bot2.clear();
    bot2.addInt32((int)actions->action_frames_vector[j].counter);
    bot2.addFloat64(actions->action_frames_vector[j].time);

    size_t size = this->actions->action_frames_vector[j].q_joints.size();
    double *ll = actions->action_frames_vector[j].q_joints.data();

    bot2.addString("commands:");
    for (int ix=0;ix<size;ix++)
    {
        bot2.addFloat64(ll[ix]);
    }
    bot2.addString("encoders:");
    for (int ix=0;ix<size;ix++)
    {
        bot2.addFloat64(encs[ix]);
    }
    bot2.addString("outputs:");
    for (int ix=0;ix<size;ix++)
    {
        bot2.addFloat64(outs[ix]);
    }
    bot2.addString("optical:");
    for (int ix=0;ix<size;ix++)
    {
        bot2.addFloat64(opts[ix]);
    }
    bot2.addString("errors:");
    for (int ix=0;ix<size;ix++)
    {
        bot2.addFloat64(errs[ix]);
    }
    bot2.addString("timestamp:");
    bot2.addFloat64(yarp::os::Time::now());
    this->port_data_out.write();
}
