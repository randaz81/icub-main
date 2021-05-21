// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// Copyright: (C) 2015 iCub Facility
// Authors: Marco Randazzo <marco.randazzo@iit.it>
// CopyPolicy: Released under the terms of the GNU GPL v2.0.

#include <bcbBattery.h>

#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <string.h>
#include <time.h>
#include <stdlib.h>
#include <cmath>

using namespace std;
//#define DEBUG_TEST 1

bool BcbBattery::open(yarp::os::Searchable& config)
{
    //debug
    yDebug("%s\n", config.toString().c_str());

    Bottle& group_general = config.findGroup("GENERAL");
    Bottle& group_serial  = config.findGroup("SERIAL_PORT");

    if (group_general.isNull())
    {
        yError() << "Insufficient parameters to BcbBattery, section GENERAL missing";
        return false;
    }

    if (group_serial.isNull())
    {
        yError() << "Insufficient parameters to BcbBattery, section SERIAL_PORT missing";
        return false;
    }

    int period=group_general.find("thread_period").asInt();

    Property prop;
    std::string ps = group_serial.toString();
    prop.fromString(ps);
    prop.put("device", "serialport");

    //open the driver
    driver.open(prop);
    if (!driver.isValid())
    {
        yError() << "Error opening PolyDriver check parameters";
#ifndef DEBUG_TEST 
        return false;
#endif
    }

    //open the serial interface
    driver.view(pSerial);
    if (!pSerial)
    {
        yError("Error opening serial driver. Device not available");
#ifndef DEBUG_TEST 
        return false;
#endif
    }

    //create the thread
    batteryReader = new batteryReaderThread(pSerial, (double)period / 1000.0);
    // Other options
    batteryReader->verboseEnable = group_general.check("verbose", Value(0), "enable/disable the verbose mode").asBool();
    batteryReader->screenEnable = group_general.check("screen", Value(0), "enable/disable the screen output").asBool();
    batteryReader->silenceSyncWarnings = group_general.check("silence_sync_warnings", Value(0), "enable/disable the print of warnings in case of sync errors.").asBool();

    //start the thread
    batteryReader->start();
    return true;
}

bool BcbBattery::close()
{
    //stop the thread
    batteryReader->stop();

    //stop the driver
    driver.close();

    return true;
}

bool batteryReaderThread::threadInit()
{
    timeStamp = yarp::os::Time::now();

    if (pSerial)
    {
        yInfo("BcbBattery starting transmission");
        startTransmission();
        yInfo("BcbBattery started successfully");
    }
    else
    {
        yError("BcbBattery pSerial == NULL");
        return false;
    }

    return true;
}

void batteryReaderThread::run()
{
    double timeNow=yarp::os::Time::now();

    //read battery data.
    if (pSerial)
    {
    search_h:
        output[8] = pSerial->receiveChar(serial_buff[8]);
        if (output[8] == 0 || serial_buff[8] != '\r')
        {
            if (!silenceSyncWarnings) yWarning("BcbBattery sync error r");
            if (closeRequested) return;
            goto search_h;
        }
        output[9] = pSerial->receiveChar(serial_buff[9]);
        if (output[9]==0 || serial_buff[9] != '\n')
        {
            if (!silenceSyncWarnings) yWarning("BcbBattery sync error n");
            if (closeRequested) return;
            goto search_h;
        }
        output[0] = pSerial->receiveChar(serial_buff[0]);
        if (output[0] == 0 || serial_buff[0] != '\0')
        {
            if (closeRequested) return;
            goto search_h;
        }
        output[1] = pSerial->receiveChar(serial_buff[1]); //voltage
        output[2] = pSerial->receiveChar(serial_buff[2]); //voltage
        output[3] = pSerial->receiveChar(serial_buff[3]); //current
        output[4] = pSerial->receiveChar(serial_buff[4]); //current
        output[5] = pSerial->receiveChar(serial_buff[5]); //charge
        output[6] = pSerial->receiveChar(serial_buff[6]); //charge
        output[7] = pSerial->receiveChar(serial_buff[7]); //status
    }
    else
    {
        yError("BcbBattery pSerial == NULL");
    }

    if (verboseEnable) 
    {
        char hexBuffer[31];
        char decBuffer[41];
        for (size_t i = 0; i < 10; ++i)
        {
            if (output[i])
            {
                sprintf(hexBuffer + 3*i,"%02X ", (unsigned int)(serial_buff[i] & 0xFF));
                sprintf(decBuffer + 4*i,"%03u ", (unsigned int)(serial_buff[i] & 0xFF));
            }
            else
            {
                sprintf(hexBuffer + 3*i,"-- ");
                sprintf(decBuffer + 4*i,"--- ");
            }
        }
        yDebug("BcbBattery::run() serial_buffer is: (hex) %s, (dec) %s", hexBuffer, decBuffer);
    }

    //add REAL checksum verification.
    //This is just a simplified check
    for (size_t i = 0; i < 10; ++i)
    {
        if (!output[i])
        {
            yError ("Checksum failed");
            return;
        }
    }

#if DEBUG_TEST
    battery_voltage = 40.0;
    battery_current = 5.0;
    battery_charge = 72.0;
    battery_temperature = 35.0;
    backpack_status = 0;
#else
    battery_voltage = ((unsigned char) serial_buff[1] * 256 + (unsigned char) serial_buff[2])/1000.0;
    battery_current = ((unsigned char) serial_buff[3] * 256 + (unsigned char) serial_buff[4])/1000.0;
    battery_charge =  ((unsigned char) serial_buff[5] * 256 + (unsigned char) serial_buff[6]);
    backpack_status = (unsigned char) serial_buff[7];
#endif

    // print data to screen
    if (screenEnable)
    {
        time_t rawtime;
        struct tm * timeinfo;
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        char* battery_timestamp = asctime(timeinfo);
        char buff[1024];
        sprintf(buff, "battery status: %+6.1fA   % 6.1fV   charge:% 6.1f%%    time: %s", battery_current, battery_voltage, battery_charge, battery_timestamp);
        yDebug("BcbBattery::run() log_buffer is: %s", buff);
    }

    //empty the buffer
    serial_buff[0] = 0;
    pSerial->flush();
}

bool BcbBattery::getBatteryVoltage(double &voltage)
{
    voltage = batteryReader->battery_voltage;
    return true;
}

bool BcbBattery::getBatteryCurrent(double &current)
{
    current = batteryReader->battery_current;
    return true;
}

bool BcbBattery::getBatteryCharge(double &charge)
{
    charge = batteryReader->battery_charge;
    return true;
}

bool BcbBattery::getBatteryStatus(Battery_status &status)
{
    //The BCB battery indicator does not provide this info, so we simply return BATTERY_OK_IN_USE
    status=Battery_status::BATTERY_OK_IN_USE;
    return true;
}

bool BcbBattery::getBatteryTemperature(double &temperature)
{
    //yError("Not yet implemented");
    temperature = std::nan("");
    return false;
}

bool BcbBattery::getBatteryInfo(string &info)
{
    info = batteryReader->battery_info;
    return true;
}

void batteryReaderThread::threadRelease()
{
    stopTransmission();
}

void batteryReaderThread::startTransmission()
{
    //start the transmission
    char cmd = 0x01;
    bool ret = pSerial->send(&cmd, 1);
    if (ret == false)
    {
        yError("BcbBattery problems starting the transmission");
        return;
    }

    //empty the buffer
    pSerial->flush();
}

void batteryReaderThread::stopTransmission()
{
    if (pSerial)
    {
        char c = 0x00;
        bool ret = pSerial->send(&c, 1);
        if (ret == false) { yError("BcbBattery problems while stopping the transmission"); }
    }
}
