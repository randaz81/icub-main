// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup icub_hardware_modules 
 * \defgroup canbusmotioncontrol canbusmotioncontrol
 *
 * This device contains code which handles communication to 
 * the motor control boards on a CAN bus. It 
 * converts requests from function calls into CAN bus messages for
 * the motor control boards. A thread monitors the bus for incoming
 * messages and dispatches replies to calling threads.
 *
 * Comunication with the CAN bus is done through the standard
 * YARP ICanBus interface.
 *
 * Copyright (C) 2010 RobotCub Consortium.
 *
 * Author: Lorenzo Natale
 *
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

//
// $Id: CanBusMotionControl.h,v 1.16 2009/07/29 13:12:29 nat Exp $
//
//

#ifndef __CanBusMotionControlh__
#define __CanBusMotionControlh__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/ControlBoardInterfacesImpl.h>
#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/CanBusInterface.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Stamp.h>
#include <string>
#include <list>

#include <iCub/FactoryInterface.h>


namespace yarp{
    namespace dev{
        class CanBusMotionControl;
        class CanBusMotionControlParameters;
    }
}

class ThreadPool2;
class RequestsQueue;
/**
* \file CanBusMotionControl.h 
* class for interfacing with a generic can device driver.
*/

/**
* \include UserDoc_dev_motorcontrol.dox
*/

/**
* @ingroup dev_impl_motor
*
* The PlxCan motion controller device driver.
* Contains a thread that takes care of polling the can bus for incoming messages.
*/

/**
* The open parameter class containing the initialization values.
*/
class yarp::dev::CanBusMotionControlParameters
{
private:
    CanBusMotionControlParameters (const CanBusMotionControlParameters&);
    void operator= (const CanBusMotionControlParameters&);



public:
    /**
    * Constructor (please make sure you use the constructor to allocate
    * memory).
    * @param nj is the number of controlled joints/axes.
    */
    CanBusMotionControlParameters ();
    
    /**
    * Destructor, with memory deallocation.
    */
    ~CanBusMotionControlParameters ();

    bool setBroadCastMask(yarp::os::Bottle &list, int MASK);

    bool fromConfig(yarp::os::Searchable &config);
    bool alloc(int nj);

    int _txQueueSize;
    int _rxQueueSize;
    int _txTimeout;
    int _rxTimeout;
    int *_broadcast_mask;

    int _networkN;								/** network number */
    int _njoints;								/** number of joints/axes/controlled motors */
    unsigned char *_destinations;       		/** destination addresses */
    unsigned char _my_address;					/** my address */
    int _polling_interval;						/** thread polling interval [ms] */
    int _timeout;								/** number of cycles before timing out */

    int *_axisMap;                              /** axis remapping lookup-table */
    double *_angleToEncoder;                    /** angle to encoder conversion factors */
    double *_zeros;                             /** encoder zeros */
    Pid *_pids;                                 /** initial gains */
	Pid *_tpids;								/** initial torque gains */
	bool _tpidsEnabled;							/** abilitation for torque gains */
    double *_limitsMin;                         /** joint limits, max*/
    double *_limitsMax;                         /** joint limits, min*/
    double *_currentLimits;                     /** current limits */
	int *_velocityShifts;                       /** velocity shifts */
	int *_velocityTimeout;                      /** velocity shifts */
	int *_torqueSensorId;						/** Id of associated Joint Torque Sensor */
	int *_torqueSensorChan;						/** Channel of associated Joint Torque Sensor */
	double *_maxTorque;						    /** Max torque of a joint */
	double *_newtonsToSensor;                   /** Newtons to force sensor units conversion factors */
};

class AnalogData
{
private:
    double *_data;
    int _size;
    int _bufferSize;
public:
    AnalogData(int ch, int buffsize): _data(0), _size(ch), _bufferSize(buffsize)
    {
        _data=new double[_bufferSize];
        for(int k=0;k<_bufferSize;k++)
            _data[k]=0;
    }
    ~AnalogData()
    {
        delete [] _data;
    }

    inline double &operator[](int i)
    { return _data[i]; }

    inline int size() 
    { return _size; }

    inline double *getBuffer()
    {return _data;}
};


#include <yarp/os/Semaphore.h>
typedef int AnalogDataFormat;

class CanBackDoor;

class AnalogSensor: public yarp::dev::IAnalogSensor,
                    public yarp::dev::DeviceDriver
{
public:
    enum AnalogDataFormat
    {
        ANALOG_FORMAT_8,
        ANALOG_FORMAT_16,
    };

	enum SensorStatus
    {
        ANALOG_IDLE=0,
        ANALOG_OK=1,
		ANALOG_NOT_RESPONDING=-1,
		ANALOG_SATURATION=-2,
		ANALOG_ERROR=-3,
    };

private:
    // debug messages
    unsigned int counterSat;
    unsigned int counterError;
    unsigned int counterTimeout;
    int rate;

    ////////////////////
    AnalogData *data;
	short status;
	double timeStamp;
	double* scaleFactor;
    yarp::os::Semaphore mutex;
    AnalogDataFormat dataFormat;
    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
	yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;
    short boardId;
	short useCalibration;
	bool  isVirtualSensor; //RANDAZ

    bool decode8(const unsigned char *msg, int id, double *data);
    bool decode16(const unsigned char *msg, int id, double *data);

public:
	CanBackDoor* backDoor; //RANDAZ

	AnalogSensor();
    ~AnalogSensor();
    bool handleAnalog(void *);

    void resetCounters()
    {
        counterSat=0;
        counterError=0;
        counterTimeout=0;
    }

    void getCounters(unsigned int &sat, unsigned int &err, unsigned int &to)
    {
        sat=counterSat;
        err=counterError;
        to=counterTimeout;
    }

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }

    short getId()
    { return boardId;}

	short getStatus()
    { return status;}

    bool isOpen()
    {
        if (data)
            return true;
        else
            return false;
    }

 	short getUseCalibration()
		{return useCalibration;}
	double* getScaleFactor()
		{return scaleFactor;}
	double getScaleFactor(int chan)
		{
			if (chan>=0 && chan<data->size())
				return scaleFactor[chan];
			else
				return 0;
		}

    bool open(int channels, AnalogDataFormat f, short bId, short useCalib, bool isVirtualSensor);

    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
	virtual int calibrateSensor();
    virtual int calibrateSensor(const double *v)
    {
        return calibrateSensor();
    }

    virtual int calibrateChannel(int ch)
    {
        return calibrateChannel(ch, 0);
    }
    /////////////////////////////////
};

class axisTorqueHelper
{
	int  jointsNum;
	int* torqueSensorId;						/** Id of associated Joint Torque Sensor */
	int* torqueSensorChan;						/** Channel of associated Joint Torque Sensor */
	double* maximumTorque;
	double* newtonsToSensor;

	public:
	axisTorqueHelper(int njoints, int* id, int* chan, double* maxTrq, double* newtons2sens );
	inline int getTorqueSensorId (int jnt)
	{
		if (jnt>=0 && jnt<jointsNum) return torqueSensorId[jnt];
		return 0;
	}						
	inline int getTorqueSensorChan (int jnt)
	{
		if (jnt>=0 && jnt<jointsNum) return torqueSensorChan[jnt];
		return 0;
	}
	inline double getMaximumTorque (int jnt)
	{
		if (jnt>=0 && jnt<jointsNum) return maximumTorque[jnt];
		return 0;
	}						
	inline double getNewtonsToSensor (int jnt)
	{
		if (jnt>=0 && jnt<jointsNum) return newtonsToSensor[jnt];
		return 0;
	}
	inline int getNumberOfJoints ()
	{ 
		return jointsNum;
	}
	inline ~axisTorqueHelper()
	{
		delete [] torqueSensorId;
		delete [] torqueSensorChan;
		delete [] maximumTorque;
		delete [] newtonsToSensor;
		torqueSensorId=0;						
		torqueSensorChan=0;						
		maximumTorque=0;						
		newtonsToSensor=0;						
	}
};

class yarp::dev::CanBusMotionControl:public DeviceDriver,
            public os::RateThread, 
            public IPidControlRaw, 
            public IPositionControlRaw,
            public IControlCalibration2Raw,
            public IVelocityControlRaw, 
            public IEncodersRaw, 
            public IAmplifierControlRaw,
            public IControlCalibrationRaw,
            public IControlDebug,
            public IControlLimitsRaw,
			public ITorqueControlRaw,
			public IImpedanceControlRaw,
			public IOpenLoopControlRaw,
            public IControlModeRaw,
            public ImplementPositionControl<CanBusMotionControl, IPositionControl>,
            public ImplementVelocityControl<CanBusMotionControl, IVelocityControl>,
            public ImplementPidControl<CanBusMotionControl, IPidControl>,
            public ImplementEncoders<CanBusMotionControl, IEncoders>,
            public ImplementControlCalibration<CanBusMotionControl, IControlCalibration>,    
            public ImplementControlCalibration2<CanBusMotionControl, IControlCalibration2>,
            public ImplementAmplifierControl<CanBusMotionControl, IAmplifierControl>,
            public ImplementControlLimits<CanBusMotionControl, IControlLimits>,
			public ImplementTorqueControl,
			public ImplementImpedanceControl,
			public ImplementOpenLoopControl,
            public ImplementControlMode,
            public IFactoryInterface
{
private:
    CanBusMotionControl(const CanBusMotionControl&);
    void operator=(const CanBusMotionControl&);

    void handleBroadcasts();
 
    double previousRun;
    double averagePeriod;
    double averageThreadTime;
    double currentRun;
    int myCount;
    double lastReportTime;
    os::Stamp stampEncoders;

    std::list<AnalogSensor *> analogSensors;

    yarp::os::ConstString canDevName;

    AnalogSensor *instantiateAnalog(yarp::os::Searchable& config, std::string id);
    void finiAnalog(AnalogSensor *s);

public:
    /**
    * Default constructor. Construction is done in two stages, first build the
    * object and then open the device driver.
    */
    CanBusMotionControl();

    /**
    * Destructor.
    */
    virtual ~CanBusMotionControl();

    /**
    * Open the device driver and start communication with the hardware.
    * @param config is a Searchable object containing the list of parameters.
    * @return true on success/failure.
    */
    virtual bool open(yarp::os::Searchable& config);

    /**
    * Open the device driver.
    * @param par is the parameter structure 
    * @return true/false on success/failure.
    */ 
    bool open(const CanBusMotionControlParameters &par);


    /**
    * Closes the device driver.
    * @return true on success.
    */
    virtual bool close(void);

    ////////////// IFactoryInterface
    yarp::dev::DeviceDriver *createDevice(yarp::os::Searchable& config);
    
    ///////////// PID INTERFACE
    //
    virtual bool setPidRaw(int j, const Pid &pid);
    virtual bool setPidsRaw(const Pid *pids);
    virtual bool setReferenceRaw(int j, double ref);
    virtual bool setReferencesRaw(const double *refs);
    virtual bool setErrorLimitRaw(int j, double limit);
    virtual bool setErrorLimitsRaw(const double *limits);
    virtual bool getErrorRaw(int j, double *err);
    virtual bool getErrorsRaw(double *errs);
    virtual bool getOutputRaw(int j, double *out);
    virtual bool getOutputsRaw(double *outs);
    virtual bool getPidRaw(int j, Pid *pid);
    virtual bool getPidsRaw(Pid *pids);
    virtual bool getReferenceRaw(int j, double *ref);
    virtual bool getReferencesRaw(double *refs);
    virtual bool getErrorLimitRaw(int j, double *limit);
    virtual bool getErrorLimitsRaw(double *limits);
    virtual bool resetPidRaw(int j);
    virtual bool disablePidRaw(int j);
    virtual bool enablePidRaw(int j);
    virtual bool setOffsetRaw(int j, double v);
    //
    /////////////////////////////// END PID INTERFACE

    //
    /// POSITION CONTROL INTERFACE RAW
    virtual bool getAxes(int *ax);
    virtual bool setPositionModeRaw();
    virtual bool positionMoveRaw(int j, double ref);
    virtual bool positionMoveRaw(const double *refs);
    virtual bool relativeMoveRaw(int j, double delta);
    virtual bool relativeMoveRaw(const double *deltas);
    virtual bool checkMotionDoneRaw(bool *flag);
    virtual bool checkMotionDoneRaw(int j, bool *flag);
    virtual bool setRefSpeedRaw(int j, double sp);
    virtual bool setRefSpeedsRaw(const double *spds);
    virtual bool setRefAccelerationRaw(int j, double acc);
    virtual bool setRefAccelerationsRaw(const double *accs);
    virtual bool getRefSpeedRaw(int j, double *ref);
    virtual bool getRefSpeedsRaw(double *spds);
    virtual bool getRefAccelerationRaw(int j, double *acc);
    virtual bool getRefAccelerationsRaw(double *accs);
    virtual bool stopRaw(int j);
    virtual bool stopRaw();
    //
    /////////////////////////////// END Position Control INTERFACE

	//
    /// TORQUE CONTROL INTERFACE RAW
//    virtual bool getAxes(int *ax);
    virtual bool getRefTorqueRaw(int j, double *ref_trq);
    virtual bool getRefTorquesRaw(double *ref_trqs);
    virtual bool setRefTorqueRaw(int j, double ref_trq);
    virtual bool setRefTorquesRaw(const double *ref_trqs);
    virtual bool getTorqueRaw(int j, double *trq);
    virtual bool getTorquesRaw(double *trqs);
    virtual bool getTorqueRangeRaw(int j, double *min, double *max);
    virtual bool getTorqueRangesRaw(double *min, double *max);

    virtual bool setTorquePidRaw(int j, const Pid &pid);
    virtual bool setTorquePidsRaw( const Pid *pid);
    virtual bool getTorquePidRaw(int j, Pid *pid);
    virtual bool getTorquePidsRaw(Pid *pids);
    virtual bool resetTorquePidRaw(int j);
    virtual bool disableTorquePidRaw(int j);
    virtual bool enableTorquePidRaw(int j);
    virtual bool setTorqueOffsetRaw(int j, double v);

    virtual bool getTorqueErrorLimitRaw(int j, double *limit);
    virtual bool getTorqueErrorLimitsRaw(double *limits);
    virtual bool setTorqueErrorLimitRaw(int j, double limit);
    virtual bool setTorqueErrorLimitsRaw(const double *limits);
    virtual bool getTorqueErrorRaw(int j, double *err);
    virtual bool getTorqueErrorsRaw(double *errs);
	virtual bool setTorqueModeRaw();
    virtual bool getTorquePidOutputRaw(int j, double *v);
    virtual bool getTorquePidOutputsRaw(double *v);
 
    //
    /////////////////////////////// END Torque Control INTERFACE

	//
    /// IMPEDANCE CONTROL INTERFACE RAW
    virtual bool getImpedanceRaw(int j, double *stiff, double *damp, double *offs);  
    virtual bool setImpedanceRaw(int j, double  stiff, double  damp, double  offs);   
    virtual bool getImpedanceOffsetRaw(int j, double *offs);  
    virtual bool setImpedanceOffsetRaw(int j, double  offs);   

	//
    /////////////////////////////// END Impedance Control INTERFACE

    // ControlMode
    virtual bool setPositionModeRaw(int j);
    virtual bool setVelocityModeRaw(int j);
    virtual bool setTorqueModeRaw(int j);
	virtual bool setImpedancePositionModeRaw(int j);
	virtual bool setImpedanceVelocityModeRaw(int j);
	virtual bool setOpenLoopModeRaw(int j);
    virtual bool getControlModeRaw(int j, int *v);
	virtual bool getControlModesRaw(int* v);

	///////////// OpenLoop control interface raw
    ///
	virtual bool setOpenLoopModeRaw();
    virtual bool setOutputRaw(int axis, double v);
    virtual bool setOutputsRaw(const double *v);
    //virtual bool getOutputRaw(int j, double *out); //already in PID interface
    //virtual bool getOutputsRaw(double *outs);      //already in PID interface
    //
    /////////////////////////////// END Velocity Control INTERFACE

    ///////////// Velocity control interface raw
    ///
    virtual bool setVelocityModeRaw();
    virtual bool velocityMoveRaw(int j, double sp);
    virtual bool velocityMoveRaw(const double *sp);
    //
    /////////////////////////////// END Velocity Control INTERFACE

	//Shift factors for velocity control
	bool setVelocityShift(int j, double val);
	//Timeout factors for velocity control
	bool setVelocityTimeout(int j, double val);


    //////////////////////// BEGIN EncoderInterface
    //
    virtual bool resetEncoderRaw(int j);
    virtual bool resetEncodersRaw();
    virtual bool setEncoderRaw(int j, double val);
    virtual bool setEncodersRaw(const double *vals);
    virtual bool getEncoderRaw(int j, double *v);
    virtual bool getEncodersRaw(double *encs);
    virtual bool getEncoderSpeedRaw(int j, double *sp);
    virtual bool getEncoderSpeedsRaw(double *spds);
    virtual bool getEncoderAccelerationRaw(int j, double *spds);
    virtual bool getEncoderAccelerationsRaw(double *accs);
    //
    ///////////////////////// END Encoder Interface

    ////// Amplifier interface
    //
    virtual bool enableAmpRaw(int j);
    virtual bool disableAmpRaw(int j);
    virtual bool getCurrentsRaw(double *vals);
    virtual bool getCurrentRaw(int j, double *val);
    virtual bool setMaxCurrentRaw(int j, double val);
    virtual bool getAmpStatusRaw(int *st);
    virtual bool getAmpStatusRaw(int j, int *st);

    //
    /////////////// END AMPLIFIER INTERFACE

    ////// calibration
    virtual bool calibrateRaw(int j, double p);
    virtual bool doneRaw(int j);
    virtual bool calibrate2Raw(int axis, unsigned int type, double p1, double p2, double p3);

    /// IControlDebug Interface
    virtual bool setPrintFunction(int (*f) (const char *fmt, ...));
    virtual bool loadBootMemory();
    virtual bool saveBootMemory();

    /////// Limits
    virtual bool setLimitsRaw(int axis, double min, double max);
    virtual bool getLimitsRaw(int axis, double *min, double *max);

protected:
    bool setBCastMessages (int axis, unsigned int v);

protected:
    void *system_resources;
    yarp::os::Semaphore _mutex;
    yarp::os::Semaphore _done;
    ICanBus *canController;

    bool _writerequested;
    bool _noreply;
    bool _opened;
    ThreadPool2 *threadPool;

    /**
    * filter for recurrent messages.
    */
    int _filter;

    /**
    * helper function to check whether the enabled flag is on or off.
    * @param axis is the axis to check for.
    * @return true if the axis is enabled and processing of the message
    * can in fact continue.
    */
    inline bool ENABLED (int axis);

    virtual void run(void);
    virtual bool threadInit();
    virtual void threadRelease();

    // helper functions
    bool _writeWord16 (int msg, int axis, short s);
    bool _writeWord16Ex (int msg, int axis, short s1, short s2, bool check);
    bool _readWord16 (int msg, int axis, short& value);
    bool _readWord16Array (int msg, double *out);
    bool _readDWord (int msg, int axis, int& value);
    bool _readDWordArray (int msg, double *out);
    bool _writeDWord (int msg, int axis, int value);
    bool _writeNone  (int msg, int axis);
	bool _writeByte8 (int msg, int axis, int value);
    bool _writeByteWords16(int msg, int axis, unsigned char value, short s1, short s2, short s3);
	axisTorqueHelper *_axisTorqueHelper;

    // internal stuff.
    double *_ref_speeds;		// used for position control.
    double *_command_speeds;	// used for velocity control.
    double *_ref_accs;			// for velocity control, in position min jerk eq is used.
	double *_ref_torques;		// for torque control.
    double *_ref_positions;		// for position control.

    enum { MAX_SHORT = 32767, MIN_SHORT = -32768, MAX_INT = 0x7fffffff, MIN_INT = 0x80000000 };
    enum { CAN_SKIP_ADDR = 0x80 };

    inline short S_16(double x) const 
    {
        if (x <= double(-(MAX_SHORT))-1)
            return MIN_SHORT;
        else
            if (x >= double(MAX_SHORT))
                return MAX_SHORT;
        else
            if  (x>0)
				return short(x + .5);
			else
				return short(x - .5);
    }

    inline int S_32(double x) const
    {
        if (x <= double(-(MAX_INT))-1.0)
            return MIN_INT;
        else
            if (x >= double(MAX_INT))
                return MAX_INT;
        else
            if  (x>0)
				return int(x + .5);
			else
				return int(x - .5);
    }
};

#endif
