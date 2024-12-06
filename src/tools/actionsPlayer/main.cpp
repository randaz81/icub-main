/*
 * SPDX-FileCopyrightText: 2024 Istituto Italiano di Tecnologia (IIT)
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <deque>
#include <cmath>
#include <map>
#include <mutex>

#include "robotDriver.h"
#include "action.h"
#include "broadcasting_thread.h"
#include "working_thread.h"

// ******************** THE MODULE
class scriptModule: public yarp::os::RFModule
{
protected:
    yarp::os::Port      m_rpcPort;
    std::string         m_name;
    bool                m_verbose;
    std::map<std::string,robotDriver*>  m_robotControllers;
    std::map<std::string,action_class> m_actions;
    WorkingThread       m_wthread;
    //BroadcastingThread  m_bthread;

    std::string         m_current_action_id;

    public:
    scriptModule() 
    {
        m_verbose=true;
    }

    ~scriptModule()
    {
        for (auto it = m_robotControllers.begin(); it != m_robotControllers.end(); it++)
        {
            if (it->second)
            {
                delete it->second;
                it->second = nullptr;
            }
        }
        yInfo() << "cleanup complete";
    }

    bool chooseActionByName(std::string id)
    {
        if (m_actions.find(id) == m_actions.end())
        {
            yError() << "action id not found";
            return false;
        }
        m_current_action_id = id;

        action_class *action = &m_actions[id];
        std::string controller_name = action->controller_name;
        robotDriver *driver = m_robotControllers[controller_name];

        if (!driver || !action)
        {
            yError() << "invalid driver/action pointer";
            return false;
        }
        yInfo() << "action selected:" << id;
        yDebug() << "action controller:" << controller_name;
        yDebug() << "number of action frames:" << action->action_frames_vector.size();
        return m_wthread.action_change(action, driver);
    }

    std::string show_actions()
    {
        std::string ss = "actions:\n";
        size_t i = 0;
        for (auto it=m_actions.begin(); it!=m_actions.end();it++)
        {
            ss = ss + "(" + std::to_string(i++) + ") " + it->second.action_name + "\n";
        }
        return ss;
    }

    bool loadConfiguration(std::string filename)
    {
        yarp::os::Property p;
        if (!p.fromConfigFile(filename))
        {
            yError() << "Unable to read configuration file!";
            return false;
        }

        yarp::os::Bottle& bot_cont = p.findGroup("CONTROLLERS");
        if (bot_cont.size() == 0)
        {
            yError() << "Unable to read CONTROLLERS section";
            return false;
        }
        size_t num_of_controllers = bot_cont.size();
        for (size_t i = 1; i < num_of_controllers; i++)
        {
            yDebug() << bot_cont.get(i).toString();
            yarp::os::Bottle* bot_cont_elem = bot_cont.get(i).asList();
            size_t num_of_celems = bot_cont_elem->size();
            if (bot_cont_elem && num_of_celems == 3)
            {
                //parse a line of the controllers section
                std::string controller_name = bot_cont_elem->get(0).toString();
                std::string remoteControlBoards = bot_cont_elem->get(1).toString();
                std::string axesNames = bot_cont_elem->get(2).toString();
                robotDriver* rob = new robotDriver;
                yarp::os::Property rmoptions;
                rmoptions.put("remoteControlBoards", bot_cont_elem->get(1));
                rmoptions.put("axesNames", bot_cont_elem->get(2));
                rmoptions.put("localPortPrefix", m_name + "/controller/" + controller_name);

                //configure the controller
                bool rob_ok = true;
                rob_ok &= rob->configure(rmoptions);
                rob_ok &= rob->init();
                if (!rob_ok)
                {
                    yError() << "Unable to initialize controller" << controller_name;
                    return false;
                }
                //put the controller in the list
                m_robotControllers[controller_name] = rob;
            }
            else
            {
                yError() << "Invalid entry in CONTROLLERS section";
                return false;
            }
        }

        yarp::os::Bottle& bot_action = p.findGroup("ACTIONS");
        if (bot_action.size() == 0)
        {
            yError() << "Unable to read ACTIONS section";
            return false;
        }
        for (size_t i = 1; i < bot_action.size(); i++)
        {
            yDebug() << bot_action.get(i).toString();
            yarp::os::Bottle* bot_act_elem = bot_action.get(i).asList();
            size_t num_of_aelems = bot_act_elem->size();
            if (bot_act_elem && num_of_aelems==3)
            {
                //parse a line of the ACTIONS section
                std::string action_name = bot_act_elem->get(0).toString();
                std::string controller_name = bot_act_elem->get(1).toString();
                std::string action_file_name = bot_act_elem->get(2).toString();

                //check if the controller name exists
                if (m_robotControllers.find(controller_name) == m_robotControllers.end())
                {
                    yError() << controller_name << "in action" << action_name << "does not exists";
                    return false;
                }

                //load the action file
                action_class tmpAction;
                tmpAction.action_name = action_name;
                tmpAction.controller_name = controller_name;
                size_t njoints = m_robotControllers[controller_name]->getNJoints();
                
                if (!tmpAction.openFile(action_file_name, njoints, 0.010))
                {
                    yError() << "Unable to parse file";
                    return false;
                }

                //put the action in the list
                m_actions[action_name] = tmpAction;
            }
            else
            {
                yError() << "Invalid entry in ACTIONS section";
                return false;
            }
        }

        yInfo() << "configuration file successfully loaded";
        return true;
    }

    virtual bool configure(yarp::os::ResourceFinder &rf)
    {
        std::string test_string = rf.toString();

        //generic configuration
        if (rf.check("name"))
            m_name=std::string("/")+rf.find("name").asString().c_str();
        else
            m_name="/trajectoryPlayer";

        //rpc port
        m_rpcPort.open((m_name+"/rpc").c_str());
        attach(m_rpcPort);

        //set the configuration for parameter execute
        if (rf.check("execute")==true)
        {
            yInfo() << "Enabling iPid->setReference() controller";
            m_wthread.enable_execute_joint_command = true;
        }
        else
        {
            yInfo() << "Not using iPid->setReference() controller";
            m_wthread.enable_execute_joint_command = false;
        }

        //set the configuration for parameter period
        if (rf.check("period")==true)
        {
            double period = rf.find("period").asFloat64();
            yInfo() << "Thread period set to " << period << "s";
            m_wthread.setPeriod(period);
        }

        //open the configuration file
        if (rf.check("filename")==true)
        {
            if (rf.find("filename").isString())
            {
                std::string filename = rf.find("filename").asString();
                bool b = loadConfiguration(filename);
                if (!b)
                {
                    yError() << "Configuration error!";
                    return false;
                }
            }
            else
            {
                yError() << "`filename` option syntax error.";
                return false;
            }
        }
        else
        {
            yWarning() << "`filename` option not found. No sequence files loaded.";
        }

        //check if actions are valid
        if (m_actions.empty())
        {
            yInfo() << "There are no actions!";
            return false;
        }

        //select the first action
        yInfo() << "automatically selecting the first action";
        std::string first_action_name;
        first_action_name = this->m_actions.begin()->first;
        this->chooseActionByName(first_action_name);

        //start the thread
        if (!m_wthread.start())
        {
            yError() << "Working thread did not start, queue will not work";
        }
        else
        {
            yInfo() << "Working thread started";
        }

        yInfo() << "module successfully configured. ready.";
        return true;
    }

    virtual bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply)
    {
        bool ret=true;

        if (command.size()!=0)
        {
            std::string cmdstring = command.get(0).asString().c_str();
            {
                if  (cmdstring == "help")
                {
                    std::cout << "Available commands:"          << std::endl;
                    std::cout << "=== commands for current action ===="          << std::endl;
                    std::cout << "start" << std::endl;
                    std::cout << "stop"  << std::endl;
                    std::cout << "reset" << std::endl;
                    std::cout << "clear" << std::endl;
                    std::cout << "forever" << std::endl;
                    std::cout << "print" << std::endl;
                    std::cout << "=== general commands ====" << std::endl;
                    std::cout << "choose_action <id>" << std::endl;
                    std::cout << "play <id>"<< std::endl;
                    std::cout << "show_actions" << std::endl;
                    reply.addVocab32("many");
                    reply.addVocab32("ack");
                    reply.addString("Available commands:");
                    reply.addString("=== commands for current action ====");
                    reply.addString("start");
                    reply.addString("stop");
                    reply.addString("reset");
                    reply.addString("forever");
                    reply.addString("print");
                    reply.addString("=== general commands ====");
                    reply.addString("choose_action <id>");
                    reply.addString("play <id>");
                    reply.addString("show_actions");
                }
                else if  (cmdstring == "start")
                {
                    bool b = this->m_wthread.action_start();
                    reply.addVocab32("ack");
                }
                else if  (cmdstring == "forever")
                {
                    bool b = this->m_wthread.action_forever();
                    reply.addVocab32("ack");
                }
                else if  (cmdstring == "stop")
                {
                    bool b = this->m_wthread.action_stop();
                    reply.addVocab32("ack");
                }
                else if  (cmdstring == "reset")
                {
                    bool b = this->m_wthread.action_reset();
                    reply.addVocab32("ack");
                }
                else if  (cmdstring == "print")
                {
                    bool b = this->m_wthread.action_print();
                    reply.addVocab32("ack");
                }
                else if (cmdstring == "choose_action")
                {
                    std::string action_id = command.get(1).asString();
                    bool b = this->chooseActionByName(action_id);
                    reply.addVocab32("ack");
                }
                else if (cmdstring == "play")
                {
                    std::string action_id = command.get(1).asString();
                    bool b = this->chooseActionByName(action_id);
                    if (b)
                    {
                        bool b1 = this->m_wthread.action_start();
                    }
                    reply.addVocab32("ack");
                }
                else if (cmdstring == "show_actions")
                {
                    std::string actions_str = this->show_actions();
                    std::string current_action_name;
                    bool b = m_wthread.action_getname(current_action_name);
                    reply.addVocab32("ack");
                    yInfo() << "current_action: " <<current_action_name;
                    yInfo() << actions_str;
                }
                else
                {
                    reply.addVocab32("nack");
                    ret = false;
                }
            }
        }
        else
        {
            reply.addVocab32("nack");
            ret = false;
        }

        return ret;
    }

    virtual bool close()
    {
        m_rpcPort.interrupt();
        m_rpcPort.close();

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};

//--------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("trajectoryPlayer");
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        yInfo() << "Options:";
        yInfo() << "\t--name         <moduleName>: set new module name";
        yInfo() << "\t--filename     <filename>:   the configuration file";
        yInfo() << "\t--execute      activate the iPid->setReference() control";
        yInfo() << "\t--period       <period>: the period in s of the internal thread";
        yInfo() << "\t--verbose      to display additional infos";
        return 0;
    }

    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        yError() << "yarp.checkNetwork() failed.";
        return -1;
    }

    scriptModule mod;
 
    return mod.runModule(rf);
}
