/**
 * This file is created at Almende B.V. It is open-source software and part of the Common 
 * Hybrid Agent Platform (CHAP). A toolbox with a lot of open-source tools, ranging from 
 * thread pools and TCP/IP components to control architectures and learning algorithms. 
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless,
 * we personally strongly object against this software being used by the military, in the
 * bio-industry, for animal experimentation, or anything that violates the Universal
 * Declaration of Human Rights.
 *
 * Copyright (c) 2010 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @company Almende B.V.
 */

// general C/C++ headers
#include <vector>
#include <string>
#include <sstream>

// middleware specific headers
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Bottle.h>

// namespaces and typedefs
using namespace yarp::os;

// recommended namespace: "rur"
// do not forget to add "using namespace rur;" to your .cpp file
namespace rur {

struct Param {
  std::string module_id;
  int parameter;
};

typedef std::vector<int> long_seq;

typedef std::vector<float> float_seq;

// The generated class. Do not modify or add class members
// Either derive from this class and implement Tick() or
// use a separate helper class to store state information.
// All information for the operation of the module should 
// be obtained over the defined ports
class autoPilot {
private:
  Network yarp;
  std::string module_id;
  
  // private storage for portCommandValue
  int portCommandValue;
  // the port portCommand itself
  BufferedPort<Bottle> *portCommand;
  
  // the port portStatus itself
  BufferedPort<Bottle> *portStatus;
  
  // the port portToMapSelf itself
  BufferedPort<Bottle> *portToMapSelf;
  
  // private storage for portFromMapSelfValues;
  std::vector<float> *portFromMapSelfValues;
  // the port portFromMapSelf itself
  BufferedPort<Bottle> *portFromMapSelf;
  
  // private storage for portFromWayPointPlannerValues;
  std::vector<float> *portFromWayPointPlannerValues;
  // the port portFromWayPointPlanner itself
  BufferedPort<Bottle> *portFromWayPointPlanner;
  
  // User-defined structs (automatically allocated later)
  Param *cliParam;

public:
  // The constructor needs to be called, also when you derive from this class
  autoPilot() {
    cliParam = new Param();
    portCommand = new BufferedPort<Bottle>();
    portCommand->setStrict();
    portCommand->writeStrict();
    portStatus = new BufferedPort<Bottle>();
    portStatus->setStrict();
    portStatus->writeStrict();
    portToMapSelf = new BufferedPort<Bottle>();
    portToMapSelf->setStrict();
    portToMapSelf->writeStrict();
    portFromMapSelfValues = new std::vector<float>();
    portFromMapSelf = new BufferedPort<Bottle>();
    portFromMapSelf->setStrict();
    portFromMapSelf->writeStrict();
    portFromWayPointPlannerValues = new std::vector<float>();
    portFromWayPointPlanner = new BufferedPort<Bottle>();
    portFromWayPointPlanner->setStrict();
    portFromWayPointPlanner->writeStrict();
  }
  
  ~autoPilot() {
    delete portCommand;
    delete portStatus;
    delete portToMapSelf;
    delete portFromMapSelfValues;
    delete portFromMapSelf;
    delete portFromWayPointPlannerValues;
    delete portFromWayPointPlanner;
    delete cliParam;
  }
  
  // This is the function you will need to implement.
  void Tick(); 
  
  
  // After construction you will need to call this function first
  // it opens the YARP ports
  void Init(std::string module_id) {
    this->module_id = module_id;
    
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/autopilot" << module_id << "/command";
      portCommand->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/autopilot" << module_id << "/status";
      portStatus->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/autopilot" << module_id << "/tomapself";
      portToMapSelf->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/autopilot" << module_id << "/frommapself";
      portFromMapSelf->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/autopilot" << module_id << "/fromwaypointplanner";
      portFromWayPointPlanner->open(portName.str().c_str());
    }
  }
  
  // Before destruction you will need to call this function first
  // it closes the YARP ports
  void Close() {
    portCommand->close();
    portStatus->close();
    portToMapSelf->close();
    portFromMapSelf->close();
    portFromWayPointPlanner->close();
  }
  
  // Function to get Param struct (to subsequently set CLI parameters)
  inline Param *GetParam() { return cliParam; };
  
protected:
  // All subsequent functions should be called from "within" this module
  // From either the Tick() routine itself, or Tick() in a derived class
  
  inline int *readCommand(bool blocking=true) {
    Bottle *b = portCommand->read(blocking);
    if (b != NULL) { 
      portCommandValue = b->get(0).asInt();
      return &portCommandValue;
    }
    return NULL;
  }
  
  inline void writeStatus(const int val) {
    Bottle &valPrepare = portStatus->prepare();
    valPrepare.clear();
    valPrepare.addInt(val);
    portStatus->write(true);
  }
  
  inline void writeToMapSelf(const float_seq &seq) {
    Bottle &seqPrepare = portToMapSelf->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portToMapSelf->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readFromMapSelf(bool blocking=true) {
    Bottle *b = portFromMapSelf->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portFromMapSelfValues->push_back(b->get(i).asDouble());
      }
    }
    return portFromMapSelfValues;
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readFromWayPointPlanner(bool blocking=true) {
    Bottle *b = portFromWayPointPlanner->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portFromWayPointPlannerValues->push_back(b->get(i).asDouble());
      }
    }
    return portFromWayPointPlannerValues;
  }
  
};
}

