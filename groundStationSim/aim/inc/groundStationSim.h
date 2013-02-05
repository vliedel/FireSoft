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
class groundStationSim {
private:
  Network yarp;
  std::string module_id;
  
  // private storage for portCommandValue
  int portCommandValue;
  // the port portCommand itself
  BufferedPort<Bottle> *portCommand;
  
  // the port portStatus itself
  BufferedPort<Bottle> *portStatus;
  
  // private storage for portSimValues;
  std::vector<float> *portSimValues;
  // the port portSim itself
  BufferedPort<Bottle> *portSim;
  
  // the port portToSim itself
  BufferedPort<Bottle> *portToSim;
  
  // the port portToMapUavs itself
  BufferedPort<Bottle> *portToMapUavs;
  
  // the port portToGuiInterface itself
  BufferedPort<Bottle> *portToGuiInterface;
  
  // private storage for portFromGuiInterfaceValues;
  std::vector<float> *portFromGuiInterfaceValues;
  // the port portFromGuiInterface itself
  BufferedPort<Bottle> *portFromGuiInterface;
  
  // User-defined structs (automatically allocated later)
  Param *cliParam;

public:
  // The constructor needs to be called, also when you derive from this class
  groundStationSim() {
    cliParam = new Param();
    portCommand = new BufferedPort<Bottle>();
    portCommand->setStrict();
    portCommand->writeStrict();
    portStatus = new BufferedPort<Bottle>();
    portStatus->setStrict();
    portStatus->writeStrict();
    portSimValues = new std::vector<float>();
    portSim = new BufferedPort<Bottle>();
    portSim->setStrict();
    portSim->writeStrict();
    portToSim = new BufferedPort<Bottle>();
    portToSim->setStrict();
    portToSim->writeStrict();
    portToMapUavs = new BufferedPort<Bottle>();
    portToMapUavs->setStrict();
    portToMapUavs->writeStrict();
    portToGuiInterface = new BufferedPort<Bottle>();
    portToGuiInterface->setStrict();
    portToGuiInterface->writeStrict();
    portFromGuiInterfaceValues = new std::vector<float>();
    portFromGuiInterface = new BufferedPort<Bottle>();
    portFromGuiInterface->setStrict();
    portFromGuiInterface->writeStrict();
  }
  
  ~groundStationSim() {
    delete portCommand;
    delete portStatus;
    delete portSimValues;
    delete portSim;
    delete portToSim;
    delete portToMapUavs;
    delete portToGuiInterface;
    delete portFromGuiInterfaceValues;
    delete portFromGuiInterface;
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
      portName << "/groundstationsim" << module_id << "/command";
      portCommand->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/status";
      portStatus->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/sim";
      portSim->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/tosim";
      portToSim->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/tomapuavs";
      portToMapUavs->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/toguiinterface";
      portToGuiInterface->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/groundstationsim" << module_id << "/fromguiinterface";
      portFromGuiInterface->open(portName.str().c_str());
    }
  }
  
  // Before destruction you will need to call this function first
  // it closes the YARP ports
  void Close() {
    portCommand->close();
    portStatus->close();
    portSim->close();
    portToSim->close();
    portToMapUavs->close();
    portToGuiInterface->close();
    portFromGuiInterface->close();
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
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readSim(bool blocking=true) {
    Bottle *b = portSim->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portSimValues->push_back(b->get(i).asDouble());
      }
    }
    return portSimValues;
  }
  
  inline void writeSim(const float_seq &seq) {
    Bottle &seqPrepare = portSim->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portSim->write(true);
  }
  
  inline void writeToSim(const float_seq &seq) {
    Bottle &seqPrepare = portToSim->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portToSim->write(true);
  }
  
  inline void writeToMapUavs(const float_seq &seq) {
    Bottle &seqPrepare = portToMapUavs->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portToMapUavs->write(true);
  }
  
  inline void writeToGuiInterface(const float_seq &seq) {
    Bottle &seqPrepare = portToGuiInterface->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portToGuiInterface->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readFromGuiInterface(bool blocking=true) {
    Bottle *b = portFromGuiInterface->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portFromGuiInterfaceValues->push_back(b->get(i).asDouble());
      }
    }
    return portFromGuiInterfaceValues;
  }
  
};
}

