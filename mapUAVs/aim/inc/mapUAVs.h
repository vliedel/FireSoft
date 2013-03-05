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
class mapUAVs {
private:
  Network yarp;
  std::string module_id;
  
  // the port portToRadio itself
  BufferedPort<Bottle> *portToRadio;
  
  // private storage for portFromRadioValues;
  std::vector<float> *portFromRadioValues;
  // the port portFromRadio itself
  BufferedPort<Bottle> *portFromRadio;
  
  // User-defined structs (automatically allocated later)
  Param *cliParam;

public:
  // The constructor needs to be called, also when you derive from this class
  mapUAVs() {
    cliParam = new Param();
    portToRadio = new BufferedPort<Bottle>();
    portToRadio->setStrict();
    portToRadio->writeStrict();
    portFromRadioValues = new std::vector<float>();
    portFromRadio = new BufferedPort<Bottle>();
    portFromRadio->setStrict();
    portFromRadio->writeStrict();
  }
  
  ~mapUAVs() {
    delete portToRadio;
    delete portFromRadioValues;
    delete portFromRadio;
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
      portName << "/mapuavs" << module_id << "/toradio";
      portToRadio->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/mapuavs" << module_id << "/fromradio";
      portFromRadio->open(portName.str().c_str());
    }
  }
  
  // Before destruction you will need to call this function first
  // it closes the YARP ports
  void Close() {
    portToRadio->close();
    portFromRadio->close();
  }
  
  // Function to get Param struct (to subsequently set CLI parameters)
  inline Param *GetParam() { return cliParam; };
  
protected:
  // All subsequent functions should be called from "within" this module
  // From either the Tick() routine itself, or Tick() in a derived class
  
  inline void writeToRadio(const int cmd) {
    Bottle &cmdPrepare = portToRadio->prepare();
    cmdPrepare.clear();
    cmdPrepare.addInt(cmd);
    portToRadio->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readFromRadio(bool blocking=true) {
    Bottle *b = portFromRadio->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portFromRadioValues->push_back(b->get(i).asDouble());
      }
    }
    return portFromRadioValues;
  }
  
};
}

