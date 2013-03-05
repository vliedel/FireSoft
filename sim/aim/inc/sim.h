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
class sim {
private:
  Network yarp;
  std::string module_id;
  
  // private storage for portCommandValue
  int portCommandValue;
  // the port portCommand itself
  BufferedPort<Bottle> *portCommand;
  
  // private storage for portGroundStationValues;
  std::vector<float> *portGroundStationValues;
  // the port portGroundStation itself
  BufferedPort<Bottle> *portGroundStation;
  
  // private storage for portFromGroundStationValues;
  std::vector<float> *portFromGroundStationValues;
  // the port portFromGroundStation itself
  BufferedPort<Bottle> *portFromGroundStation;
  
  // the port portRadioCommand0 itself
  BufferedPort<Bottle> *portRadioCommand0;
  
  // private storage for portRadioState0Values;
  std::vector<float> *portRadioState0Values;
  // the port portRadioState0 itself
  BufferedPort<Bottle> *portRadioState0;
  
  // the port portAutoPilotCommand0 itself
  BufferedPort<Bottle> *portAutoPilotCommand0;
  
  // private storage for portAutoPilotState0Values;
  std::vector<float> *portAutoPilotState0Values;
  // the port portAutoPilotState0 itself
  BufferedPort<Bottle> *portAutoPilotState0;
  
  // the port portRadioCommand1 itself
  BufferedPort<Bottle> *portRadioCommand1;
  
  // private storage for portRadioState1Values;
  std::vector<float> *portRadioState1Values;
  // the port portRadioState1 itself
  BufferedPort<Bottle> *portRadioState1;
  
  // the port portAutoPilotCommand1 itself
  BufferedPort<Bottle> *portAutoPilotCommand1;
  
  // private storage for portAutoPilotState1Values;
  std::vector<float> *portAutoPilotState1Values;
  // the port portAutoPilotState1 itself
  BufferedPort<Bottle> *portAutoPilotState1;
  
  // the port portRadioCommand2 itself
  BufferedPort<Bottle> *portRadioCommand2;
  
  // private storage for portRadioState2Values;
  std::vector<float> *portRadioState2Values;
  // the port portRadioState2 itself
  BufferedPort<Bottle> *portRadioState2;
  
  // the port portAutoPilotCommand2 itself
  BufferedPort<Bottle> *portAutoPilotCommand2;
  
  // private storage for portAutoPilotState2Values;
  std::vector<float> *portAutoPilotState2Values;
  // the port portAutoPilotState2 itself
  BufferedPort<Bottle> *portAutoPilotState2;
  
  // the port portRadioCommand3 itself
  BufferedPort<Bottle> *portRadioCommand3;
  
  // private storage for portRadioState3Values;
  std::vector<float> *portRadioState3Values;
  // the port portRadioState3 itself
  BufferedPort<Bottle> *portRadioState3;
  
  // the port portAutoPilotCommand3 itself
  BufferedPort<Bottle> *portAutoPilotCommand3;
  
  // private storage for portAutoPilotState3Values;
  std::vector<float> *portAutoPilotState3Values;
  // the port portAutoPilotState3 itself
  BufferedPort<Bottle> *portAutoPilotState3;
  
  // the port portRadioCommand4 itself
  BufferedPort<Bottle> *portRadioCommand4;
  
  // private storage for portRadioState4Values;
  std::vector<float> *portRadioState4Values;
  // the port portRadioState4 itself
  BufferedPort<Bottle> *portRadioState4;
  
  // the port portAutoPilotCommand4 itself
  BufferedPort<Bottle> *portAutoPilotCommand4;
  
  // private storage for portAutoPilotState4Values;
  std::vector<float> *portAutoPilotState4Values;
  // the port portAutoPilotState4 itself
  BufferedPort<Bottle> *portAutoPilotState4;
  
  // the port portRadioCommand5 itself
  BufferedPort<Bottle> *portRadioCommand5;
  
  // private storage for portRadioState5Values;
  std::vector<float> *portRadioState5Values;
  // the port portRadioState5 itself
  BufferedPort<Bottle> *portRadioState5;
  
  // the port portAutoPilotCommand5 itself
  BufferedPort<Bottle> *portAutoPilotCommand5;
  
  // private storage for portAutoPilotState5Values;
  std::vector<float> *portAutoPilotState5Values;
  // the port portAutoPilotState5 itself
  BufferedPort<Bottle> *portAutoPilotState5;
  
  // the port portRadioCommand6 itself
  BufferedPort<Bottle> *portRadioCommand6;
  
  // private storage for portRadioState6Values;
  std::vector<float> *portRadioState6Values;
  // the port portRadioState6 itself
  BufferedPort<Bottle> *portRadioState6;
  
  // the port portAutoPilotCommand6 itself
  BufferedPort<Bottle> *portAutoPilotCommand6;
  
  // private storage for portAutoPilotState6Values;
  std::vector<float> *portAutoPilotState6Values;
  // the port portAutoPilotState6 itself
  BufferedPort<Bottle> *portAutoPilotState6;
  
  // the port portRadioCommand7 itself
  BufferedPort<Bottle> *portRadioCommand7;
  
  // private storage for portRadioState7Values;
  std::vector<float> *portRadioState7Values;
  // the port portRadioState7 itself
  BufferedPort<Bottle> *portRadioState7;
  
  // the port portAutoPilotCommand7 itself
  BufferedPort<Bottle> *portAutoPilotCommand7;
  
  // private storage for portAutoPilotState7Values;
  std::vector<float> *portAutoPilotState7Values;
  // the port portAutoPilotState7 itself
  BufferedPort<Bottle> *portAutoPilotState7;
  
  // the port portRadioCommand8 itself
  BufferedPort<Bottle> *portRadioCommand8;
  
  // private storage for portRadioState8Values;
  std::vector<float> *portRadioState8Values;
  // the port portRadioState8 itself
  BufferedPort<Bottle> *portRadioState8;
  
  // the port portAutoPilotCommand8 itself
  BufferedPort<Bottle> *portAutoPilotCommand8;
  
  // private storage for portAutoPilotState8Values;
  std::vector<float> *portAutoPilotState8Values;
  // the port portAutoPilotState8 itself
  BufferedPort<Bottle> *portAutoPilotState8;
  
  // the port portRadioCommand9 itself
  BufferedPort<Bottle> *portRadioCommand9;
  
  // private storage for portRadioState9Values;
  std::vector<float> *portRadioState9Values;
  // the port portRadioState9 itself
  BufferedPort<Bottle> *portRadioState9;
  
  // the port portAutoPilotCommand9 itself
  BufferedPort<Bottle> *portAutoPilotCommand9;
  
  // private storage for portAutoPilotState9Values;
  std::vector<float> *portAutoPilotState9Values;
  // the port portAutoPilotState9 itself
  BufferedPort<Bottle> *portAutoPilotState9;
  
  // User-defined structs (automatically allocated later)
  Param *cliParam;

public:
  // The constructor needs to be called, also when you derive from this class
  sim() {
    cliParam = new Param();
    portCommand = new BufferedPort<Bottle>();
    portCommand->setStrict();
    portCommand->writeStrict();
    portGroundStationValues = new std::vector<float>();
    portGroundStation = new BufferedPort<Bottle>();
    portGroundStation->setStrict();
    portGroundStation->writeStrict();
    portFromGroundStationValues = new std::vector<float>();
    portFromGroundStation = new BufferedPort<Bottle>();
    portFromGroundStation->setStrict();
    portFromGroundStation->writeStrict();
    portRadioCommand0 = new BufferedPort<Bottle>();
    portRadioCommand0->setStrict();
    portRadioCommand0->writeStrict();
    portRadioState0Values = new std::vector<float>();
    portRadioState0 = new BufferedPort<Bottle>();
    portRadioState0->setStrict();
    portRadioState0->writeStrict();
    portAutoPilotCommand0 = new BufferedPort<Bottle>();
    portAutoPilotCommand0->setStrict();
    portAutoPilotCommand0->writeStrict();
    portAutoPilotState0Values = new std::vector<float>();
    portAutoPilotState0 = new BufferedPort<Bottle>();
    portAutoPilotState0->setStrict();
    portAutoPilotState0->writeStrict();
    portRadioCommand1 = new BufferedPort<Bottle>();
    portRadioCommand1->setStrict();
    portRadioCommand1->writeStrict();
    portRadioState1Values = new std::vector<float>();
    portRadioState1 = new BufferedPort<Bottle>();
    portRadioState1->setStrict();
    portRadioState1->writeStrict();
    portAutoPilotCommand1 = new BufferedPort<Bottle>();
    portAutoPilotCommand1->setStrict();
    portAutoPilotCommand1->writeStrict();
    portAutoPilotState1Values = new std::vector<float>();
    portAutoPilotState1 = new BufferedPort<Bottle>();
    portAutoPilotState1->setStrict();
    portAutoPilotState1->writeStrict();
    portRadioCommand2 = new BufferedPort<Bottle>();
    portRadioCommand2->setStrict();
    portRadioCommand2->writeStrict();
    portRadioState2Values = new std::vector<float>();
    portRadioState2 = new BufferedPort<Bottle>();
    portRadioState2->setStrict();
    portRadioState2->writeStrict();
    portAutoPilotCommand2 = new BufferedPort<Bottle>();
    portAutoPilotCommand2->setStrict();
    portAutoPilotCommand2->writeStrict();
    portAutoPilotState2Values = new std::vector<float>();
    portAutoPilotState2 = new BufferedPort<Bottle>();
    portAutoPilotState2->setStrict();
    portAutoPilotState2->writeStrict();
    portRadioCommand3 = new BufferedPort<Bottle>();
    portRadioCommand3->setStrict();
    portRadioCommand3->writeStrict();
    portRadioState3Values = new std::vector<float>();
    portRadioState3 = new BufferedPort<Bottle>();
    portRadioState3->setStrict();
    portRadioState3->writeStrict();
    portAutoPilotCommand3 = new BufferedPort<Bottle>();
    portAutoPilotCommand3->setStrict();
    portAutoPilotCommand3->writeStrict();
    portAutoPilotState3Values = new std::vector<float>();
    portAutoPilotState3 = new BufferedPort<Bottle>();
    portAutoPilotState3->setStrict();
    portAutoPilotState3->writeStrict();
    portRadioCommand4 = new BufferedPort<Bottle>();
    portRadioCommand4->setStrict();
    portRadioCommand4->writeStrict();
    portRadioState4Values = new std::vector<float>();
    portRadioState4 = new BufferedPort<Bottle>();
    portRadioState4->setStrict();
    portRadioState4->writeStrict();
    portAutoPilotCommand4 = new BufferedPort<Bottle>();
    portAutoPilotCommand4->setStrict();
    portAutoPilotCommand4->writeStrict();
    portAutoPilotState4Values = new std::vector<float>();
    portAutoPilotState4 = new BufferedPort<Bottle>();
    portAutoPilotState4->setStrict();
    portAutoPilotState4->writeStrict();
    portRadioCommand5 = new BufferedPort<Bottle>();
    portRadioCommand5->setStrict();
    portRadioCommand5->writeStrict();
    portRadioState5Values = new std::vector<float>();
    portRadioState5 = new BufferedPort<Bottle>();
    portRadioState5->setStrict();
    portRadioState5->writeStrict();
    portAutoPilotCommand5 = new BufferedPort<Bottle>();
    portAutoPilotCommand5->setStrict();
    portAutoPilotCommand5->writeStrict();
    portAutoPilotState5Values = new std::vector<float>();
    portAutoPilotState5 = new BufferedPort<Bottle>();
    portAutoPilotState5->setStrict();
    portAutoPilotState5->writeStrict();
    portRadioCommand6 = new BufferedPort<Bottle>();
    portRadioCommand6->setStrict();
    portRadioCommand6->writeStrict();
    portRadioState6Values = new std::vector<float>();
    portRadioState6 = new BufferedPort<Bottle>();
    portRadioState6->setStrict();
    portRadioState6->writeStrict();
    portAutoPilotCommand6 = new BufferedPort<Bottle>();
    portAutoPilotCommand6->setStrict();
    portAutoPilotCommand6->writeStrict();
    portAutoPilotState6Values = new std::vector<float>();
    portAutoPilotState6 = new BufferedPort<Bottle>();
    portAutoPilotState6->setStrict();
    portAutoPilotState6->writeStrict();
    portRadioCommand7 = new BufferedPort<Bottle>();
    portRadioCommand7->setStrict();
    portRadioCommand7->writeStrict();
    portRadioState7Values = new std::vector<float>();
    portRadioState7 = new BufferedPort<Bottle>();
    portRadioState7->setStrict();
    portRadioState7->writeStrict();
    portAutoPilotCommand7 = new BufferedPort<Bottle>();
    portAutoPilotCommand7->setStrict();
    portAutoPilotCommand7->writeStrict();
    portAutoPilotState7Values = new std::vector<float>();
    portAutoPilotState7 = new BufferedPort<Bottle>();
    portAutoPilotState7->setStrict();
    portAutoPilotState7->writeStrict();
    portRadioCommand8 = new BufferedPort<Bottle>();
    portRadioCommand8->setStrict();
    portRadioCommand8->writeStrict();
    portRadioState8Values = new std::vector<float>();
    portRadioState8 = new BufferedPort<Bottle>();
    portRadioState8->setStrict();
    portRadioState8->writeStrict();
    portAutoPilotCommand8 = new BufferedPort<Bottle>();
    portAutoPilotCommand8->setStrict();
    portAutoPilotCommand8->writeStrict();
    portAutoPilotState8Values = new std::vector<float>();
    portAutoPilotState8 = new BufferedPort<Bottle>();
    portAutoPilotState8->setStrict();
    portAutoPilotState8->writeStrict();
    portRadioCommand9 = new BufferedPort<Bottle>();
    portRadioCommand9->setStrict();
    portRadioCommand9->writeStrict();
    portRadioState9Values = new std::vector<float>();
    portRadioState9 = new BufferedPort<Bottle>();
    portRadioState9->setStrict();
    portRadioState9->writeStrict();
    portAutoPilotCommand9 = new BufferedPort<Bottle>();
    portAutoPilotCommand9->setStrict();
    portAutoPilotCommand9->writeStrict();
    portAutoPilotState9Values = new std::vector<float>();
    portAutoPilotState9 = new BufferedPort<Bottle>();
    portAutoPilotState9->setStrict();
    portAutoPilotState9->writeStrict();
  }
  
  ~sim() {
    delete portCommand;
    delete portGroundStationValues;
    delete portGroundStation;
    delete portFromGroundStationValues;
    delete portFromGroundStation;
    delete portRadioCommand0;
    delete portRadioState0Values;
    delete portRadioState0;
    delete portAutoPilotCommand0;
    delete portAutoPilotState0Values;
    delete portAutoPilotState0;
    delete portRadioCommand1;
    delete portRadioState1Values;
    delete portRadioState1;
    delete portAutoPilotCommand1;
    delete portAutoPilotState1Values;
    delete portAutoPilotState1;
    delete portRadioCommand2;
    delete portRadioState2Values;
    delete portRadioState2;
    delete portAutoPilotCommand2;
    delete portAutoPilotState2Values;
    delete portAutoPilotState2;
    delete portRadioCommand3;
    delete portRadioState3Values;
    delete portRadioState3;
    delete portAutoPilotCommand3;
    delete portAutoPilotState3Values;
    delete portAutoPilotState3;
    delete portRadioCommand4;
    delete portRadioState4Values;
    delete portRadioState4;
    delete portAutoPilotCommand4;
    delete portAutoPilotState4Values;
    delete portAutoPilotState4;
    delete portRadioCommand5;
    delete portRadioState5Values;
    delete portRadioState5;
    delete portAutoPilotCommand5;
    delete portAutoPilotState5Values;
    delete portAutoPilotState5;
    delete portRadioCommand6;
    delete portRadioState6Values;
    delete portRadioState6;
    delete portAutoPilotCommand6;
    delete portAutoPilotState6Values;
    delete portAutoPilotState6;
    delete portRadioCommand7;
    delete portRadioState7Values;
    delete portRadioState7;
    delete portAutoPilotCommand7;
    delete portAutoPilotState7Values;
    delete portAutoPilotState7;
    delete portRadioCommand8;
    delete portRadioState8Values;
    delete portRadioState8;
    delete portAutoPilotCommand8;
    delete portAutoPilotState8Values;
    delete portAutoPilotState8;
    delete portRadioCommand9;
    delete portRadioState9Values;
    delete portRadioState9;
    delete portAutoPilotCommand9;
    delete portAutoPilotState9Values;
    delete portAutoPilotState9;
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
      portName << "/sim" << module_id << "/command";
      portCommand->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/groundstation";
      portGroundStation->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/fromgroundstation";
      portFromGroundStation->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand0";
      portRadioCommand0->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate0";
      portRadioState0->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand0";
      portAutoPilotCommand0->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate0";
      portAutoPilotState0->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand1";
      portRadioCommand1->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate1";
      portRadioState1->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand1";
      portAutoPilotCommand1->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate1";
      portAutoPilotState1->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand2";
      portRadioCommand2->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate2";
      portRadioState2->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand2";
      portAutoPilotCommand2->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate2";
      portAutoPilotState2->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand3";
      portRadioCommand3->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate3";
      portRadioState3->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand3";
      portAutoPilotCommand3->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate3";
      portAutoPilotState3->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand4";
      portRadioCommand4->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate4";
      portRadioState4->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand4";
      portAutoPilotCommand4->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate4";
      portAutoPilotState4->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand5";
      portRadioCommand5->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate5";
      portRadioState5->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand5";
      portAutoPilotCommand5->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate5";
      portAutoPilotState5->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand6";
      portRadioCommand6->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate6";
      portRadioState6->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand6";
      portAutoPilotCommand6->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate6";
      portAutoPilotState6->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand7";
      portRadioCommand7->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate7";
      portRadioState7->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand7";
      portAutoPilotCommand7->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate7";
      portAutoPilotState7->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand8";
      portRadioCommand8->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate8";
      portRadioState8->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand8";
      portAutoPilotCommand8->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate8";
      portAutoPilotState8->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiocommand9";
      portRadioCommand9->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/radiostate9";
      portRadioState9->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotcommand9";
      portAutoPilotCommand9->open(portName.str().c_str());
    }
    {
      std::stringstream portName; portName.str(); portName.clear();
      portName << "/sim" << module_id << "/autopilotstate9";
      portAutoPilotState9->open(portName.str().c_str());
    }
  }
  
  // Before destruction you will need to call this function first
  // it closes the YARP ports
  void Close() {
    portCommand->close();
    portGroundStation->close();
    portFromGroundStation->close();
    portRadioCommand0->close();
    portRadioState0->close();
    portAutoPilotCommand0->close();
    portAutoPilotState0->close();
    portRadioCommand1->close();
    portRadioState1->close();
    portAutoPilotCommand1->close();
    portAutoPilotState1->close();
    portRadioCommand2->close();
    portRadioState2->close();
    portAutoPilotCommand2->close();
    portAutoPilotState2->close();
    portRadioCommand3->close();
    portRadioState3->close();
    portAutoPilotCommand3->close();
    portAutoPilotState3->close();
    portRadioCommand4->close();
    portRadioState4->close();
    portAutoPilotCommand4->close();
    portAutoPilotState4->close();
    portRadioCommand5->close();
    portRadioState5->close();
    portAutoPilotCommand5->close();
    portAutoPilotState5->close();
    portRadioCommand6->close();
    portRadioState6->close();
    portAutoPilotCommand6->close();
    portAutoPilotState6->close();
    portRadioCommand7->close();
    portRadioState7->close();
    portAutoPilotCommand7->close();
    portAutoPilotState7->close();
    portRadioCommand8->close();
    portRadioState8->close();
    portAutoPilotCommand8->close();
    portAutoPilotState8->close();
    portRadioCommand9->close();
    portRadioState9->close();
    portAutoPilotCommand9->close();
    portAutoPilotState9->close();
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
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readGroundStation(bool blocking=true) {
    Bottle *b = portGroundStation->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portGroundStationValues->push_back(b->get(i).asDouble());
      }
    }
    return portGroundStationValues;
  }
  
  inline void writeGroundStation(const float_seq &seq) {
    Bottle &seqPrepare = portGroundStation->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portGroundStation->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readFromGroundStation(bool blocking=true) {
    Bottle *b = portFromGroundStation->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portFromGroundStationValues->push_back(b->get(i).asDouble());
      }
    }
    return portFromGroundStationValues;
  }
  
  inline void writeRadioCommand0(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand0->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand0->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState0(bool blocking=true) {
    Bottle *b = portRadioState0->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState0Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState0Values;
  }
  
  inline void writeAutoPilotCommand0(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand0->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand0->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState0(bool blocking=true) {
    Bottle *b = portAutoPilotState0->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState0Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState0Values;
  }
  
  inline void writeRadioCommand1(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand1->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand1->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState1(bool blocking=true) {
    Bottle *b = portRadioState1->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState1Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState1Values;
  }
  
  inline void writeAutoPilotCommand1(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand1->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand1->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState1(bool blocking=true) {
    Bottle *b = portAutoPilotState1->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState1Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState1Values;
  }
  
  inline void writeRadioCommand2(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand2->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand2->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState2(bool blocking=true) {
    Bottle *b = portRadioState2->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState2Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState2Values;
  }
  
  inline void writeAutoPilotCommand2(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand2->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand2->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState2(bool blocking=true) {
    Bottle *b = portAutoPilotState2->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState2Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState2Values;
  }
  
  inline void writeRadioCommand3(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand3->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand3->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState3(bool blocking=true) {
    Bottle *b = portRadioState3->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState3Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState3Values;
  }
  
  inline void writeAutoPilotCommand3(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand3->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand3->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState3(bool blocking=true) {
    Bottle *b = portAutoPilotState3->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState3Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState3Values;
  }
  
  inline void writeRadioCommand4(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand4->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand4->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState4(bool blocking=true) {
    Bottle *b = portRadioState4->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState4Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState4Values;
  }
  
  inline void writeAutoPilotCommand4(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand4->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand4->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState4(bool blocking=true) {
    Bottle *b = portAutoPilotState4->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState4Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState4Values;
  }
  
  inline void writeRadioCommand5(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand5->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand5->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState5(bool blocking=true) {
    Bottle *b = portRadioState5->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState5Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState5Values;
  }
  
  inline void writeAutoPilotCommand5(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand5->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand5->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState5(bool blocking=true) {
    Bottle *b = portAutoPilotState5->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState5Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState5Values;
  }
  
  inline void writeRadioCommand6(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand6->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand6->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState6(bool blocking=true) {
    Bottle *b = portRadioState6->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState6Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState6Values;
  }
  
  inline void writeAutoPilotCommand6(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand6->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand6->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState6(bool blocking=true) {
    Bottle *b = portAutoPilotState6->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState6Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState6Values;
  }
  
  inline void writeRadioCommand7(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand7->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand7->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState7(bool blocking=true) {
    Bottle *b = portRadioState7->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState7Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState7Values;
  }
  
  inline void writeAutoPilotCommand7(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand7->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand7->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState7(bool blocking=true) {
    Bottle *b = portAutoPilotState7->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState7Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState7Values;
  }
  
  inline void writeRadioCommand8(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand8->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand8->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState8(bool blocking=true) {
    Bottle *b = portRadioState8->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState8Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState8Values;
  }
  
  inline void writeAutoPilotCommand8(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand8->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand8->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState8(bool blocking=true) {
    Bottle *b = portAutoPilotState8->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState8Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState8Values;
  }
  
  inline void writeRadioCommand9(const float_seq &seq) {
    Bottle &seqPrepare = portRadioCommand9->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portRadioCommand9->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readRadioState9(bool blocking=true) {
    Bottle *b = portRadioState9->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portRadioState9Values->push_back(b->get(i).asDouble());
      }
    }
    return portRadioState9Values;
  }
  
  inline void writeAutoPilotCommand9(const float_seq &seq) {
    Bottle &seqPrepare = portAutoPilotCommand9->prepare();
    seqPrepare.clear();
    for (int i = 0; i < seq.size(); ++i) {
      seqPrepare.addDouble(seq[i]);
    }
    portAutoPilotCommand9->write(true);
  }
  
  // Remark: caller is responsible for evoking vector.clear()
  inline std::vector<float> *readAutoPilotState9(bool blocking=true) {
    Bottle *b = portAutoPilotState9->read(blocking);
    if (b != NULL) { 
      for (int i = 0; i < b->size(); ++i) {
        portAutoPilotState9Values->push_back(b->get(i).asDouble());
      }
    }
    return portAutoPilotState9Values;
  }
  
};
}

