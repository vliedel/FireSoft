/**
 * @brief
 * @file CSensorCam.h
 *
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
 * Copyright © 2012 Bart van Vliet <bart@almende.com>
 *
 * @author        Bart van Vliet
 * @date          Jul 25, 2012
 * @project       FireSwarm
 * @company       Distributed Organisms B.V.
 * @case          Swarm robots
 */

#ifndef CSENSORCAM_H_
#define CSENSORCAM_H_

#include "sensorCam.h"
#include "StructToFromCont.h"
#include "Defs.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <errno.h>

#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

namespace rur {

struct SensorCamConfig
{
	long TickTime;
	bool Debug;

	void load(const std::string &filename)
	{
		boost::property_tree::ptree pt;
		read_json(filename, pt);
		TickTime = pt.get<long>("sensorCam.TickTime");
		Debug = pt.get<bool>("sensorCam.Debug");
	}
};

struct buffer {
	void *	start;
	size_t	length;
};

enum controlEnum {
	CTRL_BACKLIGHT,
	CTRL_BRIGHTNESS,
	CTRL_CONTRAST,
	CTRL_EXPOSURE,
	CTRL_GAIN,
	CTRL_GAMMA,
	CTRL_POWERLINE,
	CTRL_SATURATION,
	CTRL_SHARPNESS,
	CTRL_WHITEBALANCE,
	CTRL_AUTO_GAIN,
	CTRL_AUTO_EXPOSURE, // Control that only works for the mt9v032
	CTRL_NUMCONTROLS
};

class CSensorCam : public sensorCam
{
	public:
		SensorCamConfig config;
		~CSensorCam();
		void Init(std::string module_id);
		void Tick();

    private:
        int fileDescriptorCam;
        int fileDescriptorSub;
        int xioctl(int fd, int request, void *arg);

        std::string deviceName;
        std::string subdeviceName;

        int framesPerBurst;
        unsigned int numBuffers;
        struct buffer* buffers;
        struct v4l2_buffer* buffersV4L2;
};

}
#endif // CSENSORCAM_H_
