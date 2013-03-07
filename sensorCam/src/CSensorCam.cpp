/**
 * @brief
 * @file CSensorCam.cpp
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

// #include <asm/types.h>          // for videodev2.h
// #include <linux/videodev2.h>


#include "CSensorCam.h"
#include "Protocol.h"



#define CLEAR(x) memset(&(x), 0, sizeof(x))

using namespace rur;

CSensorCam::~CSensorCam()
{
    //deinit camera


    /***************  unmap memory  *************/

    unsigned int i;

	for (i = 0; i < numBuffers; ++i)
	if (-1 == munmap(buffers[i].start, buffers[i].length))
		exit(91);

	free(buffers);

    std::cout << "Memory freed" << std::endl;

    /***************  close devices  ************/
    close(fileDescriptorCam);
	fileDescriptorCam = -1;
	close(fileDescriptorSub);
	fileDescriptorSub = -1;

	std::cout << "Devices closed" << std::endl;

}

void CSensorCam::Init(std::string module_id)
{
	sensorCam::Init(module_id);
	config.load("config.json");

	/***************  initialize members  *******/

    fileDescriptorCam = -1;
    fileDescriptorSub = -1;

    ///TODO: make sure we have the right deviceName and also have the pipeline setup correctly! -> ask Bart
    deviceName = ""; // $(media-ctl -e "OMAP3 ISP CCDC output") find out what this resolves to...
    subdeviceName = "/dev/v4l-subdev8";

    framesPerBurst = 8;

	/***************  open devices  *************/

    struct stat st;                            //Hopefully this stuff will never be triggered; otherwise look at capture-burst-raw.c from Bart
	if (-1 == stat(deviceName.c_str(), &st))            //what these errors mean
		exit(11);

	if (!S_ISCHR (st.st_mode))
		exit(12);

	fileDescriptorCam = open(deviceName.c_str(), O_RDWR, 0);
	if (-1 == fileDescriptorCam)
		exit(13);

	// Open subdev
	if (-1 == stat(subdeviceName.c_str(), &st))
		exit(14);

	if (!S_ISCHR (st.st_mode))
		exit(15);

	fileDescriptorSub = open(subdeviceName.c_str(), O_RDWR, 0);
	if (-1 == fileDescriptorSub)
		exit(16);

    std::cout << "Devices opened!" << std::endl;

    /***************  init devices  *************/

    struct v4l2_capability cap;
	if (-1 == xioctl(fileDescriptorCam, VIDIOC_QUERYCAP, &cap))
        exit(17);

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        exit(18);

	// Store minimum and maximum values of controls
	///TODO fix this
	struct v4l2_queryctrl qctrl;
	CLEAR(qctrl);

    int	controlID[CTRL_NUMCONTROLS];  //move this stuff to header
    int	controlMin[CTRL_NUMCONTROLS];
    int	controlMax[CTRL_NUMCONTROLS];

	int i;
	for (i=0; i<CTRL_NUMCONTROLS; i++)
	{
		qctrl.id = controlID[i];
		if (xioctl(fileDescriptorSub, VIDIOC_QUERYCTRL, &qctrl) < 0)
			continue;
		controlMin[i] = qctrl.minimum;
		controlMax[i] = qctrl.maximum;
	}
	//printf("%i exp min=%i max=%i   gain min=%i max=%i\n", cam, controlMin[cam][CTRL_EXPOSURE], controlMax[cam][CTRL_EXPOSURE], controlMin[cam][CTRL_GAIN], controlMax[cam][CTRL_GAIN]);

	// Check MMAP readout method
	if (!(cap.capabilities & V4L2_CAP_STREAMING))
        exit(19);

	// Select video input, video standard and tune here.
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	CLEAR(cropcap);
	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (0 == xioctl(fileDescriptorCam, VIDIOC_CROPCAP, &cropcap))
	{
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fileDescriptorCam, VIDIOC_S_CROP, &crop))
            std::cout << "Warning: s/t wrong with cropping (1)" << std::endl;
	}
	else
	{
		std::cout << "Warning: s/t wrong with cropping (2)" << std::endl;
	}

	// Select pixel format, and frame size
	struct v4l2_format format;

	CLEAR(format);
	format.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width       = 752;
	format.fmt.pix.height      = 480;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	//format[cam].fmt.pix.pixelformat = pixelFormat[cam];
	format.fmt.pix.field       = V4L2_FIELD_INTERLACED; ///is this right? i doubt it..

	// TODO: subdev or dev? [from bart, prolly right this way]
	if (-1 == xioctl(fileDescriptorCam, VIDIOC_S_FMT, &format))
		exit(20);

    if ((format.fmt.pix.bytesperline < format.fmt.pix.width * 2) ||
        (format.fmt.pix.sizeimage < format.fmt.pix.bytesperline * format.fmt.pix.height) ||
        (format.fmt.pix.width != 752) || (format.fmt.pix.height != 480) )
        exit(21); //see section "Buggy driver paranoia" in Barts code


	buffersV4L2 = (v4l2_buffer*) malloc(framesPerBurst * sizeof(struct v4l2_buffer));
	if (buffersV4L2 == NULL)
        exit(22);

    std::cout << "Device initialization done!" << std::endl;

    /***************  init MMAP memory  *********/

	struct v4l2_requestbuffers req;

	CLEAR(req);
	req.count               = framesPerBurst+1;
	req.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory              = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fileDescriptorCam, VIDIOC_REQBUFS, &req))
        exit(23);

	if (req.count < framesPerBurst)
        exit(24);

	buffers = (rur::buffer*) calloc(req.count, sizeof(*buffers));

	if (!buffers)
        exit(25);

	for (numBuffers = 0; numBuffers < req.count; ++numBuffers)
	{
		struct v4l2_buffer buf;
		CLEAR(buf);
		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = numBuffers;

		if (-1 == xioctl(fileDescriptorCam, VIDIOC_QUERYBUF, &buf))
			exit(26);

		buffers[numBuffers].length = buf.length;
		buffers[numBuffers].start =
			mmap(NULL /* start anywhere */,
				buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommended */,
				fileDescriptorCam, buf.m.offset);

		if (MAP_FAILED == buffers[numBuffers].start)
			/* If you do not exit here you should unmap() and free() the buffers mapped so far. */
			exit(27);
	}

}

void CSensorCam::Tick()
{
/*
	int* cmd = readCommand(false);
	if (cmd != NULL)
	{
	}
	*/



	usleep(config.TickTime);
}

int CSensorCam::xioctl(int fd, int request, void *arg)
{
	int r;
	do r = ioctl(fd, request, arg);
	while (-1 == r && EINTR == errno);  //if interrupted system call, try again
	return r;
}
