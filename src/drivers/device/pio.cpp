/**
 * @file pio.cpp
 *
 * Base class for devices accessed via PIO to registers.
 */

#include "device_nuttx.h"

namespace device
{

PIO::PIO(const char *name,
	 const char *devname,
	 uint32_t base,
	 int irq) :
	// base class
	CDev(name, devname, irq),//注册设备类
	// public
	// protected
	// private
	_base(base)
{
}

PIO::~PIO()
{
	// nothing to do here...
}

int
PIO::init()
{
	int ret = OK;

	// base class init first
	ret = CDev::init();//字符设备初始化

	return ret;
}

} // namespace device


