/**
 * @file device.cpp
 *
 * Fundamental driver base class for the device framework.
 */

#include "device_nuttx.h"
#include "px4_log.h"

#include <nuttx/arch.h>
#include <stdio.h>
#include <unistd.h>
//#include <drivers/drv_device.h>
#include "DevIOCTL.h"

namespace device
{
//中断分配表入口
struct irq_entry {
	int	irq;
	Device	*owner; //设备类
};
//中断分配表大小
static const unsigned	irq_nentries = 8;
//中断分配表入口
static irq_entry	irq_entries[irq_nentries];
//注册一个设备中断
static int	register_interrupt(int irq, Device *owner);
//注销一个中断
static void	unregister_interrupt(int irq);
//处理一个中断
static int	interrupt(int irq, void *context);
//设备类的构造函数
Device::Device(const char *name,  //构造函数
	       int irq) :
	// public
	// protected
	_name(name),
	_debug_enabled(false),
	// private
	_irq(irq),
	_irq_attached(false)
{
	sem_init(&_lock, 0, 1);//锁定信号初始化

    /* setup a default device ID. When bus_type is UNKNOWN the 设置一个默认的设备id号
	   other fields are invalid */
	_device_id.devid = 0;
	_device_id.devid_s.bus_type = DeviceBusType_UNKNOWN;
	_device_id.devid_s.bus = 0;
	_device_id.devid_s.address = 0;
	_device_id.devid_s.devtype = 0;
}
//析构函数
Device::~Device()
{
    sem_destroy(&_lock);  //信号销毁

    if (_irq_attached) {//中断绑定
        unregister_interrupt(_irq);//中断注销
	}
}
int
Device::init()
{
	int ret = OK;

	// If assigned an interrupt, connect it
    if (_irq) {//如果注册了一个中断，连接
		/* ensure it's disabled */
        up_disable_irq(_irq);//系统函数，保证中断失能

		/* register */
        ret = register_interrupt(_irq, this);//注册中断

		if (ret != OK) {
			goto out;
		}

        _irq_attached = true;//中断绑定成功
	}

out:
	return ret;
}

void
Device::interrupt_enable()//中断失能
{
	if (_irq_attached) {
        up_enable_irq(_irq);//系统中使能中断
	}
}

void
Device::interrupt_disable()//中断失能
{
    if (_irq_attached) {//如果中断绑定成功
		up_disable_irq(_irq);
	}
}

void
Device::interrupt(void *context)
{
	// default action is to disable the interrupt so we don't get called again
	interrupt_disable();
}

static int
register_interrupt(int irq, Device *owner)//注册中断
{
    int ret = -ENOMEM;//超出内存

    // look for a slot where we can register the interrupt寻找一个我们可以注册的中断位置
    for (unsigned i = 0; i < irq_nentries; i++) {//查找中断表
        if (irq_entries[i].irq == 0) {//如果中断入口没有被注册

			// great, we could put it here; try attaching it
            ret = irq_attach(irq, &interrupt);//尝试绑定

			if (ret == OK) {
                irq_entries[i].irq = irq;//中断号
                irq_entries[i].owner = owner;//拥有者
			}

			break;
		}
	}

	return ret;
}

static void
unregister_interrupt(int irq) //注销中断
{
	for (unsigned i = 0; i < irq_nentries; i++) {
		if (irq_entries[i].irq == irq) {
            irq_entries[i].irq = 0; //清零
			irq_entries[i].owner = nullptr;
		}
	}
}

static int
interrupt(int irq, void *context)
{
	for (unsigned i = 0; i < irq_nentries; i++) {
		if (irq_entries[i].irq == irq) {
			irq_entries[i].owner->interrupt(context);
			break;
		}
	}

	return OK;
}

int
Device::read(unsigned offset, void *data, unsigned count)
{
    return -ENODEV;//返回没有这个设备
}

int
Device::write(unsigned offset, void *data, unsigned count)
{
	return -ENODEV;
}

int
Device::ioctl(unsigned operation, unsigned &arg)
{
	switch (operation) {
	case DEVIOCGDEVICEID:
		return (int)_device_id.devid;
	}

	return -ENODEV;
}

}//namespace device
