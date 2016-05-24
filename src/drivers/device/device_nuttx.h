/**
 * @file device_nuttx.h
 *
 * Definitions for the generic base classes in the device framework.
 */

#ifndef _DEVICE_DEVICE_H
#define _DEVICE_DEVICE_H

#include <nuttx/config.h>
#include <px4_posix.h>  //像posix功能一样的虚拟字符设备

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <poll.h>

#include <nuttx/fs/fs.h>

//跟日志相关
#define DEVICE_LOG(FMT, ...) PX4_LOG_NAMED(_name, FMT, ##__VA_ARGS__)
#define DEVICE_DEBUG(FMT, ...) PX4_LOG_NAMED_COND(_name, _debug_enabled, FMT, ##__VA_ARGS__)

//名词空间封装所有设备框架类，功能和数据
namespace device __EXPORT
{
typedef struct file file_t;//系统级定义

//所有设备驱动的基类
class __EXPORT Device
{
public:
	//析构函数
		virtual ~Device();
	//中断处理
		virtual void interrupt(void *ctx);
	//直接访问方法
		virtual int init();
	//直接从设备读取
		virtual int read(unsigned address,void *data,unsigned count);
	//直接设备写入
		virtual int	write(unsigned address, void *data, unsigned count);
	//io控制
		virtual int	ioctl(unsigned operation, unsigned &arg);
	//设备总线类型
		enum DeviceBusType {
			DeviceBusType_UNKNOWN = 0,
			DeviceBusType_I2C     = 1,
			DeviceBusType_SPI     = 2,
			DeviceBusType_UAVCAN  = 3,
		};
		//设备结构
		struct DeviceStructure {
			enum DeviceBusType bus_type : 3;
				uint8_t bus: 5;    // which instance of the bus type
				uint8_t address;   // address on the bus (eg. I2C address)
				uint8_t devtype;   // device class specific device type
			};
		union DeviceId {
		struct DeviceStructure devid_s;
		uint32_t devid;
	};
protected:
	    const char	*_name;			/**< driver name *///驱动名称
	    bool		_debug_enabled;		/**< if true, debug messages are printed */ //如果为真，debug信息被打印
	    union DeviceId	_device_id;             /**< device identifier information */  //设备id信息
	//构造函数，第一个形参为设备名，第二个为中断分配
		Device(const char *name,int irq = 0);
	//中断使能
		void		interrupt_enable();
	//中断失能
		void		interrupt_disable();
	//驱动锁定
		void		lock()
		{
			do {} while (sem_wait(&_lock) != 0);
		}
	//解除驱动锁定
		void		unlock()
		{
			sem_post(&_lock);
		}
private:
		int		_irq;//中断号
		bool	_irq_attached;//中断绑定
		sem_t	_lock;//驱动锁定信号
	    /** disable copy construction for this and all subclasses *///禁止复制指令
		Device(const Device &);
	    /** disable assignment for this and all subclasses */ //禁止分配
		Device &operator = (const Device &);
		//注册中断处理句柄
		int		dev_register_interrupt(int irq);
		//注销中断处理句柄
		void		dev_unregister_interrupt();
		//中断调度
		static void	dev_interrupt(int irq, void *context);
};

//字符设备抽象类
class __EXPORT CDev : public Device
{
public:
	//构造函数
	CDev(const char *name, const char *devname, int irq = 0);
	//析构函数
	virtual ~CDev();
	virtual int	init();
	//打开设备
	virtual int	open(file_t *filp);
	//关闭一个设备
	virtual int	close(file_t *filp);
	//读取设备
	virtual ssize_t	read(file_t *filp, char *buffer, size_t buflen);
	//写入设备
	virtual ssize_t	write(file_t *filp, const char *buffer, size_t buflen);
	//逻辑寻找
	virtual off_t	seek(file_t *filp, off_t offset, int whence);
	//io控制
	virtual int	ioctl(file_t *filp, int cmd, unsigned long arg);
	//轮询设置和撤销
	virtual int	poll(file_t *filp, struct pollfd *fds, bool setup);
	//测试设备当前是否打开
	bool            is_open() { return _open_count > 0; }
protected:
	//字符设备文件操作表
	static const struct file_operations	fops;
	//检查当前设备轮询时间状态，返回值为当前设置的轮询事件
	virtual pollevent_t poll_state(file_t *filp);
	//发布新的轮询事件
	virtual void	poll_notify(pollevent_t events);
	//轮询发布内部处理
	virtual void	poll_notify_one(px4_pollfd_struct_t *fds, pollevent_t events);
	//第一次打开设备通知
	virtual int	open_first(file_t *filp);
	//最后关闭设备通知
	virtual int	close_last(file_t *filp);
	//注册设备类名字
	virtual int register_class_devname(const char *class_devname);
	//注销设备类名字
	virtual int unregister_class_devname(const char *class_devname, unsigned class_instance);
	//获取设备名字
	const char	*get_devname() { return _devname; }
	bool		_pub_blocked;		/**< true if publishing should be blocked */
private:
	 static const unsigned _max_pollwaiters = 8;//最大轮询等待数
		const char	*_devname;		/**< device node name */
		bool		_registered;		/**< true if device name was registered */
		unsigned	_open_count;		/**< number of successful opens */
		struct pollfd	*_pollset[_max_pollwaiters];//轮询文件设置
		//保存轮询等待
		int		store_poll_waiter(px4_pollfd_struct_t *fds);
		//移除一个轮询等待者
		int		remove_poll_waiter(struct pollfd *fds);
		/* do not allow copying this class */
		CDev(const CDev &);
		CDev operator=(const CDev &);
};

class __EXPORT PIO : public CDev
{
public:
	PIO(const char *name,
	    const char *devname,
	    uint32_t base,
	    int irq = 0);
	virtual ~PIO();

	virtual int	init();
protected:
	//读取一个寄存器
    uint32_t	reg(uint32_t offset)//读取一个寄存器，基地址+偏移地址
	{
		return *(volatile uint32_t *)(_base + offset);
	}
    //写入一个寄存器
	void		reg(uint32_t offset, uint32_t value)
	{
		*(volatile uint32_t *)(_base + offset) = value;
	}
	//修改一个寄存器
	void		modify(uint32_t offset, uint32_t clearbits, uint32_t setbits)
	{
		uint32_t val = reg(offset);
		val &= ~clearbits;
		val |= setbits;
		reg(offset, val);
	}
private:
	uint32_t	_base;
};

}//namespace device
// class instance for primary driver of each class
enum CLASS_DEVICE {
	CLASS_DEVICE_PRIMARY = 0,
	CLASS_DEVICE_SECONDARY = 1,
	CLASS_DEVICE_TERTIARY = 2
};

#endif /* _DEVICE_DEVICE_H */
