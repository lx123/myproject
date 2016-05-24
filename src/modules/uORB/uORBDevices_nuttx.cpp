
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <nuttx/arch.h>
#include "uORBDevices_nuttx.hpp"
#include "uORBUtils.hpp"
#include "uORBManager.hpp"
#include "uORBCommunicator.hpp"
#include <stdlib.h>

uORB::ORBMap uORB::DeviceMaster::_node_map;

uORB::DeviceNode::DeviceNode   //设备
(
    const struct orb_metadata *meta,   //元数据数组
    const char *name,    //设备名称
    const char *path,    //路径
    int priority        //优先级
) :
    CDev(name, path),    //创建字符设备
    _meta(meta),         //元数据数组
    _data(nullptr),      //数据
    _last_update(0),     //最后一次更新
    _generation(0),
	_publisher(0),
    _priority(priority),   //优先级
	_published(false),
	_IsRemoteSubscriberPresent(false),
	_subscriber_count(0)
{
	// enable debug() calls
    _debug_enabled = true;   //使能debug调用
}

uORB::DeviceNode::~DeviceNode()
{
	if (_data != nullptr) {  //删除数据
		delete[] _data;
	}

}

int
uORB::DeviceNode::open(struct file *filp)
{
	int ret;

	/* is this a publisher? */  //判断是否是发布者
	if (filp->f_oflags == O_WRONLY) {

		/* become the publisher if we can */
		lock();

		if (_publisher == 0) {
			_publisher = getpid();
			ret = OK;

		} else {
			ret = -EBUSY;
		}

		unlock();

		/* now complete the open */
		if (ret == OK) {
			ret = CDev::open(filp);

			/* open failed - not the publisher anymore */
			if (ret != OK) {
				_publisher = 0;
			}
		}

		return ret;
	}

	/* is this a new subscriber? */  //判断是否为新的订阅者
	if (filp->f_oflags == O_RDONLY) {

		/* allocate subscriber data */
		SubscriberData *sd = new SubscriberData;

		if (nullptr == sd) {
			return -ENOMEM;
		}

		memset(sd, 0, sizeof(*sd));

		/* default to no pending update */
		sd->generation = _generation;

		/* set priority */
		sd->priority = _priority;

		filp->f_priv = (void *)sd;

		ret = CDev::open(filp);

		add_internal_subscriber();

		if (ret != OK) {
			delete sd;
		}

		return ret;
	}

	/* can only be pub or sub, not both */
	return -EINVAL;
}

int
uORB::DeviceNode::close(struct file *filp)
{
	/* is this the publisher closing? */
	if (getpid() == _publisher) {
		_publisher = 0;

	} else {
		SubscriberData *sd = filp_to_sd(filp);

		if (sd != nullptr) {
			hrt_cancel(&sd->update_call);
			remove_internal_subscriber();
			delete sd;
			sd = nullptr;
		}
	}

	return CDev::close(filp);
}

ssize_t
uORB::DeviceNode::read(struct file *filp, char *buffer, size_t buflen)  //节点读取，文件句柄/缓冲区/大小
{
	SubscriberData *sd = (SubscriberData *)filp_to_sd(filp);

	/* if the object has not been written yet, return zero */
	if (_data == nullptr) {
		return 0;
	}

	/* if the caller's buffer is the wrong size, that's an error */
	if (buflen != _meta->o_size) {  //长度错误
		return -EIO;
	}

	/*
	 * Perform an atomic copy & state update
	 */
	irqstate_t flags = up_irq_save();//保存中断

	/* if the caller doesn't want the data, don't give it to them */
	if (nullptr != buffer) {
		memcpy(buffer, _data, _meta->o_size);
	}

	/* track the last generation that the file has seen */
	sd->generation = _generation;

	/* set priority */
	sd->priority = _priority;

	/*
	 * Clear the flag that indicates that an update has been reported, as
	 * we have just collected it.
	 */
	sd->update_reported = false;

	up_irq_restore(flags);

	return _meta->o_size;
}

ssize_t
uORB::DeviceNode::write(struct file *filp, const char *buffer, size_t buflen)  //文件指针null，数据，数据长度
{
	/*
	 * Writes are legal from interrupt context as long as the
	 * object has already been initialised from thread context.
	 *
	 * Writes outside interrupt context will allocate the object
	 * if it has not yet been allocated.
	 *
	 * Note that filp will usually be NULL.
	 */
	if (nullptr == _data) {
        if (!up_interrupt_context()) {  //判断是否在中断环境下

            lock();  //锁定

			/* re-check size */
            if (nullptr == _data) { //再次检查
                _data = new uint8_t[_meta->o_size];  //获得数据大小
			}

			unlock();
		}

		/* failed or could not allocate */
        if (nullptr == _data) { //不能调用内存
			return -ENOMEM;
		}
	}

	/* If write size does not match, that is an error */
    if (_meta->o_size != buflen) {  //与数据长度不符合
		return -EIO;
	}

	/* Perform an atomic copy. */
	irqstate_t flags = up_irq_save();
	memcpy(_data, buffer, _meta->o_size);
	up_irq_restore(flags);

	/* update the timestamp and generation count */
    _last_update = hrt_absolute_time();  //获取当前绝对时间
	_generation++;

	/* notify any poll waiters */
    poll_notify(POLLIN);  //发布新的轮询通知

	_published = true;

	return _meta->o_size;
}

int
uORB::DeviceNode::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	SubscriberData *sd = filp_to_sd(filp);

	switch (cmd) {
	case ORBIOCLASTUPDATE:  //时间更新
		*(hrt_abstime *)arg = _last_update;
		return OK;

	case ORBIOCUPDATED:
		*(bool *)arg = appears_updated(sd);  //检查更新
		return OK;

	case ORBIOCSETINTERVAL:  //设置时间间隔
		sd->update_interval = arg;
		return OK;

	case ORBIOCGADVERTISER:  //获取发布者
		*(uintptr_t *)arg = (uintptr_t)this;
		return OK;

	case ORBIOCGPRIORITY:   //获取优先级
		*(int *)arg = sd->priority;
		return OK;

	default:
		/* give it to the superclass */  //给到一个超类
		return CDev::ioctl(filp, cmd, arg);
	}
}

ssize_t
uORB::DeviceNode::publish
(
    const orb_metadata *meta,  //元数据结构指针
	orb_advert_t handle,
    const void *data    //数据
)
{
    uORB::DeviceNode *DeviceNode = (uORB::DeviceNode *)handle;  //打开设备节点类
	int ret;

	/* this is a bit risky, since we are trusting the handle in order to deref it */
    if (DeviceNode->_meta != meta) {  //设备节点的类结构不同
		errno = EINVAL;
		return ERROR;
	}

    /* call the DeviceNode write method with no file pointer */  //调用设备节点类写入方法，没有文件指针
    ret = DeviceNode->write(nullptr, (const char *)data, meta->o_size); //写入

    if (ret < 0) {  //写入失败
		return ERROR;
	}

    if (ret != (int)meta->o_size) { //写入的数据大小不同
		errno = EIO;
		return ERROR;
	}

	/*
     * if the write is successful, send the data over the Multi-ORB link如果写入成功，发送数据到多个ORB连接
	 */
    uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr) {
		if (ch->send_message(meta->o_name, meta->o_size, (uint8_t *)data) != 0) {
			warnx("[uORB::DeviceNode::publish(%d)]: Error Sending [%s] topic data over comm_channel",
			      __LINE__, meta->o_name);
			return ERROR;
		}
	}

	return OK;
}

pollevent_t
uORB::DeviceNode::poll_state(struct file *filp)
{
	SubscriberData *sd = filp_to_sd(filp);

	/*
	 * If the topic appears updated to the subscriber, say so.
	 */
	if (appears_updated(sd)) {
		return POLLIN;
	}

	return 0;
}

void
uORB::DeviceNode::poll_notify_one(struct pollfd *fds, pollevent_t events)
{
	SubscriberData *sd = filp_to_sd((struct file *)fds->priv);

	/*
	 * If the topic looks updated to the subscriber, go ahead and notify them.
	 */
	if (appears_updated(sd)) {
		CDev::poll_notify_one(fds, events);
	}
}

bool
uORB::DeviceNode::appears_updated(SubscriberData *sd)
{
	/* assume it doesn't look updated */
	bool ret = false;

	/* avoid racing between interrupt and non-interrupt context calls */
	irqstate_t state = up_irq_save();

	/* check if this topic has been published yet, if not bail out */
	if (_data == nullptr) {
		ret = false;
		goto out;
	}

	/*
	 * If the subscriber's generation count matches the update generation
	 * count, there has been no update from their perspective; if they
	 * don't match then we might have a visible update.
	 */
	while (sd->generation != _generation) {

		/*
		 * Handle non-rate-limited subscribers.
		 */
		if (sd->update_interval == 0) {
			ret = true;
			break;
		}

		/*
		 * If we have previously told the subscriber that there is data,
		 * and they have not yet collected it, continue to tell them
		 * that there has been an update.  This mimics the non-rate-limited
		 * behaviour where checking / polling continues to report an update
		 * until the topic is read.
		 */
		if (sd->update_reported) {
			ret = true;
			break;
		}

		/*
		 * If the interval timer is still running, the topic should not
		 * appear updated, even though at this point we know that it has.
		 * We have previously been through here, so the subscriber
		 * must have collected the update we reported, otherwise
		 * update_reported would still be true.
		 */
		if (!hrt_called(&sd->update_call)) {
			break;
		}

		/*
		 * Make sure that we don't consider the topic to be updated again
		 * until the interval has passed once more by restarting the interval
		 * timer and thereby re-scheduling a poll notification at that time.
		 */
		hrt_call_after(&sd->update_call,
			       sd->update_interval,
			       &uORB::DeviceNode::update_deferred_trampoline,
			       (void *)this);

		/*
		 * Remember that we have told the subscriber that there is data.
		 */
		sd->update_reported = true;
		ret = true;

		break;
	}

out:
	up_irq_restore(state);

	/* consider it updated */
	return ret;
}

void
uORB::DeviceNode::update_deferred()
{
	/*
	 * Instigate a poll notification; any subscribers whose intervals have
	 * expired will be woken.
	 */
	poll_notify(POLLIN);
}

void
uORB::DeviceNode::update_deferred_trampoline(void *arg)
{
	uORB::DeviceNode *node = (uORB::DeviceNode *)arg;

	node->update_deferred();
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::add_internal_subscriber()
{
	_subscriber_count++;
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count > 0) {
		ch->add_subscription(_meta->o_name, 1);
	}
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::DeviceNode::remove_internal_subscriber()
{
	_subscriber_count--;
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (ch != nullptr && _subscriber_count == 0) {
		ch->remove_subscription(_meta->o_name);
	}
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
bool uORB::DeviceNode::is_published()
{
	return _published;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_add_subscription(int32_t rateInHz)
{
	// if there is already data in the node, send this out to
	// the remote entity.
	// send the data to the remote entity.
	uORBCommunicator::IChannel *ch = uORB::Manager::get_instance()->get_uorb_communicator();

	if (_data != nullptr && ch != nullptr) { // _data will not be null if there is a publisher.
		ch->send_message(_meta->o_name, _meta->o_size, _data);
	}

	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_remove_subscription()
{
	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::DeviceNode::process_received_message(int32_t length, uint8_t *data)
{
	int16_t ret = -1;

	if (length != (int32_t)(_meta->o_size)) {
		warnx("[uORB::DeviceNode::process_received_message(%d)]Error:[%s] Received DataLength[%d] != ExpectedLen[%d]",
		      __LINE__, _meta->o_name, (int)length, (int)_meta->o_size);
		return ERROR;
	}

	/* call the devnode write method with no file pointer */
	ret = write(nullptr, (const char *)data, _meta->o_size);

	if (ret < 0) {
		return ERROR;
	}

	if (ret != (int)_meta->o_size) {
		errno = EIO;
		return ERROR;
	}

	return OK;
}

uORB::DeviceMaster::DeviceMaster(Flavor f) :
	CDev((f == PUBSUB) ? "obj_master" : "param_master",
         (f == PUBSUB) ? TOPIC_MASTER_DEVICE_PATH : PARAM_MASTER_DEVICE_PATH), //注册设备
	_flavor(f)
{
	// enable debug() calls
    _debug_enabled = true;  //失能debug调用

}

uORB::DeviceMaster::~DeviceMaster()
{
}

int
uORB::DeviceMaster::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
    case ORBIOCADVERTISE: {  //orb io控制发布主题
            const struct orb_advertdata *adv = (const struct orb_advertdata *)arg;  //存入发布者结构体
            const struct orb_metadata *meta = adv->meta;  //元数据指针
			const char *objname;
			const char *devpath;
			char nodepath[orb_maxpath];
			uORB::DeviceNode *node;

			/* set instance to zero - we could allow selective multi-pubs later based on value */
            if (adv->instance != nullptr) {  //设置instance=0；我们允许选择多个发布者基于有效
				*(adv->instance) = 0;
			}

			/* construct a path to the node - this also checks the node name */
            ret = uORB::Utils::node_mkpath(nodepath, _flavor, meta, adv->instance); //创建一个节点的路径，同时检查节点名称

            if (ret != OK) { //节点返回失败
				return ret;
			}

			/* ensure that only one advertiser runs through this critical section */
            lock(); //确保只有一个发布者运行这个关键的部分

			ret = ERROR;

            /* try for topic groups */ //尝试会话组
            const unsigned max_group_tries = (adv->instance != nullptr) ? ORB_MULTI_MAX_INSTANCES : 1; //会话数量设置为1
			unsigned group_tries = 0;

			do {
				/* if path is modifyable change try index */
				if (adv->instance != nullptr) {
					/* replace the number at the end of the string */
					nodepath[strlen(nodepath) - 1] = '0' + group_tries;
					*(adv->instance) = group_tries;
				}

				/* driver wants a permanent copy of the node name, so make one here */
                objname = strdup(meta->o_name);  //获取设备名称

                if (objname == nullptr) {  //如果对象名为空
					return -ENOMEM;
				}

				/* driver wants a permanent copy of the path, so make one here */
                devpath = strdup(nodepath);  //设备路径

                if (devpath == nullptr) { //设备路径为空
					return -ENOMEM;
				}

				/* construct the new node */
                node = new uORB::DeviceNode(meta, objname, devpath, adv->priority);  //创建一个节点类

				/* if we didn't get a device, that's bad */
                if (node == nullptr) {  //节点为空
					unlock();
					return -ENOMEM;
				}

				/* initialise the node - this may fail if e.g. a node with this name already exists */
                ret = node->init();  //节点初始化，字符设备初始化

                if (ret != OK) { //初始化失败
                    /* if init failed, discard the node */  //初始化失败取消节点
					delete node;

					if (ret == -EEXIST) {
						/* if the node exists already, get the existing one and check if
						 * something has been published yet. */
						uORB::DeviceNode *existing_node = GetDeviceNode(devpath);

						if ((existing_node != nullptr) && !(existing_node->is_published())) {
							/* nothing has been published yet, lets claim it */
							ret = OK;

						} else {
							/* otherwise: data has already been published, keep looking */
						}
					}

					/* also discard the name now */
                    free((void *)objname);  //释放对象名称
                    free((void *)devpath);  //释放设备路径

				} else {
					// add to the node map;.
                    _node_map.insert(nodepath, node); //添加节点到map，节点路径，节点类
				}

				group_tries++;

            } while (ret != OK && (group_tries < max_group_tries)); //最大组数量尝试

            if (group_tries > max_group_tries) {  //如果尝试数量大于最大组数，表示错误
				ret = -ENOMEM;
			}

			/* the file handle for the driver has been created, unlock */
            unlock();  //解锁

			return ret;
		}

	default:
		/* give it to the superclass */
		return CDev::ioctl(filp, cmd, arg);
	}
}

uORB::DeviceNode *uORB::DeviceMaster::GetDeviceNode(const char *nodepath)
{
	uORB::DeviceNode *rc = nullptr;

	if (_node_map.find(nodepath)) {
		rc = _node_map.get(nodepath);
	}

	return rc;
}


