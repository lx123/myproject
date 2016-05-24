
#include <sys/types.h>
#include <sys/stat.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include "uORBUtils.hpp"
#include "uORBManager.hpp"


//=========================  Static initializations =================
uORB::Manager *uORB::Manager::_Instance = nullptr;  //静态初始化

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
uORB::Manager *uORB::Manager::get_instance()  //获得类
{
	if (_Instance == nullptr) {
		_Instance = new uORB::Manager();  //这个类被划分到私有成员中
	}

	return _Instance;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
uORB::Manager::Manager()
	: _comm_channel(nullptr)
{
}

int uORB::Manager::orb_exists(const struct orb_metadata *meta, int instance)  //检查主题是否存在
{
	/*
	 * Generate the path to the node and try to open it. 获得节点的路径，并尝试打开它
	 */
	char path[orb_maxpath];
	int inst = instance;  //会话编号
	int ret = uORB::Utils::node_mkpath(path, PUBSUB, meta, &inst);  //创建节点路径

	if (ret != OK) {
		errno = -ret;
		return uORB::ERROR;
	}

	struct stat buffer;

	return stat(path, &buffer);
}

orb_advert_t uORB::Manager::orb_advertise(const struct orb_metadata *meta, const void *data)  //发布主题
{
    return orb_advertise_multi(meta, data, nullptr, ORB_PRIO_DEFAULT);  //发布多个主题，第一个为数据指针，第二个为数据，第四个为默认优先级
}

orb_advert_t uORB::Manager::orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
		int priority)
{
	int result, fd;
    orb_advert_t advertiser; //发布者，无返回值，句柄

	/* open the node as an advertiser */
    fd = node_open(PUBSUB, meta, data, true, instance, priority); //打开节点，发布订阅，元数据结构，数据

    if (fd == ERROR) {  //节点打开失败
		return nullptr;
	}

	/* get the advertiser handle and close the node */
    result = ioctl(fd, ORBIOCGADVERTISER, (unsigned long)&advertiser);  //获取发布者句柄，并关闭节点
	close(fd);

	if (result == ERROR) {
		return nullptr;
	}

	/* the advertiser must perform an initial publish to initialise the object */
    result = orb_publish(meta, advertiser, data); //发布主题

	if (result == ERROR) {
		return nullptr;
	}

    return advertiser; //返回发布者句柄
}

int uORB::Manager::orb_subscribe(const struct orb_metadata *meta)  //订阅主题
{
    return node_open(PUBSUB, meta, nullptr, false); //打开节点
}

int uORB::Manager::orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance)
{
	int inst = instance;
	return node_open(PUBSUB, meta, nullptr, false, &inst);
}

int uORB::Manager::orb_unsubscribe(int handle)  //关闭订阅
{
    return close(handle);  //关闭句柄
}

int uORB::Manager::orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data)
{
	return uORB::DeviceNode::publish(meta, handle, data);
}

int uORB::Manager::orb_copy(const struct orb_metadata *meta, int handle, void *buffer)  //获得数据
{
	int ret;

    ret = read(handle, buffer, meta->o_size); //读取 /句柄/缓冲区/大小

    if (ret < 0) { //读取失败
		return ERROR;
	}

    if (ret != (int)meta->o_size) { //大小不同
		errno = EIO;
		return ERROR;
	}

	return OK;
}

int uORB::Manager::orb_check(int handle, bool *updated)
{
    return ioctl(handle, ORBIOCUPDATED, (unsigned long)(uintptr_t)updated);  //检查是否有更新
}

int uORB::Manager::orb_stat(int handle, uint64_t *time)
{
	return ioctl(handle, ORBIOCLASTUPDATE, (unsigned long)(uintptr_t)time);
}

int uORB::Manager::orb_priority(int handle, int32_t *priority)
{
	return ioctl(handle, ORBIOCGPRIORITY, (unsigned long)(uintptr_t)priority);
}

int uORB::Manager::orb_set_interval(int handle, unsigned interval)
{
	return ioctl(handle, ORBIOCSETINTERVAL, interval * 1000);
}


int uORB::Manager::node_advertise
(
    const struct orb_metadata *meta, //元数据结构
    int *instance,   //null
    int priority  //优先级
)
{
	int fd = -1;
	int ret = ERROR;

	/* fill advertiser data */
    const struct orb_advertdata adv = { meta, instance, priority }; //发布者结构体

	/* open the control device */
    fd = open(TOPIC_MASTER_DEVICE_PATH, 0); //打开会话主设备路径文件

    if (fd < 0) {  //文件打开失败
		goto out;
	}

	/* advertise the object */
    ret = ioctl(fd, ORBIOCADVERTISE, (unsigned long)(uintptr_t)&adv);  //发布主题

	/* it's OK if it already exists */
    if ((OK != ret) && (EEXIST == errno)) {  //如果返回OK则已经存在
		ret = OK;
	}

out:

	if (fd >= 0) {
		close(fd);
	}

	return ret;
}

int uORB::Manager::node_open
(
    Flavor f,  //发布订阅
    const struct orb_metadata *meta,  //元数据结构
    const void *data,   //数据
    bool advertiser,   //发布者
    int *instance,     //null
    int priority      //优先级
)
{
    char path[orb_maxpath];  //路径
	int fd, ret;

	/*
	 * If meta is null, the object was not defined, i.e. it is not
	 * known to the system.  We can't advertise/subscribe such a thing.
	 */
    if (nullptr == meta) {  //如果元数据结构没有定义
		errno = ENOENT;
		return ERROR;
	}

	/*
	 * Advertiser must publish an initial value.
	 */
    if (advertiser && (data == nullptr)) {  //发布者必须发布一个原始值
		errno = EINVAL;
		return ERROR;
	}

	/*
	 * Generate the path to the node and try to open it.
	 */
    ret = uORB::Utils::node_mkpath(path, f, meta, instance); //给节点创建一个路径，并尝试打开，路径/发布订阅，元数据结构/

    if (ret != OK) { //打开失败
		errno = -ret;
		return ERROR;
	}

	/* open the path as either the advertiser or the subscriber */
    fd = open(path, (advertiser) ? O_WRONLY : O_RDONLY);  //路径在上一个程序中已经给入，发布者以仅写入的方式

	/* if we want to advertise and the node existed, we have to re-try again */
    if ((fd >= 0) && (instance != nullptr) && (advertiser)) {  //文件打开成功，instance不为0，并且为发布者
		/* close the fd, we want a new one */
        close(fd);//关闭文件，我们需要一个新的
		/* the node_advertise call will automatically go for the next free entry */
		fd = -1;
	}

    /* we may need to advertise the node... */ //我们可能需要发布节点
	if (fd < 0) {

		/* try to create the node */
        ret = node_advertise(meta, instance, priority);  //创建节点，元数据结构，null/优先级

		if (ret == OK) {
			/* update the path, as it might have been updated during the node_advertise call */
			ret = uORB::Utils::node_mkpath(path, f, meta, instance);

			if (ret != OK) {
				errno = -ret;
				return ERROR;
			}
		}

		/* on success, try the open again */
		if (ret == OK) {
			fd = open(path, (advertiser) ? O_WRONLY : O_RDONLY);
		}
	}

	if (fd < 0) {
		errno = EIO;
		return ERROR;
	}

	/* everything has been OK, we can return the handle now */
	return fd;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void uORB::Manager::set_uorb_communicator(uORBCommunicator::IChannel *channel)
{
	_comm_channel = channel;

	if (_comm_channel != nullptr) {
		_comm_channel->register_handler(this);
	}
}

uORBCommunicator::IChannel *uORB::Manager::get_uorb_communicator(void)
{
	return _comm_channel;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_add_subscription(const char *messageName,
		int32_t msgRateInHz)
{
	warnx("[posix-uORB::Manager::process_add_subscription(%d)] entering Manager_process_add_subscription: name: %s",
	      __LINE__, messageName);
	int16_t rc = 0;
	_remote_subscriber_topics.insert(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);  //创建节点

	if (ret == OK) {
		// get the node name.
		uORB::DeviceNode *node = uORB::DeviceMaster::GetDeviceNode(nodepath);

		if (node == nullptr) {
			warnx("[posix-uORB::Manager::process_add_subscription(%d)]DeviceNode(%s) not created yet",
			      __LINE__, messageName);

		} else {
			// node is present.
			node->process_add_subscription(msgRateInHz);
		}

	} else {
		rc = -1;
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_remove_subscription(
	const char *messageName)
{
	warnx("[posix-uORB::Manager::process_remove_subscription(%d)] Enter: name: %s",
	      __LINE__, messageName);
	int16_t rc = -1;
	_remote_subscriber_topics.erase(messageName);
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);

	if (ret == OK) {
		uORB::DeviceNode *node = uORB::DeviceMaster::GetDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			warnx("[posix-uORB::Manager::process_remove_subscription(%d)]Error No existing subscriber found for message: [%s]",
			      __LINE__, messageName);

		} else {
			// node is present.
			node->process_remove_subscription();
			rc = 0;
		}
	}

	return rc;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int16_t uORB::Manager::process_received_message(const char *messageName,
		int32_t length, uint8_t *data)
{
	//warnx("[uORB::Manager::process_received_message(%d)] Enter name: %s", __LINE__, messageName );

	int16_t rc = -1;
	char nodepath[orb_maxpath];
	int ret = uORB::Utils::node_mkpath(nodepath, PUBSUB, messageName);

	if (ret == OK) {
		uORB::DeviceNode *node = uORB::DeviceMaster::GetDeviceNode(nodepath);

		// get the node name.
		if (node == nullptr) {
			warnx("[uORB::Manager::process_received_message(%d)]Error No existing subscriber found for message: [%s] nodepath:[%s]",
			      __LINE__, messageName, nodepath);

		} else {
			// node is present.
			node->process_received_message(length, data);
			rc = 0;
		}
	}

	return rc;
}

bool uORB::Manager::is_remote_subscriber_present(const char *messageName)
{
	return _remote_subscriber_topics.find(messageName);
}



