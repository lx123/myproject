
#ifndef _uORBManager_hpp_
#define _uORBManager_hpp_

#include "uORBCommon.hpp"
#include "uORBDevices_nuttx.hpp"
#include <stdint.h>
#ifdef __PX4_NUTTX
#include "ORBSet.hpp"
#else
#include <string>
#include <set>
#define ORBSet std::set<std::string>
#endif

#include "uORBCommunicator.hpp"

namespace uORB
{
class Manager;  //会话管理类
}

/**
 * This is implemented as a singleton.  This class manages creating the
 * uORB nodes for each uORB topics and also implements the behavor of the
 * uORB Api's.
 */
class uORB::Manager  : public uORBCommunicator::IChannelRxHandler
{
public:
	// public interfaces for this class.

	/**
	 * Method to get the singleton instance for the uORB::Manager.  //获得单个的类的方法
	 * @return uORB::Manager*
	 */
	static uORB::Manager *get_instance();  //返回一个Manager类

	// ==== uORB interface methods ====  uORB接口方法
	/**
	 * Advertise as the publisher of a topic. 发布一个发布者主题
	 *
	 * This performs the initial advertisement of a topic; it creates the topic
	 * node in /obj if required and publishes the initial data.执行初始发公告主题，创建一个会话节点在 /orb下如果需要，并且发布初始数据
	 *
	 * Any number of advertisers may publish to a topic; publications are atomic
	 * but co-ordination between publishers is not provided by the ORB.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param data    A pointer to the initial data to be published.
	 *      For topics updated by interrupt handlers, the advertisement
	 *      must be performed from non-interrupt context.
	 * @return    nullptr on error, otherwise returns an object pointer
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return nullptr and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise(const struct orb_metadata *meta, const void *data); //发布数据

	/**
	 * Advertise as the publisher of a topic.  发布一个主题
	 *
	 * This performs the initial advertisement of a topic; it creates the topic
	 * node in /obj if required and publishes the initial data.
	 *
	 * Any number of advertisers may publish to a topic; publications are atomic 任意数量的会话会发布一个主题
	 * but co-ordination between publishers is not provided by the ORB.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param data    A pointer to the initial data to be published.
	 *      For topics updated by interrupt handlers, the advertisement
	 *      must be performed from non-interrupt context.
	 * @param instance  Pointer to an integer which will yield the instance ID (0-based)
	 *      of the publication.
	 * @param priority  The priority of the instance. If a subscriber subscribes multiple
	 *      instances, the priority allows the subscriber to prioritize the best
	 *      data source as long as its available.
	 * @return    ERROR on error, otherwise returns a handle
	 *      that can be used to publish to the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE with no corresponding ORB_DECLARE)
	 *      this function will return -1 and set errno to ENOENT.
	 */
	orb_advert_t orb_advertise_multi(const struct orb_metadata *meta, const void *data, int *instance,
					 int priority) ;


	/**
	 * Publish new data to a topic.  向一个主题发布新的数据
	 *
	 * The data is atomically published to the topic and any waiting subscribers
	 * will be notified.  Subscribers that are not waiting can check the topic
	 * for updates using orb_check and/or orb_stat.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @handle    The handle returned from orb_advertise.
	 * @param data    A pointer to the data to be published.
	 * @return    OK on success, ERROR otherwise with errno set accordingly.
	 */
	int  orb_publish(const struct orb_metadata *meta, orb_advert_t handle, const void *data) ;

	/**
	 * Subscribe to a topic.  订阅一个主题
	 *
	 * The returned value is a file descriptor that can be passed to poll()
	 * in order to wait for updates to a topic, as well as topic_read,
	 * orb_check and orb_stat.
	 *
	 * Subscription will succeed even if the topic has not been advertised;
	 * in this case the topic will have a timestamp of zero, it will never
	 * signal a poll() event, checking will always return false and it cannot
	 * be copied. When the topic is subsequently advertised, poll, check,
	 * stat and copy calls will react to the initial publication that is
	 * performed as part of the advertisement.
	 *
	 * Subscription will fail if the topic is not known to the system, i.e.
	 * there is nothing in the system that has declared the topic and thus it
	 * can never be published.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @return    ERROR on error, otherwise returns a handle
	 *      that can be used to read and update the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
	 *      this function will return -1 and set errno to ENOENT.
	 */
	int  orb_subscribe(const struct orb_metadata *meta) ;  //订阅一个主题

	/**
	 * Subscribe to a multi-instance of a topic.  订阅一个多主题会话
	 *
	 * The returned value is a file descriptor that can be passed to poll()
	 * in order to wait for updates to a topic, as well as topic_read,
	 * orb_check and orb_stat.
	 *
	 * Subscription will succeed even if the topic has not been advertised;
	 * in this case the topic will have a timestamp of zero, it will never
	 * signal a poll() event, checking will always return false and it cannot
	 * be copied. When the topic is subsequently advertised, poll, check,
	 * stat and copy calls will react to the initial publication that is
	 * performed as part of the advertisement.
	 *
	 * Subscription will fail if the topic is not known to the system, i.e.
	 * there is nothing in the system that has declared the topic and thus it
	 * can never be published.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param instance  The instance of the topic. Instance 0 matches the
	 *      topic of the orb_subscribe() call, higher indices
	 *      are for topics created with orb_publish_multi().
	 * @return    ERROR on error, otherwise returns a handle
	 *      that can be used to read and update the topic.
	 *      If the topic in question is not known (due to an
	 *      ORB_DEFINE_OPTIONAL with no corresponding ORB_DECLARE)
	 *      this function will return -1 and set errno to ENOENT.
	 */
	int  orb_subscribe_multi(const struct orb_metadata *meta, unsigned instance) ;

	/**
	 * Unsubscribe from a topic.  取消订阅一个主题
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @return    OK on success, ERROR otherwise with errno set accordingly.
	 */
	int  orb_unsubscribe(int handle) ;

	/**
	 * Fetch data from a topic.
	 *
	 * This is the only operation that will reset the internal marker that
	 * indicates that a topic has been updated for a subscriber. Once poll
	 * or check return indicating that an updaet is available, this call
	 * must be used to update the subscription.
	 *
	 * @param meta    The uORB metadata (usually from the ORB_ID() macro)
	 *      for the topic.
	 * @param handle  A handle returned from orb_subscribe.
	 * @param buffer  Pointer to the buffer receiving the data, or NULL
	 *      if the caller wants to clear the updated flag without
	 *      using the data.
	 * @return    OK on success, ERROR otherwise with errno set accordingly.
	 */
	int  orb_copy(const struct orb_metadata *meta, int handle, void *buffer) ;  //从主题中获得数据

	/**
	 * Check whether a topic has been published to since the last orb_copy.
	 *
	 * This check can be used to determine whether to copy the topic when
	 * not using poll(), or to avoid the overhead of calling poll() when the
	 * topic is likely to have updated.
	 *
	 * Updates are tracked on a per-handle basis; this call will continue to
	 * return true until orb_copy is called using the same handle. This interface
	 * should be preferred over calling orb_stat due to the race window between
	 * stat and copy that can lead to missed updates.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param updated Set to true if the topic has been updated since the
	 *      last time it was copied using this handle.
	 * @return    OK if the check was successful, ERROR otherwise with
	 *      errno set accordingly.
	 */
	int  orb_check(int handle, bool *updated) ;  //检查一个主题是否更新

	/**
	 * Return the last time that the topic was updated.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param time    Returns the absolute time that the topic was updated, or zero if it has
	 *      never been updated. Time is measured in microseconds.
	 * @return    OK on success, ERROR otherwise with errno set accordingly.
	 */
	int  orb_stat(int handle, uint64_t *time) ;  //返回最后一次更新的时间

	/**
	 * Check if a topic has already been created.
	 *
	 * @param meta    ORB topic metadata.
	 * @param instance  ORB instance
	 * @return    OK if the topic exists, ERROR otherwise with errno set accordingly.
	 */
	int  orb_exists(const struct orb_metadata *meta, int instance) ;  //检查一个主题是否被创建

	/**
	 * Return the priority of the topic
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param priority  Returns the priority of this topic. This is only relevant for
	 *      topics which are published by multiple publishers (e.g. mag0, mag1, etc.)
	 *      and allows a subscriber to automatically pick the topic with the highest
	 *      priority, independent of the startup order of the associated publishers.
	 * @return    OK on success, ERROR otherwise with errno set accordingly.
	 */
	int  orb_priority(int handle, int32_t *priority) ;  //返回主题的优先级

	/**
	 * Set the minimum interval between which updates are seen for a subscription.
	 *
	 * If this interval is set, the subscriber will not see more than one update
	 * within the period.
	 *
	 * Specifically, the first time an update is reported to the subscriber a timer
	 * is started. The update will continue to be reported via poll and orb_check, but
	 * once fetched via orb_copy another update will not be reported until the timer
	 * expires.
	 *
	 * This feature can be used to pace a subscriber that is watching a topic that
	 * would otherwise update too quickly.
	 *
	 * @param handle  A handle returned from orb_subscribe.
	 * @param interval  An interval period in milliseconds.
	 * @return    OK on success, ERROR otherwise with ERRNO set accordingly.
	 */
	int  orb_set_interval(int handle, unsigned interval) ;  //设置最小订阅间隔，如果设置了最小间隔，订阅者将不会看到在一个周期内多余一次的更新

	/**
	 * Method to set the uORBCommunicator::IChannel instance.
	 * @param comm_channel
	 *  The IChannel instance to talk to remote proxies.
	 * @note:
	 *  Currently this call only supports the use of one IChannel
	 *  Future extensions may include more than one IChannel's.
	 */
	void set_uorb_communicator(uORBCommunicator::IChannel *comm_channel);

	/**
	 * Gets the uORB Communicator instance.
	 */
	uORBCommunicator::IChannel *get_uorb_communicator(void);  //获得会话实例

	/**
	 * Utility method to check if there is a remote subscriber present
	 * for a given topic
	 */
	bool is_remote_subscriber_present(const char *messageName);

private: // class methods 类的方法
	/**
	 * Advertise a node; don't consider it an error if the node has
	 * already been advertised.
	 *
	 * @todo verify that the existing node is the same as the one
	 *       we tried to advertise.
	 */
	int
	node_advertise  //节点发布
	(
		const struct orb_metadata *meta,
		int *instance = nullptr,
		int priority = ORB_PRIO_DEFAULT
	);

	/**
	 * Common implementation for orb_advertise and orb_subscribe.
	 *
	 * Handles creation of the object and the initial publication for
	 * advertisers.
	 */
	int
	node_open
	(
		Flavor f,
		const struct orb_metadata *meta,
		const void *data,
		bool advertiser,
		int *instance = nullptr,
		int priority = ORB_PRIO_DEFAULT
	);

private: // data members  数据成员
	static Manager *_Instance;
	// the communicator channel instance.
	uORBCommunicator::IChannel *_comm_channel;
	ORBSet _remote_subscriber_topics;

private: //class methods  类的方法
	Manager();  //构造函数

	/**
	   * Interface to process a received AddSubscription from remote.
	   * @param messageName
	   *  This represents the uORB message Name; This message Name should be
	   *  globally unique.
	   * @param msgRate
	   *  The max rate at which the subscriber can accept the messages.
	   * @return
	   *  0 = success; This means the messages is successfully handled in the
	   *    handler.
	   *  otherwise = failure.
	   */
	virtual int16_t process_add_subscription(const char *messageName,
			int32_t msgRateInHz);

	/**
	 * Interface to process a received control msg to remove subscription
	 * @param messageName
	 *  This represents the uORB message Name; This message Name should be
	 *  globally unique.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *    handler.
	 *  otherwise = failure.
	 */
	virtual int16_t process_remove_subscription(const char *messageName);

	/**
	 * Interface to process the received data message.
	 * @param messageName
	 *  This represents the uORB message Name; This message Name should be
	 *  globally unique.
	 * @param length
	 *  The length of the data buffer to be sent.
	 * @param data
	 *  The actual data to be sent.
	 * @return
	 *  0 = success; This means the messages is successfully handled in the
	 *    handler.
	 *  otherwise = failure.
	 */
	virtual int16_t process_received_message(const char *messageName,
			int32_t length, uint8_t *data);

};

#endif /* _uORBManager_hpp_ */
