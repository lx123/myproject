
/**
 * @file Subscription.h
 *
 */

#pragma once

#include <assert.h>

#include <uORB/uORB.h>
#include <containers/List.hpp>
#include <systemlib/err.h>

namespace uORB
{

/**
 * Base subscription warapper class, used in list traversal
 * of various subscriptions.
 */
class __EXPORT SubscriptionBase  //订阅者基类
{
public:
// methods

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param instance The instance for multi sub.
	 */
	SubscriptionBase(const struct orb_metadata *meta,
			 unsigned interval = 0, unsigned instance = 0);

	/**
	 * Check if there is a new update.
	 * */
	bool updated();  //检查是否有更新

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	void update(void *data);  //更新结构体

	/**
	 * Deconstructor
	 */
	virtual ~SubscriptionBase();

// accessors
	const struct orb_metadata *getMeta() { return _meta; }
	int getHandle() { return _handle; }
protected:
// accessors
	void setHandle(int handle) { _handle = handle; }
// attributes
	const struct orb_metadata *_meta;
	int _instance;
	int _handle;
private:
	// disallow copy
	SubscriptionBase(const SubscriptionBase &other);
	// disallow assignment
	SubscriptionBase &operator=(const SubscriptionBase &other);
};

/**
 * alias class name so it is clear that the base class
 */
typedef SubscriptionBase SubscriptionTiny;

/**
 * The subscription base class as a list node.
 */
class __EXPORT SubscriptionNode :   //订阅者节点

	public SubscriptionBase,
	public ListNode<SubscriptionNode *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from the ORB_ID()
	 * 	macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param instance The instance for multi sub.
	 * @param list 	A pointer to a list of subscriptions
	 * 	that this should be appended to.
	 */
	SubscriptionNode(const struct orb_metadata *meta,
			 unsigned interval = 0,
			 int instance = 0,
			 List<SubscriptionNode *> *list = nullptr) :
		SubscriptionBase(meta, interval, instance),
		_interval(interval)
	{
		if (list != nullptr) { list->add(this); }
	}

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual void update() = 0;
// accessors
	unsigned getInterval() { return _interval; }
protected:
// attributes
	unsigned _interval;

};

/**
 * Subscription wrapper class
 */
template<class T>
class __EXPORT Subscription :
	public SubscriptionNode
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param interval  The minimum interval in milliseconds
	 * 	between updates
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	Subscription(const struct orb_metadata *meta,
		     unsigned interval = 0,
		     int instance = 0,
		     List<SubscriptionNode *> *list = nullptr);

	/**
	 * Deconstructor
	 */
	virtual ~Subscription();


	/**
	 * Create an update function that uses the embedded struct.
	 */
	void update();

	/*
	 * This function gets the T struct data
	 * */
	const T &get();
private:
	T _data;
};

} // namespace uORB



