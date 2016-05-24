
/**
 * @file Publication.h
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
 * Base publication wrapper class, used in list traversal
 * of various publications.
 */
class __EXPORT PublicationBase  //发布者基类
{
public:

	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub/sub, 0-based, -1 means
	 * 	don't publish as multi
	 */
	PublicationBase(const struct orb_metadata *meta,
			int priority = -1);

	/**
	 * Update the struct
	 * @param data The uORB message struct we are updating.
	 */
	void update(void *data);  //更新结构体

	/**
	 * Deconstructor
	 */
	virtual ~PublicationBase();

// accessors
	const struct orb_metadata *getMeta() { return _meta; }  //获得元数组
	orb_advert_t getHandle() { return _handle; }  //获得句柄
protected:
	// disallow copy   不接受复制
	PublicationBase(const PublicationBase &other);
	// disallow assignment
	PublicationBase &operator=(const PublicationBase &other);
// accessors
	void setHandle(orb_advert_t handle) { _handle = handle; }  //设置句柄
// attributes
	const struct orb_metadata *_meta;
	int _priority;  //优先级
	int _instance;  //会话标号
	orb_advert_t _handle;  //句柄
};

/**
 * alias class name so it is clear that the base class
 * can be used by itself if desired
 */
typedef PublicationBase PublicationTiny;   //取个别名

/**
 * The publication base class as a list node.
 */
class __EXPORT PublicationNode :   //发布者节点
	public PublicationBase,
	public ListNode<PublicationNode *>
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub, 0-based.
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	PublicationNode(const struct orb_metadata *meta,   //构造函数
			int priority = -1,
			List<PublicationNode *> *list = nullptr);

	/**
	 * This function is the callback for list traversal
	 * updates, a child class must implement it.
	 */
	virtual void update() = 0;
};

/**
 * Publication wrapper class
 */
template<class T>  //模板声明后的值是什么类型，T就是什么类型
class __EXPORT Publication :
	public PublicationNode
{
public:
	/**
	 * Constructor
	 *
	 * @param meta The uORB metadata (usually from
	 * 	the ORB_ID() macro) for the topic.
	 * @param priority The priority for multi pub, 0-based.
	 * @param list A list interface for adding to
	 * 	list during construction
	 */
	Publication(const struct orb_metadata *meta,
		    int priority = -1,
		    List<PublicationNode *> *list = nullptr)  :
		PublicationNode(meta, priority, list),
		_data()
	{
	}

	/**
	 * Deconstructor
	 **/
	virtual ~Publication() {};

	/*
	 * This function gets the T struct
	 * */
	T &get() { return _data; }

	/**
	 * Create an update function that uses the embedded struct.
	 */
	void update()  //更新
	{
		PublicationBase::update((void *)(&_data));
	}
private:
	T _data;
};

} // namespace uORB



