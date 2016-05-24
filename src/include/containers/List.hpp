/**
 * @file List.hpp
 *
 * A linked list. 文件列表
 */

#pragma once

template<class T>
class __EXPORT ListNode  //列表节点
{
public:
	ListNode() : _sibling(nullptr)
	{
	}
	virtual ~ListNode() {};
	void setSibling(T sibling) { _sibling = sibling; }
	T getSibling() { return _sibling; }
	T get()
	{
		return _sibling;
	}
protected:
	T _sibling;
private:
	// forbid copy
	ListNode(const ListNode &other);
	// forbid assignment
	ListNode &operator = (const ListNode &);
};

template<class T>
class __EXPORT List  //列表
{
public:
	List() : _head()
	{
	}
	virtual ~List() {};
	void add(T newNode)
	{
		newNode->setSibling(getHead());
		setHead(newNode);
	}
	T getHead() { return _head; }
protected:
	void setHead(T &head) { _head = head; }
	T _head;
private:
	// forbid copy
	List(const List &other);
	// forbid assignment
	List &operator = (const List &);
};
