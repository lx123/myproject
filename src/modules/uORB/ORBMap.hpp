
#pragma once

#include <string.h>
#include <stdlib.h>


namespace uORB
{
class DeviceNode;   //声明类，设备节点
class ORBMap;       //ORB地图
}

class uORB::ORBMap  //类定义
{
public:
	struct Node {  //节点结构体
		struct Node *next;   //下一个结构体
		const char *node_name;   //节点名称
		uORB::DeviceNode *node;   //节点类
	};

	ORBMap() :      //构造函数
		_top(nullptr),   //初始化
		_end(nullptr)
	{ }
	~ORBMap()   //析构函数
	{
		while (_top != nullptr) {  //节点扫描，然后释放
			unlinkNext(_top);  //下一个

			if (_top->next == nullptr) { //如果下一个指针为空
				free((void *)_top->node_name);  //释放节点
				free(_top);
				_top = nullptr;
				_end = nullptr;
			}
		}
	}
    void insert(const char *node_name, uORB::DeviceNode *node)  //插入节点
	{
		Node **p;  //节点指针

		if (_top == nullptr) {   //如果顶部为空
			p = &_top; //放到顶部

		} else {
			p = &_end->next;  //插入尾部
		}

		*p = (Node *)malloc(sizeof(Node));  //申请内存空间

		if (_end) { //如果存在尾部
			_end = _end->next;  //尾部为下一个指针

		} else {
			_end = _top; //没有尾部，直接给头部
		}
//在尾部加入节点
		_end->next = nullptr; //新尾部后面为空
		_end->node_name = strdup(node_name); //设备节点路径
		_end->node = node;  //节点类
	}

	bool find(const char *node_name)   //查找节点
	{
		Node *p = _top;  //给出头部

		while (p) {  //依次查找
			if (strcmp(p->node_name, node_name) == 0) {  //找到相同的节点名称
				return true;  //已经找到
			}

			p = p->next;  //下一个节点
		}

		return false;  //没有找到
	}

	uORB::DeviceNode *get(const char *node_name)   //获取节点
	{
		Node *p = _top;  //头部给出

		while (p) {  //扫描
			if (strcmp(p->node_name, node_name) == 0) {  //找到节点
				return p->node; //给出节点类
			}

			p = p->next;  //下一个
		}

		return nullptr;  //返回空
	}

	void unlinkNext(Node *a)  //取消下一个连接，取消a后面的节点
	{
		Node *b = a->next; //给出下一个节点地址

		if (b != nullptr) {  //如果下一个节点地址不为空
			if (_end == b) { //最后一个节点等于a的下一个节点
				_end = a;   //删除最后一个节点
			}

			a->next = b->next; //如果b节点不为空，b的下一个节点接在a后
			free((void *)b->node_name);
			free(b);  //释放
		}
	}

private:  //私有成员
	Node *_top;  //头部节点
	Node *_end;  //尾部节点
};

