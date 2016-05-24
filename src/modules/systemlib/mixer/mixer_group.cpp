
/**
 * @file mixer_group.cpp
 *
 * Mixer collection.
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include "mixer.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)

MixerGroup::MixerGroup(ControlCallback control_cb, uintptr_t cb_handle) :
	Mixer(control_cb, cb_handle),
	_first(nullptr)
{
}

MixerGroup::~MixerGroup()
{
	reset();
}

void
MixerGroup::add_mixer(Mixer *mixer)  //添加一个混控器到一个组
{
	Mixer **mpp; //定义一个类二级指针

	mpp = &_first;  //组链表

	while (*mpp != nullptr) { //如果链表不为空
		mpp = &((*mpp)->_next); //取下一个链表，一直找到空的链表
	}

	*mpp = mixer;  //添加混控器
	mixer->_next = nullptr; //下一个链表对象为空
}

void
MixerGroup::reset()  //移除组里所有的混控器
{
	Mixer *mixer;

	/* discard sub-mixers */
	while (_first != nullptr) { //扫描所有的链表
		mixer = _first;
		_first = mixer->_next;
		delete mixer;  //删除对象
		mixer = nullptr;  //指针清空
	}
}

unsigned
MixerGroup::mix(float *outputs, unsigned space, uint16_t *status_reg)  //混控
{
	Mixer	*mixer = _first;  //首个混控器地址
	unsigned index = 0;  //索引

	while ((mixer != nullptr) && (index < space)) { //一直扫描到 混控器为空或者索引大于空间
		index += mixer->mix(outputs + index, space - index, status_reg);  //迭代的过程
		mixer = mixer->_next;
	}

	return index; //返回当前索引
}

unsigned
MixerGroup::count()  //混控器数量
{
	Mixer	*mixer = _first;
	unsigned index = 0;

	while ((mixer != nullptr)) {  //扫描整个链表
		mixer = mixer->_next;
		index++;
	}

	return index;  //返回混控器数量
}

void
MixerGroup::groups_required(uint32_t &groups) //引用
{
	Mixer	*mixer = _first;

	while (mixer != nullptr) {  //对链表扫描
		mixer->groups_required(groups);
		mixer = mixer->_next;
	}
}

int
MixerGroup::load_from_buf(const char *buf, unsigned &buflen)  //从缓冲区中载入
{
	int ret = -1;
	const char *end = buf + buflen; //末尾指针

	/*
	 * Loop until either we have emptied the buffer, or we have failed to
	 * allocate something when we expected to.
	 */
	while (buflen > 0) {//扫描整个缓冲区
		Mixer *m = nullptr;
		const char *p = end - buflen; //缓冲区首地址
		unsigned resid = buflen;

		/*
		 * Use the next character as a hint to decide which mixer class to construct. 使用下一个字符作为线索决定构建哪一个混控器
		 */
		switch (*p) { //第一个字符
		case 'Z':  //0，空混控器
			m = NullMixer::from_text(p, resid);
			break;

		case 'M': //简单加法混控器
			m = SimpleMixer::from_text(_control_cb, _cb_handle, p, resid);
			break;

		case 'R':  //多旋翼混控器
			m = MultirotorMixer::from_text(_control_cb, _cb_handle, p, resid);
			break;

		default:
			/* it's probably junk or whitespace, skip a byte and retry */ //有可能是垃圾或者空格，跳过一个字节并且重试
			buflen--;
			continue;
		}

		/*
		 * If we constructed something, add it to the group.
		 */
		if (m != nullptr) { //如果构造了任何一个，添加它到组
			add_mixer(m);  //添加混控器到组

			/* we constructed something */
			ret = 0; //返回真，构造了

			/* only adjust buflen if parsing was successful */
			buflen = resid; //如果解析成功，调整缓冲区长度
			debug("SUCCESS - buflen: %d", buflen);

		} else {

			/*
			 * There is data in the buffer that we expected to parse, but it didn't,
			 * so give up for now.
			 */
			break;
		}
	}

	/* nothing more in the buffer for us now */
	return ret;
}
