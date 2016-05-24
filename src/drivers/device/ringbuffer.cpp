/**
 * @file ringbuffer.cpp
 *
 * A flexible ringbuffer class.灵活的环形缓冲器
 */

#include "ringbuffer.h"
#include <string.h>

namespace ringbuffer
{

RingBuffer::RingBuffer(unsigned num_items, size_t item_size) :  //构造函数，需要缓冲区数量和每个缓冲区的大小
	_num_items(num_items),
	_item_size(item_size),
	_buf(new char[(_num_items + 1) * item_size]),
	_head(_num_items),  //缓冲区头指针
	_tail(_num_items)   //缓冲区尾指针
{}

RingBuffer::~RingBuffer()
{
	if (_buf != nullptr) {
		delete[] _buf;  //删除缓冲区
	}
}

unsigned
RingBuffer::_next(unsigned index)
{
	return (0 == index) ? _num_items : (index - 1); //从最大到零循环计数
}

bool
RingBuffer::empty()
{
	return _tail == _head; //指针头=尾，缓冲区为空
}

bool
RingBuffer::full()
{
	return _next(_head) == _tail;  //判断下一个缓冲区
}

unsigned
RingBuffer::size()
{
	return (_buf != nullptr) ? _num_items : 0; //返回缓冲区数量
}

void
RingBuffer::flush()  //填充缓冲区
{
	while (!empty()) { //一直判断是否为空
		get(NULL);
	}
}

bool
RingBuffer::put(const void *val, size_t val_size)  //无类型数据，和大小
{
	unsigned next = _next(_head);

	if (next != _tail) {  //没有到达尾部
		if ((val_size == 0) || (val_size > _item_size)) { //如果需要填充的数据大小==0或者大于缓冲区大小
			val_size = _item_size;  //填满缓冲区
		}

		memcpy(&_buf[_head * _item_size], val, val_size); //存入缓冲区
		_head = next;
		return true;

	} else {
		return false;
	}
}

bool
RingBuffer::put(int8_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint8_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int16_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint16_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int32_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint32_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(int64_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(uint64_t val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(float val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::put(double val)
{
	return put(&val, sizeof(val));
}

bool
RingBuffer::force(const void *val, size_t val_size)  //强制写入
{
	bool overwrote = false;

	for (;;) {
		if (put(val, val_size)) {  //写入成功则跳出循环，缓冲区一直满可能就死在这
			break;
		}

		get(NULL);
		overwrote = true;
	}

	return overwrote;
}

bool
RingBuffer::force(int8_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint8_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int16_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint16_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int32_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint32_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(int64_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(uint64_t val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(float val)
{
	return force(&val, sizeof(val));
}

bool
RingBuffer::force(double val)
{
	return force(&val, sizeof(val));
}

#define __PX4_SBCAP __sync_bool_compare_and_swap

bool
RingBuffer::get(void *val, size_t val_size)
{
	if (_tail != _head) {//缓冲区不为空
		unsigned candidate;
		unsigned next;

		if ((val_size == 0) || (val_size > _item_size)) {
			val_size = _item_size;
		}

		do {
			/* decide which element we think we're going to read */
			candidate = _tail;

			/* and what the corresponding next index will be */
			next = _next(candidate);

			/* go ahead and read from this index */
			if (val != NULL) {
				memcpy(val, &_buf[candidate * _item_size], val_size);
			}

			/* if the tail pointer didn't change, we got our item */
		} while (!__PX4_SBCAP(&_tail, candidate, next));

		return true;

	} else {
		return false;
	}
}

bool
RingBuffer::get(int8_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint8_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int16_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint16_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int32_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint32_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(int64_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(uint64_t &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(float &val)
{
	return get(&val, sizeof(val));
}

bool
RingBuffer::get(double &val)
{
	return get(&val, sizeof(val));
}

unsigned
RingBuffer::space(void)
{
	unsigned tail, head;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		head = _head;
		tail = _tail;
	} while (head != _head);

	return (tail >= head) ? (_num_items - (tail - head)) : (head - tail - 1);
}

unsigned
RingBuffer::count(void)
{
	/*
	 * Note that due to the conservative nature of space(), this may
	 * over-estimate the number of items in the buffer.
	 */
	return _num_items - space();
}

bool
RingBuffer::resize(unsigned new_size)
{
	char *old_buffer;
	char *new_buffer = new char [(new_size + 1) * _item_size];

	if (new_buffer == nullptr) {
		return false;
	}

	old_buffer = _buf;
	_buf = new_buffer;
	_num_items = new_size;
	_head = new_size;
	_tail = new_size;
	delete[] old_buffer;
	return true;
}

void
RingBuffer::print_info(const char *name)
{
	printf("%s	%u/%lu (%u/%u @ %p)\n",
	       name,
	       _num_items,
	       (unsigned long)_num_items * _item_size,
	       _head,
	       _tail,
	       _buf);
}


} // namespace ringbuffer
