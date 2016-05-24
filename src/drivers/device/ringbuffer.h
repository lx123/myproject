/**
 * @file ringbuffer.h
 *
 * A flexible ringbuffer class.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace ringbuffer __EXPORT
{

class RingBuffer
{
public:
	RingBuffer(unsigned ring_size, size_t entry_size);
	virtual ~RingBuffer();

	/**
	 * Put an item into the buffer.
	 *
	 * @param val		Item to put
	 * @return		true if the item was put, false if the buffer is full
	 */
	bool			put(const void *val, size_t val_size = 0);  //放入一个项目到缓冲区，根据数据呈现多态性

	bool			put(int8_t val);
	bool			put(uint8_t val);
	bool			put(int16_t val);
	bool			put(uint16_t val);
	bool			put(int32_t val);
	bool			put(uint32_t val);
	bool			put(int64_t val);
	bool			put(uint64_t val);
	bool			put(float val);
	bool			put(double val);

	/**
	 * Force an item into the buffer, discarding an older item if there is not space.
	 *
	 * @param val		Item to put
	 * @return		true if an item was discarded to make space
	 */
	bool			force(const void *val, size_t val_size = 0);  //如果没有空格，强制一个项目进入缓冲器，丢弃旧的数据

	bool			force(int8_t val);
	bool			force(uint8_t val);
	bool			force(int16_t val);
	bool			force(uint16_t val);
	bool			force(int32_t val);
	bool			force(uint32_t val);
	bool			force(int64_t val);
	bool			force(uint64_t val);
	bool			force(float val);
	bool			force(double val);

	/**
	 * Get an item from the buffer.
	 *
	 * @param val		Item that was gotten
	 * @return		true if an item was got, false if the buffer was empty.
	 */
	bool			get(void *val, size_t val_size = 0); //缓冲区中获得一个数据

	bool			get(int8_t &val);
	bool			get(uint8_t &val);
	bool			get(int16_t &val);
	bool			get(uint16_t &val);
	bool			get(int32_t &val);
	bool			get(uint32_t &val);
	bool			get(int64_t &val);
	bool			get(uint64_t &val);
	bool			get(float &val);
	bool			get(double &val);

	/*
	 * Get the number of slots free in the buffer.
	 *
	 * @return		The number of items that can be put into the buffer before
	 *			it becomes full.
	 */
	unsigned		space(void);  //缓冲区中剩余的空间

	/*
	 * Get the number of items in the buffer.
	 *
	 * @return		The number of items that can be got from the buffer before
	 *			it becomes empty.
	 */
	unsigned		count(void);  //缓冲区中已有数目

	/*
	 * Returns true if the buffer is empty.
	 */
	bool			empty();  //检查缓冲区是否为空

	/*
	 * Returns true if the buffer is full.
	 */
	bool			full();   //缓冲区是否满

	/*
	 * Returns the capacity of the buffer, or zero if the buffer could
	 * not be allocated.
	 */
	unsigned		size();  //缓冲区大小

	/*
	 * Empties the buffer.
	 */
	void			flush();  //清空缓冲区

	/*
	 * resize the buffer. This is unsafe to be called while
	 * a producer or consuming is running. Caller is responsible
	 * for any locking needed
	 *
	 * @param new_size	new size for buffer
	 * @return		true if the resize succeeds, false if
	 * 			not (allocation error)
	 */
	bool			resize(unsigned new_size);  //改变缓冲区大小

	/*
	 * printf() some info on the buffer
	 */
	void			print_info(const char *name);  //打印缓冲区相关

private:
	unsigned		_num_items;  //项目数量
	const size_t		_item_size;  //项目大小
	char			*_buf;
	volatile unsigned	_head;	/**< insertion point in _item_size units */
	volatile unsigned	_tail;	/**< removal point in _item_size units */

	unsigned		_next(unsigned index);

	/* we don't want this class to be copied */
	RingBuffer(const RingBuffer &);
	RingBuffer operator=(const RingBuffer &);
};

} // namespace ringbuffer

