
#include "uORBUtils.hpp"
#include <stdio.h>
#include <errno.h>

int uORB::Utils::node_mkpath  //多主题
(
	char *buf,   //路径
	Flavor f,    //发布订阅
	const struct orb_metadata *meta,  //元数据结构
	int *instance    //null
)
{
	unsigned len;

	unsigned index = 0;

	if (instance != nullptr) {  //如果不为空
		index = *instance;
	}

	len = snprintf(buf, orb_maxpath, "/%s/%s%d",
		       (f == PUBSUB) ? "obj" : "param",
		       meta->o_name, index);  //创建路径文件，并写入，路径名根据元结构体名称

	if (len >= orb_maxpath) {  //如果长度大于最大路径
		return -ENAMETOOLONG;
	}

	return OK;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
int uORB::Utils::node_mkpath(char *buf, Flavor f,  //单主题
			     const char *orbMsgName)
{
	unsigned len;

	unsigned index = 0;

	len = snprintf(buf, orb_maxpath, "/%s/%s%d", (f == PUBSUB) ? "obj" : "param",
		       orbMsgName, index);  //名字直接由字符串给定

	if (len >= orb_maxpath) {  //长度大于最大的orb路径
		return -ENAMETOOLONG;
	}

	return OK;
}
