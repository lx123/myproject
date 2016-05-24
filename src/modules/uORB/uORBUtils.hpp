#ifndef _uORBUtils_hpp_
#define _uORBUtils_hpp_

#include "uORBCommon.hpp"
//#include <string>

namespace uORB
{
class Utils;
}

class uORB::Utils  //工具包类
{
public:
	static int node_mkpath
	(
		char *buf,
		Flavor f,
		const struct orb_metadata *meta,
		int *instance = nullptr
	); //函数的多态性

	/**
	 * same as above except this generators the path based on the string.
	 */
	static int node_mkpath(char *buf, Flavor f, const char *orbMsgName);  //基于字符串，创建路径

};

#endif // _uORBUtils_hpp_
