/**
 * @file px4_getopt.cpp
 * Minimal, thread safe version of getopt  获取最小线程安全版本线程
 */

#include <px4_getopt.h>
#include <stdio.h>

// check if p is a valid option and if the option takes an arg  检查p是一个有效的选项，如果参数存在arg中
static char isvalidopt(char p, const char *options, int *takesarg)//判断是一个有效的选项
{
	int idx = 0;//索引
	*takesarg = 0;//标记

	while (options[idx] != 0 && p != options[idx]) {//选项中找p中的参数，选项可能不止一个，搜索选项
		++idx;
	}

	if (options[idx] == 0) {//没有找到选项
		return '?';//返回未知选项
	}

	if (options[idx + 1] == ':') {//如果选项找到，并且下一个为：号，则表明选项后面有参数
		*takesarg = 1;//参数标记
	}

	return options[idx];//返回找到的选项
}

// reorder argv and put non-options at the end重新排列参数，让没有用到的选项在底部
static int reorder(int argc, char **argv, const char *options)
{
	char *tmp_argv[argc];//临时保存选项参数
	char c;
	int idx = 1;//索引
	int tmpidx = 1;//临时索引
	int takesarg;//标记，如果选项后面有参数

	// move the options to the front 将选项移动到前面
	while (idx < argc && argv[idx] != 0) {//扫描参数选项
		if (argv[idx][0] == '-') {//找到选项标记
			c = isvalidopt(argv[idx][1], options, &takesarg);//标记的下一位就是选项，检查是否有效

			if (c == '?') {//如果选项未知
				return 1;//返回错误
			}

			tmp_argv[tmpidx] = argv[idx];//保存到临时变量中
			tmpidx++;//临时变量中数据加1

			if (takesarg) {//如果当前参数后面紧跟着一个参数
				tmp_argv[tmpidx] = argv[idx + 1];//保存临时变量中
				// printf("tmp_argv[%d] = %s\n", tmpidx, tmp_argv[tmpidx]);
				tmpidx++;
				idx++;
			}
		}

		idx++;
	}

	// Add non-options to the end到这，所有的参数都存入临时变量
	idx = 1;

	while (idx < argc && argv[idx] != 0) {
		if (argv[idx][0] == '-') {
			c = isvalidopt(argv[idx][1], options, &takesarg);

			if (c == '?') {
				return c;
			}

			if (takesarg) {
				idx++;
			}

		} else {
			tmp_argv[tmpidx] = argv[idx];//将选项后面没有参数的放到后面
			tmpidx++;
		}

		idx++;
	}

	// Reorder argv重新排列选项
	for (idx = 1; idx < argc; idx++) {
		argv[idx] = tmp_argv[idx];
	}

	return 0;
}

//供外部调用
// px4_getopt 对命令参数进行解析，一个个返回
//
// returns:
//            the valid option character有效的字符选项
//            '?' if any option is unknown不明的选项
//            -1 if no remaining options如果没有剩下选项返回-1
//
// If the option takes an arg, myoptarg will be updated accordingly.如果选项接收到，myoptarg立即相应的更新
// After each call to px4_getopt, myoptind in incremented to the next
// unparsed arg index.
// Argv is changed to put all options and option args at the beginning,
// followed by non-options.
//
__EXPORT int px4_getopt(int argc, char *argv[], const char *options, int *myoptind, const char **myoptarg)
{
	char *p;
	char c;
	int takesarg;

	if (*myoptind == 1)//第一次调用此函数
		if (reorder(argc, argv, options) != 0) {//重新排列参数选项，并保存在argv中
			return (int)'?';
		}

	p = argv[*myoptind];//参数索引

	if (*myoptarg == 0) {
		*myoptarg = argv[*myoptind];
	}

	if (p && options && myoptind && p[0] == '-') {
		c = isvalidopt(p[1], options, &takesarg);//判断是否为有效参数

		if (c == '?') {
			return (int)c;
		}

		*myoptind += 1;//参数调用计数

		if (takesarg) {//选项后面有参数
			*myoptarg = argv[*myoptind];//选项后面的参数
			*myoptind += 1;//参数索引加1
		}

		return (int)c;//返回参数选项
	}

	return -1;
}

