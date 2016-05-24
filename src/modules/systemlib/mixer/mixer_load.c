
/**
 * @file mixer_load.c
 *
 * Programmable multi-channel mixer library. 可编程的多通道混控器库
 */

#include <nuttx/config.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <systemlib/err.h>

#include "mixer_load.h"

int load_mixer_file(const char *fname, char *buf, unsigned maxlen)  //载入混控文件
{
	FILE		*fp;  //文件指针
	char		line[120];

	/* open the mixer definition file */
	fp = fopen(fname, "r");  //打开混控器定义的文件

	if (fp == NULL) {
		warnx("file not found");
		return -1;
	}

	/* read valid lines from the file into a buffer */  //从文件中读取有效值到缓冲区
	buf[0] = '\0';

	for (;;) {

		/* get a line, bail on error/EOF */ //获取一行，放弃 error/EOF
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {  //读取一行
			break;
		}

		/* if the line doesn't look like a mixer definition line, skip it */
		if ((strlen(line) < 2) || !isupper(line[0]) || (line[1] != ':')) {//如果一行字节小于2，或者第一个字母不是大写，或者第二个字符不为冒号，取下一行
			continue;
		}

		/* compact whitespace in the buffer */  //压缩缓冲区中的空格
		char *t, *f;

		for (f = line; *f != '\0'; f++) {  //扫描整个缓冲区
			/* scan for space characters */  //扫描空格字符
			if (*f == ' ') { //找到空格
				/* look for additional spaces */ //寻找额外的空格
				t = f + 1;

				while (*t == ' ') { //空格中的几个空格
					t++;
				}

				if (*t == '\0') { //扫描的结尾
					/* strip trailing whitespace */
					*f = '\0'; //消除后面的空格

				} else if (t > (f + 1)) {
					memmove(f + 1, t, strlen(t) + 1); //移除一个空格中的多个空格
				}
			}
		}

		/* if the line is too long to fit in the buffer, bail */  //如果一行太长充满缓冲区，放弃
		if ((strlen(line) + strlen(buf) + 1) >= maxlen) {
			warnx("line too long");  //行太长，关闭文件，返回-1
			fclose(fp);
			return -1;
		}

		/* add the line to the buffer */
		strcat(buf, line); //将行放入缓冲区
	}

	fclose(fp);
	return 0;
}

