/**
 * @file px4_getopt.h
 * Thread safe version of getopt
 * 目的为了获得命令的选项
 */

#pragma once

__BEGIN_DECLS//外部声明

int px4_getopt(int argc, char *argv[], const char *options, int *myoptind, const char **myoptarg);

__END_DECLS
