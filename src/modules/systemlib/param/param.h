
/**
 * @file param.h
 *
 * Global parameter store.  全局参数存储
 *
 * Note that a number of API members are marked const or pure; these
 * assume that the set of parameters cannot change, or that a parameter
 * cannot change type or size over its lifetime.  If any of these assumptions
 * are invalidated, the attributes should be re-evaluated.
 */

#ifndef _SYSTEMLIB_PARAM_PARAM_H
#define _SYSTEMLIB_PARAM_PARAM_H

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

/** Maximum size of the parameter backing file */
#define PARAM_FILE_MAXSIZE	4096  //参数备份文件最大数目大小

__BEGIN_DECLS

/**
 * Parameter types.  参数类型
 */
typedef enum param_type_e {
	/* globally-known parameter types */
	PARAM_TYPE_INT32 = 0,  //32位整型
	PARAM_TYPE_FLOAT,  //浮点型

    /* structure parameters; size is encoded in the type value */  //参数结构体，大小是解码类型数值的大小
	PARAM_TYPE_STRUCT = 100,  //参数类型结构体
	PARAM_TYPE_STRUCT_MAX = 16384 + PARAM_TYPE_STRUCT,  //参数类型结构体最大字节数

	PARAM_TYPE_UNKNOWN = 0xffff  //未知的参数类型
} param_type_t;

/**
 * Parameter handle.  参数句柄
 *
 * Parameters are represented by parameter handles, which can
 * be obtained by looking up (or creating?) parameters.
 */
typedef uintptr_t	param_t;

/**
 * Handle returned when a parameter cannot be found.
 */
#define PARAM_INVALID	((uintptr_t)0xffffffff)  //没有找到参数，返回无效句柄

/**
 * Magic handle for hash check param
 */
#define PARAM_HASH      ((uintptr_t)INT32_MAX)  //哈希值检查参数

/**
 * Look up a parameter by name.  通过名字寻找参数
 *
 * @param name		The canonical name of the parameter being looked up. 有效名称
 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
 *			This call will also set the parameter as "used" in the system, which is used
 *			to e.g. show the parameter via the RC interface
 */
__EXPORT param_t	param_find(const char *name);

/**
 * Look up a parameter by name.
 *
 * @param name		The canonical name of the parameter being looked up.
 * @return		A handle to the parameter, or PARAM_INVALID if the parameter does not exist.
 */
__EXPORT param_t	param_find_no_notification(const char *name);

/**
 * Return the total number of parameters.
 *
 * @return		The number of parameters.
 */
__EXPORT unsigned	param_count(void);  //返回所有参数的数量

/**
 * Return the actually used number of parameters.  //返回实际使用的参数数量
 *
 * @return		The number of parameters.
 */
__EXPORT unsigned	param_count_used(void);

/**
 * Wether a parameter is in use in the system.
 *
 * @return		True if it has been written or read
 */
__EXPORT bool		param_used(param_t param);  //判断是否在系统中使用了参数

/**
 * Look up a parameter by index.
 *
 * @param index		An index from 0 to n, where n is param_count()-1.
 * @return		A handle to the parameter, or PARAM_INVALID if the index is out of range.
 */
__EXPORT param_t	param_for_index(unsigned index); //通过索引查找参数

/**
 * Look up an used parameter by index.
 *
 * @param param		The parameter to obtain the index for.
 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
 */
__EXPORT param_t	param_for_used_index(unsigned index);  //通过索引查找一个使用的参数

/**
 * Look up the index of a parameter.
 *
 * @param param		The parameter to obtain the index for.
 * @return		The index, or -1 if the parameter does not exist.
 */
__EXPORT int		param_get_index(param_t param);

/**
 * Look up the index of an used parameter.
 *
 * @param param		The parameter to obtain the index for.
 * @return		The index of the parameter in use, or -1 if the parameter does not exist.
 */
__EXPORT int		param_get_used_index(param_t param);

/**
 * Obtain the name of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The name assigned to the parameter, or NULL if the handle is invalid.
 */
__EXPORT const char	*param_name(param_t param);   //获得参数名称

/**
 * Test whether a parameter's value has changed from the default.
 *
 * @return		If true, the parameter's value has not been changed from the default.
 */
__EXPORT bool		param_value_is_default(param_t param);  //判断默认的参数是否被改变

/**
 * Test whether a parameter's value has been changed but not saved.
 *
 * @return		If true, the parameter's value has not been saved.
 */
__EXPORT bool		param_value_unsaved(param_t param);  //判断是否有参数已经改变但是没有保存

/**
 * Obtain the type of a parameter.  获得参数类型
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The type assigned to the parameter.
 */
__EXPORT param_type_t	param_type(param_t param);

/**
 * Determine the size of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		The size of the parameter's value.
 */
__EXPORT size_t		param_size(param_t param);  //确定参数大小

/**
 * Copy the value of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		Where to return the value, assumed to point to suitable storage for the parameter type.
 *			For structures, a bitwise copy of the structure is performed to this address.
 * @return		Zero if the parameter's value could be returned, nonzero otherwise.
 */
__EXPORT int		param_get(param_t param, void *val);  //复制参数的数值

/**
 * Set the value of a parameter.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 *			For structures, the pointer is assumed to point to a structure to be copied.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
__EXPORT int		param_set(param_t param, const void *val);  //设置参数的数值

/**
 * Set the value of a parameter, but do not trigger an auto-save
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 *			For structures, the pointer is assumed to point to a structure to be copied.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
__EXPORT int		param_set_no_autosave(param_t param, const void *val);  //设置参数数值，但不触发自动保存

/**
 * Set the value of a parameter, but do not notify the system about the change.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @param val		The value to set; assumed to point to a variable of the parameter type.
 *			For structures, the pointer is assumed to point to a structure to be copied.
 * @return		Zero if the parameter's value could be set from a scalar, nonzero otherwise.
 */
__EXPORT int		param_set_no_notification(param_t param, const void *val);  //设置一个有效的参数，但是不通知系统这个变化

/**
 * Reset a parameter to its default value.
 *
 * This function frees any storage used by struct parameters, and returns the parameter
 * to its default value.
 *
 * @param param		A handle returned by param_find or passed by param_foreach.
 * @return		Zero on success, nonzero on failure
 */
__EXPORT int		param_reset(param_t param);  //复位一个参数到默认值

/**
 * Reset all parameters to their default values.
 *
 * This function also releases the storage used by struct parameters.
 */
__EXPORT void		param_reset_all(void);  //复位所有的参数到默认值


/**
 * Reset all parameters to their default values except for excluded parameters.
 *
 * This function also releases the storage used by struct parameters.
 *
 * @param excludes			Array of param names to exclude from resetting. Use a wildcard
 *							at the end to exclude parameters with a certain prefix.
 * @param num_excludes		The number of excludes provided.
 */
__EXPORT void		param_reset_excludes(const char *excludes[], int num_excludes);  //复位所有没有执行的参数

/**
 * Export changed parameters to a file.
 *
 * @param fd		File descriptor to export to.
 * @param only_unsaved	Only export changed parameters that have not yet been exported.
 * @return		Zero on success, nonzero on failure.
 */
__EXPORT int		param_export(int fd, bool only_unsaved);  //输出一个已经改变的参数到文件

/**
 * Import parameters from a file, discarding any unrecognized parameters.
 *
 * This function merges the imported parameters with the current parameter set.
 *
 * @param fd		File descriptor to import from.  (Currently expected to be a file.)
 * @return		Zero on success, nonzero if an error occurred during import.
 *			Note that in the failure case, parameters may be inconsistent.
 */
__EXPORT int		param_import(int fd);  //从文件导入一个参数，

/**
 * Load parameters from a file.
 *
 * This function resets all parameters to their default values, then loads new
 * values from a file.
 *
 * @param fd		File descriptor to import from.  (Currently expected to be a file.)
 * @return		Zero on success, nonzero if an error occurred during import.
 *			Note that in the failure case, parameters may be inconsistent.
 */
__EXPORT int		param_load(int fd);  //从文件中载入参数

/**
 * Apply a function to each parameter.
 *
 * Note that the parameter set is not locked during the traversal. It also does
 * not hold an internal state, so the callback function can block or sleep between
 * parameter callbacks.
 *
 * @param func		The function to invoke for each parameter.
 * @param arg		Argument passed to the function.
 * @param only_changed	If true, the function is only called for parameters whose values have
 *			been changed from the default.
 * @param only_changed	If true, the function is only called for parameters which have been
 *			used in one of the running applications.
 */
__EXPORT void		param_foreach(void (*func)(void *arg, param_t param), void *arg, bool only_changed, bool only_used);

/**
 * Set the default parameter file name.
 *
 * @param filename	Path to the default parameter file.  The file is not require to
 *			exist.
 * @return		Zero on success.
 */
__EXPORT int 		param_set_default_file(const char *filename);//设置默认的参数文件名称

/**
 * Get the default parameter file name.
 *
 * @return		The path to the current default parameter file; either as
 *			a result of a call to param_set_default_file, or the
 *			built-in default.
 */
__EXPORT const char	*param_get_default_file(void);   //获得默认参数文件名称

/**
 * Save parameters to the default file.
 *
 * This function saves all parameters with non-default values.
 *
 * @return		Zero on success.
 */
__EXPORT int 		param_save_default(void);  //保存参数到默认的文件

/**
 * Load parameters from the default parameter file.
 *
 * @return		Zero on success.
 */
__EXPORT int 		param_load_default(void);  //从默认文件中载入参数

/**
 * Generate the hash of all parameters and their values
 *
 * @return		CRC32 hash of all param_ids and values
 */
__EXPORT uint32_t	param_hash_check(void); //获取所有参数和数值的哈希值

/*
 * Macros creating static parameter definitions.
 *
 * Note that these structures are not known by name; they are
 * collected into a section that is iterated by the parameter
 * code.
 *
 * Note that these macros cannot be used in C++ code due to
 * their use of designated initializers.  They should probably
 * be refactored to avoid the use of a union for param_value_u.
 */

/** define an int32 parameter */
#define PARAM_DEFINE_INT32(_name, _default)

/** define a float parameter */
#define PARAM_DEFINE_FLOAT(_name, _default)

/** define a parameter that points to a structure */
#define PARAM_DEFINE_STRUCT(_name, _default)

/**
 * Parameter value union.  参数值联合体
 */
union param_value_u {
	void		*p;
	int32_t		i;
	float		f;
};

/**
 * Static parameter definition structure.
 *
 * This is normally not used by user code; see the PARAM_DEFINE macros
 * instead.
 */
struct param_info_s {
	const char	*name

// GCC 4.8 and higher don't implement proper alignment of static data on
// 64-bit. This means that the 24-byte param_info_s variables are
// 16 byte aligned by GCC and that messes up the assumption that
// sequential items in the __param segment can be addressed as an array.
// The assumption is that the address of the second parameter is at
// &param[0]+sizeof(param[0]). When compiled with clang it is
// true, with gcc is is not true.
// See https://llvm.org/bugs/show_bug.cgi?format=multiple&id=18006
// The following hack is for GCC >=4.8 only. Clang works fine without
// this.
#ifdef __PX4_POSIX
	__attribute__((aligned(16)));
#else
	;
#endif
	param_type_t	type;
	union param_value_u val;
};

__END_DECLS

#endif
