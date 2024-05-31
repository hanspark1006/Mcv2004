/*
 * macros_common.h
 *
 *  Created on: Feb 6, 2024
 *      Author: catsa
 */

#ifndef SRC_M_EVENT_MACROS_COMMON_H_
#define SRC_M_EVENT_MACROS_COMMON_H_

#define case_str(name) case name : return #name;

#define BREAK_IF_ERROR(err_code) \
if ((err_code) != 0) \
{ \
	LOG_ERR(__FILE__); \
    LOG_ERR(" - ERROR. line: %d, with error code %d \r\n", __LINE__, err_code); \
    break; \
}

#define CONTINUE_IF_ERROR(err_code) \
if ((err_code) != 0) \
{ \
	LOG_ERR(__FILE__); \
    LOG_ERR(" - ERROR. line: %d, with error code %d \r\n", __LINE__, err_code); \
    continue; \
}

#define RETURN_IF_ERROR(err_code) \
if ((err_code) != 0) \
{ \
	LOG_ERR(__FILE__); \
    LOG_ERR(" - ERROR. line: %d, with error code %d \r\n", __LINE__, err_code); \
    return (err_code); \
}

#define REPORT_IF_ERROR(err_code) \
if ((err_code) != 0) \
{ \
	LOG_ERR(__FILE__); \
    LOG_ERR(" - ERROR. line: %d, with error code %d \r\n", __LINE__, err_code); \
}

#define NULL_PARAM_CHECK(param)					\
if ((param) == NULL)							\
{												\
	LOG_ERR(__FILE__);							\
	LOG_ERR(" - NULL. line: %d\r\n", __LINE__);	\
	return -EINVAL;								\
}

#endif /* SRC_M_EVENT_MACROS_COMMON_H_ */
