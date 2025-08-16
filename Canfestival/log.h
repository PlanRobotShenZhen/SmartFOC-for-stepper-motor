/*
 * log.h
 *
 *  Created on: 2019年10月6日
 *      Author: pc
 */

#ifndef LOG_H_
#define LOG_H_


#define CONFIG_LOG_DEFAULT_LEVEL 3
#define TAG  "DEFAULT"

typedef enum {
    LOG_NONE = 0,   /*!< No log output */
    LOG_ERROR,      /*!< Critical errors, software module can not recover on its own */
    LOG_WARN,       /*!< Error conditions from which recovery measures have been taken */
    LOG_INFO,       /*!< Information messages which describe normal flow of events */
    LOG_DEBUG,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    LOG_VERBOSE,    /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */

    LOG_MAX
} log_level_t;

#define LOG_LEVEL(level, tag, f_name, line, format, ...) do {                     \
        if (level==LOG_ERROR )          { log_printf(LOG_ERROR,      tag, f_name, line, format, ##__VA_ARGS__); } \
        else if (level==LOG_WARN )      { log_printf(LOG_WARN,       tag, f_name, line, format, ##__VA_ARGS__); } \
        else if (level==LOG_DEBUG )     { log_printf(LOG_DEBUG,      tag, f_name, line, format, ##__VA_ARGS__); } \
        else if (level==LOG_VERBOSE )   { log_printf(LOG_VERBOSE,    tag, f_name, line, format, ##__VA_ARGS__); } \
        else                            { log_printf(LOG_INFO,       tag, f_name, line, format, ##__VA_ARGS__); } \
    } while(0)

#define LOG_LEVEL_LOCAL(level, tag, f_name, line, format, ...) do {               \
        if ( LOG_LOCAL_LEVEL >= level ) LOG_LEVEL(level, tag, f_name, line, format, ##__VA_ARGS__); \
    } while(0)

#if DEBUG
  #define LOGE(format, ... ) LOG_LEVEL_LOCAL(LOG_ERROR,   TAG, __FUNCTION__ , __LINE__, format, ##__VA_ARGS__)
  #define LOGW(format, ... ) LOG_LEVEL_LOCAL(LOG_WARN,    TAG, __FUNCTION__ , __LINE__, format, ##__VA_ARGS__)
  #define LOGI(format, ... ) LOG_LEVEL_LOCAL(LOG_INFO,    TAG, __FUNCTION__ , __LINE__, format, ##__VA_ARGS__)
  #define LOGD(format, ... ) LOG_LEVEL_LOCAL(LOG_DEBUG,   TAG, __FUNCTION__ , __LINE__, format, ##__VA_ARGS__)
  #define LOGV(format, ... ) LOG_LEVEL_LOCAL(LOG_VERBOSE, TAG, __FUNCTION__ , __LINE__, format, ##__VA_ARGS__)
#else
  #define LOGE(format, ... )
  #define LOGW(format, ... )
  #define LOGI(format, ... )
  #define LOGD(format, ... )
  #define LOGV(format, ... )
#endif

void log_printf(const int log_level,const char * tag, const char *func, const int line, const char *fmt, ...);

#endif /* LOG_H_ */
