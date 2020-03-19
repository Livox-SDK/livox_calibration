#ifndef __LOG_CLIENT_H__
#define __LOG_CLIENT_H__

#include <stdint.h>

typedef enum
{
    DEBUG   = 20,
    INFO    = 40,
    WARN    = 80,
    ERROR   = 100,
    FATAL   = 120,
    RAW     = 200,
}LOG_LEVEL;

#if 1
/* Modify below by yourself!!! */
/* dlog client */
#define MODULE              "mlc"
#define MODULE_ID           0
#define SUBMODULE           "log"
#define LEVEL               DEBUG        //print level need >= LEVEL
#define DLOG_CLIENT_INIT()  dlog_client_init(MODULE, MODULE_ID, LEVEL)
#define DLOG_JSON_PATH      "/home/livox/Desktop/dlog_config.json"

//#define DLOG(submod, fmt, arg...)       dlog(LEVEL, MODULE, #submod, fmt, ##arg)

#define ALOAM_MLC_DEBUG(fmt,arg...)          dlog(DEBUG, MODULE, SUBMODULE, "[DEBUG] " fmt,##arg)
#define ALOAM_MLC_INFO(fmt,arg...)           dlog(INFO, MODULE, SUBMODULE, "[INFO] " fmt,##arg)
#define ALOAM_MLC_WARN(fmt,arg...)           dlog(WARN, MODULE, SUBMODULE, "[WARN] " fmt,##arg)
#define ALOAM_MLC_ERROR(fmt,arg...)          dlog(ERROR, MODULE, SUBMODULE, "[ERROR] " fmt,##arg)
#define ALOAM_MLC_FATAL(fmt,arg...)          dlog(FATAL, MODULE, SUBMODULE, "[FATAL] " fmt,##arg)
#endif

#ifdef __cplusplus
extern "C"{
#endif
/* API */
/**
 * @brief dlog_client_init
 *
 * create an dlog client, register to dcos and dlog server.
 * @param [in] module name
 * @param [in] module id
 * @param [in] threhold print level to be set
 * @return 0 if succeed.
 * @note
 * -# dlog server will create a dir named as module name to storage log info.
 * -# module id is determined by you, it is just like module id in dcos.
 * -# log to be stored in server, print level need >= threhold print level.
 */
int dlog_client_init(const char *module, uint8_t mid, LOG_LEVEL print_level);
int dlog_client_init_with_param(const char *module, uint8_t mid, LOG_LEVEL print_level, const char* json_path);


/**
 * @brief dlog
 *
 * send log info to dlog server by dcos.
 * @param [in] level
 * @param [in] module name, which module the log belong to
 * @param [in] sub module name
 * @return send bytes if succeed, else return 0.
 * @note
 * -# dlog server will receive log info and store them in /var/dlog/module/submod/ dir.
 * -# if module name is "HAL", submod name could be "LOG", "CAMERA", "GPS", "IMU"....
 */
int dlog(LOG_LEVEL level, const char *module, const char *submod, const char *fmt, ...);
int vdlog(LOG_LEVEL level, const char *module, const char *submod, const char *fmt, va_list args);

int dcos_log(uint8_t mid, const char *fmt, ...);
int dlog_add_rule(const char *module, const char* json_path);
void dlog_set_print_level(LOG_LEVEL print_level);
LOG_LEVEL dlog_get_print_level(void);

typedef void *dlog_fp_t;
/* bin */
dlog_fp_t dlog_bin_fopen(const char *name, const char* mode);
int dlog_bin_fwrite(dlog_fp_t fp, const void *buf, size_t size);
int dlog_bin_fclose(dlog_fp_t fp);

/* frame */
dlog_fp_t dlog_frame_fopen(const char *name, const char *mode);
int dlog_frame_fwrite(dlog_fp_t fp, const void *buf, size_t buf_size, const void *pos_header, size_t pos_header_size);
int dlog_frame_fclose(dlog_fp_t fp);

#ifdef __cplusplus
}
#endif
#endif
