#ifndef __SYS_NOTIFY_H__
#define __SYS_NOTIFY_H__

#ifdef __cplusplus
extern "C" {
#endif

enum module_id {
	ModuleDiagr = 1,
	Module_I2C,
	Module_VIO,
	Module_BPU,
	Module_SOUND,
	Module_BIF,
	Module_ETH,
	ModuleIdMax = 1000,
};

/*
 * sys notify type defines, may be add with
 * the improvement of system
 */
#define BPU_FREQ_CHANGE(core_id) (50 + core_id)

/*
 * register callback, which will be called when  notify come 
 */
typedef void (*notify_cb)(void *data, int len);
int sys_notify_register(int module_id, int type_id, notify_cb cb);

void sys_notify_unregister(int module_id, int type);

#ifdef __cplusplus
}
#endif

#endif
