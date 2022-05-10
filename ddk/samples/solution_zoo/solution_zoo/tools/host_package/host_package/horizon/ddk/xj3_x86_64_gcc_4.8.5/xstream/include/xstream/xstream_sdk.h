/**
 * Copyright (c) 2018 Horizon Robotics. All rights reserved.
 * @brief     provides xstream framework interface
 * @file      xstream_sdk.h
 * @author    chuanyi.yang
 * @email     chuanyi.yang@horizon.ai
 * @version   0.0.0.1
 * @date      2018.11.21
 */
#ifndef XSTREAM_XSTREAM_SDK_H_
#define XSTREAM_XSTREAM_SDK_H_

#include <string>
#include <vector>

#include "xstream/version.h"
#include "xstream/xstream_data.h"

namespace xstream {

/**
 * Example Usage:
 * xstream::XStreamSDK *flow = xstream::XStreamSDK::CreateSDK();
 * flow->SetConfig("config_file", config);
 * flow->Init();
 * InputDataPtr inputdata(new InputData());
 * // ... construct inputdata
 * auto out = flow->SyncPredict(inputdata);
 * // PrintOut(out);
 * // ... handle outputdata
 * delete flow;
 */

/// XStream API
class XSTREAM_EXPORT XStreamSDK {
 public:
  /// Base class(derived class need to be defined, such as XStreamFlow)
  virtual ~XStreamSDK() {}
  /// creat SDK instance
  static XStreamSDK *CreateSDK();

  /**
   * @description: Set the configuration of the entire workflow.
   *               Currently supported configurations including workflow
   * configuration files, performance statistics profiler related functions,
   * timeout alarms, and release of intermediate data within frames.
   * @param:
   * 1) The key[in] is "config_file", and the value[in] is set to the workflow
   * configuration file path, which defines the entire workflow data flow.
   * 2) The key[in] is "config_string", and the value[in] is set to the workflow
   *  configuration json string, which defines the entire workflow data flow.
   * 3) The key[in] is "profiler" and the value[in] is "on", which means that
   * the performance statistics function is turned on. "off" means off, and the
   * default is off.
   * 4) The key[in] is "profiler_file", and the value[in] is the
   * path of the performance statistics output file, which is used to set the
   * path name of the performance statistics file. When the performance
   * statistics function is turned on, the output file path needs to be set.
   * 5) The key[in] is "profiler_time_interval", and the value[in] is the
   * minimum time interval for statistics, in microseconds. If the
   * time-consuming statistics somewhere is less than the value, this
   * time-consuming record will not be added to the performance statistics
   * results. The default value is 3000 microseconds, which is 3 milliseconds.
   * 6) The key[in] is "profiler_fps_interval", and the value[in] is the minimum
   * time interval for statistical frame rate, in microseconds. If the time
   * taken to count Fps somewhere is less than the value, this fps record will
   * not be added to the performance statistics results. The default is 200*1000
   * microseconds, which is 200 milliseconds.
   * 7) The key[in] is "free_framedata"
   * and the value[in] is "on", which means that an intermediate data in the
   * frame that is no longer needed in the subsequent node nodes will be
   * released as soon as possible. Turning on this configuration can reduce the
   * peak memory usage. Off by default.
   * 8) The key[in] is "time_monitor", and
   * the value[in] is the maximum time for processing each frame of data in the
   * input frame, in milliseconds. If the value is not 0, the timeout alarm
   * function is turned on. The value defaults to 0, which means that the
   * timeout alarm function is disabled.
   * @return:
   * The interface returns 0 to indicate that the setting is successful, and
   * returns -1 to indicate that the setting fails.
   */
  virtual int SetConfig(const std::string &key, const std::string &value) = 0;

  /**
   * @description: Initialize the workflow, note that Init() must be executed
   * after calling SetConfig.
   * @param: none
   * @return:
   * Returns 0 to indicate that the initialization is successful, otherwise it
   * indicates that the initialization failed.
   */
  virtual int Init() = 0;
  /// Update InputParam for specified method instance
  /**
   * @description: Update the parameters of the node.
   * @param unique_name[in],the unique name of a node (corresponding to the node
   * name in the workflow configuration), which means to update the parameters
   * of the node.
   * @param ptr[in], The parameter information to be updated.
   * @return:
   * Return 0 to indicate that the parameter update is successful, otherwise it
   * fails.
   */
  virtual int UpdateConfig(const std::string &unique_name,
                           InputParamPtr ptr) = 0;

  /**
   * @description: Get the parameters of a node.
   * @param unique_name[in],the unique name of a node (corresponding to the node
   * name in the workflow configuration), which means to get the parameters of
   * the node.
   * @return: InputParamPtr.
   */
  virtual InputParamPtr GetConfig(const std::string &unique_name) const = 0;
  /// Get Version of specified method
  virtual std::string GetVersion(const std::string &unique_name) const = 0;

  /**
   * @description: SyncPredict Func for SingleOutput mode. Run the interface
   * synchronously and transfer data. The interface will block until the entire
   * workflow is processed, and the output result of the workflow is returned
   * through the function return value.
   * @param {InputDataPtr} input[in], The input data to be processed, which
   * needs to be encapsulated by the develper.
   * @return Return the OuputDataptr, the output result of workflow.
   */
  virtual OutputDataPtr SyncPredict(InputDataPtr input) = 0;

  /**
   * @description: SyncPredict Func for MultiOutput mode.When data is
   * transferred, the interface will block until the entire workflow is
   * processed, and the output result of the workflow is returned through the
   * function return value.
   * @param {InputDataPtr} input[in], The input data to be processed, which
   * needs to be encapsulated by the develper.
   * @return Returns the OutputDataPtr array, and outputs multiple sets of
   * output results of workflow.
   */
  virtual std::vector<OutputDataPtr> SyncPredict2(InputDataPtr input) = 0;

  /**
   *  Set callback func, only support async mode.When using a synchronous
   * interface, the interface is invalid. Note that SetCallback must be executed
   * after calling Init, otherwise the node and other information is not yet
   * complete, and setting the callback function will fail.
   *
   * Note: XStreamSDK should be inited(Init()) before SetCallback()
   * @param callback [in], callback function.
   * @param name [in], workflow node name;
   * name = "": default param, set callback for final outputdata;
   * name: node name, set callback for specifed node, callback func handle the
   * node's outputdata
   * @return:
   * The interface returns 0 to indicate that the setting is successful,
   * otherwise it indicates that the setting has failed.
   */
  virtual int SetCallback(XStreamCallback callback,
                          const std::string &name = "") = 0;
  /**
   * @description: Asynchronous predict function, return immediately after
   * calling the AsyncPredict interface, and the output result of workflow is
   * processed through the callback function.
   * @param {InputDataPtr} input[in], The input data to be processed, which
   * needs to be encapsulated by the develper.
   * @return:
   * If the interface return value is greater than or equal to 0, it means that
   * the task is successfully added to the queue, and the return code indicates
   * the sequence_id of the input source; if the interface return value is less
   * than 0, it means that the task queue is full and adding a task failed.
   */
  virtual int64_t AsyncPredict(InputDataPtr input) = 0;

  /**
   * @description: Get the length of the current task queue in the framework,
   * that is, the number of unfinished tasks.
   * @param None
   * @return Returns the number of unfinished tasks.
   */
  virtual int64_t GetTaskNum() = 0;
};

}  // namespace xstream

#endif  // XSTREAM_XSTREAM_SDK_H_
