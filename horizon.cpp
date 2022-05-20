#include <iostream>
#include <string.h>
#include <algorithm>
#include <iomanip>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

//#include <map>
//#include <queue>
//#include <utility>

#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"


static const char* type[] = {
"airplane",
"automobile",
"bird",
"cat",
"deer",
"dog",
"frog",
"horse",
"ship",
"truck"
};


int32_t change_image_2_tensor_as_nv12(std::string &image_file,hbDNNTensor *input_tensor) {

  hbDNNTensor *input = input_tensor;
  hbDNNTensorProperties Properties = input->properties;
  //int tensor_id = 0;
  int input_h = Properties.validShape.dimensionSize[1];
  int input_w = Properties.validShape.dimensionSize[2];
  if (Properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    input_h = Properties.validShape.dimensionSize[2];
    input_w = Properties.validShape.dimensionSize[3];
  }
std::cout  << "image_file  "<<image_file<<std::endl;
  cv::Mat bgr_mat = cv::imread(image_file, cv::IMREAD_COLOR);
  if (bgr_mat.empty()) {
     std::cout << "image file not exist!" <<std::endl;
    return -1;
  }
  // resize
  cv::Mat mat;
  mat.create(input_h, input_w, bgr_mat.type());
  cv::resize(bgr_mat, mat, mat.size(), 0, 0);
  // convert to YUV420
  if (input_h % 2 || input_w % 2) {
    std::cout  << "input img height and width must aligned by 2!"<<std::endl;
    return -1;
  }
  cv::Mat yuv_mat;
  cv::cvtColor(mat, yuv_mat, cv::COLOR_BGR2YUV_I420);
  uint8_t *nv12_data = yuv_mat.ptr<uint8_t>();
  
  // copy y data
  auto data = input->sysMem[0].virAddr;
  std::cout << "data = " <<data<<std::endl;
  int32_t y_size = input_h * input_w;
  
  std::cout << "0000000000000000000000000000000000" << std::endl;
  std::cout << "y_size  " << y_size << std::endl;
  std::cout << "nv12_data= " << nv12_data << std::endl;
  printf("*nv12_data= %d \n",*nv12_data);
  //data = (uint8_t *)malloc(y_size);
  memcpy(reinterpret_cast<uint8_t *>(data), nv12_data, y_size);
  std::cout << "mew-data  " << data << std::endl;
  // copy uv data
  int32_t uv_height = input_h / 2;
  int32_t uv_width = input_w / 2;
  std::cout << "ooooooooo" << std::endl; 
  uint8_t *nv12 = reinterpret_cast<uint8_t *>(data) + y_size;//y_size 1024
  printf("*nv12=%d \n",*nv12);
  uint8_t *u_data = nv12_data + y_size;
  uint8_t *v_data = u_data + uv_height * uv_width;
  std::cout << "uv_width * uv_height = " <<uv_width * uv_height<<std::endl;//256
  for (int32_t i = 0; i < uv_width * uv_height; i++) {//  0-255
    
    *nv12++ = *u_data++;
    printf("*u_data=%d \n",*u_data);
    printf("*第一个*nv12=%d \n",*nv12);    
    *nv12++ = *v_data++;
    printf("*v_data=%d \n",*v_data);
    printf("第二个*nv12=%d \n",*nv12);
    /*
    *u_data = *u_data++;
    *nv12 = *nv12++;
    *nv12 = *u_data;
    std::cout << "第一个nv12 : " <<*nv12<<"--地址: "<<nv12<<std::endl;
    std::cout << "u_data : " <<*u_data<<"--地址: "<<u_data<<std::endl;
    *v_data=*v_data++;
    *nv12=*nv12++;
    *nv12=*v_data;
    std::cout << "第二个nv12 : " <<*nv12<<"--地址: "<<nv12<<std::endl;
    std::cout << "v_data : " <<*v_data<<"--地址: "<<v_data<<std::endl;
    */
    std::cout << "i =  " <<i<<std::endl;
  }
  std::cout << "kkkkkkkkk" << std::endl;
  return 0;
}


int main(int argc, char **argv) {
  // 第一步加载模型
  hbPackedDNNHandle_t packed_dnn_handle;
  const char* model_file_name= "./mobilenetv1.bin";
  hbDNNInitializeFromFiles(&packed_dnn_handle, &model_file_name, 1);

  // 第二步获取模型名称
  const char **model_name_list;
  int model_count = 0;
  hbDNNGetModelNameList(&model_name_list, &model_count, packed_dnn_handle);

  // 第三步获取dnn_handle
  hbDNNHandle_t dnn_handle;
  hbDNNGetModelHandle(&dnn_handle, packed_dnn_handle, model_name_list[0]);

  // 第四步准备输入数据
  hbDNNTensor input;
  hbDNNTensorProperties input_properties;
  hbDNNGetInputTensorProperties(&input_properties, dnn_handle, 0);
  input.properties = input_properties;
  auto &mem = input.sysMem[0];

  int yuv_length = 224 * 224 * 3;
  hbSysAllocCachedMem(&mem, yuv_length);
  if (argc < 2)
    {
        std::cerr << "no input config file" << std::endl;
    }
  std::string image_test_file = argv[1];
  change_image_2_tensor_as_nv12(image_test_file,&input);//hbDNNTensor *input_tensor
  //memcpy(mem.virAddr, yuv_data, yuv_length);
  //hbSysFlushMem(&mem, HB_SYS_MEM_CACHE_CLEAN);

  // 第五步准备模型输出数据的空间
  int output_count;
  hbDNNGetOutputCount(&output_count, dnn_handle);
  hbDNNTensor *output = new hbDNNTensor[output_count];
  for (int i = 0; i < output_count; i++) {
  hbDNNTensorProperties &output_properties = output[i].properties;
  hbDNNGetOutputTensorProperties(&output_properties, dnn_handle, i);

  // 获取模型输出尺寸
  int out_aligned_size = 4;
  for (int j = 0; j < output_properties.alignedShape.numDimensions; j++) {
    out_aligned_size =
        out_aligned_size * output_properties.alignedShape.dimensionSize[j];
  }

  hbSysMem &mem = output[i].sysMem[0];
  hbSysAllocCachedMem(&mem, out_aligned_size);
}

  // 第六步推理模型
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  hbDNNInfer(&task_handle,
              &output,
              &input,
              dnn_handle,
              &infer_ctrl_param);

  // 第七步等待任务结束
  hbDNNWaitTaskDone(task_handle, 0);
  //第八步解析模型输出，例子就获取mobilenetv1的top1分类
  float max_prob = -1.0;
  int max_prob_type_id = 0;
  hbSysFlushMem(&(output->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  float *scores = reinterpret_cast<float *>(output->sysMem[0].virAddr);
  int *shape = output->properties.validShape.dimensionSize;
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++)
    {
      if (scores[i] < max_prob)
        continue;
      max_prob = scores[i];
      max_prob_type_id = i;
    }
  std::cout << "type: " << type[max_prob_type_id] << " max_prob: " <<
  max_prob << std::endl;
// 释放内存

hbSysFreeMem(&(input.sysMem[0]));
hbSysFreeMem(&(output->sysMem[0]));

  // 释放模型
  hbDNNReleaseTask(task_handle);
  hbDNNRelease(packed_dnn_handle);

  return 0;
}