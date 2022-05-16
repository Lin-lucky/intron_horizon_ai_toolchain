#include <iostream>
#include <string.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

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

/*
int Changejpgtoyuv420()
{
	  IplImage *pstImage = NULL;
		IplImage *pstYUVImage = NULL;
		FILE *fp = NULL;
	
		pstImage = cvLoadImage("../test_cifr10/9999.jpg", CV_LOAD_IMAGE_COLOR);
    std::cout << "**************" <<std::endl;
		fp = fopen("../test_cifr10", "wb");
		pstYUVImage = cvCreateImage(cvSize(pstImage->width, pstImage->height), IPL_DEPTH_8U, 3);
	
		cvCvtColor(pstImage, pstYUVImage, CV_BGR2YUV);
		
		for(int i = 0; i < pstImage->width * pstImage->height; i++)
		{
			//提取Y分量
			fwrite(&pstYUVImage->imageData[i*3], 1 , 1, fp);
			//提取U分量
			//fwrite(&pstYUVImage->imageData[i*3+2], 1 , 1, fp);
			//提取V分量
			//fwrite(&pstYUVImage->imageData[i*3+1], 1 , 1, fp);
		}
	
		for(int i = 0; i <	pstImage->height; i = i+2)
		{
			for(int j = 0; j < pstImage->width; j= j+2)
			{
				//提取U分量
				fwrite(&pstYUVImage->imageData[3*(i*pstImage->width + j)+2], 1 , 1, fp);
			}
		}
		
		for(int i = 0; i <	pstImage->height; i = i+2)
		{
			for(int j = 0; j < pstImage->width; j = j+2)
			{
				//提取V分量
				fwrite(&pstYUVImage->imageData[3*(i*pstImage->width + j)+1], 1 , 1, fp);
			}
		}
		
		cvShowImage("Win",pstImage);
	
		cvWaitKey(0);
		cvReleaseImage(&pstImage);
		cvReleaseImage(&pstYUVImage);
		fclose(fp);
		return 0;

}
*/
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
  
  //IplImage *pstImage = NULL;
  //IplImage *pstYUVImage = NULL;
  //Changejpgtoyuv420();
  
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
  for (auto i = 0; i < shape[1] * shape[2] * shape[3]; i++) {
    std::cout << "NO==scores[i]: " << scores[i] << std::endl;
    std::cout << "NO==max_prob: " << max_prob << std::endl;
      if(scores[i] < max_prob)
        continue;
    max_prob = scores[i];
    std::cout << "for==max_prob: " << max_prob << std::endl;
    max_prob_type_id = i;
    std::cout << "for==max id: " << max_prob_type_id << std::endl;
  }

  std::cout << "type: " << type[max_prob_type_id] << " max_prob: " << max_prob << std::endl;

  std::cout << "max id: " << max_prob_type_id << std::endl;
  // 释放内存
  hbSysFreeMem(&(input.sysMem[0]));
  std::cout << "************************" << std::endl;
  hbSysFreeMem(&(output->sysMem[0]));
  std::cout << "++++++++++++++++++++++++" << std::endl;
  // 释放模型
  hbDNNReleaseTask(task_handle);
  std::cout << "2222222222222222222222222" << std::endl;
  hbDNNRelease(packed_dnn_handle);
  std::cout << "999999999999999999999999" << std::endl;

  return 0;
}
