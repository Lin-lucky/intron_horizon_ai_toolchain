#include "video_source/video_source_type.h"
#include "xstream/vision_type.h"
#include <memory>

inline void ConvertPym2Msg(
    std::shared_ptr<videosource::PyramidFrame> &input,
    std::shared_ptr<xstream::PyramidImageFrame> &output) {
    auto pym_msg_img = std::make_shared<xstream::PyramidImageFrame>();
    pym_msg_img->channel_id_ = input->channel_id_;
    pym_msg_img->time_stamp_ = input->time_stamp_;
    pym_msg_img->frame_id_ = input->frame_id_;
    pym_msg_img->context_ = input->pym_context_;
    int ds_layer_num = static_cast<int>(input->bl_ds_.size());
    int us_layer_num = static_cast<int>(input->roi_us_.size());
    pym_msg_img->img_.ds_pym_layer = ds_layer_num;
    pym_msg_img->img_.us_pym_layer = us_layer_num;
    pym_msg_img->img_.frame_id = input->frame_id_;
    pym_msg_img->img_.timestamp = input->time_stamp_;
    pym_msg_img->img_.cam_id = input->channel_id_;
    {
      pym_msg_img->img_.src_img.width = input->src_info_.width;
      pym_msg_img->img_.src_img.height = input->src_info_.height;
      pym_msg_img->img_.src_img.step = input->src_info_.stride;
      pym_msg_img->img_.src_img.y_paddr = input->src_info_.y_paddr;
      pym_msg_img->img_.src_img.c_paddr = input->src_info_.c_paddr;
      pym_msg_img->img_.src_img.y_vaddr = input->src_info_.y_vaddr;
      pym_msg_img->img_.src_img.c_vaddr = input->src_info_.c_vaddr;
    }
    for (int i = 0; i < ds_layer_num; ++i) {
      pym_msg_img->img_.down_scale[i].width =
        input->bl_ds_[i].width;
      pym_msg_img->img_.down_scale[i].height =
        input->bl_ds_[i].height;
      pym_msg_img->img_.down_scale[i].step =
        input->bl_ds_[i].stride;
      pym_msg_img->img_.down_scale[i].y_paddr =
        input->bl_ds_[i].y_paddr;
      pym_msg_img->img_.down_scale[i].c_paddr =
        input->bl_ds_[i].c_paddr;
      pym_msg_img->img_.down_scale[i].y_vaddr =
        input->bl_ds_[i].y_vaddr;
      pym_msg_img->img_.down_scale[i].c_vaddr =
        input->bl_ds_[i].c_vaddr;
    }
    for (int i = 0; i < us_layer_num; ++i) {
      pym_msg_img->img_.up_scale[i].width = input->roi_us_[i].width;
      pym_msg_img->img_.up_scale[i].height = input->roi_us_[i].height;
      pym_msg_img->img_.up_scale[i].step = input->roi_us_[i].stride;
      pym_msg_img->img_.up_scale[i].y_paddr = input->roi_us_[i].y_paddr;
      pym_msg_img->img_.up_scale[i].c_paddr = input->roi_us_[i].c_paddr;
      pym_msg_img->img_.up_scale[i].y_vaddr = input->roi_us_[i].y_vaddr;
      pym_msg_img->img_.up_scale[i].c_vaddr = input->roi_us_[i].c_vaddr;
    }
    int ds_main_layer_num = ds_layer_num / 4;
    for (int i = 0, j = 0; i < ds_main_layer_num; i++) {
      j = i * 4;
      pym_msg_img->img_.down_scale_main[i].width =
        input->bl_ds_[j].width;
      pym_msg_img->img_.down_scale_main[i].height =
        input->bl_ds_[j].height;
      pym_msg_img->img_.down_scale_main[i].step =
        input->bl_ds_[j].stride;
      pym_msg_img->img_.down_scale_main[i].y_paddr =
        input->bl_ds_[j].y_paddr;
      pym_msg_img->img_.down_scale_main[i].c_paddr =
        input->bl_ds_[j].c_paddr;
      pym_msg_img->img_.down_scale_main[i].y_vaddr =
        input->bl_ds_[j].y_vaddr;
      pym_msg_img->img_.down_scale_main[i].c_vaddr =
        input->bl_ds_[j].c_vaddr;
    }
    output = pym_msg_img;
}
