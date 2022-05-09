# this script run model_inference sample

function run_roi_task() {
    cd roi_task
    chmod +x roi_resizer_task
    ./roi_resizer_task ./configs/video_source/x3dev/feedback/x3_feedback_1080p_chn0.json ./configs/age_gender_config.json
    cd ..
}

function run_tensor_task() {
    cd tensor_task
    chmod +x tensor_task
    ./tensor_task ./configs/1080p.nv12 1080 1920
    cd ..
}

echo "run model_inference sample start."
export LD_LIBRARY_PATH=../lib:$LD_LIBRARY_PATH
run_roi_task
run_tensor_task
echo "run model_inference sample end."