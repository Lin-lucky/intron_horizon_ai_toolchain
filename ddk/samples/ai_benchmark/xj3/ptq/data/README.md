### data

You should mount preprocess data on the board, and do soft link here. Preprocess data is for model accuracy test.
You can read workflow_accuracy.json,  "image_list_file" indicates the processed data which will be loaded.

How to mount file "/my_nfs_root" (server mechine) on board path "/mnt" ?

  PC server(root authority is necessary, IP is: 192.168.1.1):
    1、edit /etc/exports, add content: 
      /my_nfs_root *(insecure,rw,sync,all_squash,anonuid=1000,anongid=1000,no_subtree_check)

    2、validate set:
      exportfs -a -r
  *notes:"/my_nfs_root" is the file which can be mount on other machine*

  On Board:
    1、mount -o remount rw /
    2、mkdir /mnt
    3、mount -t nfs 192.168.1.1:/my_nfs_root /mnt -o nolock
  *notes:"/mnt" is the file path you want to mount on this board, 192.168.1.1 is server IP*


How to produce lst file? (such as coco.lst)
  find * -name ../../../coco/ >> coco.lst
  *note: path "../../../coco/" contains preprocessed data set for model*

How to produce preprocess data ?
  hb_eval_preprocess --model_name model_name --image_dir data_dir --output_dir output_dir
  *note: Env is your server machine, make sure you have installed horizon-tc-ui*
