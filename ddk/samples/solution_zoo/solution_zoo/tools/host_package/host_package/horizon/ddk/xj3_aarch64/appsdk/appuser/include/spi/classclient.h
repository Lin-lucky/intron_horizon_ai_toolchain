
#ifndef CLASSCLIENT_H
#define CLASSCLIENT_H
#include <vector>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <dirent.h>
#include <time.h>
#include <iostream>
#include <sys/time.h>
#include <fcntl.h>
#include "zmq.h"

#define PER_TIMES 100

class Sub{
public:
  Sub(std::string filepath){
    if (filepath == "") {
      std::cout << "no zmq ipc file input" << std::endl;
    }
    file_path = filepath;
    if ((context = zmq_ctx_new()) == NULL) {
      std::cout << "sub context init failed" << std::endl;
    }
    InitSub();
  }
  ~Sub(){
	  zmq_close(suber);
    zmq_ctx_destroy(context);
  }
  void SetSub(bool inite){sub_inite = inite;}
  int InitSub(){
	  if(access(file_path.c_str(),F_OK) == -1){
	    if(creat(file_path.c_str(), 0755) < 0){
	  	std::cout << "create file failed" << std::endl;
	  	return -1;
	    }
	  }
	  std::string addr = "ipc://" + file_path;
    if ((suber = zmq_socket(context, ZMQ_SUB)) == NULL){
      std::cout << " sub socket init failed" << std::endl;
	    zmq_ctx_destroy(context);
      return -1;
    }
	  int hwm = 5;
	  int rc = zmq_setsockopt(suber, ZMQ_RCVHWM, &hwm, sizeof(hwm));
	  if(rc < 0){
	  	std::cout<<"set recvhwm failed"<<std::endl;
	  	return -1;
	  }
	  int rcvbuf = 16 * 1024;
	  rc = zmq_setsockopt(suber, ZMQ_RCVBUF, &rcvbuf, sizeof(rcvbuf));
	  if(rc < 0){
	  	std::cout<<"set recv buf failed" << std::endl;
	  	return -1;
	  }
    zmq_setsockopt(suber, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(suber, addr.c_str()) < 0){
      std::cout << "sub connect failed: "<<zmq_strerror(errno) << std::endl;
	  return -1;
    }
    std::cout << "connect to: " << addr << std::endl;
	  sub_inite = true;
    return 0;
  }

  std::string file_path;
  bool sub_inite = false;
  void* context;
  void* suber;
  void* puber;
};

#endif  // CLASSCLIENT_H