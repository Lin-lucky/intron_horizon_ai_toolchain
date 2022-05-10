
#ifndef CLASSSERVER_H
#define CLASSSERVER_H
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

class Pub{
public:
  Pub(std::string filepath){
    if (filepath == "") {
      std::cout << "no zmq ipc file input" << std::endl;
    }
    file_path = filepath;
	  if ((context = zmq_ctx_new()) == NULL){
      std::cout << "pub context init failed" << std::endl;
    }
    InitPub();
  }
  ~Pub(){
	  zmq_close(puber);
    zmq_ctx_destroy(context);
  }
  int InitPub(){
	  if(access(file_path.c_str(),F_OK) == -1){
	    if(creat(file_path.c_str(), 0755) < 0){
	  	std::cout << "create file failed" << std::endl;
	  	return -1;
	    }
	  }
	  std::string addr = "ipc://" + file_path;
      if ((puber = zmq_socket(context, ZMQ_PUB)) == NULL){
        std::cout << " pub socket init failed" << std::endl;
        return -1;
      }
	  int hwm = 10;
	  int rc = zmq_setsockopt(puber,ZMQ_SNDHWM,&hwm,sizeof(hwm));
	  if(rc < 0){
	  	std::cout<<"set sndhwm failed"<<std::endl;
	  	return -1;
	  }

	  int linger = 1000;
	  rc = zmq_setsockopt(puber,ZMQ_LINGER,&linger,sizeof(linger));
	  if(rc < 0){
	  	std::cout<<"set linger failed"<<std::endl;
	  	return -1;
	  }

	  int sndbuf = 16 * 1024;
	  rc = zmq_setsockopt(puber,ZMQ_SNDBUF,&sndbuf,sizeof(sndbuf));
	  if(rc < 0){
	  	std::cout<<"set sndbuf failed"<<std::endl;
	  	return -1;
	  }

	  if (zmq_bind(puber, addr.c_str()) < 0){
        std::cout << "pub bind failed: "<<zmq_strerror(errno) << std::endl;
	    return -1;
    }

	  pub_inite = true;
    usleep(150000);
    return 0;
  }

  std::string file_path;
  bool pub_inite = false;
  void* context;
  void* puber;
};

#endif  // CLASSSERVER_H