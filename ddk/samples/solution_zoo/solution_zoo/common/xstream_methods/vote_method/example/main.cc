/**
 * Copyright (c) 2019 Horizon Robotics. All rights reserved.
 * @brief     test
 * @author    yaoyao.sun
 * @date      2019.04.12
 */

#include <iostream>
#include <map>
#include <string>

#include "hobotlog/hobotlog.hpp"

typedef int (*example_fn)(int, char **);

extern int TestAntiSpfCalculate(int argc, char **argv);

int PrintUsage() {
  std::cout << "Usage: example [command] [args]\n"
               "\n"
               "Run a specified example.\n"
               "\n"
               "Command:\n"
               "  anti_spf_calculate\n"
               "\n"
               "Extra:\n"
               " \n"
               "You can use tools such as valrind to check memory leaks\n"
               "valgrind --leak-check=full"
               " --show-leak-kinds=all example [command] [subcommand] "
            << std::endl;
  return 0;
}

int main(int argc, char **argv) {
  TestAntiSpfCalculate(argc, argv);
  return 0;
}
