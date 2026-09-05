//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ert_main.cpp
//
// Code generated for Simulink model 'cmdvel2gazebo'.
//
// Model version                  : 1.73
// Simulink Coder version         : 8.12 (R2017a) 16-Feb-2017
// C/C++ source code generated on : Tue May 22 14:50:02 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: Custom Processor->Custom
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include <stdio.h>
#include <stdlib.h>
#include "cmdvel2gazebo.h"
#include "cmdvel2gazebo_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#include <rclcpp/rclcpp.hpp>
#include "slros_initialize.h"
#define UNUSED(x)                      x = x

// Function prototype declaration
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
unsigned long threadJoinStatus[8];
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(cmdvel2gazebo_M) == (NULL)) &&
    !rtmGetStopRequested(cmdvel2gazebo_M);
  while (runModel) {
    sem_wait(&baserateTaskSem);
    cmdvel2gazebo_step();

    // Get model outputs here
    runModel = (rtmGetErrorStatus(cmdvel2gazebo_M) == (NULL)) &&
      !rtmGetStopRequested(cmdvel2gazebo_M);
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(cmdvel2gazebo_M, "stopping the model");
  runModel = false;
  sem_post(&stopSem);
}

void *terminateTask(void *arg)
{
  UNUSED(arg);

  terminatingmodel = 1;

  runModel = 0;

  sem_post(&stopSem);

  pthread_exit((void *)0);

  return NULL;
}

int main(int argc, char **argv)
{

  slros_node_init(argc, argv);

  rtmSetErrorStatus(cmdvel2gazebo_M, 0);


  // Initialize model
  cmdvel2gazebo_initialize();


  // Start generated RTOS scheduler
  myRTOSInit(0.02, 0);


  // ROS2 executor
  rclcpp::spin(SLROSNodePtr);


  // Shutdown ROS2
  rclcpp::shutdown();


  sem_post(&stopSem);

  cmdvel2gazebo_terminate();

  rclcpp::shutdown();

  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
