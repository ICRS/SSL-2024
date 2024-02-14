/*
 * task_motors.c
 *
 *  Created on: Feb 11, 2024
 *      Author: bens1
 */

#include "tasks.h"

void startMotorTask(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
}
