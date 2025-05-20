/**
 * @file task_scheduler.c
 * @brief Task scheduler implementation for STM32F429ZI
 * @desc Phase 1: Foundation - Basic task management and dispatcher
 */

#include "task_scheduler.h"
#include "uart.h"
#include "systick.h"
#include <string.h>
#include <stdio.h>

/* ====================== Global Variables ====================== */

/* Task array - supports MAX_TASKS concurrent tasks */
static Task_t tasks[MAX_TASKS];

/* Task counter for unique IDs */
static uint8_t nextTaskId = 0;

/* Registered task count */
static uint8_t registeredTaskCount = 0;

/* ====================== Initialization Functions ====================== */

void TaskScheduler_Init(void) {
    /* Clear all task entries */
    memset(tasks, 0, sizeof(tasks));

    /* Reset counters */
    nextTaskId = 0;
    registeredTaskCount = 0;

    /* Log initialization */
    UART_SendString("\r\n[SCHEDULER] Task scheduler initialized\r\n");
}

/* =================== Task Registration and Management =================== */

TaskError_t TaskScheduler_RegisterTask(void (*taskFunc)(void),
                                      uint32_t period_ms,
                                      uint8_t priority,
                                      const char* name,
                                      uint8_t* taskId) {
    /* Check if we have space for more tasks */
    if (registeredTaskCount >= MAX_TASKS) {
        return TASK_ERROR_FULL;
    }

    /* Validate function pointer */
    if (taskFunc == NULL) {
        return TASK_ERROR_INVALID_PARAM;
    }

    /* Find empty slot */
    uint8_t slot = MAX_TASKS;
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].state == TASK_STATE_STOPPED && tasks[i].taskFunc == NULL) {
            slot = i;
            break;
        }
    }

    if (slot == MAX_TASKS) {
        return TASK_ERROR_FULL;
    }

    /* Initialize task */
    tasks[slot].taskFunc = taskFunc;
    tasks[slot].period_ms = period_ms;
    tasks[slot].lastExecuted = systick_counter;
    tasks[slot].maxExecutionTime = 0;
    tasks[slot].maxJitter = 0;
    tasks[slot].state = TASK_STATE_READY;
    tasks[slot].priority = priority;
    tasks[slot].taskId = nextTaskId++;

    /* Set name if provided, otherwise use default */
    if (name != NULL) {
        strncpy(tasks[slot].name, name, sizeof(tasks[slot].name) - 1);
        tasks[slot].name[sizeof(tasks[slot].name) - 1] = '\0';
    } else {
        snprintf(tasks[slot].name, sizeof(tasks[slot].name), "Task_%d", tasks[slot].taskId);
    }

    /* Return task ID if requested */
    if (taskId != NULL) {
        *taskId = tasks[slot].taskId;
    }

    registeredTaskCount++;

    /* Log registration */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[SCHEDULER] Registered task '%s' (ID: %d, Period: %lums)\r\n",
             tasks[slot].name, tasks[slot].taskId, tasks[slot].period_ms);
    UART_SendString(debug_msg);

    return TASK_OK;
}

TaskError_t TaskScheduler_UnregisterTask(uint8_t taskId) {
    /* Find task by ID */
    uint8_t slot = MAX_TASKS;
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId && tasks[i].taskFunc != NULL) {
            slot = i;
            break;
        }
    }

    if (slot == MAX_TASKS) {
        return TASK_ERROR_NOT_FOUND;
    }

    /* Log unregistration */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[SCHEDULER] Unregistering task '%s' (ID: %d)\r\n",
             tasks[slot].name, tasks[slot].taskId);
    UART_SendString(debug_msg);

    /* Clear task entry */
    memset(&tasks[slot], 0, sizeof(Task_t));
    registeredTaskCount--;

    return TASK_OK;
}

/* ======================= Task Control Functions ======================= */

TaskError_t TaskScheduler_StartTask(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            tasks[i].state = TASK_STATE_READY;
            tasks[i].lastExecuted = systick_counter;

            /* Log start */
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[SCHEDULER] Started task '%s' (ID: %d)\r\n",
                     tasks[i].name, taskId);
            UART_SendString(debug_msg);

            return TASK_OK;
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

TaskError_t TaskScheduler_StopTask(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            tasks[i].state = TASK_STATE_STOPPED;

            /* Log stop */
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[SCHEDULER] Stopped task '%s' (ID: %d)\r\n",
                     tasks[i].name, taskId);
            UART_SendString(debug_msg);

            return TASK_OK;
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

/* ===================== Scheduler Execution Functions ===================== */

void TaskScheduler_RunDispatcher(void) {
    uint32_t currentTime = systick_counter;

    /* First pass: Check all tasks and find the highest priority ready task */
    uint8_t selectedTask = MAX_TASKS;
    uint8_t highestPriority = 255; /* Lowest priority number */

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        /* Skip empty or stopped tasks */
        if (tasks[i].taskFunc == NULL || tasks[i].state != TASK_STATE_READY) {
            continue;
        }

        /* Check if task period has elapsed */
        if ((currentTime - tasks[i].lastExecuted) >= tasks[i].period_ms) {
            /* Select higher priority task (lower priority number) */
            if (tasks[i].priority < highestPriority) {
                highestPriority = tasks[i].priority;
                selectedTask = i;
            }
        }
    }

    /* Execute selected task if found */
    if (selectedTask < MAX_TASKS) {
        Task_t* task = &tasks[selectedTask];

        /* Update state to running */
        task->state = TASK_STATE_RUNNING;

        /* Record start time for execution measurement */
        uint32_t startTime = systick_counter;

        /* Execute task function */
        task->taskFunc();

        /* Record execution time */
        uint32_t endTime = systick_counter;
        uint32_t executionTime = endTime - startTime;

        /* Update max execution time if needed */
        if (executionTime > task->maxExecutionTime) {
            task->maxExecutionTime = executionTime;
        }

        /* Calculate jitter - difference between scheduled and actual execution */
        uint32_t scheduledTime = task->lastExecuted + task->period_ms;
        uint32_t actualTime = startTime;
        uint32_t jitter = (actualTime > scheduledTime) ?
                          (actualTime - scheduledTime) :
                          (scheduledTime - actualTime);

        /* Update max jitter if needed */
        if (jitter > task->maxJitter) {
            task->maxJitter = jitter;
        }

        /* Update last executed time */
        task->lastExecuted = currentTime;

        /* Return to ready state */
        task->state = TASK_STATE_READY;
    }
}

bool TaskScheduler_HasPendingTasks(void) {
    uint32_t currentTime = systick_counter;

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        /* Skip empty or stopped tasks */
        if (tasks[i].taskFunc == NULL || tasks[i].state != TASK_STATE_READY) {
            continue;
        }

        /* Check if task period has elapsed */
        if ((currentTime - tasks[i].lastExecuted) >= tasks[i].period_ms) {
            return true;
        }
    }

    return false;
}

/* ================== Information and Debugging Functions ================== */

uint8_t TaskScheduler_GetTaskCount(void) {
    return registeredTaskCount;
}

TaskError_t TaskScheduler_GetTaskInfo(uint8_t taskId, TaskInfo_t* info) {
    /* Validate info pointer */
    if (info == NULL) {
        return TASK_ERROR_INVALID_PARAM;
    }

    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId && tasks[i].taskFunc != NULL) {
            /* Copy task information */
            strcpy(info->name, tasks[i].name);
            info->period_ms = tasks[i].period_ms;
            info->lastExecuted = tasks[i].lastExecuted;
            info->maxExecutionTime = tasks[i].maxExecutionTime;
            info->maxJitter = tasks[i].maxJitter;
            info->state = tasks[i].state;
            info->priority = tasks[i].priority;
            info->taskId = tasks[i].taskId;

            return TASK_OK;
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

void TaskScheduler_PrintStatus(void) {
    char debug_msg[200];

    UART_SendString("\r\n=== Task Scheduler Status ===\r\n");
    snprintf(debug_msg, sizeof(debug_msg),
             "Registered Tasks: %d/%d\r\n", registeredTaskCount, MAX_TASKS);
    UART_SendString(debug_msg);

    UART_SendString("Task Details:\r\n");
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskFunc != NULL) {
            const char* stateStr[] = {"STOPPED", "READY", "RUNNING", "SUSPENDED"};
            snprintf(debug_msg, sizeof(debug_msg),
                     "  [%d] %s: State=%s, Pri=%d, Period=%lums, LastExec=%lums, MaxExecTime=%lums, MaxJitter=%lums\r\n",
                     tasks[i].taskId, tasks[i].name, stateStr[tasks[i].state],
                     tasks[i].priority, tasks[i].period_ms, tasks[i].lastExecuted,
                     tasks[i].maxExecutionTime, tasks[i].maxJitter);
            UART_SendString(debug_msg);
        }
    }
    UART_SendString("============================\r\n");
}
// Add these at the bottom of task_scheduler.c

/* ======================= Task Control Functions (Phase 2) ======================= */

TaskError_t TaskScheduler_SuspendTask(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            /* Only suspend if task is currently ready or running */
            if (tasks[i].state == TASK_STATE_READY || tasks[i].state == TASK_STATE_RUNNING) {
                tasks[i].state = TASK_STATE_SUSPENDED;

                /* Log suspension */
                char debug_msg[100];
                snprintf(debug_msg, sizeof(debug_msg),
                         "[SCHEDULER] Suspended task '%s' (ID: %d)\r\n",
                         tasks[i].name, taskId);
                UART_SendString(debug_msg);

                return TASK_OK;
            } else {
                return TASK_ERROR_INVALID_ID;
            }
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

TaskError_t TaskScheduler_ResumeTask(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            /* Only resume if task is currently suspended */
            if (tasks[i].state == TASK_STATE_SUSPENDED) {
                tasks[i].state = TASK_STATE_READY;
                tasks[i].lastExecuted = systick_counter; /* Reset timing */

                /* Log resumption */
                char debug_msg[100];
                snprintf(debug_msg, sizeof(debug_msg),
                         "[SCHEDULER] Resumed task '%s' (ID: %d)\r\n",
                         tasks[i].name, taskId);
                UART_SendString(debug_msg);

                return TASK_OK;
            } else {
                return TASK_ERROR_INVALID_ID;
            }
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

/* ==================== Task Configuration Functions (Phase 2) ==================== */

TaskError_t TaskScheduler_SetPeriod(uint8_t taskId, uint32_t period_ms) {
    /* Validate new period */
    if (period_ms == 0) {
        return TASK_ERROR_INVALID_PARAM;
    }

    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            tasks[i].period_ms = period_ms;
            tasks[i].lastExecuted = systick_counter; /* Reset timing */

            /* Log period change */
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[SCHEDULER] Changed period for task '%s' to %lums\r\n",
                     tasks[i].name, period_ms);
            UART_SendString(debug_msg);

            return TASK_OK;
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

TaskError_t TaskScheduler_SetPriority(uint8_t taskId, uint8_t priority) {
    /* Validate priority range */
    if (priority > TASK_PRIORITY_IDLE) {
        return TASK_ERROR_INVALID_PARAM;
    }

    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            tasks[i].priority = priority;

            /* Log priority change */
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[SCHEDULER] Changed priority for task '%s' to %d\r\n",
                     tasks[i].name, priority);
            UART_SendString(debug_msg);

            return TASK_OK;
        }
    }

    return TASK_ERROR_NOT_FOUND;
}

// Add these after your existing Phase 2 functions:

/* =================== Performance Monitoring Functions (Phase 3) =================== */

uint32_t TaskScheduler_GetTaskExecutionTime(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            return tasks[i].maxExecutionTime;
        }
    }

    return 0; /* Task not found */
}

uint32_t TaskScheduler_GetMaxJitter(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            return tasks[i].maxJitter;
        }
    }

    return 0; /* Task not found */
}

void TaskScheduler_ResetStatistics(uint8_t taskId) {
    /* Find task by ID */
    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskId == taskId) {
            tasks[i].maxExecutionTime = 0;
            tasks[i].maxJitter = 0;

            /* Log reset */
            char debug_msg[100];
            snprintf(debug_msg, sizeof(debug_msg),
                     "[SCHEDULER] Reset statistics for task '%s'\r\n",
                     tasks[i].name);
            UART_SendString(debug_msg);

            return;
        }
    }
}

/* ===================== System Control Functions (Phase 3) ===================== */

void TaskScheduler_SuspendAll(void) {
    uint8_t suspended_count = 0;

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskFunc != NULL &&
            (tasks[i].state == TASK_STATE_READY || tasks[i].state == TASK_STATE_RUNNING)) {
            tasks[i].state = TASK_STATE_SUSPENDED;
            suspended_count++;
        }
    }

    /* Log mass suspension */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[SCHEDULER] Suspended %d tasks\r\n", suspended_count);
    UART_SendString(debug_msg);
}

void TaskScheduler_ResumeAll(void) {
    uint8_t resumed_count = 0;
    uint32_t currentTime = systick_counter;

    for (uint8_t i = 0; i < MAX_TASKS; i++) {
        if (tasks[i].taskFunc != NULL && tasks[i].state == TASK_STATE_SUSPENDED) {
            tasks[i].state = TASK_STATE_READY;
            tasks[i].lastExecuted = currentTime; /* Reset timing */
            resumed_count++;
        }
    }

    /* Log mass resumption */
    char debug_msg[100];
    snprintf(debug_msg, sizeof(debug_msg),
             "[SCHEDULER] Resumed %d tasks\r\n", resumed_count);
    UART_SendString(debug_msg);
}
