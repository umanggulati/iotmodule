/**
 * @file task_scheduler.h
 * @brief Simple priority-based task scheduler for STM32F429ZI
 * @desc Implements a cooperative task scheduler using SysTick timer
 */

#ifndef TASK_SCHEDULER_H
#define TASK_SCHEDULER_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum number of tasks the scheduler can handle */
#define MAX_TASKS           8

/* Task priorities (0 = highest priority) */
#define TASK_PRIORITY_HIGH      0
#define TASK_PRIORITY_NORMAL    1
#define TASK_PRIORITY_LOW       2
#define TASK_PRIORITY_IDLE      3

/* Task states */
typedef enum {
    TASK_STATE_STOPPED = 0,
    TASK_STATE_READY,
    TASK_STATE_RUNNING,
    TASK_STATE_SUSPENDED
} TaskState_t;

/* Task status codes */
typedef enum {
    TASK_OK = 0,
    TASK_ERROR_FULL,
    TASK_ERROR_NOT_FOUND,
    TASK_ERROR_INVALID_ID,
    TASK_ERROR_ALREADY_EXISTS,
    TASK_ERROR_INVALID_PARAM
} TaskError_t;

/* Task control block */
typedef struct {
    void (*taskFunc)(void);        /* Task function pointer */
    uint32_t period_ms;           /* Task period in milliseconds */
    uint32_t lastExecuted;        /* Last execution time */
    uint32_t maxExecutionTime;    /* Max execution time measured */
    uint32_t maxJitter;           /* Maximum scheduling jitter */
    TaskState_t state;            /* Current task state */
    uint8_t priority;             /* Task priority (0 = highest) */
    uint8_t taskId;               /* Unique task identifier */
    char name[16];                /* Task name for debugging */
} Task_t;

/* Task information structure for debugging */
typedef struct {
    char name[16];
    uint32_t period_ms;
    uint32_t lastExecuted;
    uint32_t maxExecutionTime;
    uint32_t maxJitter;
    TaskState_t state;
    uint8_t priority;
    uint8_t taskId;
} TaskInfo_t;

/* ====================== Initialization Functions ====================== */

/**
 * @brief Initialize the task scheduler
 * @param None
 * @return None
 * @note Must be called before registering any tasks
 */
void TaskScheduler_Init(void);

/* =================== Task Registration and Management =================== */

/**
 * @brief Register a new task with the scheduler
 * @param taskFunc: Function pointer to the task
 * @param period_ms: Task execution period in milliseconds
 * @param priority: Task priority (0 = highest)
 * @param name: Task name for debugging (optional, can be NULL)
 * @param taskId: Pointer to store assigned task ID
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_RegisterTask(void (*taskFunc)(void),
                                      uint32_t period_ms,
                                      uint8_t priority,
                                      const char* name,
                                      uint8_t* taskId);

/**
 * @brief Unregister a task from the scheduler
 * @param taskId: ID of the task to unregister
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_UnregisterTask(uint8_t taskId);

/* ======================= Task Control Functions ======================= */

/**
 * @brief Start a stopped task
 * @param taskId: ID of the task to start
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_StartTask(uint8_t taskId);

/**
 * @brief Stop a running task
 * @param taskId: ID of the task to stop
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_StopTask(uint8_t taskId);

/**
 * @brief Suspend a task (temporarily pause without losing state)
 * @param taskId: ID of the task to suspend
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_SuspendTask(uint8_t taskId);

/**
 * @brief Resume a suspended task
 * @param taskId: ID of the task to resume
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_ResumeTask(uint8_t taskId);

/* ==================== Task Configuration Functions ==================== */

/**
 * @brief Change the execution period of a task
 * @param taskId: ID of the task
 * @param period_ms: New period in milliseconds
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_SetPeriod(uint8_t taskId, uint32_t period_ms);

/**
 * @brief Change the priority of a task
 * @param taskId: ID of the task
 * @param priority: New priority (0 = highest)
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_SetPriority(uint8_t taskId, uint8_t priority);

/* ===================== Scheduler Execution Functions ===================== */

/**
 * @brief Run the task dispatcher (main scheduler loop)
 * @param None
 * @return None
 * @note Should be called periodically from the main loop
 */
void TaskScheduler_RunDispatcher(void);

/**
 * @brief Check if any tasks are pending execution
 * @param None
 * @return bool: true if pending tasks exist
 */
bool TaskScheduler_HasPendingTasks(void);

/* ================== Information and Debugging Functions ================== */

/**
 * @brief Get information about a specific task
 * @param taskId: ID of the task
 * @param info: Pointer to TaskInfo_t structure to fill
 * @return TaskError_t: Task operation result
 */
TaskError_t TaskScheduler_GetTaskInfo(uint8_t taskId, TaskInfo_t* info);

/**
 * @brief Get the number of registered tasks
 * @param None
 * @return uint8_t: Number of registered tasks
 */
uint8_t TaskScheduler_GetTaskCount(void);

/**
 * @brief Print scheduler status via UART (debugging)
 * @param None
 * @return None
 */
void TaskScheduler_PrintStatus(void);

/* =================== Performance Monitoring Functions =================== */

/**
 * @brief Get the maximum execution time of a task
 * @param taskId: ID of the task
 * @return uint32_t: Maximum execution time in milliseconds
 */
uint32_t TaskScheduler_GetTaskExecutionTime(uint8_t taskId);

/**
 * @brief Get the maximum jitter of a task
 * @param taskId: ID of the task
 * @return uint32_t: Maximum jitter in milliseconds
 */
uint32_t TaskScheduler_GetMaxJitter(uint8_t taskId);

/**
 * @brief Reset performance statistics for a task
 * @param taskId: ID of the task
 * @return None
 */
void TaskScheduler_ResetStatistics(uint8_t taskId);

/* ===================== System Control Functions ===================== */

/**
 * @brief Suspend all tasks
 * @param None
 * @return None
 */
void TaskScheduler_SuspendAll(void);

/**
 * @brief Resume all suspended tasks
 * @param None
 * @return None
 */
void TaskScheduler_ResumeAll(void);

#endif /* TASK_SCHEDULER_H */
