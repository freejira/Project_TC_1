#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <pthread.h>
#include <mqueue.h>
#include <stddef.h>
#include <stdint.h>
/* --- [1. 데이터 타입 및 공유 구조체 정의] --- */
typedef enum
{
        SYS_IDLE = 0,
        SYS_IDENTIFY,
        SYS_AUTHENTICATE,
        SYS_NEGOTIATE,
        SYS_ACTIVE,
        SYS_FAULT,
        SYS_QUARANTINE
} SystemState_t;

typedef enum
{
        FAULT_NONE = 0,
        FAULT_IDENTIFY_TIMEOUT,
        FAULT_AUTH_FAIL,
        FAULT_AUTH_TIMEOUT,
        FAULT_POWER_REJECT,
        FAULT_NEGOTIATE_TIMEOUT,
        FAULT_POWER_VIOLATION_MAX
} FaultCode_t;

typedef enum
{
        MODULE_NONE = 0,
        MODULE_GENERAL,      // Module A: 일반 배송 모듈
        MODULE_COLD_CHAIN,   // Module B: 냉장 배송 모듈
        MODULE_UNKNOWN       // 등록되지 않았거나 알 수 없는 모듈
} ModuleType_t;

typedef struct
{
        pthread_mutex_t mutex;  // 프로세스 간 공유할 뮤텍스

        /* System State */
        SystemState_t system_state;
        ModuleType_t module_type;
        FaultCode_t latest_fault;

        uint32_t module_id;

        uint8_t dock_detected;
        uint8_t auth_result;
        uint8_t power_granted;
        uint8_t module_function_enabled;
        /* Driving */
        float target_speed_rpm;
        float current_speed_rpm;
        uint16_t motor_pwm_duty;

        /* Power Policy */
        float requested_power_w;
        float granted_power_w;
        float reported_power_w;
        uint8_t power_violation_count;

        /* Module A */
        float pressure_value;

        /* Module B */
        float target_temp_c;
        float current_temp_c;
        uint8_t peltier_pwm;
        uint8_t fan_pwm;

        /* Warning */
        uint8_t warning_flag;
        uint8_t sleep_flag;
} SystemSharedData_t;

/* --- [2. IPC 리소스 상수 정의] --- */
#define SHM_NAME        "/sys_shared_memory"
#define MQ_NAME         "/sys_message_queue"
#define MQ_MAX_MSG      10
#define MQ_MSG_SIZE     256

/* --- [3. 자식 프로세스 정보] --- */
typedef struct {
        const char *name;       // 실행 파일 이름 및 로그용
        const char *path;       // 현재 디렉토리 기준 상대 경로
        pid_t pid;              // 생성된 PID
} ProcessInfo;

static ProcessInfo g_processes[] = {
        // {"gui_proc",  "./gui_proc",  -1},
        {"ai_proc",   "python AI_init.py",   -1},
        // {"sec_proc",  "./sec_proc",  -1},
        // {"comm_proc", "./comm_proc", -1},
        // {"log_proc", "./log_proc", -1}
};
const int NUM_PROCESSES = sizeof(g_processes) / sizeof(g_processes[0]);

static SystemSharedData_t *g_shm_ptr = MAP_FAILED;
static mqd_t g_mq = (mqd_t)-1;

/* --- [4. 자원 정리 및 자식 프로세스 강제 종료 함수] --- */
void cleanup_resources_and_kill_children(void) {
        printf("\n[INIT] Terminating remaining children and cleaning up resources...\n");

        /* 1. 남은 자식 프로세스들을 강제 종료 */
        for (int i = 0; i < NUM_PROCESSES; i++) {
                if (g_processes[i].pid > 0) {
                        printf("[INIT] Killing %s (PID: %d)\n", g_processes[i].name, g_processes[i].pid);
                        kill(g_processes[i].pid, SIGKILL);
                        /* 킬 후 즉시 waitpid로 수집 */
                        waitpid(g_processes[i].pid, NULL, 0);
                        g_processes[i].pid = -1;
                }
        }
        /* 2. 뮤텍스 해제 및 공유메모리 매핑 해제 */
        if (g_shm_ptr != MAP_FAILED) {
                pthread_mutex_destroy(&g_shm_ptr->mutex);
                munmap(g_shm_ptr, sizeof(SystemSharedData_t));
                shm_unlink(SHM_NAME);
                printf("[INIT] Shared Memory & Mutex destroyed.\n");
        }

        /* 3. 메시지 큐 삭제 */
        if (g_mq != (mqd_t)-1) {
                mq_close(g_mq);
                mq_unlink(MQ_NAME);
                printf("[INIT] Message Queue destroyed.\n");
        }
}

/* --- [5. 초기화 함수들] --- */
int init_shared_memory_and_mutex(void) {
        shm_unlink(SHM_NAME); // 이전 찌꺼기 삭제
        int shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
        if (shm_fd < 0) {
                perror("[INIT] shm_open failed");
                return -1;
        }

        if (ftruncate(shm_fd, sizeof(SystemSharedData_t)) == -1) {
                perror("[INIT] ftruncate failed");
                close(shm_fd);
                return -1;
        }

        g_shm_ptr = mmap(NULL, sizeof(SystemSharedData_t), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        close(shm_fd);
        if (g_shm_ptr == MAP_FAILED) {
                perror("[INIT] mmap failed");
                return -1;
        }

        memset(g_shm_ptr, 0, sizeof(SystemSharedData_t));

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

        if (pthread_mutex_init(&g_shm_ptr->mutex, &attr) != 0) {
                perror("[INIT] pthread_mutex_init failed");
                pthread_mutexattr_destroy(&attr);
                return -1;
        }
        pthread_mutexattr_destroy(&attr);

        g_shm_ptr->system_state = 1;
        printf("[INIT] Shared Memory & Mutex initialized successfully.\n");
        return 0;
}

int init_message_queue(void) {
        struct mq_attr attr;
        attr.mq_flags = 0;
        attr.mq_maxmsg = MQ_MAX_MSG;
        attr.mq_msgsize = MQ_MSG_SIZE;
        attr.mq_curmsgs = 0;

        mq_unlink(MQ_NAME); // 이전 찌꺼기 삭제
        g_mq = mq_open(MQ_NAME, O_CREAT | O_RDWR, 0666, &attr);
        if (g_mq == (mqd_t)-1) {
                perror("[INIT] mq_open failed");
                return -1;
        }
        printf("[INIT] Message Queue initialized successfully.\n");
        return 0;
}

/* --- [6. 프로세스 생성 함수 (현재 디렉토리 실행)] --- */
pid_t spawn_process(ProcessInfo *proc) {
        pid_t pid = fork();
        if (pid < 0) {
        perror("[INIT] fork failed");
        return -1;
        }
        else if (pid == 0) {
                printf("[CHILD] Executing %s (%s)...\n", proc->name, proc->path);

                if (strcmp(proc->name, "ai_proc") == 0) {
                        const char *home = getenv("HOME");
                        char py[256], script[256];
                        snprintf(py, sizeof(py),
                                "%s/drowsy_env_312/bin/python", home ? home : "");
                        snprintf(script, sizeof(script),
                                "%s/my/AI/AI_init.py", home ? home : "");
                        execl(py, py, script, (char *)NULL);
                        fprintf(stderr, "[CHILD CRITICAL] Failed to exec %s %s: %s\n",
                                py, script, strerror(errno));
                } else {
                        execl(proc->path, proc->name, (char *)NULL);
                        fprintf(stderr, "[CHILD CRITICAL] Failed to exec %s: %s\n",
                                proc->path, strerror(errno));
                }
                exit(EXIT_FAILURE);
        }

        /* [Parent Process] */
        proc->pid = pid;
        printf("[INIT] Spawned %s with PID: %d\n", proc->name, pid);
        return pid;
}

/* --- [7. Main Loop] --- */
int main(void) {
        printf("=== System Init Task Starting (Local Path, No Signals) ===\n");

        /* 1. IPC 및 동기화 자원 초기화 */
        if (init_shared_memory_and_mutex() < 0 || init_message_queue() < 0) {
                cleanup_resources_and_kill_children();
                return EXIT_FAILURE;
        }

        /* 2. 현재 디렉토리에서 4개 프로세스 순차 실행 */
        for (int i = 0; i < NUM_PROCESSES; i++) {
                if (spawn_process(&g_processes[i]) < 0) {
                        fprintf(stderr, "[INIT] Fatal error spawning %s. Aborting.\n", g_processes[i].name);
                        cleanup_resources_and_kill_children();
                        return EXIT_FAILURE;
                }
                usleep(100000); // 100ms 대기 (순차적 초기화)
        }

        /* 3. 프로세스 감시 (어느 하나라도 종료될 때까지 대기) */
        printf("[INIT] All processes running from current directory. Waiting for any child to exit...\n");

        int status;
        /* 블로킹 함수: 4개의 자식 중 누군가 하나라도 죽으면 반환 */
        pid_t dead_pid = waitpid(-1, &status, 0);
        if (dead_pid > 0) {
                /* 누가 죽었는지 식별 */
                for (int i = 0; i < NUM_PROCESSES; i++) {
                        if (g_processes[i].pid == dead_pid) {
                                g_processes[i].pid = -1; // 이미 죽었음을 표시하여 중복 kill 방지

                                if (WIFEXITED(status)) {
                                        printf("\n[INIT] Module '%s' (PID: %d) exited with status %d.\n",
                                        g_processes[i].name, dead_pid, WEXITSTATUS(status));
                                }
                                else if (WIFSIGNALED(status)) {
                                        printf("\n[INIT CRITICAL] Module '%s' (PID: %d) crashed or was killed!\n",
                                        g_processes[i].name, dead_pid);
                                }
                                break;
                        }
                }
        }
        else {
                perror("[INIT] waitpid error");
        }
        /* 4. 하나의 프로세스라도 종료되면, 시스템 전체를 중지하고 자원 회수 */
        cleanup_resources_and_kill_children();
        printf("=== System Init Task Terminated ===\n");
        return 0;
}
