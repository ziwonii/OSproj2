#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"

#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/priority_sync.h"


/* 파일 상단에 추가 */
static struct vehicle_info *global_vi = NULL;
static int g_thread_cnt, g_barrier_count;
static struct lock      g_barrier_lock;
static struct condition g_barrier_cond;


/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
    /* from A */ {
        /* to A */ {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        /* to B */ {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1}},
        /* to C */ {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        /* to D */ {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from B */ {
        /* to A */ {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        /* to B */ {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        /* to C */ {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1}},
        /* to D */ {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from C */ {
        /* to A */ {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        /* to B */ {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        /* to C */ {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        /* to D */ {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from D */ {
        /* to A */ {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1}},
        /* to B */ {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        /* to C */ {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        /* to D */ {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    }
};

void parse_vehicles(struct vehicle_info *vehicle_info, char *input) {
    int idx = 0;
    char *save_ptr;
    char *token = strtok_r(input, ":", &save_ptr);

    while (token != NULL && idx < 10) {
        struct vehicle_info *vi = &vehicle_info[idx];
        memset(vi, 0, sizeof(*vi));

        vi->id = token[0];
        vi->start = token[1];
        vi->dest = token[2];

        if (strlen(token) == 3) {
            vi->type = VEHICL_TYPE_NORMAL;
            vi->arrival = -1;
            vi->golden_time = -1;
        } else {
            vi->type = VEHICL_TYPE_AMBULANCE;
            vi->arrival = atoi(&token[3]);
            char *dot = strchr(token, '.');
            if (dot)
                vi->golden_time = atoi(dot + 1);
        }

        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = vi->position.col = -1;
        idx++;
        token = strtok_r(NULL, ":", &save_ptr);
    }

    /* 디버깅 출력 */
    printf("=== Vehicle Info ===\n");
    for (int i = 0; i < idx; i++) {
        struct vehicle_info *vi = &vehicle_info[i];
        printf("Vehicle %c: %c -> %c, %s\n",
               vi->id, vi->start, vi->dest,
               vi->type == VEHICL_TYPE_AMBULANCE ? "Ambulance" : "Normal");
    }

    /* 전역 포인터에 저장 */
    global_vi = vehicle_info; //초기화용
}

static int is_position_outside(struct position pos) {
    return pos.row == -1 || pos.col == -1;
}

static int try_move(int start, int dest, int step, struct vehicle_info *vi) {
    struct position pos_cur = vi->position;
    struct position pos_next = vehicle_path[start][dest][step];

    if (vi->state == VEHICLE_STATUS_RUNNING) {
        if (is_position_outside(pos_next)) {
            vi->position.row = vi->position.col = -1;
            lock_release_priority(&global_vi[0].map_locks[pos_cur.row][pos_cur.col]);
            return 0;
        }
    }

    lock_acquire_priority(&global_vi[0].map_locks[pos_next.row][pos_next.col], vi);
    if (vi->state == VEHICLE_STATUS_READY)
        vi->state = VEHICLE_STATUS_RUNNING;
    else
        lock_release_priority(&global_vi[0].map_locks[pos_cur.row][pos_cur.col]);

    vi->position = pos_next;
    return 1;
}

void init_on_mainthread(int thread_cnt) {
    g_thread_cnt    = thread_cnt;
    g_barrier_count = 0;
    lock_init(&g_barrier_lock);
    cond_init(&g_barrier_cond);

    /* map_locks 초기화 */
    if (global_vi) {
        for (int i = 0; i < 7; i++)
            for (int j = 0; j < 7; j++)
                lock_init(&global_vi[0].map_locks[i][j]);
    }
}

void vehicle_loop(void *_vi) {
    struct vehicle_info *vi = _vi;
    int start = vi->start - 'A';
    int dest  = vi->dest  - 'A';

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    int step = 0;
    while (1) {
        int res = try_move(start, dest, step, vi);
        if (res == 1)
            step++;
        if (res == 0) {
            // 종료된 스레드만큼 배리어 크기 줄이고 wakeup
            lock_acquire(&g_barrier_lock);
            g_thread_cnt--;
            cond_broadcast(&g_barrier_cond, &g_barrier_lock);
            lock_release(&g_barrier_lock);
            break;
        }

        /* 배리어 동기화: 모든 차량 이동 후에만 step 증가 */
        lock_acquire(&g_barrier_lock);
        g_barrier_count++;
        if (g_barrier_count == g_thread_cnt) {
            crossroads_step++;
            unitstep_changed();
            g_barrier_count = 0;
            cond_broadcast(&g_barrier_cond, &g_barrier_lock);
        } else {
            int my_step = crossroads_step;
            while (my_step == crossroads_step)
                cond_wait(&g_barrier_cond, &g_barrier_lock);
        }
        lock_release(&g_barrier_lock);
    }

    vi->state = VEHICLE_STATUS_FINISHED;
}
