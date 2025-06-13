#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/interrupt.h"
#include "devices/timer.h"

#include "projects/crossroads/crossroads.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/priority_sync.h"

/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
    /* from A */ {
        /* to A */ {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */ {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */ {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */ {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from B */ {
        /* to A */ {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */ {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */ {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */ {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from C */ {
        /* to A */ {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */ {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */ {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */ {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
    },
    /* from D */ {
        /* to A */ {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
        /* to B */ {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
        /* to C */ {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
        /* to D */ {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
    }
};

/* barrier state */
static struct lock      s_barrier_lock;
static struct condition s_barrier_cond;
static int              s_barrier_waiting;
static int              s_barrier_cnt;
static int              s_barrier_gen;

/* barrier 초기화 (init_on_mainthread에서 호출) */
static void barrier_init(int thread_cnt) {
    s_barrier_cnt     = thread_cnt;
    s_barrier_waiting = 0;
    s_barrier_gen     = 0;
    lock_init(&s_barrier_lock);
    cond_init(&s_barrier_cond);
}

/* barrier wait:
   - 마지막 스레드는 락 해제 후 인터럽트 ON 상태에서 timer_msleep 호출
   - sleep 후 다시 락 잡고 step 증가 + wakeup */
static void barrier_wait(void) {
    enum intr_level old = intr_disable();
    lock_acquire(&s_barrier_lock);

    int gen = s_barrier_gen;
    s_barrier_waiting++;
    if (s_barrier_waiting == s_barrier_cnt) {
        // 마지막 스레드가 도착하면 물리적 대기 없이
        // 논리적 타임스텝만 1 증가시키고 깨어낸다.
        crossroads_step++;
        s_barrier_waiting = 0;
        s_barrier_gen++;
        cond_broadcast(&s_barrier_cond, &s_barrier_lock);
    } else {
        while (gen == s_barrier_gen)
            cond_wait(&s_barrier_cond, &s_barrier_lock);
    }

    lock_release(&s_barrier_lock);
    intr_set_level(old);
}


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
            vi->golden_time = dot ? atoi(dot + 1) : -1;
        }

        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = vi->position.col = -1;
        idx++;
        token = strtok_r(NULL, ":", &save_ptr);
    }
}

static int is_position_outside(struct position pos) {
    return pos.row == -1 || pos.col == -1;
}

static int try_move(int start, int dest, int step, struct vehicle_info *vi) {
    struct position cur = vi->position;
    struct position nxt = vehicle_path[start][dest][step];

    if (vi->state == VEHICLE_STATUS_RUNNING && is_position_outside(nxt)) {
        /* 도착 */
        vi->position.row = vi->position.col = -1;
        lock_release_priority(&vi->map_locks[cur.row][cur.col]);
        return 0;
    }

    /* 다음 칸 락 */
    lock_acquire_priority(&vi->map_locks[nxt.row][nxt.col], vi);
    if (vi->state == VEHICLE_STATUS_READY)
        vi->state = VEHICLE_STATUS_RUNNING;
    else
        lock_release_priority(&vi->map_locks[cur.row][cur.col]);

    vi->position = nxt;
    return 1;
}

void init_on_mainthread(int thread_cnt) {
    barrier_init(thread_cnt);
}

void vehicle_loop(void *_vi) {
    extern int crossroads_step;
    struct vehicle_info *vi = _vi;
    int start = vi->start - 'A';
    int dest  = vi->dest  - 'A';
    int step  = 0, res;

    /* 앰뷸런스는 arrival 스텝까지 barrier만 대기 */
    if (vi->type == VEHICL_TYPE_AMBULANCE && vi->arrival >= 0) {
        while (crossroads_step < vi->arrival)
            barrier_wait();
    }
    int t0 = crossroads_step;

    vi->position.row = vi->position.col = -1;
    vi->state = VEHICLE_STATUS_READY;

    while ((res = try_move(start, dest, step, vi)) == 1) {
        step++;
        barrier_wait();
    }

    vi->state = VEHICLE_STATUS_FINISHED;

    /* 앰뷸런스 golden time 체크 */
    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        int took = crossroads_step - t0;
        printf("Ambulance %c %s: took %d (golden %d)\n",
               vi->id,
               (took <= vi->golden_time ? "OK" : "FAILED"),
               took, vi->golden_time);
    }
}
