#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"

#include "projects/crossroads/ats.h"
#include "projects/crossroads/crossroads.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/priority_sync.h"
#include "projects/crossroads/blinker.h"  // extern enum direction g_current

extern enum direction g_current;  // 신호등 현재 방향 (blinker.c에서 선언됨)

/* 내부 스텝 동기화 */
static struct lock      s_step_lock;
static struct condition s_step_cond;
static bool             s_stepgen_started = false;

static void step_generator (void *aux UNUSED) {
    while (true) {
        unitstep_changed();
        lock_acquire(&s_step_lock);
        crossroads_step++;
        cond_broadcast(&s_step_cond, &s_step_lock);
        lock_release(&s_step_lock);
    }
}

void init_on_mainthread (int thread_cnt UNUSED) {
    if (!s_stepgen_started) {
        s_stepgen_started = true;
        lock_init(&s_step_lock);
        cond_init(&s_step_cond);
        thread_create("step-gen", PRI_DEFAULT, step_generator, NULL);
    }
}

/* 신호등 방향 판단 함수 */
static enum direction get_direction(char start) {
    switch (start) {
        case 'A': return SOUTH;
        case 'B': return WEST;
        case 'C': return NORTH;
        case 'D': return EAST;
        default: return -1;
    }
}

/* 교차로 중앙 칸인지 확인 */
static bool is_central(struct position p) {
    return (p.row == 2 && p.col == 3) ||  // NORTH
           (p.row == 3 && p.col == 4) ||  // EAST
           (p.row == 4 && p.col == 3) ||  // SOUTH
           (p.row == 3 && p.col == 2);    // WEST
}

/* 경로 정의 */
const struct position vehicle_path[4][4][12] = {
    // from A
    { {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
      {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1}},
      {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
      {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}} },
    // from B
    { {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
      {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
      {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1}},
      {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}} },
    // from C
    { {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
      {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
      {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
      {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1}} },
    // from D
    { {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1}},
      {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
      {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
      {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}} }
};

void parse_vehicles (struct vehicle_info *vehicle_info, char *input) {
    int idx = 0;
    char *save_ptr, *tok = strtok_r(input, ":", &save_ptr);
    while (tok && idx < 10) {
        struct vehicle_info *vi = &vehicle_info[idx++];
        memset(vi, 0, sizeof *vi);
        vi->id    = tok[0];
        vi->start = tok[1];
        vi->dest  = tok[2];
        if (strlen(tok) == 3) {
            vi->type = VEHICL_TYPE_NORMAL;
            vi->arrival = -1;
            vi->golden_time = -1;
        } else {
            vi->type = VEHICL_TYPE_AMBULANCE;
            vi->arrival = atoi(&tok[3]);
            char *dot = strchr(tok, '.');
            vi->golden_time = dot ? atoi(dot + 1) : -1;
        }
        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = vi->position.col = -1;
        tok = strtok_r(NULL, ":", &save_ptr);
    }
}

static bool is_outside(struct position p) {
    return p.row < 0 || p.col < 0;
}

static int try_move(int s, int d, int step, struct vehicle_info *vi) {
    struct position cur = vi->position;
    struct position nxt = vehicle_path[s][d][step];

    if (vi->state == VEHICLE_STATUS_RUNNING && is_outside(nxt)) {
        lock_release_priority(&vi->map_locks[cur.row][cur.col]);
        vi->position.row = vi->position.col = -1;
        return 0;
    }

    lock_acquire_priority(&vi->map_locks[nxt.row][nxt.col], vi);
    if (vi->state == VEHICLE_STATUS_READY)
        vi->state = VEHICLE_STATUS_RUNNING;
    else
        lock_release_priority(&vi->map_locks[cur.row][cur.col]);

    vi->position = nxt;
    return 1;
}

void vehicle_loop(void *_vi) {
    struct vehicle_info *vi = _vi;
    int start = vi->start - 'A';
    int dest  = vi->dest  - 'A';
    int moved_steps = 0;
    bool started = false;
    int start_t = 0;
    int last_step = crossroads_step;

    vi->state = VEHICLE_STATUS_READY;
    vi->position.row = vi->position.col = -1;

    while (true) {
        lock_acquire(&s_step_lock);
        while (crossroads_step == last_step)
            cond_wait(&s_step_cond, &s_step_lock);
        last_step = crossroads_step;
        lock_release(&s_step_lock);

        if (!started) {
            if (vi->type == VEHICL_TYPE_NORMAL || last_step >= vi->arrival) {
                started = true;
                start_t = last_step;
            } else {
                continue;  // barrier는 이미 참여
            }
        }

        struct position next = vehicle_path[start][dest][moved_steps];

        // 교차로 중앙 진입 시 → 신호등 방향 확인
        if (is_central(next)) {
            enum direction my_dir = get_direction(vi->start);
            while (g_current != my_dir) {
                lock_acquire(&s_step_lock);
                while (crossroads_step == last_step)
                    cond_wait(&s_step_cond, &s_step_lock);
                last_step = crossroads_step;
                lock_release(&s_step_lock);
            }
        }

        if (try_move(start, dest, moved_steps, vi)) {
            moved_steps++;
        } else {
            vi->state = VEHICLE_STATUS_FINISHED;
            break;
        }
    }

    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        int took = last_step - start_t;
        printf("Ambulance %c %s: took %d (golden %d)\n",
            vi->id,
            (took <= vi->golden_time ? "OK" : "FAILED"),
            took, vi->golden_time);
    }
}
