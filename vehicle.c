#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"

#include "projects/crossroads/ats.h"       // unitstep_changed()
#include "projects/crossroads/crossroads.h"// extern int crossroads_step
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/priority_sync.h"

/* ========================================================
   내부 스텝 생성기: ATS 통해 1초마다 호출 → crossroads_step++ →
   모든 차량 스레드를 cond_broadcast 로 깨워 다음 스텝 진행
   ======================================================== */
static struct lock      s_step_lock;
static struct condition s_step_cond;
static bool             s_stepgen_started = false;

static void
step_generator (void *aux UNUSED)
{
    while (true)
    {
        unitstep_changed ();          // ATS 내부에서만 1초 대기
        lock_acquire (&s_step_lock);
        crossroads_step++;
        cond_broadcast (&s_step_cond, &s_step_lock);
        lock_release (&s_step_lock);
    }
}

/* init_on_mainthread: 딱 한 번만 step_generator 스레드 생성 */
void
init_on_mainthread (int thread_cnt UNUSED)
{
    if (!s_stepgen_started)
    {
        s_stepgen_started = true;
        lock_init   (&s_step_lock);
        cond_init   (&s_step_cond);
        thread_create ("step-gen", PRI_DEFAULT,
                       step_generator, NULL);
    }
}

/* ========================================================
   차량 경로 정의 (A=0, B=1, C=2, D=3)
   ======================================================== */
const struct position vehicle_path[4][4][12] = {
    /* from A */ {
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        {{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1}},
        {{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        {{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from B */ {
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        {{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        {{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1}},
        {{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from C */ {
        {{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1}},
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        {{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        {{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1}}
    },
    /* from D */ {
        {{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1}},
        {{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1}},
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1}},
        {{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1}}
    }
};

/* ========================================================
   입력 파싱 (변경 없음)
   ======================================================== */
void
parse_vehicles (struct vehicle_info *vehicle_info,
                char *input)
{
    int idx = 0;
    char *save_ptr, *tok = strtok_r (input, ":", &save_ptr);
    while (tok && idx < 10) {
        struct vehicle_info *vi = &vehicle_info[idx++];
        memset (vi, 0, sizeof *vi);
        vi->id    = tok[0];
        vi->start = tok[1];
        vi->dest  = tok[2];
        if (strlen (tok) == 3) {
            vi->type        = VEHICL_TYPE_NORMAL;
            vi->arrival     = -1;
            vi->golden_time = -1;
        } else {
            vi->type        = VEHICL_TYPE_AMBULANCE;
            vi->arrival     = atoi (&tok[3]);
            char *dot       = strchr (tok, '.');
            vi->golden_time = dot ? atoi (dot + 1) : -1;
        }
        vi->state        = VEHICLE_STATUS_READY;
        vi->position.row = vi->position.col = -1;
        tok = strtok_r (NULL, ":", &save_ptr);
    }
}

static bool
is_outside (struct position p)
{
    return p.row < 0 || p.col < 0;
}

/* ========================================================
   한 칸 이동 시도 (변경 없음)
   ======================================================== */
static int
try_move (int s, int d, int step,
          struct vehicle_info *vi)
{
    struct position cur = vi->position;
    struct position nxt = vehicle_path[s][d][step];

    if (vi->state == VEHICLE_STATUS_RUNNING
     && is_outside (nxt)) {
        lock_release_priority (&vi->map_locks
                               [cur.row][cur.col]);
        vi->position.row = vi->position.col = -1;
        return 0;
    }

    lock_acquire_priority (&vi->map_locks
                            [nxt.row][nxt.col], vi);
    if (vi->state == VEHICLE_STATUS_READY)
        vi->state = VEHICLE_STATUS_RUNNING;
    else
        lock_release_priority (&vi->map_locks
                                [cur.row][cur.col]);
    vi->position = nxt;
    return 1;
}

/* ========================================================
   차량 스레드 메인 루프
   — 매 스텝마다 step_cond 을 통해
     1초 주기 단위 스텝을 동기화
   — 앰뷸런스는 arrival 전까지 이동만 스킵,
     barrier 는 항상 참여 → step 5에서 출발
   ======================================================== */
void
vehicle_loop (void *_vi)
{
    struct vehicle_info *vi = _vi;
    int start       = vi->start - 'A';
    int dest        = vi->dest  - 'A';
    int moved_steps = 0;
    bool started    = false;
    int  start_t    = 0;
    int  last_step  = crossroads_step;

    vi->state        = VEHICLE_STATUS_READY;
    vi->position.row = vi->position.col = -1;

    while (true) {
        /* 다음 스텝 신호(wait) */
        lock_acquire (&s_step_lock);
        while (crossroads_step == last_step)
            cond_wait (&s_step_cond, &s_step_lock);
        last_step = crossroads_step;
        lock_release (&s_step_lock);

        /* 앰뷸런스 출발 지연 처리 */
        if (!started) {
            if (vi->type == VEHICL_TYPE_NORMAL
             || last_step >= vi->arrival) {
                started = true;
                start_t = last_step;
            } else {
                /* 이동 스킵하되 barrier는 이미 통과 */
                continue;
            }
        }

        /* 한 칸 이동 시도 */
        if (try_move (start, dest, moved_steps, vi)) {
            moved_steps++;
        } else {
            vi->state = VEHICLE_STATUS_FINISHED;
            break;
        }
    }

    /* 도착 후 앰뷸런스 golden time 체크 */
    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        int took = last_step - start_t;
        printf ("Ambulance %c %s: took %d (golden %d)\n",
                vi->id,
                (took <= vi->golden_time ? "OK" : "FAILED"),
                took, vi->golden_time);
    }
}
