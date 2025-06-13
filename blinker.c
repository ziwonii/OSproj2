#include "projects/crossroads/blinker.h"
#include "threads/thread.h"
#include "threads/synch.h"      /* lock_held_by_current_thread */
#include "projects/crossroads/ats.h"       /* unitstep_changed() */
#include "projects/crossroads/crossroads.h"/* crossroads_step */

static struct blinker_info *g_blinkers;
static int                  g_blinker_cnt = DIR_COUNT;
static enum direction       g_current = NORTH;

/* 신호등 스케줄러: 매 unit step마다 다음 방향을 녹색으로 */
static void
light_scheduler(void *aux UNUSED) {
    while (true) {
        unitstep_changed();  /* 1초 대기 */
        g_current = (g_current + 1) % DIR_COUNT;
    }
}

/* 각 방향별 블링커 스레드 */
static void
blinker_thread(void *aux) {
    struct blinker_info *b = aux;
    struct position pos = light_cell[b->dir];
    struct lock *ctrl = &b->map_locks[pos.row][pos.col];

    /* 1) 처음엔 모두 ‘빨간불’로: 중앙 칸 락 획득 */
    lock_acquire(ctrl);

    while (true) {
        /* 2) 다음 단계 대기 */
        unitstep_changed();

        if (g_current == b->dir) {
            /* 녹색 → 락 해제 (차량 통행 허용) */
            lock_release(ctrl);
        } else {
            /* 빨간불 → 락이 없으면 다시 획득 (차단 유지) */
            if (!lock_held_by_current_thread(ctrl))
                lock_acquire(ctrl);
        }
    }
}

void
init_blinker(struct blinker_info* blinkers,
             struct lock **map_locks,
             struct vehicle_info *vehicles) {
    g_blinkers = blinkers;
    for (int i = 0; i < DIR_COUNT; i++) {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicles  = vehicles;
        blinkers[i].dir       = (enum direction)i;
    }
}

void
start_blinker(void) {
    /* 1) 스케줄러 스레드 하나 */
    thread_create("light-sched", PRI_DEFAULT,
                  light_scheduler, NULL);

    /* 2) 4개 방향별 블링커 스레드 */
    for (int i = 0; i < DIR_COUNT; i++) {
        char name[16];
        snprintf(name, sizeof name, "blinker-%d", i);
        thread_create(name, PRI_DEFAULT,
                      blinker_thread, &g_blinkers[i]);
    }
}
