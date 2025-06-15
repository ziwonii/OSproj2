#include "projects/crossroads/blinker.h"
#include "projects/crossroads/position.h"
#include "threads/thread.h"
#include "threads/synch.h"
#include "projects/crossroads/ats.h"         // unitstep_changed()
#include "projects/crossroads/crossroads.h"  // crossroads_step

/* 외부에서 참조할 수 있도록 글로벌 변수로 선언 */
enum direction g_current = NORTH;

/* light_cell[]: 각 방향 진입 시 중앙 칸 좌표 */
const struct position light_cell[DIR_COUNT] = {
    {2, 3},  // NORTH
    {3, 4},  // EAST
    {4, 3},  // SOUTH
    {3, 2}   // WEST
};

static struct blinker_info *g_blinkers;
static int g_blinker_cnt = DIR_COUNT;

/* 신호등 스케줄러 스레드: 매 unit step마다 다음 방향 녹색 불 */
static void light_scheduler(void *aux UNUSED) {
    while (true) {
        unitstep_changed();                  
        g_current = (g_current + 1) % DIR_COUNT;  // 방향 순환
    }
}

/* 각 방향 블링커 스레드: 중앙 칸 락 제어 */
static void blinker_thread(void *aux) {
    struct blinker_info *b = aux;
    struct position pos = light_cell[b->dir];
    struct lock *ctrl = &b->map_locks[pos.row][pos.col];

    // 처음엔 모두 빨간불 (락 획득)
    lock_acquire(ctrl);

    while (true) {
        unitstep_changed();  // 단위 스텝까지 대기

        if (g_current == b->dir) {
            // 녹색: 통행 허용 → 락 해제
            lock_release(ctrl);
        } else {
            // 빨간불: 락 다시 획득해서 차단 유지
            if (!lock_held_by_current_thread(ctrl))
                lock_acquire(ctrl);
        }
    }
}

/* 신호등 정보 초기화 */
void init_blinker(struct blinker_info* blinkers,
             struct lock **map_locks,
             struct vehicle_info *vehicles) {
    g_blinkers = blinkers;
    for (int i = 0; i < DIR_COUNT; i++) {
        blinkers[i].map_locks = map_locks;
        blinkers[i].vehicles  = vehicles;
        blinkers[i].dir       = (enum direction)i;
    }
}

/* 스레드 시작: 스케줄러 + 각 방향 블링커 */
void start_blinker(void) {
    thread_create("light-sched", PRI_DEFAULT,
                  light_scheduler, NULL);

    for (int i = 0; i < DIR_COUNT; i++) {
        char name[16];
        snprintf(name, sizeof name, "blinker-%d", i);
        thread_create(name, PRI_DEFAULT,
                      blinker_thread, &g_blinkers[i]);
    }
}
