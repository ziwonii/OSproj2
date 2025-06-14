#ifndef __PROJECTS_PROJECT2_BLINKER_H__
#define __PROJECTS_PROJECT2_BLINKER_H__

#include "threads/synch.h"
#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/position.h" // struct position 필요

/** 방향 인덱스: 북(0), 동(1), 남(2), 서(3) */
enum direction { NORTH = 0, EAST, SOUTH, WEST, DIR_COUNT };
extern enum direction g_current;

/* crossroads.c에서 NUM_BLINKER로 참조하기도 하므로 alias */
#ifndef NUM_BLINKER
#define NUM_BLINKER DIR_COUNT
#endif

/* 신호등이 제어할 중앙 칸 좌표 (extern 선언) */
extern const struct position light_cell[DIR_COUNT];

struct blinker_info {
    struct lock **map_locks;
    struct vehicle_info *vehicles;
    enum direction dir;
};

void init_blinker(struct blinker_info* blinkers,
                  struct lock **map_locks,
                  struct vehicle_info *vehicles);
void start_blinker(void);

#endif /* __PROJECTS_PROJECT2_BLINKER_H__ */
