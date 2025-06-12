#include "projects/crossroads/priority_sync.h"
#include "threads/thread.h"
#include "threads/interrupt.h"
#include "threads/synch.h"
#include <list.h>
#include <stddef.h>
#include <stdlib.h>


// 내부 매핑용 구조체
struct lock_to_ps_map {
    struct lock *original;
    struct priority_semaphore ps;
    struct list_elem elem;
};

// 전역 매핑 테이블
static struct list lock_map_table;

static bool is_lock_map_initialized = false;

static struct priority_semaphore *get_or_init_ps_from_lock(struct lock *lock) {
    if (!is_lock_map_initialized) {
        list_init(&lock_map_table);
        is_lock_map_initialized = true;
    }

    struct list_elem *e;
    for (e = list_begin(&lock_map_table); e != list_end(&lock_map_table); e = list_next(e)) {
        struct lock_to_ps_map *entry = list_entry(e, struct lock_to_ps_map, elem);
        if (entry->original == lock)
            return &entry->ps;
    }

    // 매핑 없으면 새로 생성
    struct lock_to_ps_map *new_entry = malloc(sizeof(struct lock_to_ps_map));
    new_entry->original = lock;
    priority_sema_init(&new_entry->ps, 1);
    list_push_back(&lock_map_table, &new_entry->elem);

    return &new_entry->ps;
}

void lock_acquire_priority(struct lock *lock, struct vehicle_info *vi) {
    struct priority_semaphore *ps = get_or_init_ps_from_lock(lock);
    priority_sema_down(ps, vi);
}

void lock_release_priority(struct lock *lock) {
    struct priority_semaphore *ps = get_or_init_ps_from_lock(lock);
    priority_sema_up(ps);
}
