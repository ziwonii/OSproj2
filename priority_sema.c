#include "projects/crossroads/priority_sync.h"
#include "threads/interrupt.h"
#include "threads/thread.h"
#include <list.h>
#include <stddef.h>
#include <stdlib.h>
#include "projects/crossroads/vehicle.h" // vi->type

/* 정렬 비교: 앰뷸런스 우선 */
bool ambulance_first(const struct list_elem *a, const struct list_elem *b, void *aux UNUSED) {
    struct waiter *wa = list_entry(a, struct waiter, elem);
    struct waiter *wb = list_entry(b, struct waiter, elem);

    struct vehicle_info *v_a = wa->vi;
    struct vehicle_info *v_b = wb->vi;

    if (v_a->type == VEHICL_TYPE_AMBULANCE && v_b->type != VEHICL_TYPE_AMBULANCE)
        return true;
    if (v_b->type == VEHICL_TYPE_AMBULANCE && v_a->type != VEHICL_TYPE_AMBULANCE)
        return false;

    return false;
}

void priority_sema_init(struct priority_semaphore *ps, unsigned value) {
    ps->value = value;
    list_init(&ps->waiters);
}

void priority_sema_down(struct priority_semaphore *ps, struct vehicle_info *vi) {
    enum intr_level old_level = intr_disable();

    while (ps->value == 0) {
        struct waiter *w = malloc(sizeof(struct waiter));
        w->t = thread_current();
        w->vi = vi;
        list_insert_ordered(&ps->waiters, &w->elem, ambulance_first, NULL);
        thread_block();
    }

    ps->value--;
    intr_set_level(old_level);
}

void priority_sema_up(struct priority_semaphore *ps) {
    enum intr_level old_level = intr_disable();

    if (!list_empty(&ps->waiters)) {
        struct list_elem *e = list_pop_front(&ps->waiters);
        struct waiter *w = list_entry(e, struct waiter, elem);
        thread_unblock(w->t);
        free(w); // 메모리 해제
    }

    ps->value++;
    intr_set_level(old_level);
}
