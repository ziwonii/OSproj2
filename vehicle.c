#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "threads/thread.h"
#include "threads/synch.h"

#include "projects/crossroads/vehicle.h"
#include "projects/crossroads/map.h"
#include "projects/crossroads/ats.h"
#include "projects/crossroads/priority_sync.h" //우선순위 락 지원

#include "projects/crossroads/crossroads.h"   // extern int crossroads_step





/* path. A:0 B:1 C:2 D:3 */
const struct position vehicle_path[4][4][12] = {
	/* from A */ {
		/* to A */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{4,0},{4,1},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{4,0},{4,1},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from B */ {
		/* to A */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{6,4},{5,4},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{6,4},{5,4},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from C */ {
		/* to A */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{2,6},{2,5},{2,4},{2,3},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{2,6},{2,5},{2,4},{1,4},{0,4},{-1,-1},}
	},
	/* from D */ {
		/* to A */
		{{0,2},{1,2},{2,2},{2,1},{2,0},{-1,-1},},
		/* to B */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{5,2},{6,2},{-1,-1},},
		/* to C */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{4,5},{4,6},{-1,-1},},
		/* to D */
		{{0,2},{1,2},{2,2},{3,2},{4,2},{4,3},{4,4},{3,4},{2,4},{1,4},{0,4},{-1,-1},}
	}
};

void parse_vehicles(struct vehicle_info *vehicle_info, char *input) {
    int idx = 0;
    char *save_ptr;
    char *token = strtok_r(input, ":", &save_ptr);

    while (token != NULL && idx < 10) {
        struct vehicle_info *vi = &vehicle_info[idx];
        memset(vi, 0, sizeof(struct vehicle_info));

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
            if (dot != NULL) {
                vi->golden_time = atoi(dot + 1);
            }
        }

        vi->state = VEHICLE_STATUS_READY;
        vi->position.row = vi->position.col = -1;
        idx++;

        token = strtok_r(NULL, ":", &save_ptr);
    }

    // 디버깅 출력
    /*printf("=== Vehicle Info ===\n");
    for (int i = 0; i < idx; i++) {
        struct vehicle_info *vi = &vehicle_info[i];
        printf("Vehicle %c\n", vi->id);
        printf("  Start: %c\n", vi->start);
        printf("  Dest: %c\n", vi->dest);
        printf("  Type: %s\n", vi->type == VEHICL_TYPE_NORMAL ? "Normal" : "Ambulance");
        if (vi->type == VEHICL_TYPE_AMBULANCE) {
            printf("  Arrival: %d\n", vi->arrival);
            printf("  Golden Time: %d\n", vi->golden_time);
        }
        printf("---------------------\n");
    }*/
}


static int is_position_outside(struct position pos)
{
	return (pos.row == -1 || pos.col == -1);
}

/* return 0:termination, 1:success, -1:fail */
static int try_move(int start, int dest, int step, struct vehicle_info *vi)
{
	struct position pos_cur, pos_next;

	pos_next = vehicle_path[start][dest][step];
	//printf("Vehicle %c trying step %d → (%d,%d)\n", vi->id, step, pos_next.row, pos_next.col);

	pos_cur = vi->position;

	if (vi->state == VEHICLE_STATUS_RUNNING) {
		/* check termination */
		if (is_position_outside(pos_next)) {
			//printf("Vehicle %c reached end of path\n", vi->id);

			/* actual move */
			vi->position.row = vi->position.col = -1;
			/* release previous */
			//lock_release(&vi->map_locks[pos_cur.row][pos_cur.col]);
			lock_release_priority(&vi->map_locks[pos_cur.row][pos_cur.col]);

			return 0;
		}
	}

	/* lock next position */
	lock_acquire_priority(&vi->map_locks[pos_next.row][pos_next.col], vi);
	if (vi->state == VEHICLE_STATUS_READY) {
		/* start this vehicle */
		vi->state = VEHICLE_STATUS_RUNNING;
	} else {
		/* release current position */
		lock_release_priority(&vi->map_locks[pos_cur.row][pos_cur.col]);
	}
	/* update position */
	vi->position = pos_next;
	
	return 1;
}

void init_on_mainthread(int thread_cnt){
	/* Called once before spawning threads */
}

void vehicle_loop(void *_vi)
{
	int res;
	int start, dest, step;

	extern int crossroads_step;

	struct vehicle_info *vi = _vi;

	if (vi->type == VEHICL_TYPE_AMBULANCE && vi->arrival >= 0) {
    	while (crossroads_step < vi->arrival) {
			unitstep_changed();
		}
	}
	int start_time = crossroads_step;

	start = vi->start - 'A';
	dest = vi->dest - 'A';

	vi->position.row = vi->position.col = -1;
	vi->state = VEHICLE_STATUS_READY;

	step = 0;
	while (1) {
		/* vehicle main code */
		res = try_move(start, dest, step, vi);
		if (res == 1) {
			step++;
		}

		/* termination condition. */ 
		if (res == 0) {
			break;
		}

		/* unitstep change! */
		unitstep_changed();
	}	

    /* --- 도착 시점에 걸린 시간 체크 --- */
    int finish_time = crossroads_step;
    if (vi->type == VEHICL_TYPE_AMBULANCE) {
        int took = finish_time - start_time;
        if (took > vi->golden_time) {
            printf("Ambulance %c FAILED: took %d > %d\n",
        			vi->id, took, vi->golden_time);
        } else {
            printf("Ambulance %c OK: took %d <= %d\n",
                   vi->id, took, vi->golden_time);
        }
    }

	/* status transition must happen before sema_up */
	vi->state = VEHICLE_STATUS_FINISHED;
}
