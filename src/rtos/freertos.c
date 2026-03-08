// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include <target/smp.h>
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"
#include "target/cortex_m.h"
#include "target/register.h"
#include "server/gdb_server.h"

#define FREERTOS_MAX_PRIORITIES	63
#define FREERTOS_MAX_CORES		8

/* FIXME: none of the _width parameters are actually observed properly!
 * you WILL need to edit more if you actually attempt to target a 8/16/64
 * bit target!
 */

struct freertos_params {
	const char *target_name;
	const unsigned char thread_count_width;
	const unsigned char pointer_width;
	const unsigned char list_next_offset;			/* offsetof(List_t, xListEnd.pxNext) */
	const unsigned char list_width;					/* sizeof(List_t) */
	const unsigned char list_elem_next_offset;		/* offsetof(ListItem_t, pxNext) */
	const unsigned char list_elem_content_offset;	/* offsetof(ListItem_t, pvOwner) */
	const unsigned char thread_stack_offset;		/* offsetof(TCB_t, pxTopOfStack) */
	const unsigned char thread_name_offset;			/* offsetof(TCB_t, pcTaskName) - single core */
	const unsigned char thread_name_offset_smp;		/* offsetof(TCB_t, pcTaskName) - SMP (configNUMBER_OF_CORES > 1, configUSE_CORE_AFFINITY == 1) */
	const struct rtos_register_stacking *stacking_info_cm3;
	const struct rtos_register_stacking *stacking_info_cm4f;
	const struct rtos_register_stacking *stacking_info_cm4f_fpu;
	const struct rtos_register_stacking *stacking_info_cm33;
	const struct rtos_register_stacking *stacking_info_cm33_fpu;
};

/* Mutable per-session state: which SMP core index each current TCB is running on */
struct freertos_state {
	const struct freertos_params *params;
	uint32_t current_tcbs[FREERTOS_MAX_CORES]; /* TCB pointer for each running core */
	int num_cores;
	bool smp_mode;
};

static const struct freertos_params freertos_params_list[] = {
	{
	"cortex_m",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	12,						/* list_next_offset; */
	20,						/* list_width; */
	4,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	64,						/* thread_name_offset_smp; */
	&rtos_standard_cortex_m3_stacking,		/* stacking_info_cm3 */
	&rtos_standard_cortex_m4f_stacking,		/* stacking_info_cm4f */
	&rtos_standard_cortex_m4f_fpu_stacking,	/* stacking_info_cm4f_fpu */
	&rtos_standard_cortex_m33_stacking,		/* stacking_info_cm33 */
	&rtos_standard_cortex_m33_fpu_stacking,	/* stacking_info_cm33_fpu */
	},
	{
	"hla_target",			/* target_name */
	4,						/* thread_count_width; */
	4,						/* pointer_width; */
	12,						/* list_next_offset; */
	20,						/* list_width; */
	4,						/* list_elem_next_offset; */
	12,						/* list_elem_content_offset */
	0,						/* thread_stack_offset; */
	52,						/* thread_name_offset; */
	64,						/* thread_name_offset_smp; */
	&rtos_standard_cortex_m3_stacking,		/* stacking_info_cm3 */
	&rtos_standard_cortex_m4f_stacking,		/* stacking_info_cm4f */
	&rtos_standard_cortex_m4f_fpu_stacking,	/* stacking_info_cm4f_fpu */
	NULL,									/* stacking_info_cm33 (not applicable) */
	NULL,									/* stacking_info_cm33_fpu (not applicable) */
	},
};

static bool freertos_detect_rtos(struct target *target);
static int freertos_create(struct target *target);
static int freertos_update_threads(struct rtos *rtos);
static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);
static int freertos_target_for_threadid(struct connection *connection,
		int64_t thread_id, struct target **p_target);

const struct rtos_type freertos_rtos = {
	.name = "FreeRTOS",

	.detect_rtos = freertos_detect_rtos,
	.create = freertos_create,
	.update_threads = freertos_update_threads,
	.get_thread_reg_list = freertos_get_thread_reg_list,
	.get_symbol_list_to_lookup = freertos_get_symbol_list_to_lookup,
};

enum freertos_symbol_values {
	FREERTOS_VAL_PX_CURRENT_TCB = 0,
	FREERTOS_VAL_PX_READY_TASKS_LISTS = 1,
	FREERTOS_VAL_X_DELAYED_TASK_LIST1 = 2,
	FREERTOS_VAL_X_DELAYED_TASK_LIST2 = 3,
	FREERTOS_VAL_PX_DELAYED_TASK_LIST = 4,
	FREERTOS_VAL_PX_OVERFLOW_DELAYED_TASK_LIST = 5,
	FREERTOS_VAL_X_PENDING_READY_LIST = 6,
	FREERTOS_VAL_X_TASKS_WAITING_TERMINATION = 7,
	FREERTOS_VAL_X_SUSPENDED_TASK_LIST = 8,
	FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS = 9,
	FREERTOS_VAL_UX_TOP_USED_PRIORITY = 10,
	FREERTOS_VAL_X_SCHEDULER_RUNNING = 11,
	FREERTOS_VAL_PX_CURRENT_TCBS = 12,
};

struct symbols {
	const char *name;
	bool optional;
};

static const struct symbols freertos_symbol_list[] = {
	{ "pxCurrentTCB", true },   /* Optional: absent in SMP FreeRTOS builds */
	{ "pxReadyTasksLists", false },
	{ "xDelayedTaskList1", false },
	{ "xDelayedTaskList2", false },
	{ "pxDelayedTaskList", false },
	{ "pxOverflowDelayedTaskList", false },
	{ "xPendingReadyList", false },
	{ "xTasksWaitingTermination", true }, /* Only if INCLUDE_vTaskDelete */
	{ "xSuspendedTaskList", true }, /* Only if INCLUDE_vTaskSuspend */
	{ "uxCurrentNumberOfTasks", false },
	{ "uxTopUsedPriority", true }, /* Unavailable since v7.5.3 */
	{ "xSchedulerRunning", false },
	{ "pxCurrentTCBs", true },  /* Optional: present only in SMP FreeRTOS builds */
	{ NULL, false }
};

/* TODO: */
/* this is not safe for little endian yet */
/* may be problems reading if sizes are not 32 bit long integers. */
/* test mallocs for failure */

static int freertos_update_threads(struct rtos *rtos)
{
	int retval;
	unsigned int tasks_found = 0;
	const struct freertos_params *param;
	struct freertos_state *state;

	if (!rtos->rtos_specific_params)
		return -1;

	state = (struct freertos_state *) rtos->rtos_specific_params;
	param = state->params;

	if (!rtos->symbols) {
		LOG_ERROR("No symbols for FreeRTOS");
		return -3;
	}

	if (rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address == 0) {
		LOG_ERROR("Don't have the number of threads in FreeRTOS");
		return -2;
	}

	uint32_t thread_list_size = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
			&thread_list_size);
	LOG_DEBUG("FreeRTOS: Read uxCurrentNumberOfTasks at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_CURRENT_NUMBER_OF_TASKS].address,
										thread_list_size);

	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read FreeRTOS thread count from target");
		return retval;
	}

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* Determine SMP mode: SMP FreeRTOS uses pxCurrentTCBs[], single-core uses pxCurrentTCB */
	bool smp_mode = (rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCBS].address != 0);
	uint32_t current_tcbs[FREERTOS_MAX_CORES];
	int num_cores = 1;
	memset(current_tcbs, 0, sizeof(current_tcbs));
	uint32_t pointer_casts_are_bad;

	if (smp_mode) {
		/* Count the number of cores from the SMP target list */
		if (rtos->target->smp) {
			struct target_list *head;
			num_cores = 0;
			foreach_smp_target(head, rtos->target->smp_targets)
				num_cores++;
			if (num_cores < 1)
				num_cores = 1;
			if (num_cores > FREERTOS_MAX_CORES)
				num_cores = FREERTOS_MAX_CORES;
		}

		/* Read the current TCB pointer for each core from pxCurrentTCBs[] */
		rtos->current_thread = 0;
		for (int i = 0; i < num_cores; i++) {
			uint32_t tcb_ptr = 0;
			retval = target_read_u32(rtos->target,
					rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCBS].address +
					i * param->pointer_width,
					&tcb_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading pxCurrentTCBs[%d] in FreeRTOS SMP", i);
				return retval;
			}
			current_tcbs[i] = tcb_ptr;
			LOG_DEBUG("FreeRTOS: Read pxCurrentTCBs[%d] at 0x%" PRIx64 ", value 0x%" PRIx32,
					i,
					rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCBS].address + i * param->pointer_width,
					tcb_ptr);
		}

		/* Select the current_thread from whichever core caused the halt.
		 * Use the same debug_reason priority as hwthread so that a
		 * breakpoint/singlestep on core N is preferred over a core that
		 * merely halted due to SMP propagation (DBGRQ). */
		{
			struct target_list *head;
			int core_idx = 0;
			enum target_debug_reason best_reason = DBG_REASON_UNDEFINED;
			foreach_smp_target(head, rtos->target->smp_targets) {
				struct target *curr = head->target;
				if (core_idx < num_cores && current_tcbs[core_idx] != 0) {
					bool update = false;
					switch (best_reason) {
					case DBG_REASON_UNDEFINED:
						update = true;
						break;
					case DBG_REASON_SINGLESTEP:
						if (curr->debug_reason == DBG_REASON_SINGLESTEP)
							update = true;
						break;
					case DBG_REASON_BREAKPOINT:
						if (curr->debug_reason == DBG_REASON_SINGLESTEP)
							update = true;
						break;
					case DBG_REASON_WATCHPOINT:
						if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
								curr->debug_reason == DBG_REASON_BREAKPOINT)
							update = true;
						break;
					case DBG_REASON_DBGRQ:
						if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
								curr->debug_reason == DBG_REASON_WATCHPOINT ||
								curr->debug_reason == DBG_REASON_BREAKPOINT)
							update = true;
						break;
					default:
						break;
					}
					if (update) {
						best_reason = curr->debug_reason;
						rtos->current_thread = current_tcbs[core_idx];
					}
				}
				core_idx++;
			}
			/* Fall back to first valid TCB if no core has a useful debug reason */
			if (rtos->current_thread == 0) {
				for (int i = 0; i < num_cores; i++) {
					if (current_tcbs[i] != 0) {
						rtos->current_thread = current_tcbs[i];
						break;
					}
				}
			}
		}
	} else {
		/* Single-core: read pxCurrentTCB */
		retval = target_read_u32(rtos->target,
				rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
				&pointer_casts_are_bad);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading current thread in FreeRTOS thread list");
			return retval;
		}
		rtos->current_thread = pointer_casts_are_bad;
		current_tcbs[0] = pointer_casts_are_bad;
		LOG_DEBUG("FreeRTOS: Read pxCurrentTCB at 0x%" PRIx64 ", value 0x%" PRIx64,
				rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address,
				rtos->current_thread);
	}

	/* Cache running-task info for use in freertos_get_thread_reg_list */
	state->smp_mode = smp_mode;
	state->num_cores = num_cores;
	memcpy(state->current_tcbs, current_tcbs, sizeof(current_tcbs));

	/* read scheduler running */
	uint32_t scheduler_running;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_X_SCHEDULER_RUNNING].address,
			&scheduler_running);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading FreeRTOS scheduler state");
		return retval;
	}
	LOG_DEBUG("FreeRTOS: Read xSchedulerRunning at 0x%" PRIx64 ", value 0x%" PRIx32,
										rtos->symbols[FREERTOS_VAL_X_SCHEDULER_RUNNING].address,
										scheduler_running);

	if ((thread_list_size  == 0) || (rtos->current_thread == 0) || (scheduler_running != 1)) {
		/* Either : No RTOS threads - there is always at least the current execution though */
		/* OR     : No current thread - all threads suspended - show the current execution
		 * of idling */
		char tmp_str[] = "Current Execution";
		thread_list_size++;
		tasks_found++;
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %d threads", thread_list_size);
			return ERROR_FAIL;
		}
		rtos->current_thread = 1;
		rtos->thread_details->threadid = rtos->current_thread;
		rtos->thread_details->exists = true;
		rtos->thread_details->extra_info_str = NULL;
		rtos->thread_details->thread_name_str = malloc(sizeof(tmp_str));
		strcpy(rtos->thread_details->thread_name_str, tmp_str);

		if (thread_list_size == 1) {
			rtos->thread_count = 1;
			return ERROR_OK;
		}
	} else {
		/* create space for new thread details */
		rtos->thread_details = malloc(
				sizeof(struct thread_detail) * thread_list_size);
		if (!rtos->thread_details) {
			LOG_ERROR("Error allocating memory for %d threads", thread_list_size);
			return ERROR_FAIL;
		}
	}

	/* Find out how many lists are needed to be read from pxReadyTasksLists, */
	if (rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address == 0) {
		LOG_ERROR("FreeRTOS: uxTopUsedPriority is not defined, consult the OpenOCD manual for a work-around");
		return ERROR_FAIL;
	}
	uint32_t top_used_priority = 0;
	retval = target_read_u32(rtos->target,
			rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
			&top_used_priority);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("FreeRTOS: Read uxTopUsedPriority at 0x%" PRIx64 ", value %" PRIu32,
										rtos->symbols[FREERTOS_VAL_UX_TOP_USED_PRIORITY].address,
										top_used_priority);
	if (top_used_priority > FREERTOS_MAX_PRIORITIES) {
		LOG_ERROR("FreeRTOS top used priority is unreasonably big, not proceeding: %" PRIu32,
			top_used_priority);
		return ERROR_FAIL;
	}

	/* uxTopUsedPriority was defined as configMAX_PRIORITIES - 1
	 * in old FreeRTOS versions (before V7.5.3)
	 * Use contrib/rtos-helpers/FreeRTOS-openocd.c to get compatible symbol
	 * in newer FreeRTOS versions.
	 * Here we restore the original configMAX_PRIORITIES value */
	unsigned int config_max_priorities = top_used_priority + 1;

	symbol_address_t *list_of_lists =
		malloc(sizeof(symbol_address_t) * (config_max_priorities + 5));
	if (!list_of_lists) {
		LOG_ERROR("Error allocating memory for %u priorities", config_max_priorities);
		return ERROR_FAIL;
	}

	unsigned int num_lists;
	for (num_lists = 0; num_lists < config_max_priorities; num_lists++)
		list_of_lists[num_lists] = rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address +
			num_lists * param->list_width;

	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST1].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_DELAYED_TASK_LIST2].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_PENDING_READY_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_SUSPENDED_TASK_LIST].address;
	list_of_lists[num_lists++] = rtos->symbols[FREERTOS_VAL_X_TASKS_WAITING_TERMINATION].address;

	for (unsigned int i = 0; i < num_lists; i++) {
		if (list_of_lists[i] == 0)
			continue;

		/* Read the number of threads in this list */
		uint32_t list_thread_count = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i],
				&list_thread_count);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading number of threads in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read thread count for list %u at 0x%" PRIx64 ", value %" PRIu32,
										i, list_of_lists[i], list_thread_count);

		if (list_thread_count == 0)
			continue;

		/* Read the location of first list item */
		uint32_t prev_list_elem_ptr = -1;
		uint32_t list_elem_ptr = 0;
		retval = target_read_u32(rtos->target,
				list_of_lists[i] + param->list_next_offset,
				&list_elem_ptr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
			free(list_of_lists);
			return retval;
		}
		LOG_DEBUG("FreeRTOS: Read first item for list %u at 0x%" PRIx64 ", value 0x%" PRIx32,
										i, list_of_lists[i] + param->list_next_offset, list_elem_ptr);

		while ((list_thread_count > 0) && (list_elem_ptr != 0) &&
				(list_elem_ptr != prev_list_elem_ptr) &&
				(tasks_found < thread_list_size)) {
			/* Get the location of the thread structure. */
			retval = target_read_u32(rtos->target,
					list_elem_ptr + param->list_elem_content_offset,
					&pointer_casts_are_bad);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading thread list item object in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			rtos->thread_details[tasks_found].threadid = pointer_casts_are_bad;
			LOG_DEBUG("FreeRTOS: Read Thread ID at 0x%" PRIx32 ", value 0x%" PRIx64,
										list_elem_ptr + param->list_elem_content_offset,
										rtos->thread_details[tasks_found].threadid);

			/* get thread name */

			#define FREERTOS_THREAD_NAME_STR_SIZE (200)
			char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];

			/* Read the thread name */
			retval = target_read_buffer(rtos->target,
					rtos->thread_details[tasks_found].threadid +
					(smp_mode ? param->thread_name_offset_smp : param->thread_name_offset),
					FREERTOS_THREAD_NAME_STR_SIZE,
					(uint8_t *)&tmp_str);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';
			LOG_DEBUG("FreeRTOS: Read Thread Name at 0x%" PRIx64 ", value '%s'",
										rtos->thread_details[tasks_found].threadid +
										(smp_mode ? param->thread_name_offset_smp : param->thread_name_offset),
										tmp_str);

			if (tmp_str[0] == '\x00')
				strcpy(tmp_str, "No Name");

			rtos->thread_details[tasks_found].thread_name_str =
				malloc(strlen(tmp_str)+1);
			strcpy(rtos->thread_details[tasks_found].thread_name_str, tmp_str);
			rtos->thread_details[tasks_found].exists = true;

			/* Mark the task as running on whichever core it is current for */
			rtos->thread_details[tasks_found].extra_info_str = NULL;
			for (int core = 0; core < num_cores; core++) {
				if (current_tcbs[core] != 0 &&
						rtos->thread_details[tasks_found].threadid == current_tcbs[core]) {
					char running_str[32];
					if (smp_mode && num_cores > 1)
						snprintf(running_str, sizeof(running_str),
								"State: Running (Core %d)", core);
					else
						snprintf(running_str, sizeof(running_str), "State: Running");
					rtos->thread_details[tasks_found].extra_info_str =
							malloc(strlen(running_str) + 1);
					if (rtos->thread_details[tasks_found].extra_info_str)
						strcpy(rtos->thread_details[tasks_found].extra_info_str,
								running_str);
					break;
				}
			}

			tasks_found++;
			list_thread_count--;
			rtos->thread_count = tasks_found;

			prev_list_elem_ptr = list_elem_ptr;
			list_elem_ptr = 0;
			retval = target_read_u32(rtos->target,
					prev_list_elem_ptr + param->list_elem_next_offset,
					&list_elem_ptr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Error reading next thread item location in FreeRTOS thread list");
				free(list_of_lists);
				return retval;
			}
			LOG_DEBUG("FreeRTOS: Read next thread location at 0x%" PRIx32 ", value 0x%" PRIx32,
										prev_list_elem_ptr + param->list_elem_next_offset,
										list_elem_ptr);
		}
	}

	free(list_of_lists);
	return 0;
}

static int freertos_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	int retval;
	const struct freertos_params *param;
	struct freertos_state *state;
	int64_t stack_ptr = 0;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -1;

	state = (struct freertos_state *) rtos->rtos_specific_params;
	param = state->params;

	/* For currently-running tasks the TCB's saved stack is stale (the task has
	 * not been context-switched out).  Read live hardware registers from the
	 * corresponding SMP core instead. */
	if (rtos->target->smp) {
		struct target_list *head;
		int core_idx = 0;
		foreach_smp_target(head, rtos->target->smp_targets) {
			if (core_idx < state->num_cores &&
					state->current_tcbs[core_idx] == (uint32_t)thread_id) {
				struct target *core_target = head->target;
				int reg_list_size;
				struct reg **hw_reg_list;
				retval = target_get_gdb_reg_list(core_target, &hw_reg_list,
						&reg_list_size, REG_CLASS_GENERAL);
				if (retval != ERROR_OK)
					break; /* fall through to saved-context path */
				int j = 0;
				for (int i = 0; i < reg_list_size; i++) {
					if (!hw_reg_list[i] || !hw_reg_list[i]->exist ||
							hw_reg_list[i]->hidden)
						continue;
					j++;
				}
				*num_regs = j;
				*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
				if (!*reg_list) {
					free(hw_reg_list);
					return ERROR_FAIL;
				}
				j = 0;
				for (int i = 0; i < reg_list_size; i++) {
					if (!hw_reg_list[i] || !hw_reg_list[i]->exist ||
							hw_reg_list[i]->hidden)
						continue;
					if (!hw_reg_list[i]->valid)
						hw_reg_list[i]->type->get(hw_reg_list[i]);
					(*reg_list)[j].number = hw_reg_list[i]->number;
					(*reg_list)[j].size = hw_reg_list[i]->size;
					memcpy((*reg_list)[j].value, hw_reg_list[i]->value,
							DIV_ROUND_UP(hw_reg_list[i]->size, 8));
					j++;
				}
				free(hw_reg_list);
				LOG_DEBUG("FreeRTOS: thread 0x%" PRIx64
						" is running on core %d, using hardware registers",
						thread_id, core_idx);
				return ERROR_OK;
			}
			core_idx++;
		}
	}

	/* Read the stack pointer from the TCB */
	uint32_t pointer_casts_are_bad;
	retval = target_read_u32(rtos->target,
			thread_id + param->thread_stack_offset,
			&pointer_casts_are_bad);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading stack frame from FreeRTOS thread");
		return retval;
	}
	stack_ptr = pointer_casts_are_bad;
	LOG_DEBUG("FreeRTOS: Read stack pointer at 0x%" PRIx64 ", value 0x%" PRIx64,
										thread_id + param->thread_stack_offset,
										stack_ptr);

	/* Detect whether the target is ARMv8-M (Cortex-M33 etc.) using the cached
	 * part number.  The RP2350 ARM_NTZ FreeRTOS port saves PSPLIM and EXC_RETURN
	 * as the first two words of the software-saved context, so all register
	 * offsets differ from the standard CM3/CM4F layout. */
	bool is_armv8m = false;
	enum cortex_m_impl_part impl_part = cortex_m_get_impl_part(rtos->target);
	switch (impl_part) {
	case CORTEX_M23_PARTNO:
	case CORTEX_M33_PARTNO:
	case CORTEX_M35P_PARTNO:
	case CORTEX_M55_PARTNO:
	case CORTEX_M85_PARTNO:
		is_armv8m = true;
		break;
	default:
		break;
	}

	/* Check for an enabled FPU */
	int fpu_enabled = 0;
	struct armv7m_common *armv7m_target = target_to_armv7m(rtos->target);
	if (is_armv7m(armv7m_target)) {
		if ((armv7m_target->fp_feature == FPV4_SP) || (armv7m_target->fp_feature == FPV5_SP) ||
				(armv7m_target->fp_feature == FPV5_DP)) {
			uint32_t cpacr;
			retval = target_read_u32(rtos->target, FPU_CPACR, &cpacr);
			if (retval != ERROR_OK) {
				LOG_ERROR("Could not read CPACR register to check FPU state");
				return -1;
			}
			if (cpacr & 0x00F00000)
				fpu_enabled = 1;
		}
	}

	if (is_armv8m && param->stacking_info_cm33) {
		/* ARMv8-M (ARM_NTZ port): EXC_RETURN is at stack_ptr+0x04 (before r4-r11).
		 * Use it to decide between basic and FPU-extended frame. */
		if (fpu_enabled && param->stacking_info_cm33_fpu) {
			uint32_t exc_return = 0;
			retval = target_read_u32(rtos->target, stack_ptr + 0x04, &exc_return);
			if (retval != ERROR_OK) {
				LOG_OUTPUT("Error reading EXC_RETURN from FreeRTOS thread stack");
				return retval;
			}
			if ((exc_return & 0x10) == 0)
				return rtos_generic_stack_read(rtos->target,
						param->stacking_info_cm33_fpu, stack_ptr, reg_list, num_regs);
		}
		return rtos_generic_stack_read(rtos->target,
				param->stacking_info_cm33, stack_ptr, reg_list, num_regs);
	}

	/* Standard ARMv7-M (CM3/CM4F) path */
	if (fpu_enabled) {
		/* Read the LR to decide between stacking with or without FPU */
		uint32_t lr_svc = 0;
		retval = target_read_u32(rtos->target,
				stack_ptr + 0x20,
				&lr_svc);
		if (retval != ERROR_OK) {
			LOG_OUTPUT("Error reading stack frame from FreeRTOS thread");
			return retval;
		}
		if ((lr_svc & 0x10) == 0)
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f_fpu, stack_ptr, reg_list, num_regs);
		else
			return rtos_generic_stack_read(rtos->target, param->stacking_info_cm4f, stack_ptr, reg_list, num_regs);
	} else
		return rtos_generic_stack_read(rtos->target, param->stacking_info_cm3, stack_ptr, reg_list, num_regs);
}

static int freertos_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	unsigned int i;
	*symbol_list = calloc(
			ARRAY_SIZE(freertos_symbol_list), sizeof(struct symbol_table_elem));

	for (i = 0; i < ARRAY_SIZE(freertos_symbol_list); i++) {
		(*symbol_list)[i].symbol_name = freertos_symbol_list[i].name;
		(*symbol_list)[i].optional = freertos_symbol_list[i].optional;
	}

	return 0;
}

#if 0

static int freertos_set_current_thread(struct rtos *rtos, threadid_t thread_id)
{
	return 0;
}

static int freertos_get_thread_ascii_info(struct rtos *rtos, threadid_t thread_id, char **info)
{
	int retval;
	const struct freertos_params *param;

	if (!rtos)
		return -1;

	if (thread_id == 0)
		return -2;

	if (!rtos->rtos_specific_params)
		return -3;

	param = (const struct freertos_params *) rtos->rtos_specific_params;

#define FREERTOS_THREAD_NAME_STR_SIZE (200)
	char tmp_str[FREERTOS_THREAD_NAME_STR_SIZE];

	/* Read the thread name */
	retval = target_read_buffer(rtos->target,
			thread_id + param->thread_name_offset,
			FREERTOS_THREAD_NAME_STR_SIZE,
			(uint8_t *)&tmp_str);
	if (retval != ERROR_OK) {
		LOG_ERROR("Error reading first thread item location in FreeRTOS thread list");
		return retval;
	}
	tmp_str[FREERTOS_THREAD_NAME_STR_SIZE-1] = '\x00';

	if (tmp_str[0] == '\x00')
		strcpy(tmp_str, "No Name");

	*info = malloc(strlen(tmp_str)+1);
	strcpy(*info, tmp_str);
	return 0;
}

#endif

static int freertos_target_for_threadid(struct connection *connection,
		int64_t thread_id, struct target **p_target)
{
	struct target *target = get_target_from_connection(connection);
	*p_target = target;

	if (!target->rtos || !target->rtos->rtos_specific_params)
		return ERROR_OK;

	struct freertos_state *state =
		(struct freertos_state *)target->rtos->rtos_specific_params;

	/* In SMP mode, map a currently-running task to its physical core so that
	 * single-step and resume operations are directed to the correct hardware
	 * thread.  Paused tasks have no associated core; leave them on the
	 * primary target (the default). */
	if (state->smp_mode && target->smp) {
		struct target_list *head;
		int core_idx = 0;
		foreach_smp_target(head, target->smp_targets) {
			if (core_idx < state->num_cores &&
					state->current_tcbs[core_idx] == (uint32_t)thread_id) {
				*p_target = head->target;
				return ERROR_OK;
			}
			core_idx++;
		}
	}

	return ERROR_OK;
}

static bool freertos_detect_rtos(struct target *target)
{
	if ((target->rtos->symbols) &&
			(target->rtos->symbols[FREERTOS_VAL_PX_READY_TASKS_LISTS].address != 0) &&
			((target->rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCB].address != 0) ||
			 (target->rtos->symbols[FREERTOS_VAL_PX_CURRENT_TCBS].address != 0))) {
		/* looks like FreeRTOS */
		return true;
	}
	return false;
}

static int freertos_create(struct target *target)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(freertos_params_list); i++) {
		if (strcmp(freertos_params_list[i].target_name, target_type_name(target)) == 0) {
			struct freertos_state *state = calloc(1, sizeof(*state));
			if (!state) {
				LOG_ERROR("Failed to allocate FreeRTOS state");
				return ERROR_FAIL;
			}
			state->params = &freertos_params_list[i];
			target->rtos->rtos_specific_params = state;
			target->rtos->gdb_target_for_threadid = freertos_target_for_threadid;
			return ERROR_OK;
		}
	}

	LOG_ERROR("Could not find target in FreeRTOS compatibility list");
	return ERROR_FAIL;
}
