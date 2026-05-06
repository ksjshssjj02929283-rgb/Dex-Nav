#!/usr/bin/env bash
# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

set -u

INTERVAL="${1:-1}"
PROCESS_PATTERN="${2:-controller_server}"
OUTPUT_DIR="${3:-/tmp/nav_cpu_logs_$(date +%Y%m%d_%H%M%S)}"
CPU_LIST="${4:-3,4}"
EXTRA_PROCESS_PATTERN="${5:-nxnode.bin}"

mkdir -p "${OUTPUT_DIR}"

SUMMARY_LOG="${OUTPUT_DIR}/summary.log"
CPU_LOG="${OUTPUT_DIR}/cpu_cores.log"
CORE_TASKS_LOG="${OUTPUT_DIR}/cpu_core_tasks.log"
PIDSTAT_LOG="${OUTPUT_DIR}/pidstat.log"
TOP_LOG="${OUTPUT_DIR}/top_threads.log"
MPSTAT_LOG="${OUTPUT_DIR}/mpstat.log"
PROC_STAT_PREV="${OUTPUT_DIR}/.proc_stat_prev"
PROC_STAT_CURR="${OUTPUT_DIR}/.proc_stat_curr"

has_cmd() {
  command -v "$1" >/dev/null 2>&1
}

ros_ts() {
  printf '[%s.%09d]' "$(date +%s)" "$((10#$(date +%N)))"
}

log_header() {
  printf '\n===== %s %s =====\n' "$(ros_ts)" "$(date '+%F %T %z')"
}

append_text() {
  local log_file="$1"
  shift
  {
    log_header
    "$@"
  } >>"${log_file}" 2>&1
}

append_summary() {
  local pid="$1"
  local extra_pid
  extra_pid="$(pgrep -f "${EXTRA_PROCESS_PATTERN}" | head -n 1 || true)"
  {
    log_header
    printf 'pattern: %s\n' "${PROCESS_PATTERN}"
    printf 'pid: %s\n' "${pid:-N/A}"
    printf 'extra_pattern: %s\n' "${EXTRA_PROCESS_PATTERN}"
    printf 'extra_pid: %s\n' "${extra_pid:-N/A}"
    printf 'cpu_list: %s\n' "${CPU_LIST}"
    printf 'uptime/load: '
    cat /proc/loadavg
    printf 'cpu MHz (first 8 cores):\n'
    grep 'cpu MHz' /proc/cpuinfo | head -n 8
    if [[ -n "${pid}" ]]; then
      printf 'ps snapshot:\n'
      ps -p "${pid}" -o pid,ppid,psr,pcpu,pmem,stat,etime,comm,args
    fi
    if [[ -n "${extra_pid}" ]]; then
      printf 'extra ps snapshot:\n'
      ps -p "${extra_pid}" -o pid,ppid,psr,pcpu,pmem,stat,etime,comm,args
    fi
  } >>"${SUMMARY_LOG}" 2>&1
}

sample_proc_stat() {
  local output_file="$1"
  awk -v cpu_list="${CPU_LIST}" '
    BEGIN {
      n = split(cpu_list, arr, ",");
      for (i = 1; i <= n; ++i) {
        wanted["cpu" arr[i]] = 1;
      }
    }
    /^cpu[0-9]+ / {
      if ($1 in wanted) {
        print $0;
      }
    }
  ' /proc/stat >"${output_file}"
}

append_proc_stat_usage() {
  if [[ ! -f "${PROC_STAT_PREV}" ]]; then
    return
  fi

  {
    log_header
    awk '
      NR == FNR {
        prev_user[$1] = $2;
        prev_nice[$1] = $3;
        prev_system[$1] = $4;
        prev_idle[$1] = $5;
        prev_iowait[$1] = $6;
        prev_irq[$1] = $7;
        prev_softirq[$1] = $8;
        prev_steal[$1] = $9;
        prev_guest[$1] = $10;
        prev_guest_nice[$1] = $11;
        next;
      }
      {
        idle_prev = prev_idle[$1] + prev_iowait[$1];
        idle_curr = $5 + $6;
        non_idle_prev = prev_user[$1] + prev_nice[$1] + prev_system[$1] + prev_irq[$1] + prev_softirq[$1] + prev_steal[$1];
        non_idle_curr = $2 + $3 + $4 + $7 + $8 + $9;
        total_prev = idle_prev + non_idle_prev;
        total_curr = idle_curr + non_idle_curr;
        total_delta = total_curr - total_prev;
        idle_delta = idle_curr - idle_prev;
        usage = 0.0;
        if (total_delta > 0) {
          usage = (total_delta - idle_delta) * 100.0 / total_delta;
        }
        printf "%s usage=%.2f%% user=%d system=%d idle=%d iowait=%d irq=%d softirq=%d\n", $1, usage, $2 - prev_user[$1], $4 - prev_system[$1], $5 - prev_idle[$1], $6 - prev_iowait[$1], $7 - prev_irq[$1], $8 - prev_softirq[$1];
      }
    ' "${PROC_STAT_PREV}" "${PROC_STAT_CURR}"
  } >>"${CPU_LOG}" 2>&1
}

append_core_tasks() {
  {
    log_header
    ps -eLo pid,tid,psr,pcpu,pmem,stat,comm,args --sort=-pcpu | awk -v cpu_list="${CPU_LIST}" '
      BEGIN {
        n = split(cpu_list, arr, ",");
        for (i = 1; i <= n; ++i) {
          wanted[arr[i]] = 1;
        }
        print "PID TID PSR %CPU %MEM STAT COMM COMMAND";
      }
      NR == 1 {
        next;
      }
      ($3 in wanted) {
        print;
        count++;
      }
    '
    printf '\nTracked process totals on cpu_list:\n'
    ps -eLo pid,psr,pcpu,comm,args --sort=-pcpu | awk \
      -v cpu_list="${CPU_LIST}" \
      -v main_pattern="${PROCESS_PATTERN}" \
      -v extra_pattern="${EXTRA_PROCESS_PATTERN}" '
      BEGIN {
        n = split(cpu_list, arr, ",");
        for (i = 1; i <= n; ++i) {
          wanted[arr[i]] = 1;
        }
      }
      ($2 in wanted) {
        line = $0;
        if (index(line, main_pattern) > 0) {
          main_sum += $3;
        }
        if (extra_pattern != "" && index(line, extra_pattern) > 0) {
          extra_sum += $3;
        }
      }
      END {
        printf "%s on cpu[%s]: %.1f%%\n", main_pattern, cpu_list, main_sum + 0.0;
        if (extra_pattern != "") {
          printf "%s on cpu[%s]: %.1f%%\n", extra_pattern, cpu_list, extra_sum + 0.0;
        }
      }
    '
  } >>"${CORE_TASKS_LOG}" 2>&1
}

append_mpstat() {
  if has_cmd mpstat; then
    append_text "${MPSTAT_LOG}" mpstat -P "${CPU_LIST}" "${INTERVAL}" 1
  elif [[ ! -f "${OUTPUT_DIR}/.mpstat_missing_logged" ]]; then
    {
      log_header
      echo "mpstat not found, falling back to /proc/stat sampling in cpu_cores.log"
    } >>"${MPSTAT_LOG}"
    : >"${OUTPUT_DIR}/.mpstat_missing_logged"
  fi
}

append_pidstat() {
  local pid="$1"
  if [[ -z "${pid}" ]]; then
    return
  fi

  if has_cmd pidstat; then
    append_text "${PIDSTAT_LOG}" pidstat -t -p "${pid}" "${INTERVAL}" 1
  elif [[ ! -f "${OUTPUT_DIR}/.pidstat_missing_logged" ]]; then
    {
      log_header
      echo "pidstat not found"
    } >>"${PIDSTAT_LOG}"
    : >"${OUTPUT_DIR}/.pidstat_missing_logged"
  fi
}

append_top_threads() {
  local pid="$1"
  {
    log_header
    if [[ -n "${pid}" ]]; then
      top -H -b -n 1 -p "${pid}"
    else
      echo "process not found for pattern: ${PROCESS_PATTERN}"
    fi
  } >>"${TOP_LOG}" 2>&1
}

echo "Recording CPU runtime info to ${OUTPUT_DIR}"
echo "Sampling interval: ${INTERVAL}s"
echo "Process pattern: ${PROCESS_PATTERN}"
echo "Extra process pattern: ${EXTRA_PROCESS_PATTERN}"
echo "CPU list: ${CPU_LIST}"
echo "Press Ctrl+C to stop."

sample_proc_stat "${PROC_STAT_PREV}"

while true; do
  PID="$(pgrep -f "${PROCESS_PATTERN}" | head -n 1 || true)"

  append_summary "${PID}"
  sample_proc_stat "${PROC_STAT_CURR}"
  append_proc_stat_usage
  # append_core_tasks
  append_mpstat
  append_pidstat "${PID}"
  append_top_threads "${PID}"

  mv "${PROC_STAT_CURR}" "${PROC_STAT_PREV}"

  sleep "${INTERVAL}"
done
