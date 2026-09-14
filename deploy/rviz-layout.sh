#!/bin/bash
# 只操作 RViz 专用的 Xvfb 显示：把 RViz 主窗口左上角推到屏幕外，
# 使 Xvfb 屏幕可见区域恰好只剩 3D 渲染区（无菜单栏/工具栏/状态栏），
# 后端截屏即得到与网页容器同比例的纯净实时画面。
# 用法: rviz-layout.sh <render_w> <render_h> <chrome_left> <chrome_top> <chrome_right> <chrome_bottom>
set -u
RW=${1:-694}; RH=${2:-434}
CL=${3:-25}; CT=${4:-71}; CR=${5:-25}; CB=${6:-39}
WW=$((RW+CL+CR)); WH=$((CT+RH+CB)); WX=$((-CL)); WY=$((-CT))

for attempt in $(seq 1 40); do
  best=""; bestarea=0
  for w in $(xdotool search --class rviz 2>/dev/null); do
    eval "$(xdotool getwindowgeometry --shell "$w" 2>/dev/null)" || continue
    area=$((WIDTH*HEIGHT))
    if [ "$area" -gt "$bestarea" ]; then bestarea=$area; best=$w; fi
  done
  if [ -n "$best" ]; then
    xdotool windowmove "$best" "$WX" "$WY" windowsize "$best" "$WW" "$WH"
    sleep 1
    # RViz 会按窗口尺寸重排渲染面板，重复一次确保尺寸稳定。
    xdotool windowmove "$best" "$WX" "$WY" windowsize "$best" "$WW" "$WH"
    exit 0
  fi
  sleep 1
done
exit 1
