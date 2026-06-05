#!/usr/bin/env python3
import json
import math
import os
import re
import shlex
import signal
import socket
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PointStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Int32MultiArray, String, UInt8

try:
    import rosnode
except Exception:
    rosnode = None


PARAM_SPECS = {
    "initial_wait_x": (-2.0, 8.0),
    "initial_wait_y": (-5.0, 5.0),
    "initial_wait_z": (0.05, 3.0),
    "expected_frame_x": (-2.0, 8.0),
    "expected_frame_y": (-5.0, 5.0),
    "expected_frame_z": (0.05, 3.0),
    "qr_goal_x": (-2.0, 8.0),
    "qr_goal_y": (-5.0, 5.0),
    "qr_goal_z": (0.05, 3.0),
    "attack_zone_x": (-2.0, 8.0),
    "attack_zone_y": (-5.0, 5.0),
    "attack_zone_z": (0.05, 3.0),
    "attack_height": (0.05, 2.0),
}
MODE_VALUES = ("auto_detect", "expected_direct")
CONTROL_PARAM_NAMES = list(PARAM_SPECS.keys()) + ["frame_center_mode"]


INDEX_HTML = r"""<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CRAIC Mission Control</title>
  <style>
    :root {
      --bg: #0e1117;
      --panel: #171b24;
      --panel2: #1f2531;
      --line: #303846;
      --text: #e8edf5;
      --muted: #99a3b4;
      --green: #32d583;
      --amber: #fdb022;
      --red: #f97066;
      --blue: #70b8ff;
      --cyan: #5eead4;
    }
    * { box-sizing: border-box; }
    html, body { min-height: 100%; width: 100%; }
    body {
      margin: 0;
      color: var(--text);
      background: var(--bg);
      font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      overflow-x: hidden;
    }
    button, input, select { font: inherit; }
    .app { min-height: 100vh; display: grid; grid-template-rows: auto 1fr; }
    header {
      min-height: 64px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 16px;
      padding: 0 18px;
      border-bottom: 1px solid var(--line);
      background: #11151d;
      flex-wrap: wrap;
    }
    .brand { display: flex; flex-direction: column; gap: 2px; }
    .brand b { font-size: 18px; letter-spacing: 0; }
    .brand span { color: var(--muted); font-size: 12px; }
    .topbar { display: flex; align-items: center; gap: 10px; flex-wrap: wrap; justify-content: flex-end; }
    .pinbox { display: flex; align-items: center; gap: 8px; }
    input, select {
      background: #0f141d;
      border: 1px solid var(--line);
      color: var(--text);
      border-radius: 6px;
      padding: 8px 9px;
      min-width: 0;
    }
    input:focus, select:focus { outline: 1px solid var(--blue); }
    button {
      border: 1px solid var(--line);
      background: var(--panel2);
      color: var(--text);
      border-radius: 6px;
      padding: 8px 12px;
      cursor: pointer;
      min-height: 38px;
    }
    button.primary { background: #146c43; border-color: #1f8a55; }
    button.danger { background: #7a271a; border-color: #b42318; }
    button:disabled { opacity: .45; cursor: not-allowed; }
    .badge {
      padding: 5px 8px;
      border: 1px solid var(--line);
      border-radius: 999px;
      color: var(--muted);
      font-size: 12px;
      white-space: nowrap;
    }
    .badge.ok { color: var(--green); border-color: #235c44; background: #10281f; }
    .badge.warn { color: var(--amber); border-color: #6d4d12; background: #2a1f0b; }
    .badge.bad { color: var(--red); border-color: #7a271a; background: #2b1110; }
    main {
      display: grid;
      grid-template-columns: 360px minmax(300px, 1fr) minmax(500px, 540px);
      gap: 12px;
      padding: 12px;
      min-height: 0;
      height: calc(100vh - 64px);
    }
    section {
      min-height: 0;
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
    }
    section.scene {
      display: grid;
      grid-template-rows: 42px minmax(0, 1fr);
    }
    .panel-head {
      height: 42px;
      padding: 0 12px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      border-bottom: 1px solid var(--line);
      background: #151a22;
      color: #d5dbe7;
      font-weight: 650;
    }
    .panel-body { padding: 12px; overflow: auto; height: calc(100% - 42px); }
    .status-section .panel-body { overflow: auto; }
    .kv { display: grid; grid-template-columns: 96px minmax(0, 1fr); gap: 8px 10px; align-items: start; font-size: 13px; }
    .kv .k { color: var(--muted); }
    .kv > div:nth-child(even) { min-width: 0; overflow-wrap: anywhere; white-space: normal; }
    .mono { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    .scene-wrap { min-height: 0; display: grid; grid-template-rows: minmax(0, 1fr) auto; }
    canvas {
      width: 100%;
      height: 100%;
      display: block;
      background: #080b10;
      touch-action: none;
    }
    .legend {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      padding: 8px 10px;
      border-top: 1px solid var(--line);
      color: var(--muted);
      font-size: 12px;
      background: #11151d;
    }
    button.small {
      min-height: 28px;
      padding: 4px 8px;
      font-size: 12px;
    }
    .dot { width: 9px; height: 9px; border-radius: 50%; display: inline-block; margin-right: 5px; }
    .config-section .panel-body { overflow: hidden; }
    .forms { display: grid; grid-template-rows: auto auto auto auto minmax(140px, 1fr); gap: 8px; height: 100%; min-height: 0; }
    .group { border: 1px solid var(--line); border-radius: 8px; padding: 8px; background: #141923; min-height: 0; }
    .group h3 { margin: 0 0 7px; font-size: 13px; }
    .param-compact {
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 8px;
      background: #141923;
      display: grid;
      gap: 7px;
    }
    .param-table { display: grid; gap: 6px; }
    .param-row {
      display: grid;
      grid-template-columns: 78px minmax(0, 1fr);
      gap: 8px;
      align-items: end;
    }
    .param-name {
      color: #d5dbe7;
      font-size: 12px;
      line-height: 1.2;
      padding-bottom: 7px;
    }
    .row3 { display: grid; grid-template-columns: repeat(3, minmax(0, 1fr)); gap: 6px; min-width: 0; }
    .field { display: flex; flex-direction: column; gap: 2px; min-width: 0; }
    .field label { color: var(--muted); font-size: 12px; }
    .field input { padding: 5px 6px; min-height: 29px; width: 100%; }
    .mode-row { display: grid; gap: 7px; }
    .mode-compact {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 6px;
    }
    .mode-row label {
      display: flex;
      align-items: center;
      gap: 8px;
      color: #d5dbe7;
      font-size: 12px;
    }
    .mode-row input { width: auto; }
    .control-row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    button.warn { background: #5f3b10; border-color: #b7791f; }
    #poweroffState { font-size: 12px; color: var(--muted); }
    #shutdownBtn, #rebootBtn { min-height: 32px; padding: 6px 10px; }
    .preview {
      border: 1px solid var(--line);
      background: #0f141d;
      padding: 8px;
      border-radius: 6px;
      color: var(--muted);
      font-size: 12px;
      line-height: 1.32;
      min-height: 0;
    }
    .settings-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; min-height: 0; }
    .settings-title {
      color: #d5dbe7;
      margin-bottom: 6px;
      font-weight: 650;
    }
    pre {
      margin: 0;
      white-space: pre-wrap;
      color: #cbd5e1;
      font-size: 12px;
      line-height: 1.45;
    }
    .log-group { min-height: 0; display: grid; grid-template-rows: auto minmax(0, 1fr); }
    .logbox {
      min-height: 130px;
      height: 100%;
      overflow: auto;
      background: #0b0f15;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 8px;
    }
    @media (max-width: 1180px) {
      main {
        grid-template-columns: 1fr;
        grid-auto-rows: auto;
        height: auto;
      }
      .config-section .panel-body { overflow: visible; }
      section.scene {
        height: min(62vh, 560px);
        min-height: 380px;
      }
    }
    @media (max-width: 680px) {
      header {
        align-items: flex-start;
        padding: 10px;
        gap: 10px;
      }
      .brand { width: 100%; }
      .brand b { font-size: 16px; }
      .brand span { font-size: 11px; }
      .topbar {
        width: 100%;
        justify-content: flex-start;
      }
      .pinbox {
        width: 100%;
        display: grid;
        grid-template-columns: minmax(80px, 1fr) auto auto;
        gap: 8px;
      }
      .pinbox input { width: auto !important; }
      main { padding: 8px; gap: 8px; }
      .panel-body { padding: 10px; }
      .kv { grid-template-columns: 96px minmax(0, 1fr); font-size: 12px; }
      .param-row { grid-template-columns: 76px minmax(0, 1fr); gap: 6px; }
      .settings-grid { grid-template-columns: 1fr; }
      .mode-compact { grid-template-columns: 1fr; }
      .row3 { gap: 6px; }
      input { padding: 9px 7px; }
      section.scene {
        height: 430px;
        min-height: 360px;
      }
      .legend { gap: 8px; }
    }
  </style>
</head>
<body>
<div class="app">
  <header>
    <div class="brand">
      <b>CRAIC Mission Control</b>
      <span>离线网页启动器 · 状态监控 · 简易 3D 视图</span>
    </div>
    <div class="topbar">
      <span id="authBadge" class="badge bad">未解锁</span>
      <span id="runBadge" class="badge">任务未知</span>
      <div class="pinbox">
        <input id="pin" type="password" inputmode="numeric" placeholder="PIN" style="width: 96px">
        <button id="loginBtn">解锁</button>
        <button id="startBtn" class="primary" disabled>开始任务</button>
      </div>
    </div>
  </header>
  <main>
    <section class="status-section">
      <div class="panel-head">飞行与感知状态 <span id="ageBadge" class="badge">--</span></div>
      <div class="panel-body">
        <div class="kv mono">
          <div class="k">任务 step</div><div id="taskStep">--</div>
          <div class="k">任务来源</div><div id="missionSource">--</div>
          <div class="k">任务 PID</div><div id="missionPid">--</div>
          <div class="k">退出码</div><div id="lastExit">--</div>
          <div class="k">flight_state</div><div id="flightState">--</div>
          <div class="k">control_mode</div><div id="controlMode">--</div>
          <div class="k">reach_status</div><div id="reachStatus">--</div>
          <div class="k">MAVROS</div><div id="mavros">--</div>
          <div class="k">Odom xyz</div><div id="odomXyz">--</div>
          <div class="k">Yaw</div><div id="odomYaw">--</div>
          <div class="k">门框状态</div><div id="frameStatus">--</div>
          <div class="k">门框中心</div><div id="frameCenter">--</div>
          <div class="k">二维码</div><div id="qrIds">--</div>
          <div class="k">气球 world</div><div id="balloonWorld">--</div>
          <div class="k">启动命令</div><div id="lastCommand">--</div>
        </div>
      </div>
    </section>

    <section class="scene">
      <div class="panel-head">
        <span>简易 3D 点云 / 栅格地图</span>
        <span>
          <button id="resetViewBtn" class="small">重置视角</button>
          <span id="sceneStats" class="badge">--</span>
        </span>
      </div>
      <div class="scene-wrap">
        <canvas id="gl"></canvas>
        <div class="legend">
          <span><i class="dot" style="background:#5eead4"></i>点云</span>
          <span><i class="dot" style="background:#fdb022"></i>占用栅格</span>
          <span><i class="dot" style="background:#f97066"></i>膨胀栅格</span>
          <span><i class="dot" style="background:#32d583"></i>任务点</span>
          <span><i class="dot" style="background:#fef08a"></i>无人机</span>
          <span>坐标轴: 红 X / 绿 Y / 蓝 Z</span>
          <span>拖拽旋转，滚轮/双指缩放</span>
        </div>
      </div>
    </section>

    <section class="config-section">
      <div class="panel-head">本次任务参数</div>
      <div class="panel-body forms">
        <div class="param-compact">
          <div class="settings-title">可修改参数</div>
          <div class="mode-compact">
            <label><input type="radio" name="frame_center_mode" value="auto_detect" checked>先识别，超时后用预期值</label>
            <label><input type="radio" name="frame_center_mode" value="expected_direct">本次直接使用预期值</label>
          </div>
          <div class="param-table">
            <div class="param-row"><div class="param-name">起飞对中</div><div class="row3" data-group="initial_wait"></div></div>
            <div class="param-row"><div class="param-name">门框中心</div><div class="row3" data-group="expected_frame"></div></div>
            <div class="param-row"><div class="param-name">二维码点</div><div class="row3" data-group="qr_goal"></div></div>
            <div class="param-row"><div class="param-name">打击区</div><div class="row3" data-group="attack_zone"></div></div>
            <div class="param-row"><div class="param-name">击打高度</div><div class="row3" data-group="attack_height"></div></div>
          </div>
        </div>
        <div class="settings-grid">
          <div class="preview mono">
            <div class="settings-title">当前表单设置</div>
            <div id="paramPreview">等待默认参数...</div>
          </div>
          <div class="preview mono">
            <div class="settings-title">任务实际使用参数</div>
            <div id="activeParamPreview">还没有启动任务</div>
          </div>
        </div>
        <div class="group">
          <h3>任务控制</h3>
          <div class="control-row">
            <button id="stopBtn" class="warn" disabled>停止主程序</button>
            <button id="restartBtn" class="primary" disabled>停止并重新开始</button>
          </div>
        </div>
        <div class="group">
          <h3>板载电脑电源</h3>
          <div class="mode-row">
            <label id="poweroffState">输入 PIN 后可以关机或重启。</label>
            <div class="control-row">
              <button id="shutdownBtn" class="danger" disabled>关闭板载电脑</button>
              <button id="rebootBtn" class="warn" disabled>重启板载电脑</button>
            </div>
          </div>
        </div>
        <div class="group log-group">
          <h3>最近任务日志</h3>
          <div class="logbox"><pre id="logs">--</pre></div>
        </div>
      </div>
    </section>
  </main>
</div>

<script>
const PARAMS = [
  ['initial_wait_x', 'x'], ['initial_wait_y', 'y'], ['initial_wait_z', 'z'],
  ['expected_frame_x', 'x'], ['expected_frame_y', 'y'], ['expected_frame_z', 'z'],
  ['qr_goal_x', 'x'], ['qr_goal_y', 'y'], ['qr_goal_z', 'z'],
  ['attack_zone_x', 'x'], ['attack_zone_y', 'y'], ['attack_zone_z', 'z'],
  ['attack_height', 'z']
];
let token = '';
let defaults = {};
let latestScene = null;
let latestStatus = null;
let poweroffEnabled = false;
let rebootEnabled = false;

function fmt(n, d=2) {
  if (n === null || n === undefined || Number.isNaN(Number(n))) return '--';
  return Number(n).toFixed(d);
}
async function api(path, opts={}) {
  opts.headers = opts.headers || {};
  if (token) opts.headers['X-Auth-Token'] = token;
  if (opts.body && !opts.headers['Content-Type']) opts.headers['Content-Type'] = 'application/json';
  const res = await fetch(path, opts);
  const txt = await res.text();
  let data = {};
  try { data = txt ? JSON.parse(txt) : {}; } catch (e) { data = {error: txt}; }
  if (!res.ok) throw new Error(data.error || ('HTTP ' + res.status));
  return data;
}
function setBadge(el, text, cls) {
  el.textContent = text;
  el.className = 'badge ' + (cls || '');
}
function makeInput(name, label) {
  return `<div class="field"><label>${label}</label><input id="${name}" type="number" step="0.01" class="param"></div>`;
}
function buildForm() {
  document.querySelector('[data-group="initial_wait"]').innerHTML =
    makeInput('initial_wait_x','X') + makeInput('initial_wait_y','Y') + makeInput('initial_wait_z','Z');
  document.querySelector('[data-group="expected_frame"]').innerHTML =
    makeInput('expected_frame_x','X') + makeInput('expected_frame_y','Y') + makeInput('expected_frame_z','Z');
  document.querySelector('[data-group="qr_goal"]').innerHTML =
    makeInput('qr_goal_x','X') + makeInput('qr_goal_y','Y') + makeInput('qr_goal_z','Z');
  document.querySelector('[data-group="attack_zone"]').innerHTML =
    makeInput('attack_zone_x','X') + makeInput('attack_zone_y','Y') + makeInput('attack_zone_z','Z');
  document.querySelector('[data-group="attack_height"]').innerHTML =
    makeInput('attack_height','Z 高度');
  document.querySelectorAll('.param, input[name="frame_center_mode"]').forEach(el => {
    el.addEventListener('input', updatePreview);
    el.addEventListener('change', updatePreview);
  });
}
function formValues() {
  const params = {};
  PARAMS.forEach(([name]) => params[name] = Number(document.getElementById(name).value));
  params.frame_center_mode = document.querySelector('input[name="frame_center_mode"]:checked').value;
  return params;
}
function taskPointsFromForm() {
  const p = formValues();
  return {
    initial_wait: [p.initial_wait_x, p.initial_wait_y, p.initial_wait_z],
    expected_frame: [p.expected_frame_x, p.expected_frame_y, p.expected_frame_z],
    qr_goal: [p.qr_goal_x, p.qr_goal_y, p.qr_goal_z],
    attack_zone: [p.attack_zone_x, p.attack_zone_y, p.attack_zone_z]
  };
}
function updatePreview() {
  const p = formValues();
  const modeText = p.frame_center_mode === 'expected_direct' ? '直接预期' : '先识别';
  document.getElementById('paramPreview').textContent =
    `模式: ${modeText}\n` +
    `对中 (${fmt(p.initial_wait_x)},${fmt(p.initial_wait_y)},${fmt(p.initial_wait_z)})  门框 (${fmt(p.expected_frame_x)},${fmt(p.expected_frame_y)},${fmt(p.expected_frame_z)})\n` +
    `QR (${fmt(p.qr_goal_x)},${fmt(p.qr_goal_y)},${fmt(p.qr_goal_z)})  打击区 (${fmt(p.attack_zone_x)},${fmt(p.attack_zone_y)},${fmt(p.attack_zone_z)})  高度 ${fmt(p.attack_height)}`;
}
function paramsPreviewText(p, emptyText) {
  if (!p || !Object.keys(p).length) return emptyText !== undefined ? emptyText : '--';
  const mode = p.frame_center_mode === 'expected_direct' ? '直接预期' : (p.frame_center_mode === 'auto_detect' ? '先识别' : '--');
  return `模式: ${mode}\n` +
    `对中 (${fmt(p.initial_wait_x)},${fmt(p.initial_wait_y)},${fmt(p.initial_wait_z)})  门框 (${fmt(p.expected_frame_x)},${fmt(p.expected_frame_y)},${fmt(p.expected_frame_z)})\n` +
    `QR (${fmt(p.qr_goal_x)},${fmt(p.qr_goal_y)},${fmt(p.qr_goal_z)})  打击区 (${fmt(p.attack_zone_x)},${fmt(p.attack_zone_y)},${fmt(p.attack_zone_z)})  高度 ${fmt(p.attack_height)}`;
}
async function loadDefaults() {
  const data = await api('/api/defaults');
  defaults = data.defaults || {};
  buildForm();
  PARAMS.forEach(([name]) => {
    const el = document.getElementById(name);
    el.value = defaults[name] !== undefined ? defaults[name] : 0;
  });
  const mode = defaults.frame_center_mode || 'auto_detect';
  const radio = document.querySelector(`input[name="frame_center_mode"][value="${mode}"]`);
  if (radio) radio.checked = true;
  updatePreview();
}
async function login() {
  try {
    const pin = document.getElementById('pin').value;
    const data = await api('/api/auth', {method:'POST', body: JSON.stringify({pin})});
    token = data.token;
    setBadge(document.getElementById('authBadge'), '已解锁', 'ok');
    document.getElementById('startBtn').disabled = false;
    document.getElementById('restartBtn').disabled = false;
    document.getElementById('shutdownBtn').disabled = !poweroffEnabled;
    document.getElementById('rebootBtn').disabled = !rebootEnabled;
  } catch (e) {
    setBadge(document.getElementById('authBadge'), 'PIN 错误', 'bad');
    alert(e.message);
  }
}
async function startMission() {
  try {
    if (!confirm('确认开始本次飞行任务？')) return;
    const data = await api('/api/start', {method:'POST', body: JSON.stringify({params: formValues()})});
    setBadge(document.getElementById('runBadge'), '任务已启动', 'ok');
    document.getElementById('logs').textContent = data.command || '';
    document.getElementById('activeParamPreview').textContent = paramsPreviewText(data.effective_params, '--');
  } catch (e) {
    alert(e.message);
  }
}
async function shutdownHost() {
  try {
    if (!poweroffEnabled) return;
    const body = {confirm:'PIN_ONLY'};
    if (!token) {
      const pin = prompt('输入 PIN 后直接关闭板载电脑');
      if (!pin) return;
      body.pin = pin;
    }
    const data = await api('/api/shutdown', {method:'POST', body: JSON.stringify(body)});
    alert(data.message || '关机命令已发送');
  } catch (e) {
    alert(e.message);
  }
}
async function rebootHost() {
  try {
    if (!rebootEnabled) return;
    const body = {confirm:'PIN_ONLY'};
    if (!token) {
      const pin = prompt('输入 PIN 后直接重启板载电脑');
      if (!pin) return;
      body.pin = pin;
    }
    const data = await api('/api/reboot', {method:'POST', body: JSON.stringify(body)});
    alert(data.message || '重启命令已发送');
  } catch (e) {
    alert(e.message);
  }
}
async function stopMission() {
  try {
    if (!confirm('确认停止正在运行的主程序？')) return;
    const data = await api('/api/stop', {method:'POST', body: JSON.stringify({})});
    document.getElementById('logs').textContent = (data.messages || []).join('\n') || '已发送停止命令';
    await refreshStatus();
  } catch (e) {
    alert(e.message);
  }
}
async function restartMission() {
  try {
    if (!confirm('确认停止当前主程序并重新开始本次任务？')) return;
    const data = await api('/api/restart', {method:'POST', body: JSON.stringify({params: formValues()})});
    setBadge(document.getElementById('runBadge'), '任务已重启', 'ok');
    document.getElementById('logs').textContent = (data.stop?.messages || []).join('\n') + '\n' + (data.start?.command || '');
    if (data.start?.effective_params) {
      document.getElementById('activeParamPreview').textContent = paramsPreviewText(data.start.effective_params, '--');
    }
  } catch (e) {
    alert(e.message);
  }
}
function pointText(p, d=2) {
  if (!p || p.x === null || p.x === undefined) return '--';
  return `(${fmt(p.x, d)}, ${fmt(p.y, d)}, ${fmt(p.z, d)})`;
}
function updateStatusUI(data) {
  latestStatus = data;
  const mission = data.mission || {};
  const system = data.system || {};
  const uav = data.uav || {};
  const ego = uav.ego_bridge || {};
  const odom = uav.odom || {};
  const mav = uav.mavros || {};
  const per = data.perception || {};
  setBadge(document.getElementById('runBadge'),
           mission.running ? '任务运行中' : (mission.external_running ? '外部任务运行' : '任务未运行'),
           mission.running || mission.external_running ? 'ok' : '');
  const anyMissionRunning = !!(mission.running || mission.external_running);
  document.getElementById('stopBtn').disabled = !anyMissionRunning;
  document.getElementById('restartBtn').disabled = !token;
  document.getElementById('taskStep').textContent = mission.current_step || '--';
  document.getElementById('missionSource').textContent =
    mission.running ? (mission.started_by_web ? '网页启动' : '网页进程') : (mission.external_running ? '手动/外部启动' : '未运行');
  document.getElementById('missionPid').textContent = mission.pid === null || mission.pid === undefined ? '--' : String(mission.pid);
  document.getElementById('lastExit').textContent = mission.last_exit_code === null || mission.last_exit_code === undefined ? '--' : String(mission.last_exit_code);
  document.getElementById('flightState').textContent = ego.flight_state || '--';
  document.getElementById('controlMode').textContent = ego.control_mode === 1 ? '1 OVERRIDE' : (ego.control_mode === 0 ? '0 EGO' : '--');
  document.getElementById('reachStatus').textContent = ego.reach_status === undefined ? '--' : String(ego.reach_status);
  document.getElementById('mavros').textContent = `${mav.connected ? 'connected' : 'not connected'} / ${mav.armed ? 'armed' : 'disarmed'} / ${mav.mode || '--'}`;
  document.getElementById('odomXyz').textContent = pointText(odom, 3);
  document.getElementById('odomYaw').textContent = fmt(odom.yaw, 3);
  document.getElementById('frameStatus').textContent = per.frame_status || '--';
  document.getElementById('frameCenter').textContent = pointText(per.frame_center);
  document.getElementById('qrIds').textContent = per.qr_ids && per.qr_ids.length ? per.qr_ids.join(', ') : '--';
  document.getElementById('balloonWorld').textContent = pointText(per.balloon_world);
  document.getElementById('lastCommand').textContent = mission.last_command || '--';
  document.getElementById('ageBadge').textContent = new Date().toLocaleTimeString();
  document.getElementById('logs').textContent = (mission.log_tail || []).join('\n') || '--';
  poweroffEnabled = !!system.poweroff_enabled;
  rebootEnabled = !!system.reboot_enabled;
  document.getElementById('shutdownBtn').disabled = !poweroffEnabled;
  document.getElementById('rebootBtn').disabled = !rebootEnabled;
  document.getElementById('poweroffState').textContent =
    (system.poweroff_enabled || system.reboot_enabled)
      ? (token ? '已解锁，点击电源按钮会立即执行。' : '点击电源按钮输入一次 PIN 后直接执行。')
      : '后端未允许电源控制。';
  const mp = mission.params || {};
  document.getElementById('activeParamPreview').textContent =
    paramsPreviewText(mp.active, '') ||
    paramsPreviewText(mp.last_effective, '还没有启动任务');
}
async function refreshStatus() {
  try { updateStatusUI(await api('/api/status')); } catch (e) { console.log(e); }
}
async function refreshScene() {
  try {
    latestScene = await api('/api/scene');
    autoFitScene(false);
    drawScene();
  } catch (e) { console.log(e); }
}

let gl, program, bufPos, bufCol, linePos, lineCol;
let yaw = -0.65, pitch = 0.55, dist = 8.0, pan = [2.2, -0.5, 0.7];
let dragging = false, lastMouse = [0,0];
let touches = new Map(), pinchLast = 0;
let autoFitDone = false;
function initGL() {
  const canvas = document.getElementById('gl');
  gl = canvas.getContext('webgl');
  if (!gl) {
    canvas.outerHTML = '<div style="padding:20px;color:#f97066">此浏览器不支持 WebGL</div>';
    return;
  }
  const vs = `attribute vec3 p; attribute vec3 c; uniform mat4 mvp; uniform float pointSize; varying vec3 v; void main(){ gl_Position=mvp*vec4(p,1.0); gl_PointSize=pointSize; v=c; }`;
  const fs = `precision mediump float; varying vec3 v; void main(){ gl_FragColor=vec4(v,1.0); }`;
  function shader(type, src) {
    const s = gl.createShader(type); gl.shaderSource(s, src); gl.compileShader(s); return s;
  }
  program = gl.createProgram();
  gl.attachShader(program, shader(gl.VERTEX_SHADER, vs));
  gl.attachShader(program, shader(gl.FRAGMENT_SHADER, fs));
  gl.linkProgram(program);
  bufPos = gl.createBuffer(); bufCol = gl.createBuffer();
  linePos = gl.createBuffer(); lineCol = gl.createBuffer();
  canvas.addEventListener('pointerdown', e => {
    touches.set(e.pointerId, [e.clientX, e.clientY]);
    dragging = touches.size === 1;
    lastMouse = [e.clientX, e.clientY];
    canvas.setPointerCapture(e.pointerId);
    e.preventDefault();
  });
  canvas.addEventListener('pointerup', e => {
    touches.delete(e.pointerId);
    dragging = touches.size === 1;
    pinchLast = 0;
    try { canvas.releasePointerCapture(e.pointerId); } catch (_) {}
  });
  canvas.addEventListener('pointercancel', e => {
    touches.delete(e.pointerId);
    dragging = false;
    pinchLast = 0;
  });
  canvas.addEventListener('pointermove', e => {
    if (!touches.has(e.pointerId)) return;
    touches.set(e.pointerId, [e.clientX, e.clientY]);
    if (touches.size >= 2) {
      const pts = Array.from(touches.values());
      const d = Math.hypot(pts[0][0] - pts[1][0], pts[0][1] - pts[1][1]);
      if (pinchLast > 0) {
        dist = Math.max(2.0, Math.min(25.0, dist * (pinchLast / Math.max(1.0, d))));
      }
      pinchLast = d;
    } else if (dragging) {
      const dx = e.clientX - lastMouse[0], dy = e.clientY - lastMouse[1];
      lastMouse = [e.clientX, e.clientY];
      yaw += dx * 0.006;
      pitch = Math.max(-1.25, Math.min(1.25, pitch + dy * 0.006));
    }
    drawScene();
    e.preventDefault();
  });
  canvas.addEventListener('wheel', e => {
    e.preventDefault();
    dist = Math.max(2.0, Math.min(25.0, dist * (e.deltaY > 0 ? 1.08 : 0.92)));
    drawScene();
  }, {passive:false});
  window.addEventListener('resize', drawScene);
}
function resetView() {
  autoFitDone = false;
  if (!autoFitScene(true)) {
    yaw = -0.65;
    pitch = 0.55;
    dist = 8.0;
    pan = [2.2, -0.5, 0.7];
  }
  drawScene();
}
function matMul(a,b) {
  const r = new Float32Array(16);
  for (let col=0; col<4; col++) for (let row=0; row<4; row++) {
    r[col*4+row] =
      a[0*4+row] * b[col*4+0] +
      a[1*4+row] * b[col*4+1] +
      a[2*4+row] * b[col*4+2] +
      a[3*4+row] * b[col*4+3];
  }
  return r;
}
function perspective(fovy, aspect, near, far) {
  const f = 1/Math.tan(fovy/2), nf = 1/(near-far);
  return new Float32Array([f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)*nf,-1, 0,0,2*far*near*nf,0]);
}
function lookAt(eye, center, up) {
  let zx=eye[0]-center[0], zy=eye[1]-center[1], zz=eye[2]-center[2];
  let zn=1/Math.hypot(zx,zy,zz); zx*=zn; zy*=zn; zz*=zn;
  let xx=up[1]*zz-up[2]*zy, xy=up[2]*zx-up[0]*zz, xz=up[0]*zy-up[1]*zx;
  let xn=1/Math.hypot(xx,xy,xz); xx*=xn; xy*=xn; xz*=xn;
  let yx=zy*xz-zz*xy, yy=zz*xx-zx*xz, yz=zx*xy-zy*xx;
  return new Float32Array([xx,yx,zx,0, xy,yy,zy,0, xz,yz,zz,0,
    -(xx*eye[0]+xy*eye[1]+xz*eye[2]), -(yx*eye[0]+yy*eye[1]+yz*eye[2]), -(zx*eye[0]+zy*eye[1]+zz*eye[2]), 1]);
}
function addPoint(arrP, arrC, p, c) {
  if (!p || p.length < 3) return;
  arrP.push(p[0], p[1], p[2]); arrC.push(c[0], c[1], c[2]);
}
function sceneFitPoints() {
  const pts = [];
  const sc = latestScene || {};
  ['cloud', 'occupancy', 'occupancy_inflate'].forEach(key => {
    (sc[key] || []).forEach(p => {
      if (p && p.length >= 3 && Number.isFinite(p[0]) && Number.isFinite(p[1]) && Number.isFinite(p[2])) pts.push(p);
    });
  });
  Object.values(taskPointsFromForm()).forEach(p => {
    if (p && p.length >= 3 && p.every(Number.isFinite)) pts.push(p);
  });
  if (sc.drone && Number.isFinite(sc.drone.x) && Number.isFinite(sc.drone.y) && Number.isFinite(sc.drone.z)) {
    pts.push([sc.drone.x, sc.drone.y, sc.drone.z]);
  }
  return pts;
}
function autoFitScene(force=false) {
  const pts = sceneFitPoints();
  if (!pts.length) return false;
  if (autoFitDone && !force) return true;
  let minX=Infinity, minY=Infinity, minZ=Infinity, maxX=-Infinity, maxY=-Infinity, maxZ=-Infinity;
  pts.forEach(p => {
    minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
    minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    minZ = Math.min(minZ, p[2]); maxZ = Math.max(maxZ, p[2]);
  });
  pan = [(minX + maxX) * 0.5, (minY + maxY) * 0.5, (minZ + maxZ) * 0.5];
  const sx = Math.max(0.4, maxX - minX), sy = Math.max(0.4, maxY - minY), sz = Math.max(0.4, maxZ - minZ);
  const radius = Math.sqrt(sx*sx + sy*sy + sz*sz) * 0.5;
  dist = Math.max(3.0, Math.min(35.0, radius * 2.2 + 1.8));
  yaw = -0.65;
  pitch = 0.55;
  autoFitDone = true;
  return true;
}
function drawScene() {
  if (!gl || !latestScene) return;
  const canvas = gl.canvas;
  const ratio = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.floor(canvas.clientWidth * ratio));
  const h = Math.max(1, Math.floor(canvas.clientHeight * ratio));
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  const points = [], colors = [], lines = [], lineColors = [];
  const sc = latestScene;
  const cloudPoints = [], cloudColors = [];
  const occPoints = [], occColors = [];
  const inflatePoints = [], inflateColors = [];
  const taskPoints = [], taskColors = [];
  (sc.cloud || []).forEach(p => addPoint(cloudPoints, cloudColors, p, [0.18,0.92,1.00]));
  (sc.occupancy || []).forEach(p => addPoint(occPoints, occColors, p, [1.00,0.72,0.12]));
  (sc.occupancy_inflate || []).forEach(p => addPoint(inflatePoints, inflateColors, p, [1.00,0.36,0.32]));
  const task = taskPointsFromForm();
  Object.values(task).forEach(p => addPoint(taskPoints, taskColors, p, [0.20,0.95,0.55]));
  if (sc.drone && sc.drone.x !== null) addPoint(taskPoints, taskColors, [sc.drone.x, sc.drone.y, sc.drone.z], [1.0,0.94,0.54]);
  for (let i=-5;i<=8;i++) {
    lines.push(i,-5,0, i,5,0, -2,i,0, 8,i,0);
    for (let k=0;k<4;k++) lineColors.push(0.18,0.22,0.29);
  }
  lines.push(0,0,0, 1,0,0, 0,0,0, 0,1,0, 0,0,0, 0,0,1);
  lineColors.push(1,0.2,0.2, 1,0.2,0.2, 0.2,1,0.2, 0.2,1,0.2, 0.3,0.5,1, 0.3,0.5,1);
  const eye = [pan[0] + dist*Math.cos(pitch)*Math.cos(yaw), pan[1] + dist*Math.cos(pitch)*Math.sin(yaw), pan[2] + dist*Math.sin(pitch)];
  const proj = perspective(0.8, w/h, 0.05, 100);
  const view = lookAt(eye, pan, [0,0,1]);
  const mvp = matMul(proj, view);
  gl.viewport(0,0,w,h);
  gl.clearColor(0.03,0.04,0.06,1); gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT); gl.enable(gl.DEPTH_TEST);
  gl.useProgram(program);
  const locP = gl.getAttribLocation(program, 'p'), locC = gl.getAttribLocation(program, 'c');
  gl.uniformMatrix4fv(gl.getUniformLocation(program, 'mvp'), false, mvp);
  gl.bindBuffer(gl.ARRAY_BUFFER, linePos); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lines), gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(locP); gl.vertexAttribPointer(locP,3,gl.FLOAT,false,0,0);
  gl.bindBuffer(gl.ARRAY_BUFFER, lineCol); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(lineColors), gl.DYNAMIC_DRAW);
  gl.enableVertexAttribArray(locC); gl.vertexAttribPointer(locC,3,gl.FLOAT,false,0,0);
  gl.drawArrays(gl.LINES, 0, lines.length/3);
  function drawPoints(pos, col, size) {
    if (!pos.length) return;
    gl.uniform1f(gl.getUniformLocation(program, 'pointSize'), size * ratio);
    gl.bindBuffer(gl.ARRAY_BUFFER, bufPos); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(pos), gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(locP,3,gl.FLOAT,false,0,0);
    gl.bindBuffer(gl.ARRAY_BUFFER, bufCol); gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(col), gl.DYNAMIC_DRAW);
    gl.vertexAttribPointer(locC,3,gl.FLOAT,false,0,0);
    gl.drawArrays(gl.POINTS, 0, pos.length/3);
  }
  drawPoints(cloudPoints, cloudColors, 1.8);
  drawPoints(occPoints, occColors, 3.0);
  drawPoints(inflatePoints, inflateColors, 3.4);
  drawPoints(taskPoints, taskColors, 3.6);
  document.getElementById('sceneStats').textContent =
    `cloud ${sc.counts?.cloud || 0} / occ ${sc.counts?.occupancy || 0} / inflate ${sc.counts?.occupancy_inflate || 0}`;
}
document.getElementById('loginBtn').addEventListener('click', login);
document.getElementById('startBtn').addEventListener('click', startMission);
document.getElementById('stopBtn').addEventListener('click', stopMission);
document.getElementById('restartBtn').addEventListener('click', restartMission);
document.getElementById('shutdownBtn').addEventListener('click', shutdownHost);
document.getElementById('rebootBtn').addEventListener('click', rebootHost);
document.getElementById('resetViewBtn').addEventListener('click', resetView);
document.getElementById('pin').addEventListener('keydown', e => { if (e.key === 'Enter') login(); });
buildForm();
initGL();
loadDefaults().catch(e => alert(e.message));
setInterval(refreshStatus, 300);
setInterval(refreshScene, 500);
refreshStatus();
refreshScene();
</script>
</body>
</html>
"""


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_dict(x=None, y=None, z=None, stamp=None):
    age = None
    if stamp is not None:
        age = max(0.0, time.time() - stamp)
    return {"x": x, "y": y, "z": z, "age": age}


class CraicWebControl:
    def __init__(self):
        pnh = rospy.get_param
        self.host = pnh("~host", "0.0.0.0")
        self.port = int(pnh("~port", 8088))
        self.pin_code = str(pnh("~pin_code", "1234"))
        self.mission_package = pnh("~mission_package", "ego_api")
        self.mission_launch = pnh("~mission_launch", "craic_competition_demo.launch")
        self.workspace_dir = pnh("~workspace_dir", "/home/orangepi/catkin_ws")
        self.enable_poweroff = bool(pnh("~enable_poweroff", False))
        self.poweroff_command = pnh("~poweroff_command", "systemctl poweroff")
        self.enable_reboot = bool(pnh("~enable_reboot", False))
        self.reboot_command = pnh("~reboot_command", "systemctl reboot")
        self.max_cloud_points = int(pnh("~max_cloud_points", 800))
        self.max_map_points = int(pnh("~max_map_points", 800))
        self.cloud_period = float(pnh("~cloud_sample_period", 0.2))
        self.map_period = float(pnh("~map_sample_period", 0.5))
        self.cloud_accumulate_frames = max(1, int(pnh("~cloud_accumulate_frames", 1)))
        self.map_accumulate_frames = max(1, int(pnh("~map_accumulate_frames", 1)))
        self.raw_cloud_topic = pnh("~raw_cloud_topic", "/cloud_registered")
        self.occ_topic = pnh("~occupancy_topic", "/drone_0_ego_planner_node/grid_map/occupancy")
        self.inflate_topic = pnh("~occupancy_inflate_topic", "/drone_0_ego_planner_node/grid_map/occupancy_inflate")

        self.lock = threading.RLock()
        self.tokens = set()
        self.proc = None
        self.started_by_web = False
        self.current_step = ""
        self.log_tail = []
        self.last_command = ""
        self.last_exit_code = None
        self.last_effective_params = {}
        self.last_overrides = {}

        self.odom = point_dict()
        self.odom_yaw = None
        self.mavros = {"connected": False, "armed": False, "mode": ""}
        self.ego_bridge = {"flight_state": "", "control_mode": None, "reach_status": None}
        self.frame_status = ""
        self.frame_center = point_dict()
        self.qr_ids = []
        self.balloon_world = point_dict()
        self.scene = {"cloud": [], "occupancy": [], "occupancy_inflate": []}
        self.scene_buffers = {
            "cloud": deque(maxlen=self.cloud_accumulate_frames),
            "occupancy": deque(maxlen=self.map_accumulate_frames),
            "occupancy_inflate": deque(maxlen=self.map_accumulate_frames),
        }
        self.last_cloud_sample = {"cloud": 0.0, "occupancy": 0.0, "occupancy_inflate": 0.0}

        self.defaults = self._load_launch_defaults()
        self._subscribe()

    def _subscribe(self):
        rospy.Subscriber("/mavros/local_position/odom", Odometry, self._odom_cb, queue_size=1)
        rospy.Subscriber("/mavros/state", State, self._mavros_cb, queue_size=1)
        rospy.Subscriber("/ego_bridge/flight_state", String, self._flight_state_cb, queue_size=1)
        rospy.Subscriber("/ego_bridge/control_mode", UInt8, self._control_mode_cb, queue_size=1)
        rospy.Subscriber("/ego_bridge/reach_status", UInt8, self._reach_status_cb, queue_size=1)
        rospy.Subscriber("/craic/frame_status", String, self._frame_status_cb, queue_size=1)
        rospy.Subscriber("/craic/frame_center", PointStamped, self._frame_center_cb, queue_size=1)
        rospy.Subscriber("/usb_camera_vision/aruco_ids", Int32MultiArray, self._qr_cb, queue_size=1)
        rospy.Subscriber("/balloon/world_point", PointStamped, self._balloon_cb, queue_size=1)
        rospy.Subscriber(self.raw_cloud_topic, PointCloud2, lambda m: self._cloud_cb(m, "cloud", self.max_cloud_points, self.cloud_period), queue_size=1)
        rospy.Subscriber(self.occ_topic, PointCloud2, lambda m: self._cloud_cb(m, "occupancy", self.max_map_points, self.map_period), queue_size=1)
        rospy.Subscriber(self.inflate_topic, PointCloud2, lambda m: self._cloud_cb(m, "occupancy_inflate", self.max_map_points, self.map_period), queue_size=1)

    def _load_launch_defaults(self):
        defaults = {}
        try:
            package_path = subprocess.check_output(["rospack", "find", self.mission_package], text=True).strip()
            launch_path = os.path.join(package_path, "launch", self.mission_launch)
            root = ET.parse(launch_path).getroot()
            for arg in root.findall("arg"):
                name = arg.attrib.get("name")
                if name in CONTROL_PARAM_NAMES:
                    defaults[name] = arg.attrib.get("default", arg.attrib.get("value", ""))
        except Exception as exc:
            rospy.logwarn("[craic_web_control] failed to parse launch defaults: %s", exc)
        for name in PARAM_SPECS:
            defaults.setdefault(name, "0.0")
        defaults.setdefault("frame_center_mode", "auto_detect")
        return defaults

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        with self.lock:
            self.odom = point_dict(round(p.x, 3), round(p.y, 3), round(p.z, 3), time.time())
            self.odom_yaw = round(yaw, 4)

    def _mavros_cb(self, msg):
        with self.lock:
            self.mavros = {"connected": bool(msg.connected), "armed": bool(msg.armed), "mode": msg.mode}

    def _flight_state_cb(self, msg):
        with self.lock:
            self.ego_bridge["flight_state"] = msg.data

    def _control_mode_cb(self, msg):
        with self.lock:
            self.ego_bridge["control_mode"] = int(msg.data)

    def _reach_status_cb(self, msg):
        with self.lock:
            self.ego_bridge["reach_status"] = int(msg.data)

    def _frame_status_cb(self, msg):
        with self.lock:
            self.frame_status = msg.data

    def _frame_center_cb(self, msg):
        p = msg.point
        with self.lock:
            self.frame_center = point_dict(round(p.x, 3), round(p.y, 3), round(p.z, 3), time.time())

    def _qr_cb(self, msg):
        with self.lock:
            self.qr_ids = [int(v) for v in msg.data]

    def _balloon_cb(self, msg):
        p = msg.point
        with self.lock:
            self.balloon_world = point_dict(round(p.x, 3), round(p.y, 3), round(p.z, 3), time.time())

    def _cloud_cb(self, msg, key, limit, period):
        now = time.time()
        if now - self.last_cloud_sample.get(key, 0.0) < period:
            return
        self.last_cloud_sample[key] = now
        total = max(1, int(msg.width) * int(msg.height))
        frame_count = self.cloud_accumulate_frames if key == "cloud" else self.map_accumulate_frames
        per_frame_limit = max(1, int(math.ceil(float(limit) / max(1, frame_count))))
        stride = max(1, total // max(1, per_frame_limit))
        pts = []
        try:
            for idx, p in enumerate(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)):
                if idx % stride != 0:
                    continue
                pts.append([round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)])
                if len(pts) >= per_frame_limit:
                    break
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "[craic_web_control] point cloud sample failed: %s", exc)
            return
        with self.lock:
            self.scene_buffers[key].append(pts)
            fused = []
            for frame in self.scene_buffers[key]:
                fused.extend(frame)
            if len(fused) > limit:
                fused = fused[-limit:]
            self.scene[key] = fused

    def authenticate(self, pin):
        if str(pin) != self.pin_code:
            return None
        token = uuid.uuid4().hex
        with self.lock:
            self.tokens.add(token)
        return token

    def check_token(self, token):
        with self.lock:
            return bool(token) and token in self.tokens

    def mission_running(self):
        if self.proc is not None and self.proc.poll() is None:
            return True
        return False

    def external_mission_running(self):
        if rosnode is None:
            return False
        try:
            names = rosnode.get_node_names()
            return "/craic_competition_demo" in names
        except Exception:
            return False

    def _current_mission_params(self):
        params = {}
        for name in CONTROL_PARAM_NAMES:
            key = "/craic_competition_demo/%s" % name
            if rospy.has_param(key):
                value = rospy.get_param(key)
                if isinstance(value, float):
                    params[name] = "%.6g" % value
                else:
                    params[name] = str(value)
        return params

    def status(self):
        with self.lock:
            running = self.mission_running()
            external = self.external_mission_running() and not running
            active_params = self._current_mission_params() if (running or external) else {}
            return {
                "mission": {
                    "running": running,
                    "external_running": external,
                    "started_by_web": self.started_by_web,
                    "pid": self.proc.pid if self.proc is not None and running else None,
                    "current_step": self.current_step,
                    "last_exit_code": self.last_exit_code,
                    "last_command": self.last_command,
                    "log_tail": list(self.log_tail[-300:]),
                    "params": {
                        "active": active_params,
                        "last_effective": dict(self.last_effective_params),
                        "last_overrides": dict(self.last_overrides),
                    },
                },
                "system": {
                    "poweroff_enabled": self.enable_poweroff,
                    "reboot_enabled": self.enable_reboot,
                },
                "uav": {
                    "odom": dict(self.odom, yaw=self.odom_yaw),
                    "mavros": dict(self.mavros),
                    "ego_bridge": dict(self.ego_bridge),
                },
                "perception": {
                    "frame_status": self.frame_status,
                    "frame_center": dict(self.frame_center),
                    "qr_ids": list(self.qr_ids),
                    "balloon_world": dict(self.balloon_world),
                },
            }

    def scene_status(self):
        with self.lock:
            scene = {k: list(v) for k, v in self.scene.items()}
            drone = dict(self.odom, yaw=self.odom_yaw)
            counts = {k: len(v) for k, v in scene.items()}
            return dict(scene, drone=drone, counts=counts, defaults=dict(self.defaults))

    def validate_params(self, params):
        clean = {}
        effective = {}
        for name, (min_value, max_value) in PARAM_SPECS.items():
            raw_value = params[name] if name in params else self.defaults.get(name, "0.0")
            try:
                value = float(raw_value)
            except Exception:
                raise ValueError("%s must be a number" % name)
            if not math.isfinite(value):
                raise ValueError("%s must be finite" % name)
            if value < min_value or value > max_value:
                raise ValueError("%s out of range %.2f..%.2f" % (name, min_value, max_value))
            default = self.defaults.get(name, "")
            try:
                default_value = float(default)
            except Exception:
                default_value = None
            effective[name] = "%.6g" % value
            if default_value is None or abs(value - default_value) > 1e-9:
                clean[name] = ("%.6g" % value)
        mode = params.get("frame_center_mode", self.defaults.get("frame_center_mode", "auto_detect"))
        if mode not in MODE_VALUES:
            raise ValueError("invalid frame_center_mode")
        effective["frame_center_mode"] = mode
        if mode != self.defaults.get("frame_center_mode", "auto_detect"):
            clean["frame_center_mode"] = mode
        return clean, effective

    def start_mission(self, params):
        if self.mission_running() or self.external_mission_running():
            raise RuntimeError("mission is already running")
        clean, effective = self.validate_params(params)
        cmd = ["roslaunch", self.mission_package, self.mission_launch]
        for name in sorted(clean.keys()):
            cmd.append("%s:=%s" % (name, clean[name]))
        env = os.environ.copy()
        env.setdefault("PYTHONUNBUFFERED", "1")
        display_cmd = " ".join(shlex.quote(v) for v in cmd)
        proc = subprocess.Popen(
            cmd,
            cwd=self.workspace_dir,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1,
            preexec_fn=os.setsid,
            env=env,
        )
        with self.lock:
            self.proc = proc
            self.started_by_web = True
            self.current_step = "LAUNCHING"
            self.last_exit_code = None
            self.last_command = display_cmd
            self.last_effective_params = dict(effective)
            self.last_overrides = dict(clean)
            self.log_tail = ["$ " + display_cmd]
        threading.Thread(target=self._read_process_output, args=(proc,), daemon=True).start()
        return {"pid": proc.pid, "command": display_cmd, "overrides": clean, "effective_params": effective}

    def stop_mission(self):
        messages = []
        proc = None
        with self.lock:
            proc = self.proc
        if proc is not None and proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                messages.append("sent SIGINT to web roslaunch pid=%d" % proc.pid)
                try:
                    proc.wait(timeout=6.0)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    messages.append("sent SIGTERM to web roslaunch pid=%d" % proc.pid)
            except Exception as exc:
                messages.append("failed to stop web roslaunch: %s" % exc)
        if self.external_mission_running():
            try:
                out = subprocess.check_output(["rosnode", "kill", "/craic_competition_demo"], stderr=subprocess.STDOUT, universal_newlines=True, timeout=5.0)
                messages.append(out.strip() or "rosnode kill /craic_competition_demo")
            except Exception as exc:
                messages.append("failed to rosnode kill /craic_competition_demo: %s" % exc)
        if not messages:
            messages.append("no mission process detected")
        with self.lock:
            self.current_step = "STOPPED"
            self.log_tail.extend("[craic_web_control] " + msg for msg in messages)
            self.log_tail = self.log_tail[-300:]
        return {"messages": messages}

    def restart_mission(self, params):
        stop_result = self.stop_mission()
        deadline = time.time() + 8.0
        while time.time() < deadline and (self.mission_running() or self.external_mission_running()):
            time.sleep(0.2)
        start_result = self.start_mission(params)
        return {"stop": stop_result, "start": start_result}

    def _run_power_command(self, enabled, command, label, confirm):
        if not enabled:
            raise RuntimeError("%s is disabled by launch parameter" % label)
        if str(confirm).strip() not in ("SHUTDOWN", "PIN_ONLY"):
            raise RuntimeError("%s confirmation mismatch" % label)
        cmd = shlex.split(str(command))
        if not cmd:
            raise RuntimeError("empty %s command" % label)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        try:
            out, _ = proc.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            return {"accepted": True, "command": " ".join(shlex.quote(v) for v in cmd), "message": "%s command is running" % label}
        if proc.returncode != 0:
            raise RuntimeError((out or "%s command failed" % label).strip())
        return {"accepted": True, "command": " ".join(shlex.quote(v) for v in cmd), "message": (out or "").strip()}

    def shutdown_host(self, confirm):
        return self._run_power_command(self.enable_poweroff, self.poweroff_command, "poweroff", confirm)

    def reboot_host(self, confirm):
        return self._run_power_command(self.enable_reboot, self.reboot_command, "reboot", confirm)

    def _read_process_output(self, proc):
        try:
            for line in proc.stdout:
                line = line.rstrip()
                self._append_log(line)
        finally:
            code = proc.poll()
            if code is None:
                code = proc.wait()
            with self.lock:
                self.last_exit_code = int(code)
                if self.current_step not in ("MISSION_DONE", "PX4_LAND"):
                    self.current_step = "EXIT_%d" % int(code)
                self.log_tail.append("[craic_web_control] roslaunch exited code=%d" % int(code))
                self.log_tail = self.log_tail[-300:]

    def _append_log(self, line):
        task_match = re.search(r"TASK step=([A-Za-z0-9_]+)", line)
        done_match = re.search(r"MISSION_DONE|PX4_LAND", line)
        with self.lock:
            if task_match:
                self.current_step = task_match.group(1)
            elif done_match:
                self.current_step = "MISSION_DONE" if "MISSION_DONE" in line else "PX4_LAND"
            self.log_tail.append(line)
            self.log_tail = self.log_tail[-300:]


class RequestHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        rospy.loginfo("[craic_web_control] %s", fmt % args)

    @property
    def app(self):
        return self.server.app

    def _read_json(self):
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        return json.loads(raw)

    def _send_json(self, data, status=200):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self):
        body = INDEX_HTML.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _token(self):
        return self.headers.get("X-Auth-Token", "")

    def do_GET(self):
        try:
            if self.path == "/" or self.path.startswith("/?"):
                self._send_html()
            elif self.path == "/api/defaults":
                self._send_json({"defaults": self.app.defaults, "modes": list(MODE_VALUES)})
            elif self.path == "/api/status":
                self._send_json(self.app.status())
            elif self.path == "/api/scene":
                self._send_json(self.app.scene_status())
            else:
                self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, 500)

    def do_POST(self):
        try:
            data = self._read_json()
            if self.path == "/api/auth":
                token = self.app.authenticate(data.get("pin", ""))
                if not token:
                    self._send_json({"error": "invalid PIN"}, 403)
                    return
                self._send_json({"token": token})
            elif self.path == "/api/start":
                if not self.app.check_token(self._token()):
                    self._send_json({"error": "unauthorized"}, 403)
                    return
                result = self.app.start_mission(data.get("params", {}))
                self._send_json(result)
            elif self.path == "/api/stop":
                if not self.app.check_token(self._token()):
                    self._send_json({"error": "unauthorized"}, 403)
                    return
                result = self.app.stop_mission()
                self._send_json(result)
            elif self.path == "/api/restart":
                if not self.app.check_token(self._token()):
                    self._send_json({"error": "unauthorized"}, 403)
                    return
                result = self.app.restart_mission(data.get("params", {}))
                self._send_json(result)
            elif self.path == "/api/shutdown":
                if not self.app.check_token(self._token()) and str(data.get("pin", "")) != self.app.pin_code:
                    self._send_json({"error": "unauthorized"}, 403)
                    return
                result = self.app.shutdown_host(data.get("confirm", ""))
                self._send_json(result)
            elif self.path == "/api/reboot":
                if not self.app.check_token(self._token()) and str(data.get("pin", "")) != self.app.pin_code:
                    self._send_json({"error": "unauthorized"}, 403)
                    return
                result = self.app.reboot_host(data.get("confirm", ""))
                self._send_json(result)
            else:
                self._send_json({"error": "not found"}, 404)
        except Exception as exc:
            self._send_json({"error": str(exc)}, 400)


def main():
    rospy.init_node("craic_web_control", anonymous=False)
    app = CraicWebControl()
    server = ThreadingHTTPServer((app.host, app.port), RequestHandler)
    server.app = app
    server.timeout = 0.5
    access_ips = []
    try:
        access_ips = subprocess.check_output(["hostname", "-I"], text=True).split()
    except Exception:
        pass
    if not access_ips:
        try:
            access_ips = [socket.gethostbyname(socket.gethostname())]
        except Exception:
            access_ips = []
    access_urls = ["http://%s:%d" % (ip, app.port) for ip in access_ips if ip and not ip.startswith("127.")]
    if not access_urls and app.host != "0.0.0.0":
        access_urls = ["http://%s:%d" % (app.host, app.port)]
    rospy.loginfo("[craic_web_control] listening bind=http://%s:%d", app.host, app.port)
    rospy.loginfo("[craic_web_control] browser access url(s): %s", ", ".join(access_urls) if access_urls else "unknown")
    try:
        while not rospy.is_shutdown():
            server.handle_request()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
