#!/usr/bin/env python3
"""Adaptive KARIS object ID -> MoveIt plan -> RViz review -> explicit execution.

Designed for the existing G1/ROS 2 Humble container and ROS graph.
  - 'list' discovers KARIS objects.
  - Enter OBJECT_ID directly (or 'plan OBJECT_ID') to plan a wrist pre-grasp.
  - ONLY successful, confirmed MoveIt executions with fresh final wrist TF log object, desired wrist, actual wrist and pose errors to CSV + bullseye SVG.
  - 'execute' is the one explicit physical-approval command for THE SAME stored trajectory (no second typed confirmation).
  - 'discard' forgets the plan without writing a log; 'quit' exits.
  - Defaults: 100% MoveIt velocity/acceleration scaling; arm-specific accuracy
    logs and calibrated wrist offsets; one IK candidate; left hand -Y down
    and left execution available via explicit execute.

No G1 controller is started here. No execution is ever requested by 'plan'.
Right wrist +Y points downward; with mirrored Inspire FTP left-hand mounting,
left wrist -Y points downward instead. Review the real palm in RViz first.
The planner searches feasible wrist-backoff, clearance and modest yaw alternatives
when collision-checked IK fails and --max-ik-trials > 1. The arm-specific
preferred backoff is an empirical wrist offset, not a palm/fingertip TCP. The signed pelvis-Y
side offset is fixed for each planning run (not automatically searched). Automatic IK
search does NOT establish that the hand is aligned for a grasp. The table and
objects may not be in the MoveIt collision scene. No Cartesian descent/grasp.
"""

import argparse
import csv
import json
import uuid
from datetime import datetime, timezone
from pathlib import Path
from xml.sax.saxutils import escape
import math
import sys
import time
from dataclasses import dataclass
from statistics import median, mean

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.srv import GetParameters
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, DisplayTrajectory, JointConstraint
from moveit_msgs.srv import GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time
from visualization_msgs.msg import Marker

TOPIC = '/karis/object_poses_base'
ARM_PARTS = (
    'shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow',
    'wrist_roll', 'wrist_pitch', 'wrist_yaw'
)
# Inspire FTP palms are mounted with MIRRORED wrist-frame normals:
# the nominal palmar direction is +Y on the right, -Y on the left.
# Right wrist Rx(-90deg): +Y points down in pelvis.
# Left wrist Rx(+90deg): -Y points down in pelvis.
# Confirm the actual robot's hand attachment against RViz/its URDF;
# the hardware may use a customized robot description.
RIGHT_WRIST_Q = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
LEFT_WRIST_Q = (+math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
# Arm-specific, operator-selected wrist offsets (pelvis frame, metres).
# X = object X - wrist_backoff; Y = object Y + side_offset;
# Z = object Z + height. This is a wrist-link pre-grasp, not palm TCP.
# One IK trial is the default so calibration values are not silently changed.
DEFAULT_PICK_OFFSETS_M = {
    'right': {'wrist_backoff': 0.223, 'side_offset': -0.032, 'height': 0.110},
    'left':  {'wrist_backoff': 0.233, 'side_offset': +0.0225, 'height': 0.085},
}


@dataclass
class StoredPlan:
    object_id: str
    xyz: tuple
    approach_xyz: tuple
    made_at: float
    trajectory: object
    trajectory_start: object
    joint_names: tuple
    chosen_backoff: float
    chosen_height: float
    chosen_side_offset: float
    chosen_yaw_deg: float
    object_reference: object
    wrist_quaternion: tuple


def future_result(node, future, timeout, label):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError(f'{label}: timeout after {timeout:.1f}s')
    if future.exception() is not None:
        raise RuntimeError(f'{label}: {future.exception()}')
    value = future.result()
    if value is None:
        raise RuntimeError(f'{label}: empty response')
    return value


# Each CSV ROW represents one *successfully executed and measured* goal.
# Keep V7's --goal-log CLI option as a backwards-compatible alias.
ACCURACY_FIELDS = (
    'execution_id', 'timestamp_utc', 'object_id', 'arm', 'frame_id',
    'object_x_m', 'object_y_m', 'object_z_m',
    'object_qx', 'object_qy', 'object_qz', 'object_qw', 'object_orientation_valid',
    'desired_link', 'desired_x_m', 'desired_y_m', 'desired_z_m',
    'desired_qx', 'desired_qy', 'desired_qz', 'desired_qw',
    'actual_link', 'actual_x_m', 'actual_y_m', 'actual_z_m',
    'actual_qx', 'actual_qy', 'actual_qz', 'actual_qw',
    'error_x_mm', 'error_y_mm', 'error_z_mm', 'error_3d_mm',
    'rotation_error_deg', 'within_15mm', 'tf_stamp_sec', 'tf_stamp_nanosec',
    'source', 'chosen_backoff_m', 'chosen_height_m', 'chosen_yaw_deg',
)


def orientation_error_deg(desired, actual):
    def unit(q):
        if len(q) != 4 or not all(math.isfinite(float(v)) for v in q):
            raise ValueError('Invalid wrist quaternion')
        norm = math.sqrt(sum(float(v)*float(v) for v in q))
        if norm < 1e-9:
            raise ValueError('Zero-norm wrist quaternion')
        return [float(v)/norm for v in q]
    qa, qb = unit(desired), unit(actual)
    dot = min(1.0, abs(sum(a*b for a,b in zip(qa,qb))))
    return math.degrees(2 * math.acos(dot))


def accuracy_row(plan, arm, measured_xyz, measured_q, tf_stamp):
    if not all(math.isfinite(float(v)) for v in (*measured_xyz, *measured_q)):
        raise ValueError('Nonfinite measured wrist TF')
    obj = plan.object_reference
    valid_orientation = bool(getattr(obj, 'has_orientation', False))
    obj_q = None
    if valid_orientation:
        q = obj.pose.orientation
        obj_q = (float(q.x), float(q.y), float(q.z), float(q.w))
        valid_orientation = all(math.isfinite(v) for v in obj_q)
    dx, dy, dz = [(a-b)*1000 for a,b in zip(measured_xyz,plan.approach_xyz)]
    error = math.sqrt(dx*dx + dy*dy + dz*dz)
    row = {k:'' for k in ACCURACY_FIELDS}
    row.update(
        execution_id=uuid.uuid4().hex[:12],
        timestamp_utc=datetime.now(timezone.utc).isoformat(timespec='milliseconds'),
        object_id=plan.object_id, arm=arm, frame_id='pelvis',
        object_orientation_valid=str(valid_orientation).lower(),
        desired_link=f'{arm}_wrist_yaw_link', actual_link=f'{arm}_wrist_yaw_link',
        error_x_mm=f'{dx:+.3f}',error_y_mm=f'{dy:+.3f}',error_z_mm=f'{dz:+.3f}',
        error_3d_mm=f'{error:.3f}',
        rotation_error_deg=f'{orientation_error_deg(plan.wrist_quaternion,measured_q):.3f}',
        within_15mm=str(error <= 15.0).lower(),
        tf_stamp_sec=tf_stamp.sec,tf_stamp_nanosec=tf_stamp.nanosec,
        source='robot_state_publisher TF derived from /joint_states (not independent metrology)',
        chosen_backoff_m=f'{plan.chosen_backoff:.4f}',
        chosen_height_m=f'{plan.chosen_height:.4f}',
        chosen_yaw_deg=f'{plan.chosen_yaw_deg:+.1f}',
    )
    for prefix, xyz in [('object',plan.xyz),('desired',plan.approach_xyz),('actual',measured_xyz)]:
        for axis,val in zip('xyz',xyz):
            row[f'{prefix}_{axis}_m'] = f'{val:+.7f}'
    for prefix,q in [('object',obj_q if valid_orientation else None),
                     ('desired',plan.wrist_quaternion),('actual',measured_q)]:
        if q is not None:
            for axis,val in zip(('qx','qy','qz','qw'),q):
                row[f'{prefix}_{axis}'] = f'{val:+.9f}'
    return row


def bullseye_svg(csv_path, svg_path):
    """Three centered signed-error projections for *executed* goals only."""
    with Path(csv_path).open(newline='',encoding='utf-8') as f:
        samples = list(csv.DictReader(f))
    if not samples:
        return
    for r in samples:
        if not all(k in r for k in ('error_x_mm','error_y_mm','error_z_mm','error_3d_mm')):
            raise ValueError('Not an execution-accuracy CSV')
    samples = samples[-120:]
    coords = [tuple(float(r['error_'+a+'_mm']) for a in 'xyz') for r in samples]
    max_abs = max(abs(v) for xyz in coords for v in xyz)
    limit = max(25, int(math.ceil(max_abs*1.20 / 5))*5)
    W,H = 1150,610
    size=285
    top=151
    panels=[(64,0,1,'X / Y','X','Y'), (430,0,2,'X / Z','X','Z'), (796,1,2,'Y / Z','Y','Z')]
    parts=[f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}">',
           '<rect width="100%" height="100%" fill="#ffffff"/>',
           '<text x="35" y="35" font-family="sans-serif" font-size="24" font-weight="bold" fill="#14233a">G1 wrist execution error · bullseye</text>',
           '<text x="35" y="58" font-family="sans-serif" font-size="14" fill="#536579">Actual wrist TF minus desired MoveIt wrist goal, in pelvis · mm · successful executions only</text>',
           '<text x="35" y="82" font-family="sans-serif" font-size="13" fill="#536579">Center = perfect goal match. Rings = 5, 10, 15 mm; overall 15 mm tolerance applies to the 3D error.</text>']
    for left,ia,ib,title,axis_a,axis_b in panels:
        center_x=left+size/2
        center_y=top+size/2
        scale=(size/2-15)/limit
        parts.append(f'<text x="{left}" y="131" font-family="sans-serif" font-size="18" fill="#14233a">{title} error</text>')
        parts.append(f'<rect x="{left}" y="{top}" width="{size}" height="{size}" fill="#fafcff" stroke="#b1bfcc"/>')
        # Keep meaningful reference rings even when the scale expands for outliers.
        ring_values = [5,10,15]
        for ring in ring_values:
            rad = ring*scale
            parts.append(f'<circle cx="{center_x:.2f}" cy="{center_y:.2f}" r="{rad:.2f}" fill="none" stroke="#8b9eb3" stroke-width="1.2" stroke-dasharray="4 3"/>')
        parts.append(f'<line x1="{left}" y1="{center_y:.2f}" x2="{left+size}" y2="{center_y:.2f}" stroke="#94a7bc"/>')
        parts.append(f'<line x1="{center_x:.2f}" y1="{top}" x2="{center_x:.2f}" y2="{top+size}" stroke="#94a7bc"/>')
        parts.append(f'<circle cx="{center_x:.2f}" cy="{center_y:.2f}" r="2.6" fill="#101827"/>')
        parts.append(f'<text x="{left}" y="{top+size+20}" font-family="sans-serif" font-size="12" fill="#536579">{-limit:+d} mm</text>')
        parts.append(f'<text x="{left+size}" y="{top+size+20}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#536579">{limit:+d} mm</text>')
        parts.append(f'<text x="{center_x:.2f}" y="{top+size+42}" text-anchor="middle" font-family="sans-serif" font-size="13" fill="#14233a">{axis_a} error (mm) →</text>')
        parts.append(f'<text x="{left-8}" y="{top+15}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#536579">+{axis_b}</text>')
        parts.append(f'<text x="{left-8}" y="{top+size-2}" text-anchor="end" font-family="sans-serif" font-size="12" fill="#536579">−{axis_b}</text>')
        for idx,(r,xyz) in enumerate(zip(samples,coords)):
            x=center_x+xyz[ia]*scale
            y=center_y-xyz[ib]*scale
            latest=(idx==len(samples)-1)
            color='#d25736' if latest else '#427ca2'
            radius=6.3 if latest else 3.3
            opacity='1' if latest else '0.48'
            label=escape(f"{r.get('timestamp_utc','')}: {r.get('object_id','')} — ΔX={xyz[0]:+.2f}, ΔY={xyz[1]:+.2f}, ΔZ={xyz[2]:+.2f} mm | 3D={r.get('error_3d_mm','?')} mm")
            parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="{radius}" fill="{color}" stroke="white" stroke-width="{1.1 if latest else 0.5}" opacity="{opacity}"><title>{label}</title></circle>')
    latest=samples[-1]
    e=[float(latest['error_'+a+'_mm']) for a in 'xyz']
    total=float(latest['error_3d_mm'])
    rotation=float(latest['rotation_error_deg'])
    all_dist=[float(r['error_3d_mm']) for r in samples]
    n15=sum(d<=15 for d in all_dist)
    parts.append('<rect x="35" y="518" width="1080" height="72" rx="8" fill="#eef4f8"/>')
    parts.append(f'<text x="53" y="544" font-family="sans-serif" font-size="15" font-weight="bold" fill="#14233a">Latest: ΔX={e[0]:+.1f} · ΔY={e[1]:+.1f} · ΔZ={e[2]:+.1f} mm</text>')
    parts.append(f'<text x="53" y="572" font-family="sans-serif" font-size="14" fill="#14233a">3D error = {total:.1f} mm · orientation error = {rotation:.1f}°</text>')
    parts.append(f'<text x="637" y="544" font-family="sans-serif" font-size="15" fill="#14233a">Executed samples shown: {len(samples)}</text>')
    parts.append(f'<text x="637" y="572" font-family="sans-serif" font-size="14" fill="#14233a">Within 15 mm: {n15}/{len(samples)} · mean 3D error: {mean(all_dist):.1f} mm</text>')
    parts.append('</svg>')
    Path(svg_path).write_text('\n'.join(parts),encoding='utf-8')


def append_execution_accuracy(csv_path, plan, arm, actual_xyz, actual_quat, tf_stamp):
    path=Path(csv_path).expanduser()
    path.parent.mkdir(parents=True,exist_ok=True)
    # Do not corrupt existing V6/V7 files with a different CSV schema.
    if path.exists() and path.stat().st_size:
        with path.open(newline='',encoding='utf-8') as f:
            existing_fields=next(csv.reader(f),[])
        if tuple(existing_fields) != ACCURACY_FIELDS:
            raise ValueError(f'Existing CSV {path} uses a different schema. '
                             'Choose a new --accuracy-log filename, e.g. left_execution_accuracy.csv')
    row=accuracy_row(plan,arm,actual_xyz,actual_quat,tf_stamp)
    with path.open('a',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=ACCURACY_FIELDS)
        if f.tell()==0:
            writer.writeheader()
        writer.writerow(row)
    svg_path=path.with_name(path.stem+'_bullseye.svg')
    html_path=path.with_name(path.stem+'_3d.html')
    try:
        bullseye_svg(path,svg_path)
    except (OSError,ValueError,KeyError) as exc:
        print(f'WARNING: Accuracy CSV recorded, bullseye update failed: {exc}')
    try:
        accuracy_3d_html([path],html_path)
    except (OSError,ValueError,KeyError) as exc:
        print(f'WARNING: Accuracy CSV recorded, 3D HTML update failed: {exc}')
    return path,svg_path,row


def accuracy_3d_html(csv_paths, html_path):
    # Generate a self-contained, interactive 3D error-cloud report. No third-party dependencies.
    observations = []
    for csv_path in csv_paths:
        with Path(csv_path).open(newline='', encoding='utf-8') as file:
            for row in csv.DictReader(file):
                try:
                    e = tuple(float(row['error_' + a + '_mm']) for a in 'xyz')
                    distance = float(row['error_3d_mm'])
                    if not all(math.isfinite(v) for v in (*e, distance)):
                        continue
                    observations.append({
                        'x': e[0], 'y': e[1], 'z': e[2], 'r': distance,
                        'arm': row.get('arm', 'unknown'),
                        'object': row.get('object_id', ''),
                        'time': row.get('timestamp_utc', ''),
                        'id': row.get('execution_id', ''),
                        'rot': row.get('rotation_error_deg', ''),
                    })
                except (KeyError, ValueError, TypeError):
                    continue
    if not observations:
        raise ValueError('Accuracy 3D report needs at least one valid execution error row.')
    observations.sort(key=lambda row: row['time'])
    # Escape markup-sensitive chars even inside the inert JSON script tag.
    packed = json.dumps(observations, separators=(',', ':'), ensure_ascii=True).replace('<','\\u003c').replace('>','\\u003e').replace('&','\\u0026')
    html = r'''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>G1 wrist execution accuracy · interactive 3D</title>
<style>
:root{color-scheme:dark;font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial,sans-serif;background:#0d1725;color:#e7eef8}
*{box-sizing:border-box}body{margin:0;padding:22px}.wrap{max-width:1180px;margin:auto}h1{font-size:clamp(22px,3vw,32px);margin:0 0 5px}
.sub{color:#a8bed3;margin:0 0 20px;font-size:14px;line-height:1.55}.top{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:12px;margin-bottom:16px}
.stat{background:#17283b;border:1px solid #29435a;border-radius:12px;padding:15px}.stat strong{display:block;font-size:26px;font-variant-numeric:tabular-nums}.stat span{font-size:12px;color:#bed0df}
.panel{background:#122337;border:1px solid #29435a;border-radius:16px;overflow:hidden}.controls{display:flex;flex-wrap:wrap;gap:14px;align-items:center;padding:15px;border-bottom:1px solid #29435a}
.controls label{display:flex;gap:10px;align-items:center;font-size:14px}select,button,input{accent-color:#90bdea}select,button{background:#233a52;color:#fff;border:1px solid #3a5870;border-radius:7px;padding:8px 10px;font-size:14px;cursor:pointer}
button:focus-visible,select:focus-visible,input:focus-visible,canvas:focus-visible{outline:2px solid #9ad2ff;outline-offset:2px}.legend{display:flex;gap:15px;align-items:center;flex-wrap:wrap;font-size:13px;color:#c5d5e4}
.dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:5px}.left{background:#c798ff}.right{background:#ffbd77}.target{background:#fff}
canvas{display:block;width:100%;height:auto;aspect-ratio:1.65;background:radial-gradient(ellipse at center,#203951 0,#101f31 75%);touch-action:pan-y;cursor:grab}canvas:active{cursor:grabbing}
.under{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin-top:15px}.info{padding:17px;line-height:1.55}.info h2{font-size:16px;margin:0 0 9px}.info p{color:#b7cce0;font-size:13px;margin:6px 0}.mono{font-variant-numeric:tabular-nums;font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:13px;overflow-wrap:anywhere}
.note{font-size:13px;color:#9eb5c9;margin:15px 3px 0;line-height:1.55}@media(max-width:680px){body{padding:12px}.top{grid-template-columns:repeat(2,1fr)}.under{grid-template-columns:1fr}.controls{gap:11px}}
</style></head><body><main class="wrap">
<h1>G1 wrist execution accuracy · 3D</h1>
<p class="sub">Each dot = one completed MoveIt execution. Coordinates are <strong>actual wrist TF − desired wrist goal</strong> in pelvis, in millimetres. Centre (0,0,0) means perfect agreement.</p>
<div class="top"><div class="stat"><strong id="count">—</strong><span>Executions</span></div><div class="stat"><strong id="avg">—</strong><span>Mean 3D error</span></div><div class="stat"><strong id="within">—</strong><span>Within 5 mm in 3D</span></div><div class="stat"><strong id="maxerr">—</strong><span>Worst 3D error</span></div></div>
<section class="panel"><div class="controls"><label>Show <select id="arm"><option value="all">Both arms</option><option value="left">Left arm</option><option value="right">Right arm</option></select></label><label>Zoom <input id="zoom" type="range" min="60" max="180" value="100" step="5"></label><button type="button" id="reset">Reset 3D view</button><span class="legend"><span><i class="dot left"></i>Left</span><span><i class="dot right"></i>Right</span><span><i class="dot target"></i>Goal</span></span></div>
<canvas id="chart" tabindex="0" role="img" aria-label="Interactive 3D scatter plot of the signed wrist position errors in X Y Z. Drag to rotate; use zoom control to enlarge."></canvas></section>
<div class="under"><section class="panel info"><h2>Selected execution</h2><div id="selected" class="mono">Click a dot to inspect its measured error.</div></section><section class="panel info"><h2>Mean signed error · bias</h2><div id="bias" class="mono">—</div><p>XYZ axes: X red, Y green, Z blue. Dashed 5, 10 and 15 mm spheres are guide surfaces, not 1D tolerances.</p></section></div>
<p class="note">Drag left/right/up/down to rotate. Click a measurement point to inspect it; select an arm to filter. These are robot_state_publisher TF estimates derived from joint feedback and the kinematic model, not independent external metrology. Logging occurs after MoveIt reports success; that does not establish a 5 mm controller acceptance criterion.</p>
<script type="application/json" id="samples">__DATA__</script>
<script>
(()=>{
'use strict';
const root=document, data=JSON.parse(root.getElementById('samples').textContent),
canvas=root.getElementById('chart'),ctx=canvas.getContext('2d'), filter=root.getElementById('arm'),zoom=root.getElementById('zoom');
const colors={left:'#c798ff',right:'#ffbd77'}, ac={x:'#ff737c',y:'#79dda0',z:'#82b9ff'};
let yaw=-0.68,pitch=-0.52,selected=null,marks=[],drag=null;
const round=(v,n=2)=>Number(v).toFixed(n), signed=v=>(v>=0?'+':'')+round(v), get=()=>data.filter(d=>filter.value==='all'||d.arm===filter.value);
function sz(){const rect=canvas.getBoundingClientRect(),dpr=Math.min(window.devicePixelRatio||1,2);canvas.width=Math.max(300,Math.round(rect.width*dpr));canvas.height=Math.max(230,Math.round(rect.width/1.65*dpr));ctx.setTransform(dpr,0,0,dpr,0,0);return [canvas.width/dpr,canvas.height/dpr];}
function updateStats(){const arr=get(),n=arr.length,mean=(key)=>n?arr.reduce((a,d)=>a+d[key],0)/n:0;
root.getElementById('count').textContent=String(n);root.getElementById('avg').textContent=n?round(mean('r'))+' mm':'—';
root.getElementById('within').textContent=n?arr.filter(d=>d.r<=5).length+'/'+n:'—';
root.getElementById('maxerr').textContent=n?round(Math.max(...arr.map(d=>d.r)))+' mm':'—';
root.getElementById('bias').textContent=n?'X '+signed(mean('x'))+' mm  ·  Y '+signed(mean('y'))+' mm  ·  Z '+signed(mean('z'))+' mm':'No records';
if(selected&&!arr.includes(selected))selected=null;updateSelected();}
function updateSelected(){const el=root.getElementById('selected');if(!selected){el.textContent='Click a dot to inspect its measured error.';return;}
const d=selected;el.textContent=d.arm.toUpperCase()+' · '+d.object+'\n'+
'X '+signed(d.x)+' mm  |  Y '+signed(d.y)+' mm  |  Z '+signed(d.z)+' mm\n'+
'3D '+round(d.r)+' mm  |  rotation '+d.rot+'°\n'+d.time+'\nExecution '+d.id;el.style.whiteSpace='pre-line';}
function render(){const [w,h]=sz(), arr=get(), canvasSize=Math.min(w,h), span=Math.max(18,...data.flatMap(d=>[Math.abs(d.x),Math.abs(d.y),Math.abs(d.z)]))*1.2;
const sc=canvasSize*0.38/span*(Number(zoom.value)/100),cx=w*0.50,cy=h*0.50,co=Math.cos(yaw),si=Math.sin(yaw),cp=Math.cos(pitch),sp=Math.sin(pitch);
function point(x,y,z){const xx=x*co-y*si,yy=x*si+y*co;return {x:cx+xx*sc,y:cy-(yy*cp-z*sp)*sc,depth:yy*sp+z*cp};}
function line(a,b,stroke,width=1,dash=[]){ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(a.x,a.y);ctx.lineTo(b.x,b.y);ctx.stroke();ctx.setLineDash([]);}
function sphere(r,alpha){const planes=[[0,1],[0,2],[1,2]];ctx.lineWidth=1;ctx.strokeStyle='rgba(166,189,212,'+alpha+')';ctx.setLineDash([4,6]);for(const [a,b] of planes){ctx.beginPath();for(let j=0;j<=100;j++){const t=j*Math.PI*2/100,v=[0,0,0];v[a]=r*Math.cos(t);v[b]=r*Math.sin(t);const p=point(...v);if(!j)ctx.moveTo(p.x,p.y);else ctx.lineTo(p.x,p.y);}ctx.stroke();}ctx.setLineDash([]);}
ctx.clearRect(0,0,w,h);for(const r of [15,10,5])sphere(r,r===5?0.70:r===15?0.42:0.28);
for(const [axis,v] of Object.entries({x:[1,0,0],y:[0,1,0],z:[0,0,1]})){
const end=span*0.80;line(point(...v.map(x=>-x*end)),point(...v.map(x=>x*end)),ac[axis],2.1);const p=point(...v.map(x=>x*end));ctx.fillStyle=ac[axis];ctx.font='bold 16px system-ui';ctx.fillText(axis.toUpperCase()+'+',p.x+6,p.y-4);}
const origin=point(0,0,0);ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(origin.x,origin.y,5.5,0,Math.PI*2);ctx.fill();
marks=arr.map(d=>({d,p:point(d.x,d.y,d.z)})).sort((a,b)=>a.p.depth-b.p.depth);
for(const {d,p} of marks){ctx.globalAlpha=d===selected?1:0.86;ctx.fillStyle=colors[d.arm]||'#f9db81';ctx.beginPath();ctx.arc(p.x,p.y,d===selected?8:5.5,0,Math.PI*2);ctx.fill();ctx.globalAlpha=1;ctx.strokeStyle=d===selected?'#fff':'rgba(255,255,255,.55)';ctx.lineWidth=d===selected?2.3:0.9;ctx.stroke();}
ctx.fillStyle='#d4e4f2';ctx.font='13px system-ui';ctx.fillText('Origin = desired wrist position  •  rings: 5 / 10 / 15 mm',15,h-16);
}
filter.addEventListener('change',()=>{updateStats();render()});zoom.addEventListener('input',render);
root.getElementById('reset').addEventListener('click',()=>{yaw=-0.68;pitch=-0.52;zoom.value=100;render()});
canvas.addEventListener('pointerdown',e=>{drag={id:e.pointerId,x:e.clientX,y:e.clientY,moved:false};canvas.setPointerCapture(e.pointerId)});
canvas.addEventListener('pointermove',e=>{if(!drag||drag.id!==e.pointerId)return;const dx=e.clientX-drag.x,dy=e.clientY-drag.y;if(Math.abs(dx)+Math.abs(dy)>1){drag.moved=true;yaw+=dx*.009;pitch=Math.max(-1.43,Math.min(1.43,pitch+dy*.009));render();}drag.x=e.clientX;drag.y=e.clientY});
canvas.addEventListener('pointerup',e=>{if(!drag||drag.id!==e.pointerId)return;if(!drag.moved){const rect=canvas.getBoundingClientRect(),x=e.clientX-rect.left,y=e.clientY-rect.top;let best=null,dist=16**2;for(const m of marks){const dd=(m.p.x-x)**2+(m.p.y-y)**2;if(dd<dist){best=m.d;dist=dd;}}if(best){selected=best;updateSelected();render();}}drag=null});
canvas.addEventListener('keydown',e=>{if(e.key==='ArrowLeft')yaw-=.12;else if(e.key==='ArrowRight')yaw+=.12;else if(e.key==='ArrowUp')pitch=Math.min(1.43,pitch+.12);else if(e.key==='ArrowDown')pitch=Math.max(-1.43,pitch-.12);else return;e.preventDefault();render()});
window.addEventListener('resize',render);updateStats();render();
})();
</script></main></body></html>
'''
    html = html.replace('__DATA__', packed)
    Path(html_path).write_text(html, encoding='utf-8')
    return Path(html_path)


class ObjectPlanner:
    def __init__(self, arm, height, wrist_backoff, side_offset, max_plan_age, max_start_error,
                 velocity_scaling, acceleration_scaling, max_ik_trials, ik_timeout, goal_log,
                 left_palm_axis='minus-y', allow_left_execution=True):
        rclpy.init()
        self.node = Node('karis_object_review_execute')
        self.arm = arm
        self.left_palm_axis = left_palm_axis
        self.allow_left_execution = allow_left_execution
        self.height = height
        self.wrist_backoff = wrist_backoff
        self.side_offset = side_offset  # Signed Y displacement in pelvis frame (m)
        self.max_plan_age = max_plan_age
        self.max_start_error = max_start_error
        self.velocity_scaling = velocity_scaling
        self.acceleration_scaling = acceleration_scaling
        self.max_ik_trials = max_ik_trials
        self.ik_timeout = ik_timeout
        self.goal_log = goal_log
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
        self.joints = tuple(f'{arm}_{part}_joint' for part in ARM_PARTS)
        self.plan = None
        self.object_msg = None
        self.last_message_at = 0.0
        self.has_live_object_stream = False
        self.last_state = None
        self.last_state_at = 0.0
        self.class_anchors = {}  # helps follow a re-numbered detection without guessing

        # /joint_states is deliberately two publishers: body and FTP hands.
        # Filter for a complete body message, using BEST_EFFORT for compatibility.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(JointState, '/joint_states', self.on_state, qos)
        self.display_pub = self.node.create_publisher(
            DisplayTrajectory, '/display_planned_path',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.marker_pub = self.node.create_publisher(
            Marker, '/karis/manual_plan_markers',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))

        # Discover the custom object message type *after* DDS graph discovery.
        self.discover_object_type()

    def discover_object_type(self):
        print(f'Waiting for {TOPIC} ROS 2 discovery ...', flush=True)
        deadline = time.monotonic() + 18.0
        types = set()
        while time.monotonic() < deadline:
            types = {typ for name, typs in self.node.get_topic_names_and_types()
                     if name == TOPIC for typ in typs}
            if types:
                break
            rclpy.spin_once(self.node, timeout_sec=0.2)
        if len(types) != 1:
            raise RuntimeError(f'{TOPIC} not found or ambiguous: {sorted(types)}; '
                               'check dual-network ROS setup and whether Kim publishes KARIS.')
        msg_type = get_message(next(iter(types)))
        self.node.create_subscription(
            msg_type, TOPIC, self.on_objects,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        print(f'Discovered {TOPIC} [{next(iter(types))}]')

    def on_objects(self, msg):
        received = time.monotonic()
        # Two spatially separated arrivals demonstrate a live stream rather
        # than an old message still waiting in a DDS receive queue.
        if self.last_message_at and received - self.last_message_at >= 0.12:
            self.has_live_object_stream = True
        self.object_msg = msg
        self.last_message_at = received

    def on_state(self, msg):
        if len(msg.name) != len(msg.position):
            return
        q = dict(zip(msg.name, msg.position))
        if all(name in q and math.isfinite(q[name]) for name in self.joints):
            self.last_state = msg
            self.last_state_at = time.monotonic()

    def new_objects(self, seconds=6.0):
        self.object_msg = None
        self.last_message_at = 0.0
        self.has_live_object_stream = False
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.15)
            if (self.object_msg is not None and self.has_live_object_stream
                    and time.monotonic() - self.last_message_at < 0.5):
                return self.object_msg
        raise RuntimeError(f'No verified live {TOPIC} stream in {seconds:.0f}s')

    def measured_state(self, timeout=5.0):
        self.last_state = None
        self.last_state_at = 0.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.15)
            if self.last_state is not None:
                return self.last_state
        raise RuntimeError('No fresh complete G1 arm /joint_states')

    def find_object(self, name):
        # A live ObjectPoseArray can contain a tracked object with an old
        # per-object age_s. Wait briefly for a fresh observation of the
        # *same* ID rather than refusing the first stale frame. Never
        # loosen the original 2-second per-object age threshold.
        msg = self.new_objects()
        if msg.header.frame_id != 'pelvis':
            raise RuntimeError(f'Expected KARIS frame pelvis, got {msg.header.frame_id!r}')
        objects = [obj for obj in msg.objects if obj.object_id == name]
        if not objects:
            seen = ', '.join(obj.object_id for obj in msg.objects)
            raise RuntimeError(f'Object {name!r} not in latest detection. Found: {seen}')

        obj = objects[0]
        age = float(getattr(obj, 'age_s', 0.0))
        if not math.isfinite(age) or age < 0.0:
            raise RuntimeError(f'{name} has invalid KARIS age_s={age}')
        if age > 2.0:
            print(f'{name}: age_s={age:.2f} exceeds 2.00s; '
                  'waiting up to 8s for a fresh update of this ID ...')
            deadline = time.monotonic() + 8.0
            last_checked_at = self.last_message_at
            while time.monotonic() < deadline:
                rclpy.spin_once(self.node, timeout_sec=0.15)
                if self.last_message_at <= last_checked_at:
                    continue
                last_checked_at = self.last_message_at
                msg = self.object_msg
                if msg.header.frame_id != 'pelvis':
                    raise RuntimeError('KARIS reference frame changed while waiting')
                candidates = [item for item in msg.objects if item.object_id == name]
                if not candidates:
                    continue
                obj = candidates[0]
                age = float(getattr(obj, 'age_s', 0.0))
                if math.isfinite(age) and 0.0 <= age <= 2.0:
                    print(f'{name}: refreshed with age_s={age:.2f}; planning.')
                    break
            else:
                raise RuntimeError(
                    f'{name}: no fresh observation of this ID after 8s '
                    f'(last reported age_s={age:.2f}). No plan requested. '
                    'Use list to inspect IDs and age_s; check whether '
                    'KARIS is still refreshing that tracked object.')

        position = obj.pose.position
        xyz = (float(position.x), float(position.y), float(position.z))
        if not all(math.isfinite(v) for v in xyz):
            raise RuntimeError('KARIS object position is not finite')
        class_key = self.normalized_class(obj.class_name)
        self.class_anchors[class_key] = xyz
        return obj, xyz

    @staticmethod
    def normalized_class(value):
        return ''.join(c for c in str(value).lower() if c.isalnum())

    def plan_by_class(self, class_name):
        # Object IDs can change after a track is lost. A class label is stable,
        # but a class can occur multiple times. Never silently select a random
        # instance of a multiply-detected class.
        wanted = self.normalized_class(class_name)
        if not wanted:
            raise RuntimeError('Usage: plan-class CLASS_NAME')
        msg = self.new_objects()
        candidates = []
        for obj in msg.objects:
            if self.normalized_class(obj.class_name) != wanted:
                continue
            age = float(getattr(obj, 'age_s', 0.0))
            if not math.isfinite(age) or not (0 <= age <= 2.0):
                continue
            p = obj.pose.position
            xyz = (float(p.x), float(p.y), float(p.z))
            if all(math.isfinite(v) for v in xyz):
                candidates.append((obj.object_id, xyz))
        if not candidates:
            raise RuntimeError(f'No fresh objects of class {class_name!r}. Use list.')
        if len(candidates) == 1:
            chosen = candidates[0]
        else:
            previous_xyz = self.class_anchors.get(wanted)
            if previous_xyz is None:
                raise RuntimeError(f'{len(candidates)} objects share class {class_name!r}: '
                                   + ', '.join(item[0] for item in candidates)
                                   + '. Select the exact ID first with plan OBJECT_ID.')
            ranked = sorted((math.dist(xyz, previous_xyz), obj_id, xyz)
                            for obj_id, xyz in candidates)
            if ranked[0][0] > 0.30 or ranked[1][0] - ranked[0][0] < 0.05:
                raise RuntimeError('Class re-identification is ambiguous or moved too far; '
                                   'use list and plan EXACT_OBJECT_ID instead.')
            chosen = (ranked[0][1], ranked[0][2])
        print(f'Class {class_name!r} -> current detected ID {chosen[0]} '
              '(checked for ambiguity).')
        self.make_plan(chosen[0])

    def list_objects(self):
        msg = self.new_objects()
        print(f'\nKARIS objects [{msg.header.frame_id}]:')
        for obj in msg.objects:
            p = obj.pose.position
            reported_age = float(getattr(obj, 'age_s', 0.0))
            age_text = f'{reported_age:.2f}s' if math.isfinite(reported_age) else 'invalid'
            print(f'  {obj.object_id:42s}  '
                  f'{p.x:+.3f} {p.y:+.3f} {p.z:+.3f} m  '
                  f'conf={obj.confidence:.2f} mode={obj.mode} age_s={age_text}')
        if not msg.objects:
            print('  (no detections)')

    def service(self, srv_type, endpoint, timeout=8.0):
        client = self.node.create_client(srv_type, endpoint)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{endpoint} is unavailable')
        return client

    def wrist_orientation(self, yaw_degrees):
        # Apply pelvis-Z yaw to the arm-specific palm-down wrist orientation.
        # Right nominal palm = wrist +Y; left nominal palm = wrist -Y.
        # Explicit 'plus-y' retains V8 legacy orientation for comparison.
        mirrored_left = (self.arm == 'left' and self.left_palm_axis == 'minus-y')
        base_q = LEFT_WRIST_Q if mirrored_left else RIGHT_WRIST_Q
        half = math.radians(yaw_degrees) / 2.0
        c, t = math.cos(half), math.sin(half)
        x, y, z, w = base_q
        return (c*x - t*y, t*x + c*y, c*z + t*w, c*w - t*z)

    def palm_axis_name(self):
        return '-Y' if self.arm == 'left' and self.left_palm_axis == 'minus-y' else '+Y'

    def ik_candidates(self):
        # Try the requested height/backoff first with arm-specific palm-down
        # orientation. Only if max_ik_trials > 1, relax backoff, height or yaw.
        backoffs = [self.wrist_backoff]
        for step in (0.02, -0.02, 0.04, -0.04, 0.06, -0.06, 0.08, -0.08,
                     0.10, -0.10, 0.12, -0.12):
            value = round(self.wrist_backoff + step, 4)
            if 0.05 <= value <= 0.25 and all(abs(value - v) > 0.005 for v in backoffs):
                backoffs.append(value)
        heights = [self.height]
        for delta in (0.03, -0.02, 0.06):
            value = round(self.height + delta, 4)
            if 0.05 <= value <= 0.40 and all(abs(value - v) > 0.005 for v in heights):
                heights.append(value)
        # First prioritize wrist location; then alternative heights; then yaw.
        ordered = []
        ordered.extend((b, self.height, 0) for b in backoffs)
        for yaw in (15, -15):
            ordered.extend((b, self.height, yaw) for b in backoffs[:6])
        for h in heights[1:]:
            ordered.extend((b, h, 0) for b in backoffs)
        for yaw in (15, -15, 30, -30):
            for h in heights:
                ordered.extend((b, h, yaw) for b in backoffs)
        for backoff, height, yaw in ordered[:self.max_ik_trials]:
            yield backoff, height, yaw

    def compute_ik(self, client, xyz, quat, measured):
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = f'{self.arm}_arm'
        ik.ik_link_name = f'{self.arm}_wrist_yaw_link'
        ik.robot_state.joint_state = measured
        ik.robot_state.is_diff = True
        ik.avoid_collisions = True
        ik.timeout = Duration(seconds=self.ik_timeout).to_msg()
        ik.pose_stamped = PoseStamped()
        ik.pose_stamped.header.frame_id = 'pelvis'
        ik.pose_stamped.pose.position.x, ik.pose_stamped.pose.position.y, ik.pose_stamped.pose.position.z = xyz
        (ik.pose_stamped.pose.orientation.x, ik.pose_stamped.pose.orientation.y,
         ik.pose_stamped.pose.orientation.z, ik.pose_stamped.pose.orientation.w) = quat
        response = future_result(self.node, client.call_async(request),
                                 self.ik_timeout + 7.0, 'IK')
        code = int(response.error_code.val)
        if code != 1:
            return code, None
        values = dict(zip(response.solution.joint_state.name,
                          response.solution.joint_state.position))
        if any(name not in values or not math.isfinite(values[name]) for name in self.joints):
            raise RuntimeError('IK response missing valid arm joints')
        return code, values

    def choose_ik_for_live_target(self, xyz, measured):
        client = self.service(GetPositionIK, '/compute_ik')
        tried_codes = {}
        for attempt, (backoff, height, yaw) in enumerate(self.ik_candidates(), 1):
            wrist_xyz = (xyz[0] - backoff, xyz[1] + self.side_offset, xyz[2] + height)
            quat = self.wrist_orientation(yaw)
            print(f'  IK {attempt:02d}/{self.max_ik_trials}: wrist-backoff={backoff:.3f}m '
                  f'side-offset={self.side_offset:+.4f}m height={height:.3f}m '
                  f'yaw={yaw:+.0f}deg ', end='', flush=True)
            code, values = self.compute_ik(client, wrist_xyz, quat, measured)
            print(f'-> code={code}', flush=True)
            if code == 1:
                return values, wrist_xyz, backoff, height, yaw
            tried_codes[code] = tried_codes.get(code, 0) + 1
        raise RuntimeError(
            f'No collision-checked palm-down IK among {attempt} nearby poses; '
            f'MoveIt codes={tried_codes}. No plan/execution requested. '
            'The object may be unreachable or blocked in the current robot/scene pose.')

    def make_plan(self, obj_id):
        # A new requested plan invalidates the previous one, even on failure.
        self.plan = None
        obj, xyz = self.find_object(obj_id)
        print(f'\nObject {obj_id}: ({xyz[0]:+.4f}, {xyz[1]:+.4f}, {xyz[2]:+.4f}) pelvis')
        print('Finding a collision-checked wrist target automatically from LIVE object XYZ...')
        print(f'Preferred wrist offsets in pelvis: X={-self.wrist_backoff:+.3f}m, '
              f'Y={self.side_offset:+.4f}m, Z={self.height:+.3f}m.')
        print(f'Wrist nominal palm direction: {self.palm_axis_name()} points down; '
              'modest pelvis yaw may vary.')
        if self.arm == 'left' and not self.allow_left_execution:
            print('LEFT ARM PREVIEW-ONLY: execution explicitly disabled for this run.')
        print('WARNING: wrist-to-palm TCP is not calibrated. These are pre-grasp pose candidates.')

        measured = self.measured_state()
        # If the user moved the object during the IK search, update the target
        # and solve again once. We do not continuously command a moving robot.
        for refresh_index in range(2):
            joint_goal, approach, backoff, height, yaw = self.choose_ik_for_live_target(xyz, measured)
            latest_obj, latest_xyz = self.find_object(obj_id)
            displacement = math.dist(latest_xyz, xyz)
            if displacement <= 0.02:
                break
            if refresh_index == 1:
                raise RuntimeError(f'Object moved {displacement*1000:.1f}mm during IK search again. '
                                   'Hold it stationary, then plan again. No execution requested.')
            print(f'Object moved {displacement*1000:.1f}mm during search; '
                  'automatically recalculating from newest observation...', flush=True)
            xyz = latest_xyz
            obj = latest_obj
            measured = self.measured_state()
        print(f'Selected wrist target: ({approach[0]:+.4f}, {approach[1]:+.4f}, '
              f'{approach[2]:+.4f}) pelvis')
        print(f'Selected wrist backoff={backoff:.3f}m, side-offset={self.side_offset:+.4f}m, '
              f'height={height:.3f}m, yaw={yaw:+.0f}deg')
        print('NOTE: If backoff/height/yaw differ from the preferred values, verify '
              'the actual hand is correctly over the object in RViz.')

        action = ActionClient(self.node, MoveGroup, '/move_action')
        if not action.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('MoveIt /move_action server unavailable')
        goal = MoveGroup.Goal()
        goal.request.group_name = f'{self.arm}_arm'
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = self.velocity_scaling
        goal.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal.request.start_state.is_diff = True
        constraint = Constraints()
        constraint.name = f'karis_manual_{obj_id}'
        for joint in self.joints:
            part = JointConstraint()
            part.joint_name = joint
            part.position = float(joint_goal[joint])
            part.tolerance_above = 0.01
            part.tolerance_below = 0.01
            part.weight = 1.0
            constraint.joint_constraints.append(part)
        goal.request.goal_constraints = [constraint]
        goal.planning_options.plan_only = True
        goal.planning_options.replan = False
        goal.planning_options.look_around = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        print(f'Planning ONLY with MoveIt (velocity={self.velocity_scaling:.0%}, '
              f'acceleration={self.acceleration_scaling:.0%}) ...')
        handle = future_result(self.node, action.send_goal_async(goal), 10.0, 'MoveGroup goal')
        if not handle.accepted:
            raise RuntimeError('MoveIt planning request rejected')
        response = future_result(self.node, handle.get_result_async(), 45.0, 'MoveGroup result').result
        traj = response.planned_trajectory.joint_trajectory
        print(f'  MoveIt code={response.error_code.val} (1=SUCCESS); waypoints={len(traj.points)}')
        if traj.points:
            stamp = traj.points[-1].time_from_start
            planned_seconds = stamp.sec + stamp.nanosec * 1e-9
            print(f'  Planned trajectory duration: {planned_seconds:.2f}s '
                  '(controller may execute more slowly)')
        if int(response.error_code.val) != 1 or not traj.points:
            raise RuntimeError('MoveIt could not plan a valid trajectory; execution unavailable')
        if not all(name in traj.joint_names for name in self.joints):
            raise RuntimeError('Planned trajectory is missing arm joints')

        # Keep the two goal poses in memory while the operator reviews the plan.
        # No CSV/SVG writes occur during planning, re-planning, or discarding.
        wrist_quat = self.wrist_orientation(yaw)

        self.plan = StoredPlan(
            object_id=obj_id, xyz=xyz, approach_xyz=approach,
            made_at=time.monotonic(), trajectory=response.planned_trajectory,
            trajectory_start=response.trajectory_start, joint_names=tuple(traj.joint_names),
            chosen_backoff=backoff, chosen_height=height,
            chosen_side_offset=self.side_offset, chosen_yaw_deg=yaw,
            object_reference=obj, wrist_quaternion=wrist_quat)
        self.publish_plan()
        self.publish_markers()
        print(f'\nPLAN READY: {len(traj.points)} points; '+
              f'RViz: /display_planned_path (pre-grasp only).')
        print(f'Inspect actual palm direction and clearances. '
              f'Plan expires after {self.max_plan_age:.0f}s.')
        print('If the object moves, the reviewed trajectory does NOT chase it. Replan for a new target.')
        print('Object, desired wrist, measured wrist and errors logged ONLY after successful execution.')
        print('REVIEW IN RViz: type execute to explicitly APPROVE physical movement (no second prompt), '
              'or discard to reject it. The chosen plan is never executed automatically.')

    def publish_markers(self):
        if self.plan is None:
            return
        for marker_id, xyz, diameter, rgb in (
            (0, self.plan.xyz, 0.035, (1.0, 0.35, 0.0)),
            (1, self.plan.approach_xyz, 0.045, (0.1, 0.95, 0.25)),
        ):
            m = Marker()
            m.header.frame_id = 'pelvis'
            m.ns = 'karis_object_review'
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = xyz
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = diameter
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 0.9
            self.marker_pub.publish(m)

        # Remove V8's always-down GREEN marker if RViz remains open across upgrades.
        old_arrow = Marker()
        old_arrow.header.frame_id = 'pelvis'
        old_arrow.ns = 'karis_desired_wrist_axis'
        old_arrow.id = 2
        old_arrow.action = Marker.DELETE
        self.marker_pub.publish(old_arrow)

        # Visualize ACTUAL target axes (computed from goal quaternion), not
        # a hard-coded downward arrow. GREEN = wrist +Y; BLUE = nominal
        # hand-facing/palm normal, which may be wrist -Y for the left hand.
        from geometry_msgs.msg import Point
        wx, wy, wz = self.plan.approach_xyz
        qx, qy, qz, qw = self.plan.wrist_quaternion
        # R(q) @ [0, 1, 0] : desired +Y expressed in pelvis.
        plus_y = (2 * (qx*qy - qz*qw),
                  1 - 2*(qx*qx + qz*qz),
                  2 * (qy*qz + qx*qw))
        down_sign = -1 if self.palm_axis_name() == '-Y' else 1
        for idx, direction, rgb, namespace in (
            (2, plus_y, (0.0, 1.0, 0.0), 'karis_desired_wrist_plus_y'),
            (3, tuple(down_sign*v for v in plus_y), (0.0, 0.6, 1.0),
             'karis_nominal_palm_direction'),
        ):
            m = Marker()
            m.header.frame_id = 'pelvis'
            m.ns = namespace
            m.id = idx
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.points = [Point(x=wx, y=wy, z=wz),
                        Point(x=wx + direction[0] * 0.085,
                              y=wy + direction[1] * 0.085,
                              z=wz + direction[2] * 0.085)]
            m.scale.x, m.scale.y, m.scale.z = 0.008, 0.018, 0.022
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 0.9
            self.marker_pub.publish(m)

    def publish_plan(self):
        if self.plan is None:
            return
        display = DisplayTrajectory()
        display.trajectory_start = self.plan.trajectory_start
        display.trajectory = [self.plan.trajectory]
        # Keep publishing during discovery and RViz transport startup.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            self.display_pub.publish(display)
            rclpy.spin_once(self.node, timeout_sec=0.2)

    def execution_setting(self):
        client = self.service(GetParameters, '/move_group/get_parameters')
        req = GetParameters.Request()
        req.names = ['allow_trajectory_execution']
        reply = future_result(self.node, client.call_async(req), 8.0, 'execution setting')
        if len(reply.values) != 1 or reply.values[0].type != 1:
            raise RuntimeError('Cannot verify /move_group allow_trajectory_execution Boolean')
        return bool(reply.values[0].bool_value)

    def valid_at_selected_waypoints(self):
        # Recheck the collision scene before execution. This checks the
        # represented geometry; unknown tables/objects cannot be validated.
        client = self.service(GetStateValidity, '/check_state_validity')
        jt = self.plan.trajectory.joint_trajectory
        n = len(jt.points)
        sample_ids = sorted({round(i * (n - 1) / min(n - 1, 19))
                             for i in range(min(n, 20))}) if n > 1 else [0]
        for i in sample_ids:
            req = GetStateValidity.Request()
            req.group_name = f'{self.arm}_arm'
            req.robot_state.is_diff = True
            js = req.robot_state.joint_state
            js.name = list(jt.joint_names)
            js.position = list(jt.points[i].positions)
            answer = future_result(self.node, client.call_async(req), 5.0,
                                   f'collision check waypoint {i}')
            if not answer.valid:
                hits = [f'{c.contact_body_1}/{c.contact_body_2}' for c in list(answer.contacts)[:3]]
                raise RuntimeError(f'Planning scene collision/invalid state at waypoint {i}: {hits}')
        print(f'Rechecked {len(sample_ids)} trajectory waypoints against current MoveIt scene.')

    def verify_execute(self):
        if self.plan is None:
            raise RuntimeError('No stored plan. First enter an object ID (or plan OBJECT_ID).')
        elapsed = time.monotonic() - self.plan.made_at
        if elapsed > self.max_plan_age:
            self.plan = None
            raise RuntimeError(f'Plan expired ({elapsed:.0f}s). Create a fresh plan.')
        if not self.execution_setting():
            raise RuntimeError('/move_group allow_trajectory_execution is FALSE. '
                               'Plan-only preview works, but physical execution is blocked.')

        obj, xyz = self.find_object(self.plan.object_id)
        moved = math.dist(xyz, self.plan.xyz)
        if moved > 0.02:
            self.plan = None
            raise RuntimeError(f'Object moved {moved*1000:.1f}mm (>20mm); plan discarded. Replan.')

        state = self.measured_state()
        measured = dict(zip(state.name, state.position))
        trajectory = self.plan.trajectory.joint_trajectory
        first = dict(zip(trajectory.joint_names, trajectory.points[0].positions))
        first_error = max(abs(measured[j] - first[j]) for j in self.joints)
        print(f'Start state max arm-joint mismatch: {first_error:.4f} rad')
        if first_error > self.max_start_error:
            self.plan = None
            raise RuntimeError('Robot moved since planning; discarded stale trajectory. Replan.')
        self.valid_at_selected_waypoints()
        if time.monotonic() - self.plan.made_at > self.max_plan_age:
            self.plan = None
            raise RuntimeError('Plan expired during revalidation. Replan.')

    def wrist_tf_latest(self):
        """Return latest measured robot-state-publisher TF pose, or None."""
        link=f'{self.arm}_wrist_yaw_link'
        if not self.tf_buffer.can_transform('pelvis',link,Time()):
            return None
        tf=self.tf_buffer.lookup_transform('pelvis',link,Time())
        s=tf.header.stamp
        if s.sec == 0 and s.nanosec == 0:
            return None
        now_ns=self.node.get_clock().now().nanoseconds
        stamp_ns=s.sec*1_000_000_000+s.nanosec
        age=(now_ns-stamp_ns)*1e-9
        if age > 0.65 or age < -0.30:
            return None
        t=tf.transform.translation
        q=tf.transform.rotation
        xyz=(t.x,t.y,t.z)
        quat=(q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in (*xyz,*quat)):
            return None
        return xyz,quat,s,stamp_ns

    def check_measurement_available(self):
        """Fail closed before motor motion if no current wrist TF is visible."""
        deadline=time.monotonic()+3.0
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=0.12)
            if self.wrist_tf_latest() is not None:
                return
        raise RuntimeError('Fresh wrist TF (pelvis -> wrist_yaw_link) unavailable. '
                           'Cannot promise an actual-pose accuracy log; execution not requested.')

    def measured_wrist_after_execution(self, finished_ros_ns):
        """Sample fresh post-execution TF poses, report median X/Y/Z and last quaternion."""
        samples=[]
        seen=set()
        deadline=time.monotonic()+5.0
        # Allow robot-state-publisher and controller feedback to settle.
        settled_at=time.monotonic()+0.35
        while time.monotonic()<deadline:
            rclpy.spin_once(self.node,timeout_sec=0.10)
            if time.monotonic()<settled_at:
                continue
            current=self.wrist_tf_latest()
            if current is None:
                continue
            xyz,quat,stamp,stamp_ns=current
            # Reject TF samples from before execution finished, even if
            # they are still discoverable in the TF cache.
            if stamp_ns < finished_ros_ns or stamp_ns in seen:
                continue
            seen.add(stamp_ns)
            samples.append((xyz,quat,stamp))
            if len(samples)>=7:
                break
        if len(samples)<4:
            raise RuntimeError('MoveIt execution succeeded, but fewer than 4 fresh '
                               'post-execution wrist TF samples arrived. Accuracy not logged.')
        xyz=tuple(median(s[0][idx] for s in samples) for idx in range(3))
        last_quat=samples[-1][1]
        stamp=samples[-1][2]
        return xyz,last_quat,stamp,len(samples)

    def execute(self):
        if self.arm == 'left' and self.left_palm_axis == 'plus-y':
            print('LEFT ARM EXECUTION BLOCKED: plus-y is the legacy V8 orientation '
                  'associated with the observed upside-down left hand. '
                  'Use --left-palm-axis minus-y instead and review in RViz.')
            return
        if self.arm == 'left' and not self.allow_left_execution:
            print('LEFT ARM EXECUTION BLOCKED: this run was started with '
                  '--no-allow-left-execution. Restart without that flag to enable execution.')
            return
        self.verify_execute()
        if self.plan is None:
            return
        obj_id = self.plan.object_id
        print('\nPHYSICAL MOVEMENT AUTHORIZED BY THE execute COMMAND.')
        print('Operator must review RViz and clear the area BEFORE entering execute; '
              'VR/XR cannot own the arms simultaneously.')
        print(f'Wrist palm-facing direction: {self.palm_axis_name()} (model-based assumption).')
        print(f'Chosen wrist geometry: backoff={self.plan.chosen_backoff:.3f}m '
              f'side-offset={self.plan.chosen_side_offset:+.4f}m '
              f'height={self.plan.chosen_height:.3f}m '
              f'yaw={self.plan.chosen_yaw_deg:+.0f}deg (uncalibrated TCP).')
        print('Only the PRE-GRASP wrist pose is planned. No descent and no grip.')
        # Typing 'execute' is the separate explicit human approval; do NOT
        # silently auto-run from object selection. Keep all live preflight checks.
        print(f'Executing approved plan for {obj_id} after final live-state checks...')
        # Recheck the start immediately before sending the goal.
        measured = self.measured_state(timeout=3.0)
        q = dict(zip(measured.name, measured.position))
        jt = self.plan.trajectory.joint_trajectory
        first = dict(zip(jt.joint_names, jt.points[0].positions))
        if max(abs(q[j] - first[j]) for j in self.joints) > self.max_start_error:
            self.plan = None
            raise RuntimeError('Arm moved since planning. Execution canceled.')
        if time.monotonic() - self.plan.made_at > self.max_plan_age:
            self.plan = None
            raise RuntimeError('Plan expired before execution. Execution canceled.')

        self.check_measurement_available()

        action = ActionClient(self.node, ExecuteTrajectory, '/execute_trajectory')
        if not action.wait_for_server(timeout_sec=8.0):
            raise RuntimeError('MoveIt /execute_trajectory action unavailable')
        goal = ExecuteTrajectory.Goal()
        # Exactly the same trajectory shown in RViz, NOT a replanned trajectory.
        executing_plan = self.plan
        goal.trajectory = executing_plan.trajectory
        print('Sending reviewed trajectory to /execute_trajectory ...')
        execution_started_at = time.monotonic()
        handle = future_result(self.node, action.send_goal_async(goal), 10.0, 'execution goal')
        self.plan = None  # one-shot: prevent replaying the same motion
        if not handle.accepted:
            raise RuntimeError('MoveIt rejected execution goal')
        print('Execution accepted. Monitor the robot and be ready to use its physical stop.')
        try:
            wrapped = future_result(self.node, handle.get_result_async(), 180.0,
                                    'execution result')
        except KeyboardInterrupt:
            print('Requesting execution cancellation; use physical stop if needed.')
            future_result(self.node, handle.cancel_goal_async(), 5.0, 'cancel')
            raise
        elapsed = time.monotonic() - execution_started_at
        code = int(wrapped.result.error_code.val)
        action_status = int(wrapped.status)
        print(f'Execution finished: MoveIt error_code={code} '
              f'(1=SUCCESS); action_status={action_status} '
              f'(4=SUCCEEDED); elapsed={elapsed:.2f}s including action overhead')
        # A successful MoveIt action is NECESSARY but not sufficient to log
        # measured accuracy. We also require recent, post-completion wrist TF.
        if code != 1 or action_status != GoalStatus.STATUS_SUCCEEDED:
            print('No execution-accuracy data logged: MoveIt execution did not report SUCCESS.')
            return
        try:
            finished_ros_ns=self.node.get_clock().now().nanoseconds
            xyz,quat,stamp,n_samples=self.measured_wrist_after_execution(finished_ros_ns)
            csv_path,svg_path,row=append_execution_accuracy(
                self.goal_log,executing_plan,self.arm,xyz,quat,stamp)
        except (RuntimeError,OSError,ValueError,KeyError) as exc:
            print(f'WARNING: MoveIt execution succeeded, but final TF measurement/logging '
                  f'failed: {exc}. Do not replay motion just to repair the log.')
            return
        print(f'\n=== EXECUTION ACCURACY (pelvis -> {self.arm}_wrist_yaw_link) ===')
        print(f'  KARIS object: {executing_plan.xyz}')
        print(f'  Desired wrist: {executing_plan.approach_xyz}')
        print(f'  Actual TF wrist (median of {n_samples} post-execution readings): '
              f'({xyz[0]:+.5f}, {xyz[1]:+.5f}, {xyz[2]:+.5f}) m')
        print(f"  Error actual-desired: X={float(row['error_x_mm']):+.2f} mm, "
              f"Y={float(row['error_y_mm']):+.2f} mm, "
              f"Z={float(row['error_z_mm']):+.2f} mm")
        print(f"  Total 3D error={row['error_3d_mm']} mm; "
              f"orientation={row['rotation_error_deg']} deg")
        print('  IMPORTANT: Wrist TF is computed from joint feedback + model, '
              'not independent physical tracking.')
        print(f'EXECUTION ACCURACY CSV: {csv_path}')
        print(f'EXECUTION ERROR BULLSEYE SVG: {svg_path}')
        html_path=Path(csv_path).with_name(Path(csv_path).stem+'_3d.html')
        if html_path.exists():
            print(f'INTERACTIVE 3D ERROR VIEW: {html_path}')
        print(f"  Within 5 mm 3D target: {'YES' if float(row['error_3d_mm']) <= 5.0 else 'NO'} "
              '(measurement after execution; NOT a motion abort criterion)')

    def close(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', choices=('left', 'right'), default='right',
                        help='Select arm. Backoff is empirical; left minus-y / right plus-y palm direction.')
    parser.add_argument('--left-palm-axis', choices=('minus-y', 'plus-y'), default='minus-y',
                        help='Left nominal palm-facing wrist axis. Default minus-y (mirrored FTP); '
                             'plus-y restores the V8 orientation for PLAN-ONLY comparison.')
    parser.add_argument('--allow-left-execution', action=argparse.BooleanOptionalAction,
                        default=True,
                        help='Left-arm execution permitted by default after explicit execute; '
                             'use --no-allow-left-execution for preview-only testing.')
    parser.add_argument('--height', type=float, default=None,
                        help='Wrist pre-grasp Z height above detected object (m). '
                             'Defaults: right 0.110, left 0.085.')
    parser.add_argument('--wrist-backoff', type=float, default=None,
                        help='Wrist backoff behind object in pelvis X (m). '
                             'Defaults: right 0.223, left 0.233.')
    parser.add_argument('--side-offset', type=float, default=None,
                        help='Signed lateral wrist shift in pelvis Y (m): +Y left, -Y right; '
                             'defaults: right -0.032, left +0.0225. Fixed during IK search.')
    parser.add_argument('--max-ik-trials', type=int, default=1,
                        help='Maximum collision-checked IK attempts (1..60; default 1: exact calibrated target)')
    parser.add_argument('--ik-timeout', type=float, default=1.5,
                        help='Timeout per IK request in seconds (0.5..5.0; default 1.5)')
    parser.add_argument('--max-plan-age', type=float, default=90.0,
                        help='Maximum seconds between planning and execution')
    parser.add_argument('--max-start-error', type=float, default=0.06,
                        help='Maximum measured/planned starting joint mismatch (rad)')
    parser.add_argument('--velocity-scaling', type=float, default=1.00,
                        help='MoveIt planning velocity scaling (0 < value <= 1; default 1.00)')
    parser.add_argument('--acceleration-scaling', type=float, default=1.00,
                        help='MoveIt planning acceleration scaling (0 < value <= 1; default 1.00)')
    parser.add_argument('--accuracy-log', '--goal-log', dest='goal_log',
                        default=None,
                        help='Optional CSV path override. Default is '
                             '/workspace/karis_goal_logs/<arm>_execution_accuracy.csv; '
                             'one row per successful measured execution with bullseye SVG and 3D HTML')
    args = parser.parse_args()
    # Read each default from the selected arm, while allowing independent CLI
    # overrides for X, Y, Z. Never silently replace operator offsets during IK
    # search unless --max-ik-trials is explicitly raised above 1.
    defaults = DEFAULT_PICK_OFFSETS_M[args.arm]
    if args.wrist_backoff is None:
        args.wrist_backoff = defaults['wrist_backoff']
    if args.height is None:
        args.height = defaults['height']
    if args.side_offset is None:
        args.side_offset = defaults['side_offset']
    if args.goal_log is None:
        args.goal_log = f'/workspace/karis_goal_logs/{args.arm}_execution_accuracy.csv'
    if not math.isfinite(args.wrist_backoff) or not 0.05 <= args.wrist_backoff <= 0.25:
        parser.error('--wrist-backoff must be in [0.05,0.25] m')
    if not math.isfinite(args.height) or not 0.05 <= args.height <= 0.40:
        parser.error('--height must be in [0.05,0.40] m')
    if not math.isfinite(args.side_offset) or not -0.25 <= args.side_offset <= 0.25:
        parser.error('--side-offset must be in [-0.25,+0.25] m')
    if not 1 <= args.max_ik_trials <= 60:
        parser.error('--max-ik-trials must be between 1 and 60')
    if not math.isfinite(args.ik_timeout) or not 0.5 <= args.ik_timeout <= 5.0:
        parser.error('--ik-timeout must be in [0.5,5.0] seconds')
    if not math.isfinite(args.max_plan_age) or not 10 <= args.max_plan_age <= 300:
        parser.error('--max-plan-age must be in [10,300] seconds')
    if not math.isfinite(args.max_start_error) or not 0.01 <= args.max_start_error <= 0.08:
        parser.error('--max-start-error must be in [0.01,0.08] rad')
    if not math.isfinite(args.velocity_scaling) or not 0.0 < args.velocity_scaling <= 1.0:
        parser.error('--velocity-scaling must be in (0,1]')
    if not math.isfinite(args.acceleration_scaling) or not 0.0 < args.acceleration_scaling <= 1.0:
        parser.error('--acceleration-scaling must be in (0,1]')

    print(f'KARIS configuration: arm={args.arm}, wrist palm axis='
          f"{'-Y' if args.arm == 'left' and args.left_palm_axis == 'minus-y' else '+Y'}",
          f'velocity={args.velocity_scaling:.0%}, acceleration={args.acceleration_scaling:.0%}')
    print(f'Preferred wrist offset from object in pelvis: '
          f'X={-args.wrist_backoff:+.3f}m, Y={args.side_offset:+.4f}m, '
          f'Z={args.height:+.3f}m')
    print(f'IK attempts: {args.max_ik_trials} (1 = one exact calibrated wrist pose)')
    print(f'Execution accuracy log: {args.goal_log}')
    print('Physical execution requires an existing plan AND the explicit execute command.')

    client = None
    try:
        client = ObjectPlanner(args.arm, args.height, args.wrist_backoff,
                               args.side_offset, args.max_plan_age, args.max_start_error,
                               args.velocity_scaling, args.acceleration_scaling,
                               args.max_ik_trials, args.ik_timeout, args.goal_log,
                               args.left_palm_axis, args.allow_left_execution)
        print('\nKARIS → MoveIt interactive REVIEW & EXECUTE')
        print('Commands: list | OBJECT_ID | plan OBJECT_ID | plan-class CLASS_NAME | execute | discard | quit')
        print('Enter an OBJECT_ID alone to PLAN ONLY. After RViz review, typing execute APPROVES physical motion.')
        print('There is no second EXECUTE <id> prompt; execute itself is the explicit physical approval.')
        while True:
            try:
                command = input('\nkaris> ').strip()
            except EOFError:
                print('Exiting without executing.')
                break
            if not command:
                continue
            if command in ('quit', 'exit', 'q'):
                break
            try:
                if command in ('list', 'ls'):
                    client.list_objects()
                elif command.startswith('plan-class '):
                    client.plan_by_class(command[len('plan-class '):].strip())
                elif command.startswith('plan '):
                    object_id = command[5:].strip()
                    if object_id:
                        client.make_plan(object_id)
                    else:
                        print('Usage: plan OBJECT_ID')
                elif command == 'execute':
                    client.execute()
                elif command == 'discard':
                    client.plan = None
                    print('Stored plan discarded. No physical execution requested.')
                elif command == 'help':
                    print('Commands: list | OBJECT_ID | plan OBJECT_ID | plan-class CLASS_NAME | execute | discard | quit')
                    print('Typing execute triggers physical execution after live safeguards; no second confirmation.')
                else:
                    # Bare object IDs are unambiguous because reserved commands are handled above.
                    # find_object still verifies current KARIS observations and exact object ID.
                    client.make_plan(command)
            except (RuntimeError, ValueError) as exc:
                print(f'ERROR: {exc}', file=sys.stderr)
        return 0
    except (RuntimeError, ValueError) as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2
    finally:
        if client is not None:
            client.close()
        elif rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
