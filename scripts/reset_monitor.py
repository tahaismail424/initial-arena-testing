"""Per-episode pedestrian monitor for the reset fix.

Subscribes to /task_generator_node/human_states, /clock and the task reset counter. For each episode it prints,
every --period sim seconds: number of agents, how many moved > 0.2 m in the last 10 sim s, and at the first
message of an episode the max distance of each agent from the scenario start pose. Also logs Gazebo model count
(gz model --list) at each sample.
"""
import argparse
import json
import math
import subprocess
import time

import rclpy
from hunav_msgs.msg import Agents
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

p = argparse.ArgumentParser()
p.add_argument('--scenario', default='/home/tahaismail/arena_ws/src/arena/simulation-setup/worlds/school_hallway/scenarios/hallway_tame.json')
p.add_argument('--period', type=float, default=10.0)
p.add_argument('--duration', type=float, default=600.0, help='wall seconds')
a = p.parse_args()

starts = {}
try:
    sc = json.load(open(a.scenario))
    for o in sc.get('obstacles', {}).get('dynamic', []):
        starts[o['name']] = tuple(o['pos'][:2])
except Exception as e:  # noqa
    print('scenario parse failed', e)

rclpy.init()
n = rclpy.create_node('reset_monitor')
st = {'robot': None, 'reset_sim': None, 'checked1s': True, 'sim': 0.0, 'ep': 0, 'msg': None, 'hist': [], 'first_of_ep': True}


def on_clock(m):
    st['sim'] = m.clock.sec + m.clock.nanosec * 1e-9


def on_agents(m):
    st['msg'] = m
    pos = {ag.name: (ag.position.position.x, ag.position.position.y) for ag in m.agents}
    st['hist'].append((st['sim'], pos))
    st['hist'] = [h for h in st['hist'] if st['sim'] - h[0] < 15]
    if not st['checked1s'] and st['reset_sim'] is not None and st['sim'] - st['reset_sim'] >= 1.0 and m.agents:
        st['checked1s'] = True
        d = [math.dist(p_, starts[nm]) for nm, p_ in pos.items() if nm in starts]
        print(f"[ep {st['ep']}] {st['sim'] - st['reset_sim']:.1f}s after reset: {len(m.agents)} agents, max dist from scenario start "
              f"{max(d) if d else float('nan'):.2f} m, mean {sum(d)/max(len(d),1):.2f} m", flush=True)
        far = sorted(((math.dist(p_, starts[nm]), nm, p_) for nm, p_ in pos.items() if nm in starts), reverse=True)[:4]
        print('    farthest:', ', '.join(f"{nm} {dd:.2f}m at ({p_[0]:.1f},{p_[1]:.1f}) start {starts[nm]}" for dd, nm, p_ in far), flush=True)
    if st['first_of_ep'] and m.agents:
        st['first_of_ep'] = False
        d = [math.dist(p_, starts[nm]) for nm, p_ in pos.items() if nm in starts]
        print(f"[ep {st['ep']}] first non-empty human_states at sim {st['sim']:.1f}: {len(m.agents)} agents, "
              f"max dist from scenario start {max(d) if d else float('nan'):.2f} m (matched {len(d)})", flush=True)


ODOM = []


def on_odom(m):
    ODOM.append((st['sim'], (m.pose.pose.position.x, m.pose.pose.position.y)))
    del ODOM[:-400]
    old = [o for o in ODOM if st['sim'] - o[0] >= 1.0]
    if old:
        st['robot'] = old[-1][1]


def on_reset(m):
    r = st['robot']
    print(f"=== episode {st['ep']} ended at sim {st['sim']:.1f}, robot at {r[0]:.2f},{r[1]:.2f}, dist to goal (59,6) "
          f"{math.dist(r, (59.0, 6.0)):.2f} m" if r else "=== (no robot pose yet)", flush=True)
    st['reset_sim'] = st['sim']
    st['checked1s'] = False
    st['ep'] = m.data + 1
    st['first_of_ep'] = True
    st['hist'] = []
    print(f"=== reset counter {m.data} -> episode {st['ep']} at sim {st['sim']:.1f}", flush=True)


n.create_subscription(Odometry, '/task_generator_node/jackal/odom', on_odom, 10)
n.create_subscription(Clock, '/clock', on_clock, 10)
n.create_subscription(Agents, '/task_generator_node/human_states', on_agents, 10)
for t in ('/task_generator_node/task_reset', '/task_reset'):
    n.create_subscription(Int16, t, on_reset, 10)

t0 = time.time()
last = 0.0
while time.time() - t0 < a.duration:
    rclpy.spin_once(n, timeout_sec=0.2)
    if st['sim'] - last >= a.period:
        last = st['sim']
        m = st['msg']
        nag = len(m.agents) if m else -1
        moved = 0
        if st['hist'] and m:
            old = [h for h in st['hist'] if st['sim'] - h[0] >= 9.5]
            if old:
                ref = old[-1][1]
                cur = st['hist'][-1][1]
                moved = sum(1 for k, v in cur.items() if k in ref and math.dist(v, ref[k]) > 0.2)
        try:
            models = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True, timeout=20).stdout
            nm = sum(1 for l in models.splitlines() if l.strip().startswith('-'))
        except Exception:
            nm = -1
        print(f"[ep {st['ep']}] sim {st['sim']:.0f}s agents {nag} moved>0.2m/10s {moved} gz_models {nm}", flush=True)
