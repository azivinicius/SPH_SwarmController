import sys
for f in sys.argv[1:]:
    s = open(f).read()
    if "'sim_clock'" in s:
        print(f, 'already has sim_clock'); continue
    a, b = "    'test_duration': 60.0,\n}", ("    'test_duration': 60.0,\n    'max_velocity': 0.5,\n    'keepalive_s': 0.1,\n    'poll_period': 0.005,\n"
        "    'fence_xmin': float('nan'), 'fence_xmax': float('nan'),\n    'fence_ymin': float('nan'), 'fence_ymax': float('nan'),\n    'state_timeout_s': 0.0,\n}")
    c, d = "    'git_rev': 'unknown',\n}", ("    'git_rev': 'unknown',\n    'sim_clock': 'wall',\n    'gz_stats_topic': '',\n    'platform': 'sim',\n    'log_dir': '/root/dev_ws/src/logs',\n}")
    assert a in s and c in s, f + ': unexpected content, edit by hand'
    open(f, 'w').write(s.replace(a, b).replace(c, d)); print(f, 'patched')