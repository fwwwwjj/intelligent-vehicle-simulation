# Contrast Experiment Report

Paired baseline/ablation analysis for the proposed stack (`proposed_fsm_lattice_lqr`). All numbers are produced from closed-loop simulation runs; no metric is hard-coded.

## Metadata

| Field | Value |
| --- | --- |
| Timestamp (UTC) | 2026-10-20T14:12:38+00:00 |
| Git commit | e0d7643a0941a651fb2ec8ffa712c77f0f3d26ff |
| Git dirty (excluding results/) | False |
| Python version | 3.13.1 |
| Duration per run (s) | 20.0 |
| Scenarios | highway_merge, cut_in, emergency_avoidance |
| Seeds | 42–71 (30 seeds) |
| Bootstrap resamples | 2000 |
| Bootstrap seed | 0 |
| Multiple-comparison alpha | 0.05 |


## Condition Definitions

| Condition | Family | Component | Decision | Planner | Controller | Tire | Dynamic | Experimental | Description |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| proposed_fsm_lattice_lqr | proposed | full | fsm | lattice | lqr | linear | False | False | Proposed full stack: hierarchical FSM decision + Frenet lattice planner + LQR controller + linear tire model. |
| baseline_idm_centerline_pure_pursuit | baseline | full_stack | idm | centerline | pure_pursuit | linear | False | False | Classical baseline stack: IDM car-following + centerline tracking + pure-pursuit controller. |
| ablation_decision_game | ablation | decision | game | lattice | lqr | linear | False | True | Decision ablation: replace the rule-based FSM with the game-theoretic decision maker. |
| ablation_decision_idm | ablation | decision | idm | lattice | lqr | linear | False | False | Decision ablation: replace the tactical FSM with IDM adaptive cruise (no lane-change intent). |
| ablation_decision_no_lane_change | ablation | decision | fsm_no_lane_change | lattice | lqr | linear | False | False | Decision ablation: HFSM with merge/lane-change intent disabled (lane keeping + car following only). |
| ablation_planner_em | ablation | planner | fsm | em | lqr | linear | False | True | Planner ablation: replace the lattice planner with the expectation-maximisation planner. |
| ablation_planner_centerline | ablation | planner | fsm | centerline | lqr | linear | False | False | Planner ablation: replace search/optimisation with pure centerline tracking (no obstacle awareness). |
| ablation_controller_pure_pursuit | ablation | controller | fsm | lattice | pure_pursuit | linear | False | False | Controller ablation: replace LQR tracking with a pure-pursuit controller. |
| ablation_controller_mpc | ablation | controller | fsm | lattice | mpc | linear | False | True | Controller ablation: replace LQR tracking with the MPC controller. |
| ablation_model_dynamic_2dof | ablation | vehicle_model | fsm | lattice | lqr | linear | True | False | Vehicle-model ablation: closed loop with the 2-DOF dynamic model instead of the kinematic model. |
| ablation_model_pacejka | ablation | vehicle_model | fsm | lattice | lqr | pacejka | True | True | Vehicle-model ablation: 2-DOF dynamic model with a Pacejka nonlinear tire model. |


## Success Rate by Condition and Scenario

Per-run success is binary (task completion and no collision and no road departure). Intervals are Wilson score 95% CIs.

| Condition | Scenario | n | Successes | Rate | 95% CI |
| --- | --- | --- | --- | --- | --- |
| ablation_controller_mpc | cut_in | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_controller_mpc | emergency_avoidance | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_controller_mpc | highway_merge | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_controller_pure_pursuit | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_controller_pure_pursuit | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_controller_pure_pursuit | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_game | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_game | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_game | highway_merge | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_decision_idm | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_idm | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_idm | highway_merge | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_decision_no_lane_change | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_no_lane_change | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_decision_no_lane_change | highway_merge | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_model_dynamic_2dof | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_model_dynamic_2dof | emergency_avoidance | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_model_dynamic_2dof | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_model_pacejka | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_model_pacejka | emergency_avoidance | 30 | 0 | 0.000 | [0.000, 0.114] |
| ablation_model_pacejka | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_centerline | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_centerline | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_centerline | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_em | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_em | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| ablation_planner_em | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |
| baseline_idm_centerline_pure_pursuit | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| baseline_idm_centerline_pure_pursuit | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| baseline_idm_centerline_pure_pursuit | highway_merge | 30 | 0 | 0.000 | [0.000, 0.114] |
| proposed_fsm_lattice_lqr | cut_in | 30 | 30 | 1.000 | [0.886, 1.000] |
| proposed_fsm_lattice_lqr | emergency_avoidance | 30 | 30 | 1.000 | [0.886, 1.000] |
| proposed_fsm_lattice_lqr | highway_merge | 30 | 30 | 1.000 | [0.886, 1.000] |


## Metric Summary by Condition and Scenario

Values are mean ± sample standard deviation (ddof=1). `min_ttc_s` uses only finite values below 50 s; completion time only completed runs (< 999 s sentinel).

| Condition | Scenario | min distance (m) | min TTC (s) | RMS jerk (m/s^3) | max lat accel (m/s^2) | max lat error (m) |
| --- | --- | --- | --- | --- | --- | --- |
| ablation_controller_mpc | cut_in | 0.023 ± 0.061 | 0.877 ± 0.726 | 0.868 ± 0.020 | 4.003 ± 0.000 | 0.067 ± 0.000 |
| ablation_controller_mpc | emergency_avoidance | 0.000 ± 0.000 | 0.029 ± 0.003 | 8.794 ± 0.412 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_controller_mpc | highway_merge | 1.804 ± 0.011 | — | 0.208 ± 0.003 | 3.845 ± 0.508 | 0.063 ± 0.005 |
| ablation_controller_pure_pursuit | cut_in | 5.030 ± 0.154 | 1.742 ± 0.040 | 2.406 ± 0.049 | 2.852 ± 0.148 | 0.062 ± 0.004 |
| ablation_controller_pure_pursuit | emergency_avoidance | 15.522 ± 0.099 | 2.371 ± 0.000 | 2.075 ± 0.036 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_controller_pure_pursuit | highway_merge | 1.850 ± 0.000 | 20.230 ± 0.171 | 2.722 ± 0.045 | 3.001 ± 0.000 | 0.060 ± 0.000 |
| ablation_decision_game | cut_in | 3.560 ± 0.176 | 1.748 ± 0.038 | 1.175 ± 0.005 | 2.510 ± 0.000 | 0.074 ± 0.003 |
| ablation_decision_game | emergency_avoidance | 15.392 ± 0.038 | 2.371 ± 0.000 | 1.752 ± 0.001 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_game | highway_merge | 1.823 ± 0.000 | — | 0.915 ± 0.005 | 2.504 ± 0.000 | 0.075 ± 0.002 |
| ablation_decision_idm | cut_in | 5.476 ± 0.168 | 1.745 ± 0.039 | 1.959 ± 0.001 | 2.510 ± 0.000 | 0.074 ± 0.003 |
| ablation_decision_idm | emergency_avoidance | 3.101 ± 0.007 | 2.270 ± 0.003 | 1.975 ± 0.001 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_idm | highway_merge | 1.850 ± 0.000 | — | 0.948 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_no_lane_change | cut_in | 5.466 ± 0.162 | 1.748 ± 0.038 | 1.833 ± 0.038 | 2.510 ± 0.000 | 0.074 ± 0.003 |
| ablation_decision_no_lane_change | emergency_avoidance | 14.110 ± 0.133 | 2.371 ± 0.000 | 1.803 ± 0.002 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_no_lane_change | highway_merge | 1.850 ± 0.000 | — | 0.948 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_dynamic_2dof | cut_in | 5.661 ± 0.162 | 1.740 ± 0.038 | 1.825 ± 0.040 | 2.514 ± 0.276 | 0.067 ± 0.002 |
| ablation_model_dynamic_2dof | emergency_avoidance | 3.221 ± 0.056 | 2.386 ± 0.000 | 1.838 ± 0.004 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_dynamic_2dof | highway_merge | 1.851 ± 0.000 | 20.402 ± 0.156 | 2.015 ± 0.019 | 2.168 ± 0.007 | 0.078 ± 0.002 |
| ablation_model_pacejka | cut_in | 5.661 ± 0.162 | 1.738 ± 0.037 | 1.825 ± 0.040 | 2.211 ± 0.047 | 0.067 ± 0.002 |
| ablation_model_pacejka | emergency_avoidance | 3.221 ± 0.056 | 2.386 ± 0.000 | 1.838 ± 0.004 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_pacejka | highway_merge | 1.850 ± 0.000 | 20.420 ± 0.149 | 2.017 ± 0.019 | 2.061 ± 0.015 | 0.079 ± 0.002 |
| ablation_planner_centerline | cut_in | 5.452 ± 0.163 | 1.751 ± 0.038 | 1.834 ± 0.038 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_centerline | emergency_avoidance | 14.110 ± 0.133 | 2.371 ± 0.000 | 1.803 ± 0.002 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_centerline | highway_merge | 1.835 ± 0.011 | 20.407 ± 0.123 | 2.004 ± 0.022 | 2.500 ± 0.001 | 0.060 ± 0.000 |
| ablation_planner_em | cut_in | 9.869 ± 0.017 | 3.485 ± 0.030 | 1.323 ± 0.005 | 2.486 ± 0.004 | 0.014 ± 0.002 |
| ablation_planner_em | emergency_avoidance | 18.644 ± 0.042 | 2.371 ± 0.000 | 1.929 ± 0.004 | 2.163 ± 0.205 | 0.002 ± 0.000 |
| ablation_planner_em | highway_merge | 1.833 ± 0.002 | 19.912 ± 0.161 | 2.022 ± 0.018 | 2.507 ± 0.003 | 0.573 ± 0.006 |
| baseline_idm_centerline_pure_pursuit | cut_in | 5.451 ± 0.168 | 1.748 ± 0.038 | 2.118 ± 0.016 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| baseline_idm_centerline_pure_pursuit | emergency_avoidance | 3.107 ± 0.006 | 2.371 ± 0.000 | 2.557 ± 0.055 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| baseline_idm_centerline_pure_pursuit | highway_merge | 1.850 ± 0.000 | — | 0.948 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| proposed_fsm_lattice_lqr | cut_in | 5.466 ± 0.162 | 1.748 ± 0.038 | 1.833 ± 0.038 | 2.510 ± 0.000 | 0.074 ± 0.003 |
| proposed_fsm_lattice_lqr | emergency_avoidance | 14.110 ± 0.133 | 2.371 ± 0.000 | 1.803 ± 0.002 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| proposed_fsm_lattice_lqr | highway_merge | 1.836 ± 0.010 | 20.410 ± 0.124 | 2.003 ± 0.021 | 2.501 ± 0.001 | 0.060 ± 0.000 |


| Condition | Scenario | avg plan latency (ms) | p95 plan latency (ms) | completion time (s) | collision frames | planner fallback rate | controller fallback rate |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ablation_controller_mpc | cut_in | 3.024 ± 0.031 | 5.325 ± 0.107 | — | 74.433 ± 63.279 | 0.000 ± 0.000 | 0.003 ± 0.001 |
| ablation_controller_mpc | emergency_avoidance | 2.285 ± 0.017 | 2.668 ± 0.040 | — | 285.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_controller_mpc | highway_merge | 4.674 ± 0.057 | 5.656 ± 0.167 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.003 ± 0.002 |
| ablation_controller_pure_pursuit | cut_in | 2.184 ± 0.020 | 5.387 ± 0.085 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_controller_pure_pursuit | emergency_avoidance | 1.713 ± 0.015 | 1.989 ± 0.037 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_controller_pure_pursuit | highway_merge | 2.391 ± 0.013 | 3.123 ± 0.032 | 11.868 ± 0.036 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_game | cut_in | 2.280 ± 0.016 | 5.629 ± 0.074 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_game | emergency_avoidance | 1.735 ± 0.014 | 1.963 ± 0.043 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_game | highway_merge | 5.282 ± 0.026 | 7.314 ± 0.063 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_idm | cut_in | 2.073 ± 0.014 | 5.282 ± 0.077 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_idm | emergency_avoidance | 2.099 ± 0.014 | 2.554 ± 0.032 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_idm | highway_merge | 2.008 ± 0.013 | 2.986 ± 0.034 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_no_lane_change | cut_in | 2.192 ± 0.018 | 5.409 ± 0.090 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_no_lane_change | emergency_avoidance | 1.731 ± 0.007 | 2.001 ± 0.029 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_decision_no_lane_change | highway_merge | 2.016 ± 0.031 | 3.003 ± 0.046 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_dynamic_2dof | cut_in | 2.199 ± 0.016 | 5.407 ± 0.081 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_dynamic_2dof | emergency_avoidance | 1.809 ± 0.021 | 2.328 ± 0.056 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_dynamic_2dof | highway_merge | 2.359 ± 0.016 | 3.099 ± 0.030 | 11.550 ± 0.035 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_pacejka | cut_in | 2.196 ± 0.021 | 5.422 ± 0.071 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_pacejka | emergency_avoidance | 1.804 ± 0.016 | 2.321 ± 0.047 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_model_pacejka | highway_merge | 2.362 ± 0.015 | 3.120 ± 0.037 | 11.642 ± 0.032 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_centerline | cut_in | 1.559 ± 0.016 | 1.764 ± 0.049 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_centerline | emergency_avoidance | 1.549 ± 0.016 | 1.785 ± 0.043 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_centerline | highway_merge | 1.613 ± 0.009 | 1.840 ± 0.040 | 11.712 ± 0.047 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_em | cut_in | 9.072 ± 0.091 | 14.857 ± 0.563 | — | 0.000 ± 0.000 | 0.000 ± 0.001 | 0.000 ± 0.000 |
| ablation_planner_em | emergency_avoidance | 10.972 ± 0.151 | 31.068 ± 0.969 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| ablation_planner_em | highway_merge | 20.101 ± 0.152 | 25.268 ± 0.328 | 10.590 ± 0.050 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| baseline_idm_centerline_pure_pursuit | cut_in | 1.538 ± 0.018 | 1.772 ± 0.055 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| baseline_idm_centerline_pure_pursuit | emergency_avoidance | 1.536 ± 0.014 | 1.734 ± 0.037 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| baseline_idm_centerline_pure_pursuit | highway_merge | 1.580 ± 0.012 | 1.786 ± 0.037 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| proposed_fsm_lattice_lqr | cut_in | 2.172 ± 0.020 | 5.345 ± 0.076 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| proposed_fsm_lattice_lqr | emergency_avoidance | 1.714 ± 0.018 | 1.921 ± 0.036 | — | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |
| proposed_fsm_lattice_lqr | highway_merge | 2.371 ± 0.018 | 3.097 ± 0.037 | 11.717 ± 0.040 | 0.000 ± 0.000 | 0.000 ± 0.000 | 0.000 ± 0.000 |


## Paired Comparisons vs Proposed

Pairs are matched on `(scenario, seed)`; the difference is always `condition - proposed`. Positive differences mean the ablation condition reported a larger value. `p` values are Holm-Bonferroni adjusted within each condition family (success + metrics); the `significant` column marks `p_holm < alpha`.

| Condition | pairs | proposed success | condition success | risk diff | b (proposed only) | c (condition only) | McNemar p | p (Holm) | significant |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ablation_controller_mpc | 90 | 1.000 | 0.000 | -1.000 | 90 | 0 | 0.0000 | 0.0000 | yes |
| ablation_controller_pure_pursuit | 90 | 1.000 | 1.000 | +0.000 | 0 | 0 | 1.0000 | 1.0000 | no |
| ablation_decision_game | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |
| ablation_decision_idm | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |
| ablation_decision_no_lane_change | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |
| ablation_model_dynamic_2dof | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |
| ablation_model_pacejka | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |
| ablation_planner_centerline | 90 | 1.000 | 1.000 | +0.000 | 0 | 0 | 1.0000 | 1.0000 | no |
| ablation_planner_em | 90 | 1.000 | 1.000 | +0.000 | 0 | 0 | 1.0000 | 1.0000 | no |
| baseline_idm_centerline_pure_pursuit | 90 | 1.000 | 0.667 | -0.333 | 30 | 0 | 0.0000 | 0.0000 | yes |


| Condition | Metric | pairs | proposed mean | condition mean | mean diff | 95% CI | Wilcoxon p | p (Holm) | Cliff's delta | effect | significant |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| ablation_controller_mpc | min obstacle distance (m) | 90 | 7.137 | 0.609 | -6.528 | [-7.702, -5.301] | 0.0000 | 0.0000 | -0.995 | large | yes |
| ablation_controller_mpc | min TTC (s) | 60 | 2.059 | 0.453 | -1.606 | [-1.826, -1.370] | 0.0000 | 0.0000 | -0.939 | large | yes |
| ablation_controller_mpc | RMS jerk (m/s^3) | 90 | 1.880 | 3.290 | +1.410 | [+0.591, +2.244] | 0.3815 | 0.7630 | -0.333 | medium | no |
| ablation_controller_mpc | max lateral accel (m/s^2) | 90 | 1.670 | 2.616 | +0.946 | [+0.793, +1.096] | 0.0000 | 0.0000 | +0.511 | large | yes |
| ablation_controller_mpc | max lateral error (m) | 90 | 0.045 | 0.043 | -0.001 | [-0.002, -0.000] | 0.0011 | 0.0032 | -0.029 | negligible | yes |
| ablation_controller_mpc | avg planning latency (ms) | 90 | 2.086 | 3.328 | +1.242 | [+1.091, +1.401] | 0.0000 | 0.0000 | +0.778 | large | yes |
| ablation_controller_mpc | p95 planning latency (ms) | 90 | 3.454 | 4.549 | +1.095 | [+0.877, +1.316] | 0.0000 | 0.0000 | +0.423 | medium | yes |
| ablation_controller_mpc | collision frames | 90 | 0.000 | 119.811 | +119.811 | [+93.559, +146.014] | 0.0000 | 0.0000 | +0.589 | large | yes |
| ablation_controller_mpc | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_controller_mpc | controller fallback rate | 90 | 0.000 | 0.002 | +0.002 | [+0.001, +0.002] | 0.0000 | 0.0000 | +0.556 | large | yes |
| ablation_controller_pure_pursuit | min obstacle distance (m) | 90 | 7.137 | 7.467 | +0.330 | [+0.169, +0.499] | 0.0068 | 0.0341 | +0.091 | negligible | yes |
| ablation_controller_pure_pursuit | min TTC (s) | 90 | 8.176 | 8.114 | -0.062 | [-0.088, -0.039] | 0.0000 | 0.0000 | -0.122 | negligible | yes |
| ablation_controller_pure_pursuit | RMS jerk (m/s^3) | 90 | 1.880 | 2.401 | +0.522 | [+0.483, +0.560] | 0.0000 | 0.0000 | +0.993 | large | yes |
| ablation_controller_pure_pursuit | max lateral accel (m/s^2) | 90 | 1.670 | 1.951 | +0.281 | [+0.235, +0.328] | 0.0000 | 0.0000 | +0.541 | large | yes |
| ablation_controller_pure_pursuit | max lateral error (m) | 90 | 0.045 | 0.041 | -0.004 | [-0.005, -0.003] | 0.0000 | 0.0000 | -0.100 | negligible | yes |
| ablation_controller_pure_pursuit | avg planning latency (ms) | 90 | 2.086 | 2.096 | +0.010 | [+0.005, +0.015] | 0.0006 | 0.0034 | +0.096 | negligible | yes |
| ablation_controller_pure_pursuit | p95 planning latency (ms) | 90 | 3.454 | 3.500 | +0.045 | [+0.027, +0.063] | 0.0000 | 0.0000 | +0.168 | small | yes |
| ablation_controller_pure_pursuit | completion time (s) | 30 | 11.717 | 11.868 | +0.152 | [+0.145, +0.158] | 0.0000 | 0.0000 | +0.991 | large | yes |
| ablation_controller_pure_pursuit | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_controller_pure_pursuit | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_controller_pure_pursuit | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_game | min obstacle distance (m) | 90 | 7.137 | 6.925 | -0.212 | [-0.477, +0.062] | 0.0078 | 0.0389 | -0.082 | negligible | yes |
| ablation_decision_game | min TTC (s) | 60 | 2.059 | 2.059 | -0.000 | [-0.000, +0.000] | 0.6272 | 1.0000 | -0.003 | negligible | no |
| ablation_decision_game | RMS jerk (m/s^3) | 90 | 1.880 | 1.281 | -0.599 | [-0.690, -0.512] | 0.0000 | 0.0000 | -1.000 | large | yes |
| ablation_decision_game | max lateral accel (m/s^2) | 90 | 1.670 | 1.671 | +0.001 | [+0.001, +0.002] | 0.0000 | 0.0000 | +0.111 | negligible | yes |
| ablation_decision_game | max lateral error (m) | 90 | 0.045 | 0.049 | +0.005 | [+0.004, +0.006] | 0.0000 | 0.0000 | +0.237 | small | yes |
| ablation_decision_game | avg planning latency (ms) | 90 | 2.086 | 3.099 | +1.013 | [+0.756, +1.294] | 0.0000 | 0.0000 | +0.302 | small | yes |
| ablation_decision_game | p95 planning latency (ms) | 90 | 3.454 | 4.969 | +1.514 | [+1.141, +1.910] | 0.0000 | 0.0000 | +0.506 | large | yes |
| ablation_decision_game | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_game | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_game | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_idm | min obstacle distance (m) | 90 | 7.137 | 3.476 | -3.662 | [-4.765, -2.563] | 0.2173 | 1.0000 | -0.192 | small | no |
| ablation_decision_idm | min TTC (s) | 60 | 2.059 | 2.007 | -0.052 | [-0.065, -0.040] | 0.0000 | 0.0000 | -0.344 | medium | yes |
| ablation_decision_idm | RMS jerk (m/s^3) | 90 | 1.880 | 1.628 | -0.252 | [-0.371, -0.142] | 0.3815 | 1.0000 | -0.079 | negligible | no |
| ablation_decision_idm | max lateral accel (m/s^2) | 90 | 1.670 | 0.837 | -0.834 | [-1.084, -0.611] | 0.0000 | 0.0000 | -0.222 | small | yes |
| ablation_decision_idm | max lateral error (m) | 90 | 0.045 | 0.025 | -0.020 | [-0.026, -0.015] | 0.0000 | 0.0000 | -0.222 | small | yes |
| ablation_decision_idm | avg planning latency (ms) | 90 | 2.086 | 2.060 | -0.026 | [-0.091, +0.038] | 0.9695 | 1.0000 | -0.333 | medium | no |
| ablation_decision_idm | p95 planning latency (ms) | 90 | 3.454 | 3.607 | +0.152 | [+0.080, +0.224] | 0.1669 | 1.0000 | -0.044 | negligible | no |
| ablation_decision_idm | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_idm | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_idm | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_no_lane_change | min obstacle distance (m) | 90 | 7.137 | 7.142 | +0.005 | [+0.003, +0.006] | 0.0000 | 0.0001 | +0.096 | negligible | yes |
| ablation_decision_no_lane_change | min TTC (s) | 60 | 2.059 | 2.059 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_no_lane_change | RMS jerk (m/s^3) | 90 | 1.880 | 1.528 | -0.351 | [-0.456, -0.257] | 0.0000 | 0.0000 | -0.556 | large | yes |
| ablation_decision_no_lane_change | max lateral accel (m/s^2) | 90 | 1.670 | 0.837 | -0.834 | [-1.084, -0.611] | 0.0000 | 0.0000 | -0.222 | small | yes |
| ablation_decision_no_lane_change | max lateral error (m) | 90 | 0.045 | 0.025 | -0.020 | [-0.026, -0.015] | 0.0000 | 0.0000 | -0.222 | small | yes |
| ablation_decision_no_lane_change | avg planning latency (ms) | 90 | 2.086 | 1.980 | -0.106 | [-0.144, -0.072] | 0.0686 | 0.4119 | -0.204 | small | no |
| ablation_decision_no_lane_change | p95 planning latency (ms) | 90 | 3.454 | 3.471 | +0.017 | [-0.004, +0.038] | 0.1596 | 0.7982 | +0.055 | negligible | no |
| ablation_decision_no_lane_change | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_no_lane_change | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_decision_no_lane_change | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_dynamic_2dof | min obstacle distance (m) | 90 | 7.137 | 3.578 | -3.559 | [-4.662, -2.468] | 0.3815 | 1.0000 | -0.169 | small | no |
| ablation_model_dynamic_2dof | min TTC (s) | 90 | 8.176 | 8.176 | -0.000 | [-0.019, +0.021] | 0.2022 | 1.0000 | +0.054 | negligible | no |
| ablation_model_dynamic_2dof | RMS jerk (m/s^3) | 90 | 1.880 | 1.892 | +0.013 | [+0.008, +0.018] | 0.0000 | 0.0000 | +0.199 | small | yes |
| ablation_model_dynamic_2dof | max lateral accel (m/s^2) | 90 | 1.670 | 1.561 | -0.110 | [-0.155, -0.063] | 0.0000 | 0.0000 | -0.222 | small | yes |
| ablation_model_dynamic_2dof | max lateral error (m) | 90 | 0.045 | 0.048 | +0.004 | [+0.002, +0.006] | 0.0009 | 0.0055 | +0.192 | small | yes |
| ablation_model_dynamic_2dof | avg planning latency (ms) | 90 | 2.086 | 2.122 | +0.037 | [+0.026, +0.047] | 0.0000 | 0.0000 | +0.149 | small | yes |
| ablation_model_dynamic_2dof | p95 planning latency (ms) | 90 | 3.454 | 3.611 | +0.157 | [+0.117, +0.196] | 0.0000 | 0.0000 | +0.158 | small | yes |
| ablation_model_dynamic_2dof | completion time (s) | 30 | 11.717 | 11.550 | -0.167 | [-0.188, -0.147] | 0.0000 | 0.0000 | -1.000 | large | yes |
| ablation_model_dynamic_2dof | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_dynamic_2dof | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_dynamic_2dof | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_pacejka | min obstacle distance (m) | 90 | 7.137 | 3.578 | -3.560 | [-4.662, -2.468] | 0.3815 | 1.0000 | -0.169 | small | no |
| ablation_model_pacejka | min TTC (s) | 90 | 8.176 | 8.181 | +0.005 | [-0.014, +0.027] | 0.1163 | 0.5817 | +0.061 | negligible | no |
| ablation_model_pacejka | RMS jerk (m/s^3) | 90 | 1.880 | 1.893 | +0.014 | [+0.008, +0.019] | 0.0000 | 0.0000 | +0.208 | small | yes |
| ablation_model_pacejka | max lateral accel (m/s^2) | 90 | 1.670 | 1.424 | -0.246 | [-0.286, -0.208] | 0.0000 | 0.0000 | -0.444 | medium | yes |
| ablation_model_pacejka | max lateral error (m) | 90 | 0.045 | 0.049 | +0.004 | [+0.002, +0.007] | 0.0009 | 0.0054 | +0.219 | small | yes |
| ablation_model_pacejka | avg planning latency (ms) | 90 | 2.086 | 2.121 | +0.035 | [+0.026, +0.045] | 0.0000 | 0.0000 | +0.153 | small | yes |
| ablation_model_pacejka | p95 planning latency (ms) | 90 | 3.454 | 3.621 | +0.166 | [+0.128, +0.203] | 0.0000 | 0.0000 | +0.217 | small | yes |
| ablation_model_pacejka | completion time (s) | 30 | 11.717 | 11.642 | -0.075 | [-0.095, -0.057] | 0.0000 | 0.0001 | -0.856 | large | yes |
| ablation_model_pacejka | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_pacejka | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_model_pacejka | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_planner_centerline | min obstacle distance (m) | 90 | 7.137 | 7.132 | -0.005 | [-0.007, -0.003] | 0.0001 | 0.0009 | -0.060 | negligible | yes |
| ablation_planner_centerline | min TTC (s) | 90 | 8.176 | 8.176 | +0.000 | [-0.002, +0.002] | 0.0006 | 0.0034 | +0.051 | negligible | yes |
| ablation_planner_centerline | RMS jerk (m/s^3) | 90 | 1.880 | 1.881 | +0.001 | [+0.000, +0.003] | 0.0000 | 0.0001 | +0.013 | negligible | yes |
| ablation_planner_centerline | max lateral accel (m/s^2) | 90 | 1.670 | 0.833 | -0.837 | [-1.088, -0.586] | 0.0000 | 0.0000 | -0.483 | large | yes |
| ablation_planner_centerline | max lateral error (m) | 90 | 0.045 | 0.020 | -0.025 | [-0.032, -0.017] | 0.0000 | 0.0000 | -0.444 | medium | yes |
| ablation_planner_centerline | avg planning latency (ms) | 90 | 2.086 | 1.574 | -0.512 | [-0.564, -0.460] | 0.0000 | 0.0000 | -1.000 | large | yes |
| ablation_planner_centerline | p95 planning latency (ms) | 90 | 3.454 | 1.796 | -1.658 | [-1.953, -1.356] | 0.0000 | 0.0000 | -0.985 | large | yes |
| ablation_planner_centerline | completion time (s) | 30 | 11.717 | 11.712 | -0.005 | [-0.013, +0.003] | 0.2568 | 1.0000 | -0.071 | negligible | no |
| ablation_planner_centerline | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_planner_centerline | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_planner_centerline | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_planner_em | min obstacle distance (m) | 90 | 7.137 | 10.115 | +2.978 | [+2.543, +3.383] | 0.0000 | 0.0000 | +0.176 | small | yes |
| ablation_planner_em | min TTC (s) | 90 | 8.176 | 8.589 | +0.413 | [+0.215, +0.610] | 0.3815 | 1.0000 | +0.111 | negligible | no |
| ablation_planner_em | RMS jerk (m/s^3) | 90 | 1.880 | 1.758 | -0.122 | [-0.180, -0.062] | 0.3685 | 1.0000 | +0.058 | negligible | no |
| ablation_planner_em | max lateral accel (m/s^2) | 90 | 1.670 | 2.385 | +0.715 | [+0.502, +0.934] | 0.0060 | 0.0482 | -0.067 | negligible | yes |
| ablation_planner_em | max lateral error (m) | 90 | 0.045 | 0.196 | +0.152 | [+0.103, +0.205] | 0.0060 | 0.0482 | +0.111 | negligible | yes |
| ablation_planner_em | avg planning latency (ms) | 90 | 2.086 | 13.382 | +11.296 | [+10.386, +12.245] | 0.0000 | 0.0000 | +1.000 | large | yes |
| ablation_planner_em | p95 planning latency (ms) | 90 | 3.454 | 23.731 | +20.277 | [+18.608, +22.007] | 0.0000 | 0.0000 | +1.000 | large | yes |
| ablation_planner_em | completion time (s) | 30 | 11.717 | 10.590 | -1.127 | [-1.140, -1.115] | 0.0000 | 0.0000 | -1.000 | large | yes |
| ablation_planner_em | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| ablation_planner_em | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 0.1573 | 0.9438 | +0.022 | negligible | no |
| ablation_planner_em | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| baseline_idm_centerline_pure_pursuit | min obstacle distance (m) | 90 | 7.137 | 3.470 | -3.668 | [-4.768, -2.571] | 0.0039 | 0.0194 | -0.252 | small | yes |
| baseline_idm_centerline_pure_pursuit | min TTC (s) | 60 | 2.059 | 2.059 | -0.000 | [-0.003, +0.002] | 0.0000 | 0.0002 | +0.104 | negligible | yes |
| baseline_idm_centerline_pure_pursuit | RMS jerk (m/s^3) | 90 | 1.880 | 1.875 | -0.005 | [-0.169, +0.146] | 0.3815 | 1.0000 | +0.333 | medium | no |
| baseline_idm_centerline_pure_pursuit | max lateral accel (m/s^2) | 90 | 1.670 | 0.000 | -1.670 | [-1.920, -1.420] | 0.0000 | 0.0000 | -0.556 | large | yes |
| baseline_idm_centerline_pure_pursuit | max lateral error (m) | 90 | 0.045 | 0.000 | -0.045 | [-0.051, -0.038] | 0.0000 | 0.0000 | -0.556 | large | yes |
| baseline_idm_centerline_pure_pursuit | avg planning latency (ms) | 90 | 2.086 | 1.551 | -0.534 | [-0.589, -0.480] | 0.0000 | 0.0000 | -1.000 | large | yes |
| baseline_idm_centerline_pure_pursuit | p95 planning latency (ms) | 90 | 3.454 | 1.764 | -1.690 | [-1.978, -1.393] | 0.0000 | 0.0000 | -0.993 | large | yes |
| baseline_idm_centerline_pure_pursuit | collision frames | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| baseline_idm_centerline_pure_pursuit | planner fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |
| baseline_idm_centerline_pure_pursuit | controller fallback rate | 90 | 0.000 | 0.000 | +0.000 | [+0.000, +0.000] | 1.0000 | 1.0000 | +0.000 | negligible | no |


## Interpretation Notes

- **ablation_controller_mpc (experimental condition)**: success rate is significantly lower than proposed (risk diff -1.000) — degradation. Holm-significant metric changes: min obstacle distance (m) worsened (mean diff -6.528, Holm p=0.0000, Cliff's delta=-0.995); min TTC (s) worsened (mean diff -1.606, Holm p=0.0000, Cliff's delta=-0.939); max lateral accel (m/s^2) worsened (mean diff +0.946, Holm p=0.0000, Cliff's delta=+0.511); max lateral error (m) improved (mean diff -0.001, Holm p=0.0032, Cliff's delta=-0.029); avg planning latency (ms) worsened (mean diff +1.242, Holm p=0.0000, Cliff's delta=+0.778); p95 planning latency (ms) worsened (mean diff +1.095, Holm p=0.0000, Cliff's delta=+0.423); collision frames worsened (mean diff +119.811, Holm p=0.0000, Cliff's delta=+0.589); controller fallback rate worsened (mean diff +0.002, Holm p=0.0000, Cliff's delta=+0.556).
- **ablation_controller_pure_pursuit**: success rate difference (risk diff +0.000) is not significant after Holm correction. Holm-significant metric changes: min obstacle distance (m) improved (mean diff +0.330, Holm p=0.0341, Cliff's delta=+0.091); min TTC (s) worsened (mean diff -0.062, Holm p=0.0000, Cliff's delta=-0.122); RMS jerk (m/s^3) worsened (mean diff +0.522, Holm p=0.0000, Cliff's delta=+0.993); max lateral accel (m/s^2) worsened (mean diff +0.281, Holm p=0.0000, Cliff's delta=+0.541); max lateral error (m) improved (mean diff -0.004, Holm p=0.0000, Cliff's delta=-0.100); avg planning latency (ms) worsened (mean diff +0.010, Holm p=0.0034, Cliff's delta=+0.096); p95 planning latency (ms) worsened (mean diff +0.045, Holm p=0.0000, Cliff's delta=+0.168); completion time (s) worsened (mean diff +0.152, Holm p=0.0000, Cliff's delta=+0.991).
- **ablation_decision_game (experimental condition)**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: min obstacle distance (m) worsened (mean diff -0.212, Holm p=0.0389, Cliff's delta=-0.082); RMS jerk (m/s^3) improved (mean diff -0.599, Holm p=0.0000, Cliff's delta=-1.000); max lateral accel (m/s^2) worsened (mean diff +0.001, Holm p=0.0000, Cliff's delta=+0.111); max lateral error (m) worsened (mean diff +0.005, Holm p=0.0000, Cliff's delta=+0.237); avg planning latency (ms) worsened (mean diff +1.013, Holm p=0.0000, Cliff's delta=+0.302); p95 planning latency (ms) worsened (mean diff +1.514, Holm p=0.0000, Cliff's delta=+0.506).
- **ablation_decision_idm**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: min TTC (s) worsened (mean diff -0.052, Holm p=0.0000, Cliff's delta=-0.344); max lateral accel (m/s^2) improved (mean diff -0.834, Holm p=0.0000, Cliff's delta=-0.222); max lateral error (m) improved (mean diff -0.020, Holm p=0.0000, Cliff's delta=-0.222).
- **ablation_decision_no_lane_change**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: min obstacle distance (m) improved (mean diff +0.005, Holm p=0.0001, Cliff's delta=+0.096); RMS jerk (m/s^3) improved (mean diff -0.351, Holm p=0.0000, Cliff's delta=-0.556); max lateral accel (m/s^2) improved (mean diff -0.834, Holm p=0.0000, Cliff's delta=-0.222); max lateral error (m) improved (mean diff -0.020, Holm p=0.0000, Cliff's delta=-0.222).
- **ablation_model_dynamic_2dof**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: RMS jerk (m/s^3) worsened (mean diff +0.013, Holm p=0.0000, Cliff's delta=+0.199); max lateral accel (m/s^2) improved (mean diff -0.110, Holm p=0.0000, Cliff's delta=-0.222); max lateral error (m) worsened (mean diff +0.004, Holm p=0.0055, Cliff's delta=+0.192); avg planning latency (ms) worsened (mean diff +0.037, Holm p=0.0000, Cliff's delta=+0.149); p95 planning latency (ms) worsened (mean diff +0.157, Holm p=0.0000, Cliff's delta=+0.158); completion time (s) improved (mean diff -0.167, Holm p=0.0000, Cliff's delta=-1.000).
- **ablation_model_pacejka (experimental condition)**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: RMS jerk (m/s^3) worsened (mean diff +0.014, Holm p=0.0000, Cliff's delta=+0.208); max lateral accel (m/s^2) improved (mean diff -0.246, Holm p=0.0000, Cliff's delta=-0.444); max lateral error (m) worsened (mean diff +0.004, Holm p=0.0054, Cliff's delta=+0.219); avg planning latency (ms) worsened (mean diff +0.035, Holm p=0.0000, Cliff's delta=+0.153); p95 planning latency (ms) worsened (mean diff +0.166, Holm p=0.0000, Cliff's delta=+0.217); completion time (s) improved (mean diff -0.075, Holm p=0.0001, Cliff's delta=-0.856).
- **ablation_planner_centerline**: success rate difference (risk diff +0.000) is not significant after Holm correction. Holm-significant metric changes: min obstacle distance (m) worsened (mean diff -0.005, Holm p=0.0009, Cliff's delta=-0.060); min TTC (s) improved (mean diff +0.000, Holm p=0.0034, Cliff's delta=+0.051); RMS jerk (m/s^3) worsened (mean diff +0.001, Holm p=0.0001, Cliff's delta=+0.013); max lateral accel (m/s^2) improved (mean diff -0.837, Holm p=0.0000, Cliff's delta=-0.483); max lateral error (m) improved (mean diff -0.025, Holm p=0.0000, Cliff's delta=-0.444); avg planning latency (ms) improved (mean diff -0.512, Holm p=0.0000, Cliff's delta=-1.000); p95 planning latency (ms) improved (mean diff -1.658, Holm p=0.0000, Cliff's delta=-0.985).
- **ablation_planner_em (experimental condition)**: success rate difference (risk diff +0.000) is not significant after Holm correction. Holm-significant metric changes: min obstacle distance (m) improved (mean diff +2.978, Holm p=0.0000, Cliff's delta=+0.176); max lateral accel (m/s^2) worsened (mean diff +0.715, Holm p=0.0482, Cliff's delta=-0.067); max lateral error (m) worsened (mean diff +0.152, Holm p=0.0482, Cliff's delta=+0.111); avg planning latency (ms) worsened (mean diff +11.296, Holm p=0.0000, Cliff's delta=+1.000); p95 planning latency (ms) worsened (mean diff +20.277, Holm p=0.0000, Cliff's delta=+1.000); completion time (s) improved (mean diff -1.127, Holm p=0.0000, Cliff's delta=-1.000).
- **baseline_idm_centerline_pure_pursuit**: success rate is significantly lower than proposed (risk diff -0.333) — degradation. Holm-significant metric changes: min obstacle distance (m) worsened (mean diff -3.668, Holm p=0.0194, Cliff's delta=-0.252); min TTC (s) worsened (mean diff -0.000, Holm p=0.0002, Cliff's delta=+0.104); max lateral accel (m/s^2) improved (mean diff -1.670, Holm p=0.0000, Cliff's delta=-0.556); max lateral error (m) improved (mean diff -0.045, Holm p=0.0000, Cliff's delta=-0.556); avg planning latency (ms) improved (mean diff -0.534, Holm p=0.0000, Cliff's delta=-1.000); p95 planning latency (ms) improved (mean diff -1.690, Holm p=0.0000, Cliff's delta=-0.993).

Direction reminders: higher `min_obstacle_distance_m` and `min_ttc_s` are safer; lower `rms_jerk_m_s3`, `max_lateral_accel_m_s2`, `max_lateral_error_m`, planning latency, collision frames and fallback rates are better. Completion time is conditional on completing the task.

## Method Notes

- **Success interval**: Wilson score interval at 95%, computed from the per-run binary success count.
- **McNemar exact test**: two-sided exact binomial test on discordant pairs with p=0.5; `b` = proposed succeeds / condition fails, `c` = proposed fails / condition succeeds.
- **Paired bootstrap**: resamples matched differences `condition - proposed` with replacement (default 2000 resamples), percentile 95% CI, seeded for reproducibility.
- **Wilcoxon signed-rank**: two-sided, normal approximation with tie and zero-difference correction; used as the metric significance test.
- **Cliff's delta**: non-parametric effect size `P(condition > proposed) - P(condition < proposed)`; magnitudes follow Romano et al. (<0.147 negligible, <0.33 small, <0.474 medium, else large).
- **Multiple comparisons**: Holm-Bonferroni (step-down) within each condition family; reported adjusted p-values are monotone and capped at 1.
