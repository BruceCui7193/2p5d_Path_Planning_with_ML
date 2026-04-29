# Experiment 1

| trial | seed | spawn_z | roll_mean | roll_max | pitch_mean | pitch_max | bottom_contact_rate | fl_contact_rate | fr_contact_rate | rl_contact_rate | rr_contact_rate | terrain_already_exists | fail_reason |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 101 | 0.0300 | 0.0005 | 0.0011 | 0.0002 | 0.0004 | 0.0000 | 0.1200 | 0.1200 | 0.1200 | 0.1100 | False | - |
| 2 | 102 | 0.0300 | 0.0005 | 0.0011 | 0.0002 | 0.0004 | 0.0000 | 0.1200 | 0.1200 | 0.1100 | 0.1100 | False | - |
| 3 | 103 | 0.0300 | 0.0006 | 0.0011 | 0.0002 | 0.0004 | 0.0000 | 0.1200 | 0.1200 | 0.1100 | 0.1200 | False | - |
| 4 | 104 | 0.0300 | 0.0006 | 0.0011 | 0.0002 | 0.0004 | 0.0000 | 0.1100 | 0.1200 | 0.1200 | 0.1200 | False | - |


# Experiment 2

| trial | seed | progress | cmd_mean | act_forward_mean | roll_max | pitch_max | bottom_contact | stuck_fail | final_fail | fail_reason |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 201 | 0.9857 | 0.1500 | 0.1477 | 0.0003 | 0.0010 | False | False | False | - |
| 2 | 202 | 0.9821 | 0.1500 | 0.1471 | 0.0005 | 0.0023 | False | False | False | - |
| 3 | 203 | NaN | NaN | NaN | NaN | NaN | NaN | NaN | True | no wheel contact sensor samples received from Gazebo snapshot={} |
| 4 | 204 | 0.9904 | 0.1500 | 0.1478 | 0.0005 | 0.0017 | False | False | False | - |


# Experiment 3

| trial | seed | slip_max | slip_p95 | slip_median | cmd_mean | act_mean | valid_slip_sample_count |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 201 | 0.0109 | 0.0058 | 0.0021 | 0.1500 | 0.1497 | 80 |
| 2 | 202 | 0.0181 | 0.0089 | 0.0017 | 0.1500 | 0.1497 | 80 |
| 4 | 204 | 0.0175 | 0.0100 | 0.0031 | 0.1500 | 0.1495 | 80 |


# Experiment 4

| trial | seed | fl_contact_observed_rate | fr_contact_observed_rate | rl_contact_observed_rate | rr_contact_observed_rate | lift_unknown_rate | lift_rate_if_observed_only | original_q_lift | corrected_q_lift |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 201 | 0.1200 | 0.1200 | 0.1200 | 0.1100 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 2 | 202 | 0.1200 | 0.1100 | 0.1100 | 0.1200 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 4 | 204 | 0.1100 | 0.1100 | 0.1200 | 0.1200 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |


# Experiment 3 compare

| slip_max_mean | slip_p95_mean | slip_max_minus_p95_mean |
| --- | --- | --- |
| 0.0155 | 0.0082 | 0.0073 |