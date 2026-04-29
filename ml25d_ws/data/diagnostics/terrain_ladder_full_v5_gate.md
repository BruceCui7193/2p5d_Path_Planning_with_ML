| check | pass | evidence |
| --- | --- | --- |
| flat_forward all vehicles fail<=0.05 | PASS | city_small:0.00, offroad_medium:0.00, mountain_large:0.00 |
| low_bump all vehicles fail<=0.05 | PASS | city_small:0.00, offroad_medium:0.00, mountain_large:0.00 |
| max invalid_sample_rate < 0.05 | PASS | max=0.000 |
| city_small: q_pitch(5)<q_pitch(10)<q_pitch(20) | PASS | [0.2387, 0.4695, 0.9157] |
| offroad_medium: q_pitch(5)<q_pitch(10)<q_pitch(20) | PASS | [0.2029, 0.4046, 0.8063] |
| mountain_large: q_pitch(5)<q_pitch(10)<q_pitch(20) | PASS | [0.1824, 0.363, 0.7486] |
| city_small: q_roll(10)<q_roll(20) | PASS | [0.5235, 1.0] |
| offroad_medium: q_roll(10)<q_roll(20) | PASS | [0.4818, 0.9959] |
| mountain_large: q_roll(10)<q_roll(20) | PASS | [0.5134, 1.0] |
| step_15cm: city fail > mountain fail | PASS | city=1.00, mountain=0.00 |
| step_15cm: city bottom > mountain bottom | PASS | city=1.00, mountain=0.00 |
| city_small: q_lift pit_small<=pit_medium<=pit_large | PASS | [0.195, 0.2647, 0.6435] |
| offroad_medium: q_lift pit_small<=pit_medium<=pit_large | PASS | [0.0225, 0.0967, 0.141] |
| mountain_large: q_lift pit_small<=pit_medium<=pit_large | PASS | [0.057, 0.1525, 0.1552] |
| avg fail city > offroad > mountain | PASS | city=0.533, offroad=0.428, mountain=0.311 |

Final gate: PASS
