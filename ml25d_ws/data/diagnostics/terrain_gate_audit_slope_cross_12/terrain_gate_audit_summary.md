# Terrain Gate Audit

sample_count=12, combos=6, repeats=2

| vehicle_type | terrain_type | gate_pass_rate | initialization_invalid_rate | settle_time_mean | settle_time_p95 | roll_expected_mean | roll_actual_mean | roll_error_world_mean | roll_error_world_p95 | roll_error_gate_mean | roll_error_gate_p95 | pitch_expected_mean | pitch_actual_mean | pitch_error_world_mean | pitch_error_world_p95 | pitch_error_gate_mean | pitch_error_gate_p95 | linear_speed_mean | angular_speed_mean | bottom_before_action_rate | lift_before_action_rate | message_time_valid_rate |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| city_small | slope_forward_20deg | 1.0000 | 0.0000 | 0.1480 | 0.1482 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | -24.4638 | 0.0000 | 24.4638 | 24.4638 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| city_small | cross_slope_20deg | 1.0000 | 0.0000 | 0.1482 | 0.1482 | 15.0886 | 0.0000 | 15.0886 | 15.0886 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0002 | 0.0067 | 0.0000 | 0.0000 | 1.0000 |
| offroad_medium | slope_forward_20deg | 1.0000 | 0.0000 | 0.1883 | 0.1886 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | -16.8730 | 0.0000 | 16.8730 | 16.8730 | 0.0000 | 0.0000 | 0.0002 | 0.0061 | 0.0000 | 0.0000 | 1.0000 |
| offroad_medium | cross_slope_20deg | 1.0000 | 0.0000 | 0.1924 | 0.1924 | 19.7723 | 0.0000 | 19.7723 | 19.7723 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0001 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| mountain_large | slope_forward_20deg | 1.0000 | 0.0000 | 0.2128 | 0.2128 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | -19.4038 | 0.0000 | 19.4038 | 19.4038 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |
| mountain_large | cross_slope_20deg | 1.0000 | 0.0000 | 0.2088 | 0.2090 | 16.3895 | 0.0000 | 16.3895 | 16.3895 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 0.0000 | 1.0000 |