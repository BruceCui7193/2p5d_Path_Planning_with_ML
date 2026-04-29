## Mini Batch Repro Check

### stage1_n1
| first_run_fail_rate | replay_fail_rate | label_mismatch_rate | terrain_H_std | terrain_H_range |
| --- | --- | --- | --- | --- |
| 0.000000 | 0.000000 | 1.000000 | 0.000000e+00 | 0.000000e+00 |

| ok_rows | runtime_failure_rows | sample_start_time_min | sample_start_time_max | message_time_min | message_time_max |
| --- | --- | --- | --- | --- | --- |
| 9 | 51 | 3.140000 | 3.272000 | 3.140000 | 5.272000 |

Mismatch rows:
| sample_id | seed | action_id | mu | first_q_roll | first_q_pitch | first_q_lift | first_p_bottom | first_p_stuck | replay_q_roll | replay_q_pitch | replay_q_lift | replay_p_bottom | replay_p_stuck |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 1438246031 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 1 | 718890214 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 2 | 536675920 | a2 | 0.800 | 0.007400 | 0.005247 | 0.000000 | 0.000000 | 0.000000 | 0.011462 | 0.010764 | 0.000000 | 0.000000 | 0.000000 |
| 3 | 866457609 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 4 | 740044766 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 5 | 936811205 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 6 | 989671484 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 7 | 102683776 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 8 | 124667575 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 9 | 1863621002 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 10 | 1588628555 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 11 | 1371703680 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 12 | 1024079646 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 13 | 1217993871 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 14 | 448109743 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 15 | 20353054 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 16 | 1417759681 | a1 | 0.800 | 0.008500 | 0.008823 | 0.000000 | 0.000000 | 0.000000 | 0.008544 | 0.007581 | 0.000000 | 0.000000 | 0.000000 |
| 17 | 503632434 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 18 | 933559175 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 19 | 1529487126 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 20 | 35800937 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 21 | 2014747107 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 22 | 336730988 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 23 | 882755250 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 24 | 216258061 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 25 | 1405852644 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 26 | 1494922366 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 27 | 894864454 | a0 | 0.800 | 0.004774 | 0.006711 | 0.000000 | 0.000000 | 0.000000 | 0.005208 | 0.006742 | 0.000000 | 0.000000 | 0.000000 |
| 28 | 347861919 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 29 | 530840898 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 30 | 1991570991 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 31 | 263300534 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 32 | 489258893 | a2 | 0.800 | 0.005676 | 0.007084 | 0.000000 | 0.000000 | 0.000000 | 0.008055 | 0.004685 | 0.000000 | 0.000000 | 0.000000 |
| 33 | 1596532301 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 34 | 981091937 | a1 | 0.800 | 0.007563 | 0.006628 | 0.000000 | 0.000000 | 0.000000 | 0.006770 | 0.008085 | 0.000000 | 0.000000 | 0.000000 |
| 35 | 1099736371 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 36 | 502463890 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 37 | 995910252 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 38 | 2083221442 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 39 | 603262983 | a0 | 0.800 | 0.008222 | 0.006684 | 0.000000 | 0.000000 | 0.000000 | 0.004961 | 0.007479 | 0.000000 | 0.000000 | 0.000000 |
| 40 | 25246691 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 41 | 908465506 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 42 | 808918570 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 43 | 854687927 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 44 | 1109819081 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 45 | 58215051 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 46 | 289694847 | a1 | 0.800 | 0.007485 | 0.006607 | 0.000000 | 0.000000 | 0.000000 | 0.009405 | 0.007046 | 0.000000 | 0.000000 | 0.000000 |
| 47 | 556704764 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 48 | 1123345499 | a0 | 0.800 | 0.006052 | 0.005546 | 0.000000 | 0.000000 | 0.000000 | 0.007828 | 0.005374 | 0.000000 | 0.000000 | 0.000000 |
| 49 | 1578309723 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 50 | 23027976 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 51 | 1254850317 | a0 | 0.800 | 0.005540 | 0.004980 | 0.000000 | 0.000000 | 0.000000 | 0.006474 | 0.005704 | 0.000000 | 0.000000 | 0.000000 |
| 52 | 528115566 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 53 | 605659116 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 54 | 1778895647 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 55 | 527889521 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 56 | 792247550 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 57 | 1469647704 | a0 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 58 | 1303655034 | a1 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
| 59 | 1368330998 | a2 | 0.800 | nan | nan | nan | nan | nan | nan | nan | nan | nan | nan |
