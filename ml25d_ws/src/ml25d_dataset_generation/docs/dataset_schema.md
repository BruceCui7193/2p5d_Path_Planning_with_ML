# Dataset schema

Each stored sample follows the tuple:

- X_map: float32 tensor with shape (31, 31, 6)
- theta_v: float32 vector with shape (12,)
- a: float32 vector with shape (4,)
- mu: float32 scalar packed as shape (1,)
- y: float32 vector with shape (7,)
- band: string in {safe, fail, critical}
- metadata_json: utf-8 JSON string

Channel order for X_map:

1. relative height Z
2. gradient along vehicle forward axis G_u
3. gradient along vehicle lateral axis G_v
4. local roughness R
5. current body footprint mask M_body
6. action swept area mask M_swept

Label vector order y:

1. y_fail
2. q_roll
3. q_pitch
4. q_slip
5. q_lift
6. p_bottom
7. p_stuck
