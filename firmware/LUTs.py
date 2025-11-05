# 5°C-step LUT built from your NCU18XH103F60RB "Typ" Vout data,
# converted to 16-bit ADC counts assuming 3.3 V reference.
# Tuples are (adc_count, temp_c). Note: adc_count decreases as temp increases.

ADC_TO_TEMP_5C = [
    (58071, -40.0),
    (57644, -35.0),
    (57112, -30.0),
    (56460, -25.0),
    (55685, -20.0),
    (54785, -15.0),
    (53760, -10.0),
    (52610,  -5.0),
    (51335,   0.0),
    (49935,   5.0),
    (48410,  10.0),
    (46760,  15.0),
    (44985,  20.0),
    (43085,  25.0),
    (41062,  30.0),
    (38926,  35.0),
    (36683,  40.0),
    (34351,  45.0),
    (31955,  50.0),
    (29522,  55.0),
    (27083,  60.0),
    (24667,  65.0),
    (22303,  70.0),
    (20021,  75.0),
    (17850,  80.0),
    (15813,  85.0),
    (13932,  90.0),
    (12225,  95.0),
    (10701, 100.0),
    ( 9366, 105.0),
    ( 8216, 110.0),
    ( 7522, 115.0),
    ( 6777, 120.0),
    ( 6111, 125.0),
]

# Fan PWM policy: thresholds (rising) mapped to duty fractions.
# Hysteresis is handled in main code (±5°C around the current step).
TEMP_TO_DUTY = [
    (40.0, 0.35),
    (50.0, 0.50),
    (60.0, 0.65),
    (70.0, 0.80),
    (75.0, 1.00),
]
