import math


def _gen_parabola(phase: float, start: float, mid: float, end: float) -> float:
    """Gets a point on a parabola y = a x^2 + b x + c.

    The Parabola is determined by three points (0, start), (0.5, mid), (1, end) in
    the plane.

    Args:
    phase: Normalized to [0, 1]. A point on the x-axis of the parabola.
    start: The y value at x == 0.
    mid: The y value at x == 0.5.
    end: The y value at x == 1.

    Returns:
    The y value at x == phase.
    """
    mid_phase = 0.5
    delta_1 = mid - start
    delta_2 = end - start
    delta_3 = mid_phase**2 - mid_phase
    coef_a = (delta_1 - delta_2 * mid_phase) / delta_3
    coef_b = (delta_2 * mid_phase**2 - delta_1) / delta_3
    coef_c = start

    return coef_a * phase**2 + coef_b * phase + coef_c


def gen_swing_foot_trajectory(input_phase, start_pos,end_pos, max_clearance=0.011):
    """Generates the swing trajectory using a parabola.

    Args:
    input_phase: the swing/stance phase value between [0, 1].
    start_pos: The foot's position at the beginning of swing cycle.
    end_pos: The foot's desired position at the end of swing cycle.

    Returns:
    The desired foot position at the current phase.
    """
    # We augment the swing speed using the below formula. For the first half of
    # the swing cycle, the swing leg moves faster and finishes 80% of the full
    # swing trajectory. The rest 20% of trajectory takes another half swing
    # cycle. Intuitely, we want to move the swing foot quickly to the target
    # landing location and stay above the ground, in this way the control is more
    # robust to perturbations to the body that may cause the swing foot to drop
    # onto the ground earlier than expected. This is a common practice similar
    # to the MIT cheetah and Marc Raibert's original controllers.

    phase = input_phase
    # if input_phase <= 0.5:
    #   phase = 0.8 * math.sin(input_phase * math.pi)
    # else:
    #   phase = 0.8 + (input_phase - 0.5) * 0.4

    x = (1 - phase) * start_pos[0] + phase * end_pos[0]
    y = (1 - phase) * start_pos[1] + phase * end_pos[1]
    mid = max(end_pos[2], start_pos[2]) + max_clearance
    z = _gen_parabola(phase, start_pos[2], mid, end_pos[2])

    # print(x, start_pos[0], end_pos[0])

    # PyType detects the wrong return type here.
    return (x, y, z)  # pytype: disable=bad-return-type
