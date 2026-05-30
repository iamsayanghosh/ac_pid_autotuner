"""
ac_pid_autotuner.py
-------------------
PID temperature controller simulation with Twiddle-based auto-tuning.

Simulates a first-order thermal system subject to a sinusoidal ambient
disturbance. The Twiddle algorithm optimises Kp, Ki, Kd to minimise a
cost function that penalises both steady-state error and energy use.

Usage:
    python ac_pid_autotuner.py
"""

import math
import matplotlib.pyplot as plt


# ── shared simulation core ───────────────────────────────────────────────────

def _run_sim(Kp, Ki, Kd, Tset, record=False):
    """
    Core simulation loop for a first-order thermal system with PID control.

    The plant model is:
        dT/dt = (Tamb - T) / tau + k * u

    where Tamb varies sinusoidally to simulate environmental disturbance.
    Anti-windup is applied: the integrator only accumulates when the
    actuator is not saturated.

    Parameters
    ----------
    Kp : float
        Proportional gain.
    Ki : float
        Integral gain.
    Kd : float
        Derivative gain.
    Tset : float
        Desired setpoint temperature (°C).
    record : bool
        If True, returns time-series lists for plotting.
        If False, returns the scalar cost (used during optimisation).

    Returns
    -------
    record=True  : tuple (time_l, temp_l, e_l, pwr_l, nrg_l)
    record=False : float — total cost = integral(e²) + 0.2 * total_energy
    """
    T = 35          # Initial temperature (°C)
    t = 0           # Simulation time (s)
    tau = 100       # Thermal time constant of the system (s)
    k = 0.3         # Actuator gain (°C/s per unit control output)
    dt = 0.1        # Integration time step (s)
    totalsim = 500  # Total simulation duration (s)
    U_MAX = 10.0    # Actuator saturation limit (±)

    e = Tset - T
    prev_e = e
    integ = 0           # Integrator state
    energy = 0          # Cumulative energy (u² · dt)
    total_error = 0     # Cumulative squared error (e² · dt)

    if record:
        time_l, temp_l, e_l, pwr_l, nrg_l = [t], [T], [e], [0], [0]

    while t < totalsim:
        # Sinusoidal ambient temperature simulates external disturbance
        Tamb = 40 + 3 * math.sin(0.01 * t)

        e = Tset - T
        de = (e - prev_e) / dt  # Derivative of error

        # Raw PID output
        u_raw = Kp * e + Ki * integ + Kd * de

        # Clamp actuator output to physical limits
        u = max(-U_MAX, min(U_MAX, u_raw))

        # Anti-windup: only integrate when actuator is not saturated
        if abs(u_raw) < U_MAX:
            integ += e * dt

        # First-order plant update (Euler integration)
        dTdt = (Tamb - T) / tau + k * u
        T += dTdt * dt
        t += dt

        power = u * u           # Instantaneous power (proportional to u²)
        energy += power * dt
        prev_e = e
        e = Tset - T
        total_error += e * e * dt

        if record:
            time_l.append(t)
            temp_l.append(T)
            pwr_l.append(power)
            nrg_l.append(energy)
            e_l.append(e)

    if record:
        return time_l, temp_l, e_l, pwr_l, nrg_l

    # Cost balances tracking accuracy against energy consumption
    return total_error + 0.2 * energy


# ── public API ───────────────────────────────────────────────────────────────

def temp_sim(Kp, Ki, Kd, Tset):
    """
    Run the simulation and return full time-series data for plotting.

    Parameters
    ----------
    Kp, Ki, Kd : float
        PID gains.
    Tset : float
        Setpoint temperature (°C).

    Returns
    -------
    tuple : (time_l, temp_l, e_l, pwr_l, nrg_l)
        time_l  — simulation time points (s)
        temp_l  — temperature at each time step (°C)
        e_l     — tracking error at each time step (°C)
        pwr_l   — instantaneous power (u²) at each time step
        nrg_l   — cumulative energy up to each time step
    """
    return _run_sim(Kp, Ki, Kd, Tset, record=True)


def sim_cost(Kp, Ki, Kd, Tset):
    """
    Run the simulation and return only the scalar cost.

    Used by the Twiddle optimiser to evaluate candidate gain sets
    without storing time-series data.

    Parameters
    ----------
    Kp, Ki, Kd : float
        PID gains.
    Tset : float
        Setpoint temperature (°C).

    Returns
    -------
    float
        Cost = integral(e²·dt) + 0.2 * total_energy
    """
    return _run_sim(Kp, Ki, Kd, Tset, record=False)


# ── performance metrics ──────────────────────────────────────────────────────

def compute_metrics(time_list, temp_list, Tset, T_initial):
    """
    Compute standard control system performance metrics from simulation data.

    Definitions used:
        Rise Time     — time for temperature to first cross the setpoint.
        Settling Time — first time the temperature enters and stays within
                        ±2% of the total change (settling band) for at least
                        50 consecutive samples (~5 s at dt=0.1).
        Overshoot     — peak deviation beyond the setpoint, expressed as a
                        percentage of the total reference change. For a
                        cooling scenario (T_initial > Tset) overshoot means
                        the temperature drops below Tset; for heating it
                        rises above.

    Parameters
    ----------
    time_list : list of float
        Simulation time points (s).
    temp_list : list of float
        Temperature at each time point (°C).
    Tset : float
        Setpoint temperature (°C).
    T_initial : float
        Initial temperature at t=0 (°C).

    Returns
    -------
    dict with keys:
        'rise_time'     : float or None  (s)
        'settling_time' : float or None  (s)
        'overshoot_pct' : float          (%)
    """
    total_change = abs(Tset - T_initial)
    band = 0.02 * total_change          # ±2% settling band
    cooling = T_initial > Tset          # True if system needs to cool down

    rise_time     = None
    settling_time = None
    
    # Rise time (keep your existing definition)
    # Standard 10–90% rise time

    if cooling:
        T10 = T_initial - 0.1 * total_change
        T90 = T_initial - 0.9 * total_change
    else:
        T10 = T_initial + 0.1 * total_change
        T90 = T_initial + 0.9 * total_change

    t10 = None
    t90 = None
    for t, T in zip(time_list, temp_list):
        if t10 is None:
            if (cooling and T <= T10) or (not cooling and T >= T10):
                t10 = t
        if t90 is None:
            if (cooling and T <= T90) or (not cooling and T >= T90):
                t90 = t
                break

    if t10 is not None and t90 is not None:
        rise_time = t90 - t10
    else:
        rise_time = None

    # Settling time: first time temperature enters and stays within the
    # ±2% band for at least 50 consecutive samples (~5 s at dt=0.1)
    settling_time = None
    consec_required = 50
    n = len(temp_list)
    for i in range(n):
        count = 0
        for T in temp_list[i:]:
            if abs(T - Tset) <= band:
                count += 1
                if count >= consec_required:
                    settling_time = time_list[i]
                    break
            else:
                break
        if settling_time is not None:
            break
    
    # Overshoot: peak exceedance beyond the setpoint (% of total change)
    if cooling:
        # Undershoot = temp dips below Tset
        peak_error = Tset - min(temp_list)
    else:
        # Overshoot = temp rises above Tset
        peak_error = max(temp_list) - Tset

    if total_change == 0:
        overshoot_pct = 0.0
    else:
        overshoot_pct = max(0.0, (peak_error / total_change) * 100)

    return {
        'rise_time':     rise_time,
        't90': t90,
        't10': t10,
        'settling_time': settling_time,
        'overshoot_pct': overshoot_pct,
    }


# ── twiddle optimizer ────────────────────────────────────────────────────────

def twiddle(Tset, tol=0.01):
    """
    Optimise PID gains using the Twiddle (coordinate ascent) algorithm.

    Starting from an initial guess, each gain is perturbed one at a time.
    If the perturbation reduces cost, the step size grows; otherwise it
    shrinks. The loop exits when the sum of all step sizes falls below
    `tol` or the iteration cap is reached.

    Gain search bounds (enforced by clamping):
        Kp ∈ [0, 10],  Ki ∈ [0, 1],  Kd ∈ [0, 5]

    Parameters
    ----------
    Tset : float
        Setpoint temperature (°C) passed to the simulator.
    tol : float, optional
        Convergence threshold on sum(dp). Default is 0.01.

    Returns
    -------
    list : [Kp, Ki, Kd]
        Optimised PID gains.
    """
    p  = [1.0, 0.01, 0.1]   # Initial gains [Kp, Ki, Kd]
    dp = [0.5, 0.01, 0.05]  # Initial step sizes for each gain

    def clamp(p):
        """Keep gains within physically meaningful bounds."""
        p[0] = max(0, min(10, p[0]))
        p[1] = max(0, min(1,  p[1]))
        p[2] = max(0, min(5,  p[2]))

    best_cost = sim_cost(p[0], p[1], p[2], Tset)
    max_iter = 50
    iter_count = 0

    while sum(dp) > tol and iter_count < max_iter:
        for i in range(3):
            # Try increasing gain i
            p[i] += dp[i]
            clamp(p)
            cost = sim_cost(p[0], p[1], p[2], Tset)

            if cost < best_cost:
                best_cost = cost
                dp[i] *= 1.1   # Widen step — this direction is promising
            else:
                # Try decreasing gain i instead
                p[i] -= 2 * dp[i]
                clamp(p)
                cost = sim_cost(p[0], p[1], p[2], Tset)

                if cost < best_cost:
                    best_cost = cost
                    dp[i] *= 1.1   # Widen step — opposite direction works
                else:
                    # Neither direction helped; restore and shrink step
                    p[i] += dp[i]
                    clamp(p)
                    dp[i] *= 0.9

        print(f"Iter {iter_count}, cost = {best_cost:.4f}")
        iter_count += 1

    return p


# ── entry point ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    Tset = float(input("Enter desired temperature (°C): "))

    # Auto-tune gains for the given setpoint
    Kp_opt, Ki_opt, Kd_opt = twiddle(Tset)
    print("\nOptimized PID:")
    print("Kp =", Kp_opt)
    print("Ki =", Ki_opt)
    print("Kd =", Kd_opt)
    print("Final cost:", sim_cost(Kp_opt, Ki_opt, Kd_opt, Tset))

    # Re-run with recording enabled to collect data for plots
    time_list, temp_list, e_list, power_list, energy_list = temp_sim(
        Kp_opt, Ki_opt, Kd_opt, Tset
    )

    # ── performance metrics ───────────────────────────────────────────────────
    T_initial = 35.0    # Must match the initial T inside _run_sim
    metrics = compute_metrics(time_list, temp_list, Tset, T_initial)

    print("\nControl Performance Metrics:")
    if metrics['rise_time']:
        print(f"  Rise Time     : {metrics['rise_time']:.2f} s")
    else:
        print("  Rise Time     : not reached")
    if metrics['settling_time']:
        print(f"  Settling Time : {metrics['settling_time']:.2f} s")
    else:
        print("  Settling Time : not reached")
    print(f"  Overshoot     : {metrics['overshoot_pct']:.3f} %")

    # ── plot ─────────────────────────────────────────────────────────────────
    plt.figure(figsize=(10, 8))

    plt.subplot(4, 1, 1)
    plt.plot(time_list, temp_list)
    plt.axhline(y=Tset, color='r', linestyle='--', label=f'Setpoint ({Tset}°C)')
    # Mark rise time and settling time on the temperature plot
    if metrics['t90'] is not None:
        plt.axvline(x=metrics['t90'],
                    color='g',
                    linestyle=':',
                    label=f"t90 = {metrics['t90']:.1f}s")

    if metrics['t10'] is not None:
        plt.axvline(x=metrics['t10'],
                    color='purple',
                    linestyle=':',
                    label=f"t10 = {metrics['t10']:.1f}s")
    if metrics['settling_time']:
        plt.axvline(x=metrics['settling_time'], color='orange', linestyle=':',
                    label=f"Settling Time ({metrics['settling_time']:.1f}s)")
    plt.ylabel("Temperature")
    plt.title("Temperature vs Time (PID Control)")
    plt.legend()
    plt.grid()

    plt.subplot(4, 1, 2)
    plt.plot(time_list, e_list)
    plt.ylabel("Error")
    plt.title("Error vs Time")
    plt.grid()

    plt.subplot(4, 1, 3)
    plt.plot(time_list, power_list)
    plt.ylabel("Power")
    plt.title("Power vs Time")
    plt.grid()

    plt.subplot(4, 1, 4)
    plt.plot(time_list, energy_list)
    plt.xlabel("Time")
    plt.ylabel("Energy")
    plt.title("Energy vs Time")
    plt.grid()

    plt.tight_layout()
    plt.show()
