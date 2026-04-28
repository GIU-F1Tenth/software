import numpy as np
import matplotlib.pyplot as plt
import os

MODE_COLORS = {
    'STRAIGHT':  '#4C9BE8',
    'BLEND_OUT': '#A78BFA',
    'CURVE':     '#EF4444',
    'BLEND_IN':  '#F97316',
    'FALLBACK':  '#22C55E',
}


def plot_results(data: dict, outpath: str = None):
    if outpath is None:
        outpath = os.path.join(os.path.dirname(__file__), 'results', 'kayn_sim.png')
    os.makedirs(os.path.dirname(outpath), exist_ok=True)

    track  = data['track']
    times  = data['time']
    modes  = data['mode']

    fig, axes = plt.subplots(4, 1, figsize=(14, 18))
    fig.suptitle('KAYN Hybrid Controller — Simulation Results', fontsize=14, y=0.98)

    # ---- Panel 1: Path coloured by mode ----
    ax = axes[0]
    tx = [wp['x'] for wp in track]
    ty = [wp['y'] for wp in track]
    ax.plot(tx, ty, 'k--', linewidth=1, alpha=0.5, label='Reference')
    for mode, color in MODE_COLORS.items():
        mask = np.array([m == mode for m in modes])
        if mask.any():
            ax.scatter(data['x'][mask], data['y'][mask], c=color, s=4, label=mode)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Trajectory (coloured by active mode)')
    ax.legend(loc='upper right', markerscale=3)
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)

    # ---- Panel 2: Cross-track error ----
    ax = axes[1]
    prev_mode = None
    for i, (t, cte, mode) in enumerate(zip(times, data['cte'], modes)):
        color = MODE_COLORS.get(mode, 'gray')
        if i > 0:
            ax.plot([times[i-1], t], [data['cte'][i-1], cte], color=color, linewidth=1)
        if mode != prev_mode and i > 0:
            ax.axvline(t, color='gray', linewidth=0.5, alpha=0.5)
        prev_mode = mode
    ax.axhline(0,     color='black', linewidth=0.5)
    ax.axhline( 0.05, color='red', linewidth=0.5, linestyle='--', label='±0.05m')
    ax.axhline(-0.05, color='red', linewidth=0.5, linestyle='--')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('CTE [m]')
    ax.set_title('Cross-Track Error (vertical lines = mode transitions)')
    ax.legend(); ax.grid(True, alpha=0.3)

    # ---- Panel 3: Control inputs ----
    ax = axes[2]
    ax.plot(times, np.degrees(data['delta']), label='Steering [deg]', color='steelblue')
    ax2 = ax.twinx()
    ax2.plot(times, data['accel'], label='Accel [m/s²]', color='darkorange', alpha=0.7)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Steering [deg]')
    ax2.set_ylabel('Acceleration [m/s²]')
    ax.set_title('Control Inputs')
    ax.grid(True, alpha=0.3)
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2)

    # ---- Panel 4: Curvature + FSM state ----
    ax = axes[3]
    ax.plot(times, data['kappa'], color='purple', linewidth=1, label='κ [rad/m]')
    ax.axhline(0.10, color='red',    linewidth=0.8, linestyle='--', label='Enter MPC (0.10)')
    ax.axhline(0.06, color='orange', linewidth=0.8, linestyle='--', label='Exit MPC (0.06)')
    state_to_num = {'STRAIGHT': 0, 'BLEND_OUT': 1, 'CURVE': 2, 'BLEND_IN': 3, 'FALLBACK': 4}
    state_nums = [state_to_num.get(m, 0) for m in modes]
    ax2 = ax.twinx()
    ax2.step(times, state_nums, color='gray', alpha=0.3, linewidth=1, where='post')
    ax2.set_yticks(list(state_to_num.values()))
    ax2.set_yticklabels(list(state_to_num.keys()), fontsize=7)
    ax2.set_ylabel('FSM State')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Curvature κ [rad/m]')
    ax.set_title('Curvature Estimate + FSM State')
    ax.legend(loc='upper right'); ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"Plot saved: {outpath}")
    plt.close()
