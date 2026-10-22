import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch
import os

def create_diagram():
    fig, ax = plt.subplots(figsize=(14, 11), dpi=300)
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 11)
    ax.axis('off')
    fig.patch.set_facecolor('#FFFFFF')

    # Color Palette
    bg_plant = '#EBF3FB'
    border_plant = '#2B6CB0'
    header_plant = '#1A365D'

    bg_afe = '#FEFCBF'
    border_afe = '#B7791F'
    header_afe = '#744210'

    bg_core = '#FAF5FF'
    border_core = '#6B46C1'
    header_core = '#322659'

    bg_act = '#F0FFF4'
    border_act = '#276749'
    header_act = '#1C4532'

    card_bg = '#FFFFFF'
    text_dark = '#1A202C'
    text_muted = '#4A5568'
    arrow_color = '#2D3748'

    # Title Banner
    ax.text(7.0, 10.6, "EV Battery Management System (BMS) Architecture", 
            ha='center', va='center', fontsize=18, fontweight='bold', color='#1A202C')
    ax.text(7.0, 10.25, "ISO 26262:2018 ASIL-D Closed-Loop Control, EKF SoC Estimation & Active Balancing", 
            ha='center', va='center', fontsize=11, color='#4A5568', style='italic')

    # =========================================================================
    # LAYER 1: PLANT DOMAIN (Top)
    # =========================================================================
    plant_box = FancyBboxPatch((0.6, 7.8), 12.8, 2.1, boxstyle="round,pad=0.1,rounding_size=0.15",
                               facecolor=bg_plant, edgecolor=border_plant, linewidth=1.5)
    ax.add_patch(plant_box)
    ax.text(0.9, 9.65, "1. PHYSICAL PLANT DOMAIN (14S Li-ion Battery & Inverter)", 
            fontsize=11, fontweight='bold', color=header_plant, va='center')

    # Plant Sub-cards
    # Cell ECM
    c1 = FancyBboxPatch((0.9, 8.05), 3.8, 1.35, boxstyle="round,pad=0.08,rounding_size=0.1",
                        facecolor=card_bg, edgecolor=border_plant, linewidth=1)
    ax.add_patch(c1)
    ax.text(2.8, 9.15, "14-Cell 2nd-Order Thevenin ECM", fontsize=9.5, fontweight='bold', ha='center', color=text_dark)
    ax.text(2.8, 8.65, "• Dynamic R0, R1-C1 & R2-C2 Polarization\n• Non-Linear NMC OCV Polynomial\n• Arrhenius Temperature Resistance Scaling", 
            fontsize=8, ha='center', va='center', color=text_muted)

    # Thermal Model
    c2 = FancyBboxPatch((5.1, 8.05), 3.8, 1.35, boxstyle="round,pad=0.08,rounding_size=0.1",
                        facecolor=card_bg, edgecolor=border_plant, linewidth=1)
    ax.add_patch(c2)
    ax.text(7.0, 9.15, "Coupled Lumped Thermal Model", fontsize=9.5, fontweight='bold', ha='center', color=text_dark)
    ax.text(7.0, 8.65, "• Joule Heating: Q_gen = I²·R0 + Q_polar\n• Convective Cooling: Q_conv = h·A·ΔT\n• Active Chiller Simulation (>35°C)", 
            fontsize=8, ha='center', va='center', color=text_muted)

    # Inverter DC Bus
    c3 = FancyBboxPatch((9.3, 8.05), 3.8, 1.35, boxstyle="round,pad=0.08,rounding_size=0.1",
                        facecolor=card_bg, edgecolor=border_plant, linewidth=1)
    ax.add_patch(c3)
    ax.text(11.2, 9.15, "Inverter DC-Link Bus Model", fontsize=9.5, fontweight='bold', ha='center', color=text_dark)
    ax.text(11.2, 8.65, "• Decoupling Capacitance (C_link = 1500 µF)\n• Precharge Resistor (R_pre = 50 Ω)\n• Inverter Discharge Bleeder (50 kΩ)", 
            fontsize=8, ha='center', va='center', color=text_muted)

    # Arrow Plant -> AFE
    a1 = FancyArrowPatch((7.0, 7.8), (7.0, 7.15), arrowstyle='-|>', mutation_scale=15, 
                         color=arrow_color, linewidth=2)
    ax.add_patch(a1)
    ax.text(7.1, 7.48, "High-Voltage Sensing & Shunts (Raw Physical Signals)", 
            fontsize=8.5, color=text_dark, fontweight='bold', va='center')

    # =========================================================================
    # LAYER 2: ANALOG FRONT-END (AFE)
    # =========================================================================
    afe_box = FancyBboxPatch((0.6, 5.95), 12.8, 1.2, boxstyle="round,pad=0.1,rounding_size=0.15",
                             facecolor=bg_afe, edgecolor=border_afe, linewidth=1.5)
    ax.add_patch(afe_box)
    ax.text(0.9, 6.9, "2. ANALOG FRONT-END (AFE) SENSING & SIGNAL CONDITIONING", 
            fontsize=11, fontweight='bold', color=header_afe, va='center')

    afe_items = [
        ("Cell Voltages (14S)", "V_meas = 3.15V – 4.20V\nσ = 4.5 mV Gaussian Noise", 2.2),
        ("Pack Current", "I_meas = ±120 A (Hall AFE)\nσ = 0.15 A, 80 mA DC Bias", 5.4),
        ("Cell Temperatures (14S)", "T_meas = -20°C – 60°C\nNTC Thermistors, σ = 0.25°C", 8.6),
        ("Inverter Bus Voltage", "V_bus = 0V – 60V\nDC-Link Sensing Divider", 11.8)
    ]
    for title, desc, cx in afe_items:
        sc = FancyBboxPatch((cx - 1.45, 6.08), 2.9, 0.72, boxstyle="round,pad=0.05,rounding_size=0.08",
                            facecolor=card_bg, edgecolor=border_afe, linewidth=0.8)
        ax.add_patch(sc)
        ax.text(cx, 6.6, title, fontsize=8.5, fontweight='bold', ha='center', color=text_dark)
        ax.text(cx, 6.3, desc, fontsize=7.2, ha='center', va='center', color=text_muted)

    # Arrow AFE -> Core
    a2 = FancyArrowPatch((7.0, 5.95), (7.0, 5.3), arrowstyle='-|>', mutation_scale=15, 
                         color=arrow_color, linewidth=2)
    ax.add_patch(a2)
    ax.text(7.1, 5.63, "Filtered Telemetry Bus [V_cells, I_pack, T_cells, V_bus] (100 Hz / 10 Hz)", 
            fontsize=8.5, color=text_dark, fontweight='bold', va='center')

    # =========================================================================
    # LAYER 3: BMS ESTIMATOR & SAFETY CORE (ASIL-D)
    # =========================================================================
    core_box = FancyBboxPatch((0.6, 2.5), 12.8, 2.8, boxstyle="round,pad=0.1,rounding_size=0.15",
                             facecolor=bg_core, edgecolor=border_core, linewidth=1.5)
    ax.add_patch(core_box)
    ax.text(0.9, 5.05, "3. BMS FIRMWARE & ESTIMATOR CORE (ISO 26262 ASIL-D ARCHITECTURE)", 
            fontsize=11, fontweight='bold', color=header_core, va='center')

    # State Machine Card (Top Core)
    fsm_card = FancyBboxPatch((0.9, 3.9), 12.2, 0.95, boxstyle="round,pad=0.08,rounding_size=0.1",
                              facecolor=card_bg, edgecolor=border_core, linewidth=1)
    ax.add_patch(fsm_card)
    ax.text(1.1, 4.62, "ISO 26262 Supervisory State Machine & Contactor Manager", fontsize=10, fontweight='bold', color=text_dark)
    ax.text(1.1, 4.22, "• States: INIT → STANDBY → PRECHARGE (500 ms target) → NORMAL_RUN → DERATING → SAFE_FAULT\n• Safety Goals SG01–SG05: Over/Under Voltage, Overcurrent, Over-Temperature, Contactor Weld Detection (FTTI ≤ 200 ms)", 
            fontsize=8, color=text_muted)

    # Three Core Blocks Below FSM
    # EKF
    ekf_c = FancyBboxPatch((0.9, 2.65), 3.9, 1.15, boxstyle="round,pad=0.08,rounding_size=0.1",
                           facecolor=card_bg, edgecolor=border_core, linewidth=1)
    ax.add_patch(ekf_c)
    ax.text(2.85, 3.6, "Discrete Extended Kalman Filter (EKF)", fontsize=9, fontweight='bold', ha='center', color=text_dark)
    ax.text(2.85, 3.12, "• 2nd-Order ECM State Vector x = [SoC, V1, V2]ᵀ\n• Dynamic Jacobian C = [dVoc/dSoC, -1, -1]\n• Joseph-Form Covariance Stabilization\n• Steady-State SoC RMSE: 0.270%", 
            fontsize=7.5, ha='center', va='center', color=text_muted)

    # RLS SoH
    rls_c = FancyBboxPatch((5.05, 2.65), 3.9, 1.15, boxstyle="round,pad=0.08,rounding_size=0.1",
                           facecolor=card_bg, edgecolor=border_core, linewidth=1)
    ax.add_patch(rls_c)
    ax.text(7.0, 3.6, "Recursive Least Squares (RLS)", fontsize=9, fontweight='bold', ha='center', color=text_dark)
    ax.text(7.0, 3.12, "• Online Ohmic Resistance (R0) Identification\n• Exponential Forgetting Factor λ = 0.9995\n• End of Life (EOL) Resistance Growth Tracking\n• Real-Time State of Health (SoH %)", 
            fontsize=7.5, ha='center', va='center', color=text_muted)

    # Active Balancing
    bal_c = FancyBboxPatch((9.2, 2.65), 3.9, 1.15, boxstyle="round,pad=0.08,rounding_size=0.1",
                           facecolor=card_bg, edgecolor=border_core, linewidth=1)
    ax.add_patch(bal_c)
    ax.text(11.15, 3.6, "Active Balancing Controller", fontsize=9, fontweight='bold', ha='center', color=text_dark)
    ax.text(11.15, 3.12, "• Bidirectional DC-DC Charge Shuttling\n• Donor (Max SoC) → Receiver (Min SoC)\n• Max Balancing Current: 1.5 A (88% Efficiency)\n• Cell SoC Spread Reduction: 50.8%", 
            fontsize=7.5, ha='center', va='center', color=text_muted)

    # Arrow Core -> Actuation
    a3 = FancyArrowPatch((7.0, 2.5), (7.0, 1.85), arrowstyle='-|>', mutation_scale=15, 
                         color=arrow_color, linewidth=2)
    ax.add_patch(a3)
    ax.text(7.1, 2.18, "Actuation Commands & Powertrain Envelopes", 
            fontsize=8.5, color=text_dark, fontweight='bold', va='center')

    # =========================================================================
    # LAYER 4: ACTUATION & INTERFACE
    # =========================================================================
    act_box = FancyBboxPatch((0.6, 0.45), 12.8, 1.4, boxstyle="round,pad=0.1,rounding_size=0.15",
                             facecolor=bg_act, edgecolor=border_act, linewidth=1.5)
    ax.add_patch(act_box)
    ax.text(0.9, 1.62, "4. HIGH-VOLTAGE SAFETY ACTUATION & VEHICLE INTERFACE", 
            fontsize=11, fontweight='bold', color=header_act, va='center')

    # Actuation Sub-cards
    act_items = [
        ("HV Contactors", "Main Pos (+), Main Neg (-), Precharge Relay\nAutomatic Emergency Safe State Isolation (FTTI < 200ms)", 2.85),
        ("Dynamic Derating Limits", "CAN Broadcast: I_discharge_lim, I_charge_lim\nThermal (>45°C) & Voltage Warning Derating", 7.0),
        ("Active DC-DC Shuttling", "Bidirectional switched-inductor converters\nDirect cell-to-cell charge redistribution", 11.15)
    ]
    for title, desc, cx in act_items:
        ac = FancyBboxPatch((cx - 1.9, 0.58), 3.8, 0.85, boxstyle="round,pad=0.08,rounding_size=0.08",
                            facecolor=card_bg, edgecolor=border_act, linewidth=1)
        ax.add_patch(ac)
        ax.text(cx, 1.25, title, fontsize=9, fontweight='bold', ha='center', color=text_dark)
        ax.text(cx, 0.88, desc, fontsize=7.5, ha='center', va='center', color=text_muted)

    # Feedback loop arrows (Actuation back to Plant)
    # Left feedback: Contactor physical connection to Inverter
    fb_line = patches.ConnectionPatch(xyA=(1.5, 1.45), xyB=(2.0, 7.8), coordsA="data", coordsB="data",
                                      axesA=ax, axesB=ax, arrowstyle="-|>", mutation_scale=12,
                                      connectionstyle="arc3,rad=0.35", color='#3182CE', linewidth=1.5,
                                      linestyle='--', label='HV Contactor Actuation')
    ax.add_artist(fb_line)
    ax.text(0.2, 5.0, "HV Pack Isolation\n& Precharge Control", fontsize=8, color='#2B6CB0', fontweight='bold', ha='center', rotation=90)

    # Right feedback: Balancing current into cells
    fb_line2 = patches.ConnectionPatch(xyA=(12.5, 1.45), xyB=(12.0, 7.8), coordsA="data", coordsB="data",
                                       axesA=ax, axesB=ax, arrowstyle="-|>", mutation_scale=12,
                                       connectionstyle="arc3,rad=-0.35", color='#38A169', linewidth=1.5,
                                       linestyle='--', label='Balancing Current')
    ax.add_artist(fb_line2)
    ax.text(13.8, 5.0, "Bidirectional Balancing\nCharge Shuttling (±1.5A)", fontsize=8, color='#276749', fontweight='bold', ha='center', rotation=-90)

    plt.tight_layout()
    out_dir = r"C:\Users\us\Desktop\PROJECT_PRAHAS\EV-BMS-ISO26262-EKF-ActiveBalancing\docs"
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.join(out_dir, "BMS_System_Architecture.png")
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Diagram generated successfully at: {out_path}")

if __name__ == "__main__":
    create_diagram()
