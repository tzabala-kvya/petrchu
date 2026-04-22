"""
PetrChu Dashboard — Raspberry Pi Supervisory Interface
=======================================================
This is a STARTER SKELETON, not production code. It implements the full
architecture (serial reader thread + Dash UI + command sender) with mock
data so you can run it on any machine and see the layout before the
hardware is connected.

To run:
    pip install dash plotly pyserial
    python dashboard.py

Then open http://localhost:8050 in a browser.
On the Pi, Chromium will auto-open to this URL.

Architecture:
    Thread 1 (SerialReader): Reads UART, parses CSV lines, pushes to deque
    Thread 2 (DataLogger):   Pulls from deque, writes to CSV file
    Main thread (Dash):      Pulls from deque on timer, updates plots/indicators

The three are decoupled — if the UI lags, data still gets logged.
If logging fails, the UI still updates.
"""

import dash
from dash import dcc, html, callback_context
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go
from collections import deque
from datetime import datetime
import threading
import time
import math
import csv
import os

# Try to import serial; if not available, we'll use mock data
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("pyserial not installed — running with mock data")


# ============================================================================
# CONFIGURATION
# ============================================================================

# Serial port settings (change for your Pi)
SERIAL_PORT = '/dev/ttyACM0'     # Arduino Mega on Pi (usually ACM0 or USB0)
SERIAL_BAUD = 115200

# Data buffer size (how many samples to keep in memory for plotting)
# At 20 Hz, 6000 samples = 5 minutes of data in the rolling plots
BUFFER_SIZE = 6000

# UI update interval in milliseconds
# 500ms (2 Hz) is a good balance: smooth enough visually, light on CPU
# Don't go below 200ms on a Pi 4 — Plotly redraws are expensive
UI_UPDATE_MS = 500

# Telemetry field indices (must match Arduino telemetry_format.ino)
F_TIMESTAMP = 0
F_STATE = 1
F_FAULTS = 2
F_PRESSURE = 3
F_FLOW = 4
F_RPM = 5
F_VOLTAGE = 6
F_CURRENT = 7
F_POWER = 8
F_SETPOINT = 9
F_PID_OUTPUT = 10
F_PID_ERROR = 11
NUM_FIELDS = 12

# State machine mode names (index = mode number)
STATE_NAMES = [
    "INIT", "IDLE", "ARMED", "CAES CHARGE", "CAES DISCHARGE",
    "HYDRO CHARGE", "HYDRO DISCHARGE", "DUAL DISCHARGE", "FAULT", "E-STOP"
]
STATE_COLORS = [
    "#888", "#4dabf7", "#51cf66", "#ffd43b", "#ff922b",
    "#339af0", "#1c7ed6", "#e64980", "#fa5252", "#fa5252"
]

# Fault bit names (index = bit position)
FAULT_NAMES = [
    "Overpressure", "Underpressure", "Overcurrent", "Overvoltage",
    "Sensor Timeout", "Comm Timeout", "Flow Fault", "RPM Fault",
    "E-Stop Active", "Reserved", "Reserved", "Reserved",
    "Reserved", "Reserved", "Reserved", "Reserved"
]


# ============================================================================
# DATA STORAGE (thread-safe)
# ============================================================================
# Each channel gets its own deque. This is simpler than storing dicts
# and lets each plot just grab the array it needs.

data = {
    'time_s':     deque(maxlen=BUFFER_SIZE),   # seconds since start
    'state':      deque(maxlen=BUFFER_SIZE),
    'faults':     deque(maxlen=BUFFER_SIZE),
    'pressure':   deque(maxlen=BUFFER_SIZE),
    'flow':       deque(maxlen=BUFFER_SIZE),
    'rpm':        deque(maxlen=BUFFER_SIZE),
    'voltage':    deque(maxlen=BUFFER_SIZE),
    'current':    deque(maxlen=BUFFER_SIZE),
    'power':      deque(maxlen=BUFFER_SIZE),
    'setpoint':   deque(maxlen=BUFFER_SIZE),
    'pid_output': deque(maxlen=BUFFER_SIZE),
    'pid_error':  deque(maxlen=BUFFER_SIZE),
    'energy_mJ':  deque(maxlen=BUFFER_SIZE),   # cumulative energy (integrated power)
}

# Lock for thread-safe access (deques are thread-safe for append/pop,
# but we want atomic multi-field appends)
data_lock = threading.Lock()

# Communication health tracking
last_valid_packet_time = time.time()
packet_count = 0
error_count = 0

# Cumulative energy integration state
cumulative_energy_mJ = 0.0

# Event log (for the scrolling event panel)
event_log = deque(maxlen=200)

# Serial port handle (shared between reader and command sender)
ser = None


# ============================================================================
# SERIAL READER THREAD
# ============================================================================

def parse_telemetry_line(line):
    """Parse one CSV telemetry line from the Arduino.
    Returns a dict of field values, or None if the line is invalid."""
    global last_valid_packet_time, packet_count, error_count, cumulative_energy_mJ

    try:
        parts = line.strip().split(',')
        if len(parts) != NUM_FIELDS:
            error_count += 1
            return None

        # Parse each field to its expected type
        parsed = {
            'timestamp_ms': int(parts[F_TIMESTAMP]),
            'state':        int(parts[F_STATE]),
            'faults':       int(parts[F_FAULTS]),
            'pressure':     float(parts[F_PRESSURE]),
            'flow':         float(parts[F_FLOW]),
            'rpm':          int(parts[F_RPM]),
            'voltage':      int(parts[F_VOLTAGE]),
            'current':      int(parts[F_CURRENT]),
            'power':        int(parts[F_POWER]),
            'setpoint':     float(parts[F_SETPOINT]),
            'pid_output':   int(parts[F_PID_OUTPUT]),
            'pid_error':    float(parts[F_PID_ERROR]),
        }

        # Integrate power for cumulative energy
        # power is in mW, interval is ~50ms, so energy increment = power_mW * 0.050 = mJ
        cumulative_energy_mJ += parsed['power'] * 0.050

        last_valid_packet_time = time.time()
        packet_count += 1
        return parsed

    except (ValueError, IndexError) as e:
        error_count += 1
        return None


def serial_reader_thread():
    """Continuously reads from the Arduino serial port and pushes to data buffers."""
    global ser

    start_time = None

    while True:
        try:
            if ser is None or not ser.is_open:
                ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
                log_event("Serial connected")
                time.sleep(0.1)  # Let Arduino reset after connection

            line = ser.readline().decode('utf-8', errors='ignore')
            if not line.strip():
                continue

            # Skip header lines or debug messages (they won't parse as 12 fields)
            parsed = parse_telemetry_line(line)
            if parsed is None:
                continue

            # Use first valid timestamp as t=0
            if start_time is None:
                start_time = parsed['timestamp_ms']

            t_sec = (parsed['timestamp_ms'] - start_time) / 1000.0

            with data_lock:
                data['time_s'].append(t_sec)
                data['state'].append(parsed['state'])
                data['faults'].append(parsed['faults'])
                data['pressure'].append(parsed['pressure'])
                data['flow'].append(parsed['flow'])
                data['rpm'].append(parsed['rpm'])
                data['voltage'].append(parsed['voltage'] / 1000.0)     # mV → V for display
                data['current'].append(parsed['current'] / 1000.0)     # mA → A for display
                data['power'].append(parsed['power'] / 1000.0)         # mW → W for display
                data['setpoint'].append(parsed['setpoint'])
                data['pid_output'].append(parsed['pid_output'])
                data['pid_error'].append(parsed['pid_error'])
                data['energy_mJ'].append(cumulative_energy_mJ / 1000.0)  # mJ → J for display

        except serial.SerialException as e:
            log_event(f"Serial error: {e}")
            ser = None
            time.sleep(2)  # Wait before reconnecting

        except Exception as e:
            log_event(f"Reader error: {e}")
            time.sleep(0.1)


# ============================================================================
# MOCK DATA GENERATOR (for testing without hardware)
# ============================================================================

def mock_data_thread():
    """Generates fake telemetry data so you can develop the UI without hardware."""
    global cumulative_energy_mJ

    t = 0
    state = 1  # IDLE
    pressure_base = 1013  # mbar (atmospheric)

    while True:
        t += 0.05  # 20 Hz

        # Simulate a slow pressure ramp with some noise
        pressure = pressure_base + 200 * math.sin(t * 0.1) + 5 * math.sin(t * 3.7)
        flow = max(0, 2.0 + 0.5 * math.sin(t * 0.3) + 0.1 * math.sin(t * 7))
        rpm = int(1200 + 300 * math.sin(t * 0.15) + 20 * math.sin(t * 5))
        voltage = max(0, 11000 + 2000 * math.sin(t * 0.15))  # mV
        current = max(0, 500 + 200 * math.sin(t * 0.15 + 0.5))  # mA
        power = voltage * current / 1000  # mW

        setpoint = 1200  # example: pressure setpoint in mbar
        pid_error = setpoint - pressure
        pid_output = min(255, max(0, int(128 + pid_error * 0.5)))

        cumulative_energy_mJ += power * 0.050

        # Cycle through some states for demo
        if t > 30:
            state = 4  # CAES discharge
        elif t > 20:
            state = 3  # CAES charge
        elif t > 10:
            state = 2  # Armed

        with data_lock:
            data['time_s'].append(t)
            data['state'].append(state)
            data['faults'].append(0)
            data['pressure'].append(pressure)
            data['flow'].append(flow)
            data['rpm'].append(rpm)
            data['voltage'].append(voltage / 1000.0)
            data['current'].append(current / 1000.0)
            data['power'].append(power / 1000.0)
            data['setpoint'].append(setpoint)
            data['pid_output'].append(pid_output)
            data['pid_error'].append(pid_error)
            data['energy_mJ'].append(cumulative_energy_mJ / 1000.0)

        time.sleep(0.05)


# ============================================================================
# EVENT LOGGING
# ============================================================================

def log_event(message):
    """Add a timestamped event to the event log."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    event_log.appendleft(f"[{timestamp}] {message}")


# ============================================================================
# COMMAND SENDER
# ============================================================================

def send_command(cmd_string):
    """Send a command to the Arduino. Returns True if acknowledged."""
    global ser
    if ser is None or not ser.is_open:
        log_event(f"CMD FAILED (no serial): {cmd_string}")
        return False
    try:
        ser.write(f"{cmd_string}\n".encode())
        log_event(f"CMD SENT: {cmd_string}")
        # In production, you'd wait for OK/ERR response here with a timeout
        return True
    except serial.SerialException as e:
        log_event(f"CMD ERROR: {e}")
        return False


# ============================================================================
# DASH APP LAYOUT
# ============================================================================

app = dash.Dash(__name__)
app.title = "PetrChu Control Dashboard"

# --- Reusable style constants ---
CARD_STYLE = {
    'backgroundColor': '#1e1e2e',
    'borderRadius': '8px',
    'padding': '12px',
    'marginBottom': '10px',
    'border': '1px solid #333',
}
LABEL_STYLE = {
    'color': '#aaa',
    'fontSize': '11px',
    'textTransform': 'uppercase',
    'letterSpacing': '1px',
    'marginBottom': '2px',
}
VALUE_STYLE = {
    'color': '#fff',
    'fontSize': '28px',
    'fontWeight': 'bold',
    'fontFamily': 'monospace',
}
PLOT_CONFIG = {
    'displayModeBar': True,
    'modeBarButtonsToAdd': ['toImage'],  # Screenshot button
    'toImageButtonOptions': {'format': 'png', 'height': 600, 'width': 1200, 'scale': 2},
}
DARK_PLOT_LAYOUT = {
    'paper_bgcolor': '#1e1e2e',
    'plot_bgcolor': '#1e1e2e',
    'font': {'color': '#ccc', 'size': 11},
    'margin': {'l': 50, 'r': 20, 't': 30, 'b': 40},
    'xaxis': {'gridcolor': '#333', 'title': 'Time (s)'},
    'yaxis': {'gridcolor': '#333'},
    'legend': {'orientation': 'h', 'y': 1.12},
    'uirevision': 'constant',  # Preserves zoom/pan state across updates
}


app.layout = html.Div(style={'backgroundColor': '#11111b', 'minHeight': '100vh', 'padding': '15px', 'fontFamily': 'sans-serif'}, children=[

    # --- HEADER BAR ---
    html.Div(style={'display': 'flex', 'justifyContent': 'space-between', 'alignItems': 'center', 'marginBottom': '15px'}, children=[
        html.H1("PetrChu", style={'color': '#cdd6f4', 'margin': '0', 'fontSize': '24px'}),
        html.Div(id='header-status', style={'display': 'flex', 'gap': '20px', 'alignItems': 'center'}),
    ]),

    # --- FAULT ANNUNCIATOR BAR ---
    html.Div(id='fault-bar', style={
        **CARD_STYLE,
        'backgroundColor': '#1e3a1e',  # Green when no faults
        'textAlign': 'center',
        'padding': '8px',
        'fontSize': '14px',
        'color': '#51cf66',
    }, children="No Active Faults"),

    # --- MAIN CONTENT (3-column layout) ---
    html.Div(style={'display': 'grid', 'gridTemplateColumns': '280px 1fr 1fr', 'gap': '10px'}, children=[

        # ============================================================
        # LEFT COLUMN — Status indicators + Operator controls
        # ============================================================
        html.Div(children=[

            # State indicator
            html.Div(style=CARD_STYLE, children=[
                html.Div("SYSTEM STATE", style=LABEL_STYLE),
                html.Div(id='state-display', style={**VALUE_STYLE, 'fontSize': '20px'}, children="---"),
            ]),

            # Comms health
            html.Div(style=CARD_STYLE, children=[
                html.Div("COMMS HEALTH", style=LABEL_STYLE),
                html.Div(id='comms-indicator', style={'fontSize': '14px', 'color': '#888'}),
            ]),

            # Live readouts
            html.Div(style=CARD_STYLE, children=[
                html.Div("LIVE READINGS", style=LABEL_STYLE),
                html.Div(id='live-readouts', style={'fontFamily': 'monospace', 'fontSize': '13px', 'color': '#ccc', 'lineHeight': '1.8'}),
            ]),

            # --- OPERATOR CONTROLS ---
            html.Div(style={**CARD_STYLE, 'borderColor': '#f9e2af'}, children=[
                html.Div("OPERATOR CONTROLS", style={**LABEL_STYLE, 'color': '#f9e2af'}),

                # Setpoint input
                html.Div(style={'marginTop': '10px'}, children=[
                    html.Label("Setpoint:", style={'color': '#ccc', 'fontSize': '12px'}),
                    html.Div(style={'display': 'flex', 'gap': '5px'}, children=[
                        dcc.Input(id='setpoint-input', type='number', value=1500,
                                  style={'width': '100px', 'backgroundColor': '#313244', 'color': '#fff', 'border': '1px solid #555', 'borderRadius': '4px', 'padding': '5px'}),
                        html.Button("Send", id='btn-setpoint', n_clicks=0,
                                    style={'backgroundColor': '#89b4fa', 'border': 'none', 'borderRadius': '4px', 'padding': '5px 12px', 'cursor': 'pointer'}),
                    ]),
                ]),

                # PID gains
                html.Div(style={'marginTop': '10px'}, children=[
                    html.Label("PID Gains:", style={'color': '#ccc', 'fontSize': '12px'}),
                    html.Div(style={'display': 'grid', 'gridTemplateColumns': '1fr 1fr 1fr', 'gap': '5px'}, children=[
                        dcc.Input(id='kp-input', type='number', value=2.0, step=0.1, placeholder='Kp',
                                  style={'backgroundColor': '#313244', 'color': '#fff', 'border': '1px solid #555', 'borderRadius': '4px', 'padding': '5px', 'width': '100%'}),
                        dcc.Input(id='ki-input', type='number', value=0.1, step=0.01, placeholder='Ki',
                                  style={'backgroundColor': '#313244', 'color': '#fff', 'border': '1px solid #555', 'borderRadius': '4px', 'padding': '5px', 'width': '100%'}),
                        dcc.Input(id='kd-input', type='number', value=0.05, step=0.01, placeholder='Kd',
                                  style={'backgroundColor': '#313244', 'color': '#fff', 'border': '1px solid #555', 'borderRadius': '4px', 'padding': '5px', 'width': '100%'}),
                    ]),
                    html.Button("Update PID", id='btn-pid', n_clicks=0,
                                style={'marginTop': '5px', 'backgroundColor': '#a6e3a1', 'border': 'none', 'borderRadius': '4px', 'padding': '5px 12px', 'cursor': 'pointer', 'width': '100%'}),
                ]),

                # Mode / Run / Pause
                html.Div(style={'marginTop': '10px', 'display': 'flex', 'gap': '5px'}, children=[
                    html.Button("RUN", id='btn-run', n_clicks=0,
                                style={'flex': '1', 'backgroundColor': '#a6e3a1', 'border': 'none', 'borderRadius': '4px', 'padding': '8px', 'cursor': 'pointer', 'fontWeight': 'bold'}),
                    html.Button("PAUSE", id='btn-pause', n_clicks=0,
                                style={'flex': '1', 'backgroundColor': '#f9e2af', 'border': 'none', 'borderRadius': '4px', 'padding': '8px', 'cursor': 'pointer', 'fontWeight': 'bold'}),
                    html.Button("STOP", id='btn-stop', n_clicks=0,
                                style={'flex': '1', 'backgroundColor': '#f38ba8', 'border': 'none', 'borderRadius': '4px', 'padding': '8px', 'cursor': 'pointer', 'fontWeight': 'bold'}),
                ]),
            ]),

            # Session controls
            html.Div(style=CARD_STYLE, children=[
                html.Div("SESSION", style=LABEL_STYLE),
                html.Button("Add Annotation", id='btn-annotate', n_clicks=0,
                            style={'backgroundColor': '#585b70', 'color': '#ccc', 'border': 'none', 'borderRadius': '4px', 'padding': '5px 10px', 'cursor': 'pointer', 'marginTop': '5px', 'width': '100%'}),
                dcc.Input(id='annotation-text', type='text', placeholder='Note...',
                          style={'width': '100%', 'marginTop': '5px', 'backgroundColor': '#313244', 'color': '#fff', 'border': '1px solid #555', 'borderRadius': '4px', 'padding': '5px', 'boxSizing': 'border-box'}),
            ]),

            # Event log
            html.Div(style={**CARD_STYLE, 'maxHeight': '200px', 'overflowY': 'auto'}, children=[
                html.Div("EVENT LOG", style=LABEL_STYLE),
                html.Pre(id='event-log-display', style={'color': '#888', 'fontSize': '10px', 'margin': '0', 'whiteSpace': 'pre-wrap'}),
            ]),
        ]),

        # ============================================================
        # CENTER COLUMN — Primary time-series plots
        # ============================================================
        html.Div(children=[
            # Pressure + setpoint overlay
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-pressure', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
            # RPM
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-rpm', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
            # Power
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-power', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
        ]),

        # ============================================================
        # RIGHT COLUMN — Analysis plots
        # ============================================================
        html.Div(children=[
            # Voltage + Current (dual axis)
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-electrical', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
            # PID performance (error over time)
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-pid', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
            # Cumulative energy
            html.Div(style=CARD_STYLE, children=[
                dcc.Graph(id='plot-energy', config=PLOT_CONFIG, style={'height': '220px'}),
            ]),
        ]),
    ]),

    # --- UPDATE TIMER (drives all callbacks) ---
    dcc.Interval(id='update-timer', interval=UI_UPDATE_MS, n_intervals=0),
])


# ============================================================================
# CALLBACKS (Dash updates the UI by calling these on the timer interval)
# ============================================================================

@app.callback(
    [
        Output('state-display', 'children'),
        Output('state-display', 'style'),
        Output('comms-indicator', 'children'),
        Output('live-readouts', 'children'),
        Output('fault-bar', 'children'),
        Output('fault-bar', 'style'),
        Output('event-log-display', 'children'),
        Output('plot-pressure', 'figure'),
        Output('plot-rpm', 'figure'),
        Output('plot-power', 'figure'),
        Output('plot-electrical', 'figure'),
        Output('plot-pid', 'figure'),
        Output('plot-energy', 'figure'),
    ],
    [Input('update-timer', 'n_intervals')]
)
def update_dashboard(n):
    """Master update callback — fires every UI_UPDATE_MS milliseconds."""

    with data_lock:
        # Snapshot the deques into lists (fast, avoids holding the lock during plotting)
        t = list(data['time_s'])
        pressures = list(data['pressure'])
        flows = list(data['flow'])
        rpms = list(data['rpm'])
        voltages = list(data['voltage'])
        currents = list(data['current'])
        powers = list(data['power'])
        setpoints = list(data['setpoint'])
        pid_outputs = list(data['pid_output'])
        pid_errors = list(data['pid_error'])
        energies = list(data['energy_mJ'])
        states = list(data['state'])
        faults_list = list(data['faults'])

    # --- State display ---
    if states:
        current_state = states[-1]
        state_text = STATE_NAMES[current_state] if current_state < len(STATE_NAMES) else f"UNKNOWN({current_state})"
        state_style = {**VALUE_STYLE, 'fontSize': '20px', 'color': STATE_COLORS[min(current_state, len(STATE_COLORS)-1)]}
    else:
        state_text = "NO DATA"
        state_style = {**VALUE_STYLE, 'fontSize': '20px', 'color': '#888'}

    # --- Comms health ---
    age = time.time() - last_valid_packet_time
    if age < 0.5:
        comms_text = f"● CONNECTED — {packet_count} pkts, {error_count} errs"
        comms_color = '#51cf66'
    elif age < 2.0:
        comms_text = f"● STALE ({age:.1f}s) — {packet_count} pkts"
        comms_color = '#ffd43b'
    else:
        comms_text = f"● DISCONNECTED ({age:.0f}s)"
        comms_color = '#fa5252'
    comms_html = html.Span(comms_text, style={'color': comms_color})

    # --- Live readouts ---
    if pressures:
        readout_lines = [
            f"Pressure:  {pressures[-1]:7.1f} mbar",
            f"Flow:      {flows[-1]:7.2f} LPM",
            f"RPM:       {rpms[-1]:7.0f}",
            f"Voltage:   {voltages[-1]:7.3f} V",
            f"Current:   {currents[-1]:7.3f} A",
            f"Power:     {powers[-1]:7.3f} W",
            f"Energy:    {energies[-1]:7.1f} J",
        ]
        readouts = html.Pre('\n'.join(readout_lines), style={'margin': 0})
    else:
        readouts = html.Pre("Waiting for data...", style={'margin': 0, 'color': '#666'})

    # --- Fault bar ---
    active_faults = faults_list[-1] if faults_list else 0
    if active_faults == 0:
        fault_text = "No Active Faults"
        fault_style = {**CARD_STYLE, 'backgroundColor': '#1e3a1e', 'textAlign': 'center', 'padding': '8px', 'fontSize': '14px', 'color': '#51cf66'}
    else:
        fault_names = [FAULT_NAMES[i] for i in range(16) if active_faults & (1 << i)]
        fault_text = "FAULT: " + " | ".join(fault_names)
        fault_style = {**CARD_STYLE, 'backgroundColor': '#3a1e1e', 'textAlign': 'center', 'padding': '8px', 'fontSize': '14px', 'color': '#fa5252', 'fontWeight': 'bold'}

    # --- Event log ---
    log_text = '\n'.join(list(event_log)[:30])

    # === PLOTS ===
    # Helper: trim to last N seconds for rolling view
    def rolling_window(t_arr, seconds=60):
        """Returns start index for a rolling window of the last N seconds."""
        if not t_arr:
            return 0
        cutoff = t_arr[-1] - seconds
        for i, tv in enumerate(t_arr):
            if tv >= cutoff:
                return i
        return 0

    ri = rolling_window(t, 60)  # Last 60 seconds
    t_window = t[ri:]

    # --- Pressure + Setpoint ---
    fig_pressure = go.Figure()
    fig_pressure.add_trace(go.Scattergl(x=t_window, y=pressures[ri:], mode='lines', name='Pressure', line=dict(color='#89b4fa', width=2)))
    fig_pressure.add_trace(go.Scattergl(x=t_window, y=setpoints[ri:], mode='lines', name='Setpoint', line=dict(color='#f38ba8', width=1, dash='dash')))
    fig_pressure.update_layout(**DARK_PLOT_LAYOUT, title='Pressure vs Setpoint (mbar)', yaxis_title='mbar')

    # --- RPM ---
    fig_rpm = go.Figure()
    fig_rpm.add_trace(go.Scattergl(x=t_window, y=rpms[ri:], mode='lines', name='RPM', line=dict(color='#a6e3a1', width=2)))
    fig_rpm.update_layout(**DARK_PLOT_LAYOUT, title='Alternator RPM', yaxis_title='RPM')

    # --- Power ---
    fig_power = go.Figure()
    fig_power.add_trace(go.Scattergl(x=t_window, y=powers[ri:], mode='lines', name='Power', line=dict(color='#f9e2af', width=2), fill='tozeroy'))
    fig_power.update_layout(**DARK_PLOT_LAYOUT, title='Instantaneous Power (W)', yaxis_title='W')

    # --- Electrical (Voltage + Current, dual Y axis) ---
    fig_electrical = go.Figure()
    fig_electrical.add_trace(go.Scattergl(x=t_window, y=voltages[ri:], mode='lines', name='Voltage (V)', line=dict(color='#cba6f7', width=2)))
    fig_electrical.add_trace(go.Scattergl(x=t_window, y=currents[ri:], mode='lines', name='Current (A)', line=dict(color='#fab387', width=2), yaxis='y2'))
    fig_electrical.update_layout(
        **DARK_PLOT_LAYOUT,
        title='Electrical Output',
        yaxis=dict(title='Voltage (V)', gridcolor='#333', side='left'),
        yaxis2=dict(title='Current (A)', overlaying='y', side='right', gridcolor='#333'),
    )

    # --- PID Error ---
    fig_pid = go.Figure()
    fig_pid.add_trace(go.Scattergl(x=t_window, y=pid_errors[ri:], mode='lines', name='PID Error', line=dict(color='#94e2d5', width=2)))
    fig_pid.add_hline(y=0, line_dash="dash", line_color="#666")
    fig_pid.update_layout(**DARK_PLOT_LAYOUT, title='PID Error (Setpoint − Actual)', yaxis_title='Error')

    # --- Cumulative Energy ---
    fig_energy = go.Figure()
    fig_energy.add_trace(go.Scattergl(x=t, y=energies, mode='lines', name='Cumulative Energy', line=dict(color='#f2cdcd', width=2), fill='tozeroy'))
    fig_energy.update_layout(**DARK_PLOT_LAYOUT, title='Cumulative Energy Output (J)', yaxis_title='Joules')
    # NOTE: Energy plot shows ALL time, not rolling window — you want the full session integral

    return (
        state_text, state_style,
        comms_html,
        readouts,
        fault_text, fault_style,
        log_text,
        fig_pressure, fig_rpm, fig_power,
        fig_electrical, fig_pid, fig_energy,
    )


# --- Command button callbacks ---

@app.callback(Output('btn-setpoint', 'n_clicks'), Input('btn-setpoint', 'n_clicks'), State('setpoint-input', 'value'), prevent_initial_call=True)
def on_setpoint(n, val):
    if val is not None:
        send_command(f"SET:{val}")
    return 0

@app.callback(Output('btn-pid', 'n_clicks'), Input('btn-pid', 'n_clicks'),
              State('kp-input', 'value'), State('ki-input', 'value'), State('kd-input', 'value'), prevent_initial_call=True)
def on_pid(n, kp, ki, kd):
    if kp is not None: send_command(f"KP:{kp}")
    if ki is not None: send_command(f"KI:{ki}")
    if kd is not None: send_command(f"KD:{kd}")
    return 0

@app.callback(Output('btn-run', 'n_clicks'), Input('btn-run', 'n_clicks'), prevent_initial_call=True)
def on_run(n):
    send_command("RUN")
    return 0

@app.callback(Output('btn-pause', 'n_clicks'), Input('btn-pause', 'n_clicks'), prevent_initial_call=True)
def on_pause(n):
    send_command("PAUSE")
    return 0

@app.callback(Output('btn-stop', 'n_clicks'), Input('btn-stop', 'n_clicks'), prevent_initial_call=True)
def on_stop(n):
    send_command("STOP")
    return 0

@app.callback(Output('btn-annotate', 'n_clicks'), Input('btn-annotate', 'n_clicks'), State('annotation-text', 'value'), prevent_initial_call=True)
def on_annotate(n, text):
    if text:
        log_event(f"NOTE: {text}")
    return 0


# ============================================================================
# ENTRY POINT
# ============================================================================

if __name__ == '__main__':
    log_event("Dashboard starting")

    # Start the data source thread
    if SERIAL_AVAILABLE and os.path.exists(SERIAL_PORT):
        t = threading.Thread(target=serial_reader_thread, daemon=True)
        t.start()
        log_event("Serial reader started")
    else:
        t = threading.Thread(target=mock_data_thread, daemon=True)
        t.start()
        log_event("Mock data generator started (no serial port)")

    # Launch Dash server
    # host='0.0.0.0' makes it accessible from other devices on the network
    # debug=False for production; set True during development for hot-reload
    print("\n  Dashboard running at http://localhost:8050")
    print("  (accessible from other devices at http://<pi-ip>:8050)\n")
    app.run(host='0.0.0.0', port=8050, debug=False)
