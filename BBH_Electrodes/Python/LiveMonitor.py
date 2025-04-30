import tkinter as tk
from tkinter import ttk
import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.animation as animation
import threading
import datetime
import time
from scipy.signal import iirnotch, filtfilt, butter

# Global flags and data buffer
NUM_ELEC =12
NUM_CH = NUM_ELEC + 6  # Number of channels to read from the serial port
running = False         # Controls whether the serial feed is running
toggle_metric = False   # Flag for toggling testing metric calculations
data_buffer = np.empty((0, NUM_CH))  # Buffer for incoming data (assumes NUM_CH channels)
record_file = None
metrics_file = None

# Variables for channel toggles and RMS labels
channel_vars = []  # Tkinter BooleanVars for each channel's display toggle
metrics_labels = []    

snr_state = "idle"
still_data = np.empty((0, NUM_CH))
active_data = np.empty((0, NUM_CH))

snr_label = None


# Global for controlling metrics storage rate (once per second)
last_metrics_store_time = 0

ser = None
read_thread = None


########################
# Filtering Functions  #
########################

def butter_bandpass(lowcut, highcut, fs, order=4):
    """
    Design a Butterworth bandpass filter.
    :param lowcut: Lower cutoff frequency in Hz.
    :param highcut: Upper cutoff frequency in Hz.
    :param fs: Sampling frequency in Hz.
    :param order: Filter order.
    :return: Filter coefficients (b, a).
    """
    nyq = 0.5 * fs
    low = lowcut / nyq
    high = highcut / nyq
    b, a = butter(order, [low, high], btype='band', analog=False)
    return b, a

def bandpass_filter(data, lowcut=20, highcut=450, fs=12000, order=4):
    """
    Apply a bandpass filter to each channel of the data.
    :param data: 2D numpy array of shape (samples, channels).
    :param lowcut: Lower cutoff frequency in Hz (e.g., 20).
    :param highcut: Upper cutoff frequency in Hz (e.g., 450).
    :param fs: Sampling frequency in Hz.
    :param order: Filter order (e.g., 4).
    :return: Filtered 2D numpy array.
    """
    b, a = butter_bandpass(lowcut, highcut, fs, order=order)
    filtered_data = np.zeros_like(data)
    for ch in range(data.shape[1]):
        filtered_data[:, ch] = filtfilt(b, a, data[:, ch])
    return filtered_data

def notch_filter_50Hz(signal, fs=1000.0, quality=60.0):
    """Apply a 50 Hz notch filter to each column in 'signal'.
    :param signal: 2D numpy array of shape (samples, channels)
    :param fs: Sampling frequency (Hz)
    :param quality: Q-factor for the notch filter
    :return: Filtered 2D numpy array
    """
    # Design a notch filter at 50 Hz
    w0 = 50.0 / (fs / 2.0)  # Normalized frequency
    b, a = iirnotch(w0, quality)

    # Filter each channel
    filtered = np.zeros_like(signal)
    for ch in range(signal.shape[1]):
        filtered[:, ch] = filtfilt(b, a, signal[:, ch])

    return filtered


def teager_kaiser_energy(signal):
    """Apply the Teager-Kaiser Energy operator to each column.
    TKE formula for x[n]: y[n] = x[n]^2 - x[n+1]*x[n-1].
    We'll set boundary points (0, -1) to zero for simplicity.

    :param signal: 2D numpy array of shape (samples, channels)
    :return: 2D numpy array of TKE outputs
    """
    tke_out = np.zeros_like(signal)
    for ch in range(signal.shape[1]):
        x = signal[:, ch]
        for n in range(1, len(x) - 1):
            tke_out[n, ch] = x[n] * x[n] - x[n + 1] * x[n - 1]
        # The first and last remain 0.
    return tke_out


def moving_average(signal, window_size=5):
    """Apply a simple moving average to each column.

    :param signal: 2D numpy array of shape (samples, channels)
    :param window_size: Number of samples for the moving average
    :return: 2D numpy array of smoothed outputs
    """
    smoothed = np.zeros_like(signal)
    for ch in range(signal.shape[1]):
        x = signal[:, ch]
        # Use 'same' mode so the output has the same length.
        smoothed[:, ch] = np.convolve(x, np.ones(window_size) / window_size, mode='same')
    return smoothed

# Serial reading function (runs in a separate thread)
def read_serial_data():
    global running, data_buffer, record_file, ser
        # This will hold the last-seen IMU reading
    latest_imu = None
    # try:
    #     ser = serial.Serial(port, baud, timeout=1)
    #     ser.write('!connect\r'.encode())
    #     # in_line = ser.readline().decode()
        
    # except Exception as e:
    #     print("Error opening serial port:", e)
    #     return
    while running:
        try:
            # Read a line and parse it (expecting comma-separated floats)
            line = ser.readline().decode('utf-8', errors='replace').strip()
            print(line)
            if not line:
                continue
            parts = line.split('|')
            # print(parts)
            if len(parts) != 2:
                continue

            parts_elec = parts[0].split(',')
            parts_imu = parts[1].split(',')

            if len(parts_imu) == 6:
                # convert once, store for next ELEC
                latest_imu = [float(x) for x in parts_imu]
                # print(latest_imu)
                
            if len(parts_elec) == 12 and latest_imu is not None:
                # print(parts_elec)
                # Ensure we have data for all NUM_CH channels
                elec_floats = [float(x) for x in parts_elec]
                # print(elec_floats)

                new_row = elec_floats + latest_imu
                # Append new data and keep a fixed-size buffer (last 1000 samples)
                data_buffer = np.vstack([data_buffer, new_row])
                if data_buffer.shape[0] > 1000:
                    data_buffer = data_buffer[-1000:]
                if record_file is not None:
                    record_file.write(','.join(parts) + '\n')
        except Exception as e:
            print("Error reading/parsing data:", e)
    # ser.write('!DC\r'.encode())
    # ser.close()


    if record_file is not None:
        record_file.close()

# Button callback: start the feed
def start_feed():
    global running, record_file, last_metrics_store_time, metrics_file, ser, read_thread
    if running:
        return
    # 1) open the port
    try:
        ser = serial.Serial("COM4", 250000, timeout=1,
                            dsrdtr=False, rtscts=False)  # disable hardware handshakes
        time.sleep(2)                                     # wait for USB-CDC stabilization :contentReference[oaicite:9]{index=9}
        ser.reset_input_buffer()                          # clear stale data
        ser.reset_output_buffer()
        ser.write(b'!connect\r')    
    except Exception as e:
        print("Error opening serial port:", e)
        return
    
    running = True
    # Start the serial reading in a new daemon thread
    now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    r_filename = f"emg_dataLog_{now_str}.csv"
    record_file = open(r_filename, 'w')

    metrics_filename = f"Metrics_{now_str}.csv"
    metrics_file = open(metrics_filename, "w")
    # Write header for metrics file
    metrics_file.write("Timestamp;")
    metrics_file.write("\n")
    last_metrics_store_time = time.time()

    read_thread = threading.Thread(target=read_serial_data, daemon=True)
    read_thread.start()
    
    print(f"Started data feed from serial port. Recording to {r_filename}")


# Button callback: stop the feed
def stop_feed():
    global running, ser, read_thread, record_file, metrics_file
    if not running:
        return

    # 1) tell the board to stop streaming
    try:
        ser.write(b'!DC\r')
    except Exception as e:
        print("Error sending DC command:", e)

    # 2) stop the reader loop
    running = False
    # 3) wait for thread to finish
    if read_thread is not None:
        read_thread.join(timeout=1.0)

    # 4) close files
    if record_file:
        record_file.close()
    if metrics_file:
        metrics_file.close()

    # 5) close serial port
    try:
        time.sleep(0.1)                       # allow command to flush :contentReference[oaicite:5]{index=5}
        ser.reset_input_buffer()              # drop any incoming data
        ser.reset_output_buffer()             # discard any outgoing data
        ser.close()                           # close port handle
        time.sleep(0.5)   
    except:
        pass

    ser = None
    read_thread = None

    print("Stopped data feed.")

# Button callback: toggle metric computation/display
def toggle_metrics():
    global toggle_metric
    toggle_metric = not toggle_metric
    print("Toggle Metrics is now", "ON" if toggle_metric else "OFF")

############################################################
#             SNR Measurement Button/Logic                 #
############################################################

def snr_button():
    """
    Cycles through STILL -> ACTIVE -> DONE states to compute SNR.
    1) First press: start STILL phase
    2) Second press: start ACTIVE phase
    3) Third press: compute SNR and display
    4) Fourth press: resets to idle
    """
    global snr_state, still_data, active_data
    if snr_state == "idle":
        snr_state = "still"
        still_data = np.empty((0, NUM_CH))
        snr_label.config(text="SNR State: Collecting STILL data... (Press again for ACTIVE)")
    elif snr_state == "still":
        snr_state = "active"
        active_data = np.empty((0, NUM_CH))
        snr_label.config(text="SNR State: Collecting ACTIVE data... (Press again to compute SNR)")
    elif snr_state == "active":
        # Compute SNR
        snr_state = "done"
        snr_values = compute_snr(still_data, active_data)
        s_rms_vals, s_cv_vals, s_mav_vals = compute_metrics(still_data)
        a_rms_vals, a_cv_vals, a_mav_vals = compute_metrics(active_data)

        # Display SNR results
        text_lines = ["SNR Results:"]
        for ch in range(NUM_CH):
            text_lines.append(f"Ch {ch+1}: {snr_values[ch]:.2f} dB")
        snr_label.config(text="\n".join(text_lines) + "\n(Press again to reset)")
    
        # Store SNR results to metrics file
        if metrics_file is not None:
            snr_line = f"SNR;{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')};"
            metrics_file.write("still_data\n")

            start_still = still_data[:10]
            metrics_file.write("start\n")
            for ch in range(NUM_CH):
                for value in start_still[:, ch]:
                    metrics_file.write(f"{value};")
                metrics_file.write("\n")

            metrics_file.write("end\n")
            end_still = still_data[-10:]

            for ch in range(NUM_CH):
                for value in end_still[:, ch]:
                    metrics_file.write(f"{value};")
                metrics_file.write("\n")
            metrics_file.write("\n")

            metrics_file.write("active_data\n")
            start_active = active_data[:10]
            metrics_file.write("start\n")
            for ch in range(NUM_CH):
                for value in start_active[:, ch]:
                    metrics_file.write(f"{value};")
                metrics_file.write("\n")
            
            metrics_file.write("end\n")
            end_active = active_data[-10:]
            for ch in range(NUM_CH):
                for value in end_active[:, ch]:
                    metrics_file.write(f"{value};")  

            metrics_file.write("\n")

            metrics_file.write("snr_values\n")
            for val in snr_values:
                snr_line += f"{val:.2f};"
            snr_line = snr_line.rstrip(';') + "\n"
            metrics_file.write(snr_line)
            general_a_max_mav = np.max(a_mav_vals) if np.max(a_mav_vals) != 0 else 1e-12

            for ch in range(NUM_CH):
                    curr_mav_sdata = s_mav_vals[ch] 
                    curr_mav_adata = a_mav_vals[ch]
                    curr_cv_sdata = s_cv_vals[ch]
                    curr_cv_adata = a_cv_vals[ch]
                    curr_rms_sdata = s_rms_vals[ch]
                    curr_rms_adata = a_rms_vals[ch]

                    s_mean_mav = np.mean(curr_mav_sdata) if np.mean(curr_mav_sdata) != 0 else 1e-12
                    a_max_mav = np.max(curr_mav_adata) if np.max(curr_mav_adata) != 0 else 1e-12
                    s_mean_cv = np.mean(curr_cv_sdata) if np.mean(curr_cv_sdata) != 0 else 1e-12
                    a_max_cv = np.max(curr_cv_adata) if np.max(curr_cv_adata) != 0 else 1e-12
                    s_mean_rms = np.mean(curr_rms_sdata) if np.mean(curr_rms_sdata) != 0 else 1e-12
                    a_max_rms = np.max(curr_rms_adata) if np.max(curr_rms_adata) != 0 else 1e-12
                    
                    mav_perc = (a_max_mav / s_mean_mav) * 100
                    general_mav_perc = (a_max_mav / general_a_max_mav) * 100
                    cv_perc = (a_max_cv / s_mean_cv) * 100
                    rms_perc = (a_max_rms / s_mean_rms) * 100

                    perc_line = f"Ch{ch+1}_MAV_perc;{mav_perc:.2f};Ch{ch+1}_MAV_general_perc;{general_mav_perc:.2f};Ch{ch+1}_CV_perc;{cv_perc:.2f};Ch{ch+1}_RMS_perc;{rms_perc:.2f}\n"
                    metrics_file.write(perc_line)

            metrics_file.flush()
    else:
        # snr_state == "done"
        snr_state = "idle"
        snr_label.config(text="SNR State: Idle. (Press to start STILL)")

def compute_snr(still_arr, active_arr):
    """
    Compute SNR = 20*log10(RMS_active / RMS_still) for each channel.
    If still or active arrays are empty, returns an array of NaNs.
    """
    if still_arr.shape[0] == 0 or active_arr.shape[0] == 0:
        return np.full((NUM_CH,), np.nan)

    # RMS of STILL
    rms_still = np.sqrt(np.mean(still_arr**2, axis=0))
    # RMS of ACTIVE
    rms_active = np.sqrt(np.mean(active_arr**2, axis=0))

    # Avoid division by zero
    rms_still[rms_still == 0] = 1e-12

    # SNR in dB
    snr_db = 20 * np.log10(rms_active / rms_still)
    return snr_db



############################################################
#          Metrics Computation (RMS, CV, MAV)              #
############################################################

def compute_metrics(data):
    """
    Computes RMS, CV, MAV for each channel over 'data'.
    Returns arrays of shape (NUM_CH,).
    """
    # If data is empty, return zeros
    if data.shape[0] == 0:
        ch_count = data.shape[1] if data.shape[1] else NUM_CH
        return (np.zeros(ch_count), np.zeros(ch_count), np.zeros(ch_count))

    # RMS
    rms = np.sqrt(np.mean(data**2, axis=0))

    # CV = std(x) / mean(x)  (watch for zero mean)
    mean_vals = np.mean(data, axis=0)
    std_vals = np.std(data, axis=0)
    # To avoid division by zero
    mean_vals[mean_vals == 0] = 1e-12
    cv = std_vals / np.abs(mean_vals)

    # MAV = mean(|x|)
    mav = np.mean(np.abs(data), axis=0)

    return (rms, cv, mav)

# Set up the Tkinter window
root = tk.Tk()
root.title("HD-EMG Live Feed")
# Variable for lowpass filter toggle
lowpass_var = tk.BooleanVar(value=False)
# Create a matplotlib figure for plotting
fig, ax = plt.subplots()
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

# Initialize plot lines for NUM_CH channels
lines = []
colors = plt.cm.viridis(np.linspace(0, 1, NUM_CH))
for ch in range(NUM_CH):
    (line,) = ax.plot([], [], color=colors[ch], label=f"Channel {ch+1}")
    lines.append(line)

ax.set_xlim(0, 500)  # Display last 1000 samples on X-axis

ax.set_ylim(-5000, 5000)
#ax.set_ylim(-4000, 4000000)

ax.set_xlabel("Sample")
ax.set_ylabel("Filtered Value")
ax.legend(loc="upper right")

# Create a control frame for the Start, Stop, and Toggle Metrics buttons
control_frame = ttk.Frame(root)
control_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)

start_btn = ttk.Button(control_frame, text="Start Feed", command=start_feed)
start_btn.pack(side=tk.LEFT, padx=5)

stop_btn = ttk.Button(control_frame, text="Stop Feed", command=stop_feed)
stop_btn.pack(side=tk.LEFT, padx=5)

toggle_metrics_btn = ttk.Button(control_frame, text="Toggle Metrics", command=toggle_metrics)
toggle_metrics_btn.pack(side=tk.LEFT, padx=5)

lowpass_chk = ttk.Checkbutton(control_frame, text="Bandpass Filter", variable=lowpass_var)
lowpass_chk.pack(side=tk.LEFT, padx=5)

# Button for SNR measurement
snr_btn = ttk.Button(control_frame, text="SNR Measurement", command=lambda: snr_button())
snr_btn.pack(side=tk.LEFT, padx=5)

# Create a frame for channel toggles (checkboxes)
toggle_frame = ttk.Frame(root)
toggle_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)

# Metrics display frame
metrics_frame = ttk.Frame(root)
metrics_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)

# Initialize toggles and RMS labels for each channel
for ch in range(NUM_CH):
    var = tk.BooleanVar(value=True)
    channel_vars.append(var)
    # Checkbutton to toggle channel visibility
    chk = ttk.Checkbutton(toggle_frame, text=f"Channel {ch+1}", variable=var)
    chk.pack(anchor='w')
    
    # Label to display the Metrics values for the channel
    lbl = ttk.Label(metrics_frame, text=f"Channel {ch+1} RMS: N/A")
    lbl.pack(anchor='w')
    metrics_labels.append(lbl)

# Label to show SNR state and results
snr_label = ttk.Label(root, text="SNR State: Idle. (Press to start STILL)")
snr_label.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=5)


# Update function for the animation
def update(frame):
    global data_buffer, still_data, active_data, snr_state, last_metrics_store_time

    if data_buffer.shape[0] == 0:
        return lines

    # If lowpass filter toggle is enabled, filter the data
    if lowpass_var.get():
        # grab elec and imu separately
        elec = data_buffer[:, :12]
        imu  = data_buffer[:, 12:]

        # filter the elec channels
        filtered12 = notch_filter_50Hz(elec)

        # compute a common row-count
        n_rows = min(filtered12.shape[0], imu.shape[0])

        # slice both to the last n_rows samples
        f12 = filtered12[-n_rows:, :]
        iu  = imu         [-n_rows:, :]

        # stitch back together
        data_to_plot = np.concatenate([f12, iu], axis=1)
    else:
        data_to_plot = data_buffer


    x = np.arange(data_to_plot.shape[0])
    for ch, line in enumerate(lines):
        if channel_vars[ch].get():  # If toggled on, display data
            line.set_visible(True)
            line.set_data(x, data_to_plot[:, ch])
        else:
            line.set_visible(False)
        # line.set_data(x, filtered_data[:, ch])

    # 3) Collect data for SNR if in STILL or ACTIVE
    #    We accumulate *all* the new data in the relevant array.
    #    In this example, we simply re-copy the entire buffer each time,
    #    but you might want to store incremental data or do a ring buffer.
    if snr_state == "still":
        # Append current data to still_data
        still_data = np.vstack([still_data, data_to_plot])
    elif snr_state == "active":
        # Append current data to active_data
        active_data = np.vstack([active_data, data_to_plot])

    # 4) Compute and update metrics (RMS, CV, MAV) if toggled
    if toggle_metric:
        # Use the data_to_plot for these computations
        rms_vals, cv_vals, mav_vals = compute_metrics(data_to_plot)
        for ch in range(NUM_CH):
            metrics_labels[ch].config(
                text=(
                    f"Ch {ch+1} -> "
                    f"RMS: {rms_vals[ch]:.2f}; "
                    f"CV: {cv_vals[ch]:.2f}; "
                    f"MAV: {mav_vals[ch]:.2f}"
                )
            )
    return lines

# Create the animation (updates every 30 ms)
ani = animation.FuncAnimation(fig, update, interval=60, blit=True)



root.mainloop()
