import sounddevice as sd
import vosk
import json
import tkinter as tk
from threading import Thread
import time
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import difflib
import os
import transmitter
import time

mode="VOICE"


# Path to your downloaded Vosk model (update this path as needed)
MODEL_PATH = r"E:\Abu Samaha\Graduation Project\AI_Modes_final_codes\2-web_sockets\project_scripts\vosk-model-small-en-us-0.15"

# Verify the model path exists
if not os.path.exists(MODEL_PATH):
    print("Error: Model path does not exist!")
    exit(1)

# Load the Vosk model
vosk.SetLogLevel(-1)  # Suppress Vosk logs
try:
    model = vosk.Model(MODEL_PATH)
except Exception as e:
    print("Error loading model:", e)
    exit(1)

# Flag to control the main loop
stop_flag = False

# Initialize the recognizer with a sample rate of 16kHz
recognizer = vosk.KaldiRecognizer(model, 16000)

# Variable to track the last partial result and its timestamp (for debounce)
last_partial = ""
last_partial_time = 0

# Buffer for amplitude data for plotting
amplitude_data = []

def find_commands(recognized_text):

    words = recognized_text.split()
    detected_commands = []
    # List of exact-match commands
    command_list = ["stop", "move", "right", "left" ,"forward"]
    for word in words:
        if word in command_list:
            detected_commands.append(word)
        # Special handling for 'left': accept 'live' or similar words
        elif word.lower() == "live" or difflib.SequenceMatcher(None, word.lower(), "left").ratio() > 0.8:
            detected_commands.append("left")

        elif word.lower() == "for" or difflib.SequenceMatcher(None, word.lower(), "forward").ratio() > 0.8:
            detected_commands.append("forward")
    return detected_commands

def audio_callback(indata, frames, time_info, status):
    global last_partial, amplitude_data, last_partial_time
    if status:
        print(status)  # Print any audio callback errors

    # Convert audio data to bytes and calculate amplitude
    audio_data = indata.tobytes()
    amplitude = np.linalg.norm(indata)
    amplitude_data.append(amplitude)
    if len(amplitude_data) > 100:
        amplitude_data.pop(0)  # Limit buffer size for smooth plotting

    if audio_data:
        start_recognition_time = time.time()
        if recognizer.AcceptWaveform(audio_data):
            # Full recognition result
            result = json.loads(recognizer.Result())
            recognized_text = result.get("text", "")
            recognition_time = time.time() - start_recognition_time
            if recognized_text:
                commands_detected = find_commands(recognized_text)
                if commands_detected:
                    for cmd in commands_detected:
                        message=f"{cmd.upper()}|{mode}"
                        print(f"Command received: {cmd.upper()} (Processing time: {recognition_time:.4f} seconds)")
                        transmitter.send_command(message)

                else:
                    print("No commands detected in the recognized text.")
            last_partial = ""
        else:
            # Partial recognition result
            partial_result = json.loads(recognizer.PartialResult())
            current_partial = partial_result.get('partial', '')
            current_time = time.time()
            if current_partial and current_partial != last_partial and (current_time - last_partial_time) > 0.5:
                # First, check if the partial text contains any commands
                partial_commands = find_commands(current_partial)
                if partial_commands:
                    for cmd in partial_commands:
                        print(f"Partial command detected: ")
                else:
                    # If no command is detected, print truncated partial text (up to 10 letters)
                    truncated_partial = current_partial[:10]
                    if len(truncated_partial.strip()) >= 3:
                        print(f"Partial: {truncated_partial}")
                last_partial_time = current_time
                last_partial = current_partial

def update_plot():
    global amplitude_data
    if amplitude_data:
        ax.clear()
        ax.plot(amplitude_data, color='blue')
        ax.set_title("Amplitude of Detected Voice")
        ax.set_xlabel("Time")
        ax.set_ylabel("Amplitude")
        ax.set_ylim(0, max(1, max(amplitude_data)))  # Dynamic y-axis limit
        canvas.draw()
    if not stop_flag:
        root.after(50, update_plot)  # Schedule next update

def main_loop():
    global stop_flag
    with sd.InputStream(samplerate=16000, channels=1, dtype='int16', callback=audio_callback):
        print("Listening... Speak now!")
        while not stop_flag:
            sd.sleep(100)
    print("Program stopped.")

def stop_program():
    global stop_flag
    stop_flag = True
    root.destroy()  # Close the GUI window

# Create the GUI
root = tk.Tk()
root.title("Speech Recognition with Amplitude Plot")

# Create a Matplotlib figure for amplitude plotting
fig = Figure(figsize=(6, 4), dpi=100)
ax = fig.add_subplot(111)
canvas = FigureCanvasTkAgg(fig, master=root)
canvas.get_tk_widget().pack()

# Add a button to stop the program
stop_button = tk.Button(root, text="Stop", command=stop_program, font=("Arial", 14), bg="red", fg="white")
stop_button.pack(pady=20)

# Run the main loop in a separate thread
thread = Thread(target=main_loop)
thread.daemon = True
thread.start()

# Start updating the plot
update_plot()

# Run the Tkinter GUI event loop
root.mainloop()
