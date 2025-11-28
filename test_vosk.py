from vosk import Model, KaldiRecognizer
import sounddevice as sd
import serial
import json
import time

MODEL_PATH = r"C:\Users\User\Downloads\vosk-model-small-en-us-0.15\vosk-model-small-en-us-0.15"
SERIAL_PORT = "COM3"  # Change this to your Arduino COM port
BAUD_RATE = 9600
# =====================

# Connect to Arduino
arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
# Wait for Arduino
time.sleep(2)

# Load Vosk model
model = Model(MODEL_PATH)
rec = KaldiRecognizer(model, 16000)

# Speech callback
def callback(indata, frames, time_info, status):
    data = bytes(indata)
    if rec.AcceptWaveform(data):
        text = json.loads(rec.Result())['text']
        if text:
            print(f"Recognized: {text}")
            handle_command(text.lower())

# define command
def handle_command(text):
    # define command mapping
    commands = {
        # LED Basic
        "led on": "LED_ON",
        "led off": "LED_OFF",
        # LED modes
        "led solid": "LED_SOLID",
        "led blink fast": "LED_FAST",
        "led fast blink": "LED_FAST",
        "led blink slow": "LED_SLOW",
        "led slow blink": "LED_SLOW",

        # LCD basic
        "lcd on": "LCD_ON",
        "lcd off": "LCD_OFF",
        # LCD modes
        "lcd welcome": "LCD_WELCOME",
        "lcd ok": "LCD_OK",
        "lcd all systems ok": "LCD_OK",
        "lcd reset": "LCD_RESET",
        "lcd resetting": "LCD_RESET",

        # Motor basic
        "motor on": "MOTOR_ON",
        "motor off": "MOTOR_OFF",
        # Motor speeds
        "motor slow": "MOTOR_SLOW",
        "motor medium": "MOTOR_MED",
        "motor fast": "MOTOR_FAST",

        # Servo basic
        "servo on": "SERVO_ON",
        "servo off": "SERVO_OFF",
        # Servo angles
        "servo 45": "SERVO_45",
        "servo forty five": "SERVO_45",
        "servo 90": "SERVO_90",
        "servo ninety": "SERVO_90",
        "servo 180": "SERVO_180",
        "servo one eighty": "SERVO_180",
        # Natural forms
        "move servo": "SERVO_ON"
    }
    for phrase, cmd in commands.items():
        if phrase in text:
            print(f"arduino: {cmd}")
            arduino.write((cmd + "\n").encode())
            return
    print("No valid command found.")

# listening loop
with sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16', channels=1, callback=callback):
    print("🎤 Say command (Ctrl+C to stop)")
    while True:
        pass