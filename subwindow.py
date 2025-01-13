import tkinter as tk
import threading
from tkinter import Label
from tkinter import font as tkfont

def show_commands():
    """Create a separate Tkinter window for commands."""
    sub_window = tk.Tk()
    sub_window.title("Commands")
    sub_window.geometry("300x200")

    custom_font = tkfont.Font(family="Helvetica", size=12, weight="bold")

    commands = [
        "Left Click: Set Start/End/Barrier",
        "Right Click: Remove Barrier",
        "Space: Start Pathfinding",
        "Q: Generate Random Map",
        "C: Clear Grid",
    ]
    for command in commands:
        Label(
            sub_window, 
            text=command,
            font=custom_font,
            padx=10,
            pady=5
            ).pack()

    sub_window.mainloop()
# def main():
#     threading.Thread(target=show_commands, daemon=True).start()

#     while True:
#         pass

# if __name__ == "__main__":
#     main()