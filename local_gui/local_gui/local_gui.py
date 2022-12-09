import tkinter as tk
from tkinter import ttk

class LocalGui:
    def __init__(self,root):
        self.root = root
        # self.root.title("Local GUI")

    style_unpressed = ttk.Style()
    style_unpressed.configure(
        "unpressed.TButton",
        background = "gray",
        foreground = "black",
        font = ("", 40),
        relief = "flat",
        borderwidth = 0,
        padding = 0
    )

    style_pressed = ttk.Style()
    style_pressed.configure(
        "pressed.TButton",
        background = "black",
        foreground = "black",
        font = ("", 40),
        relief = "flat",
        borderwidth = 0,
        padding = 0
    )

    def start_window(self):

        # self.root.place(x = 0, y = 0)
        self.b0 = ttk.Button(
            self.root,
            text = "AUTO",
            command = self.button_clicked,
            style = "unpressed.TButton"
            )
        self.b0.pack()

        self.b1 = ttk.Button(
            self.root,
            text = "MANUAL",
            command = self.button_clicked,
            style="unpressed.TButton"
            )
        self.b1.pack()

        self.b2 = ttk.Button(
            self.root,
            text = "PRECISION",
            command = self.button_clicked,
            style="unpressed.TButton"
            )
        self.b2.pack()

        self.b3 = ttk.Button(
            self.root,
            text = "CLIMB",
            command = self.button_clicked,
            style="unpressed.TButton"
            )
        self.b3.pack()

        # self.b0.place(
        #     x = 136, y = 84,
        #     width = 385,
        #     height = 174)
        # self.b1.place(
        #     x = 557, y = 84,
        #     width = 385,
        #     height = 174)
        # self.b2.place(
        #     x = 978, y = 84,
        #     width = 385,
        #     height = 174)
        # self.b3.place(
        #     x = 1399, y = 84,
        #     width = 385,
        #     height = 174)

        self.root.resizable(True, True)
        self.root.mainloop()

    # change button style when clicked
    def button_clicked(self):
        if self.b0["style"] == "unpressed.TButton":
            self.b0["style"] = "pressed.TButton"
        else:
            self.b0["style"] = "unpressed.TButton"

        if self.b1["style"] == "unpressed.TButton":
            self.b1["style"] = "pressed.TButton"
        else:
            self.b1["style"] = "unpressed.TButton"

        if self.b2["style"] == "unpressed.TButton":
            self.b2["style"] = "pressed.TButton"
        else:
            self.b2["style"] = "unpressed.TButton"

        if self.b3["style"] == "unpressed.TButton":
            self.b3["style"] = "pressed.TButton"
        else:
            self.b3["style"] = "unpressed.TButton"


if __name__ == "__main__":
    app = tk.Tk()
    # app.geometry("1980x1080")
    window = LocalGui(app)
    window.start_window()
    app.mainloop()


