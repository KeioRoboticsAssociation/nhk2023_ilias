import tkinter

class LocalGui:
    def __init__(self):
        self.root = tkinter.Tk()
        self.root.title("Local GUI")
        self.root.geometry("400x300")
        self.root.mainloop()

if __name__ == "__main__":
    LocalGui()
