from Tkinter import *

root = Tk()
var = StringVar()
label = Label( root, textvariable=var, relief=RAISED )

def task():
    var.set("hello")
    root.after(2000, task)  # reschedule event in 2 seconds

var.set("Hey!? How are you doing?")
label.pack()
root.after(2000, task)
root.mainloop()