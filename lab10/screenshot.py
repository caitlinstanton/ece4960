import pygetwindow
import time
import pyautogui
import PIL
import os, sys, subprocess


def open_file(filename):
    if sys.platform == "win32":
        os.startfile(filename)
    else:
        opener = "open" if sys.platform == "darwin" else "xdg-open"
        subprocess.call([opener, filename])


# get screensize
x, y = pyautogui.size()
print(f"width={x}\theight={y}")

x2, y2 = pyautogui.size()
x2, y2 = int(str(x2)), int(str(y2))
print(x2 // 2)
print(y2 // 2)

# find new window title
z1 = pygetwindow.getAllTitles()
time.sleep(1)
print(len(z1))
# test with pictures folder
open_file("Pictures")
time.sleep(1)
z2 = pygetwindow.getAllTitles()
print(len(z2))
time.sleep(1)
z3 = [x for x in z2 if x not in z1]
z3 = ''.join(z3)
time.sleep(3)

# also able to edit z3 to specified window-title string like: "Sublime Text (UNREGISTERED)"
my = pygetwindow.getWindowsWithTitle(z3)[0]
# quarter of screen screensize
x3 = x2 // 2
y3 = y2 // 2
my.resizeTo(x3, y3)
# top-left
my.moveTo(0, 0)
time.sleep(3)
my.activate()
time.sleep(1)

# save screenshot
p = pyautogui.screenshot()
p.save(r'Pictures////p.png')

# edit screenshot
im = PIL.Image.open('Pictures//p.png')
im_crop = im.crop((0, 0, x3, y3))
im_crop.save('Pictures//p.jpg', quality=100)

# close window
time.sleep(1)
my.close()