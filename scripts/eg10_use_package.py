import sys
print(sys.path)

try:
    import list_ops as lo
    print(lo.spam)
except ModuleNotFoundError as e:
    print(e)

import sys
new_path = 'D:/Work/ian-lecture/scripts/package'
if new_path not in sys.path:
    sys.path.append(new_path)
print(sys.path)
import list_ops as lo
print(lo.spam)

create = 2
import os
import shutil
tempdir = "D:/tempdir"
tempfile1 = "D:/tempdir/tempfile1.txt"
tempfile2 = "D:/tempdir/tempfile2.txt"

if create == 1:
    os.mkdir(tempdir)
    try:
        os.mkdir(tempdir)
    except FileExistsError as e:
        print(e)

    with open(tempfile1, "w") as f:
        f.write("blurblurblur1")
    with open(tempfile2, "w") as f:
        f.write("blurblurblur2")
    print("two files created")

elif create == 0:
    os.remove(tempfile1)
    try:
        os.remove(tempfile1)
    except FileNotFoundError as e:
        print(e)

    shutil.rmtree(tempdir)
    try:
        shutil.rmtree(tempdir)
    except FileNotFoundError as e:
        print(e)
    shutil.rmtree(tempdir, ignore_errors=True)
    print("rmtree non-existing dir but no error")


import os
import shutil
tempdir = "D:/tempdir"
tempfile1 = "D:/tempdir/tempfile1.txt"
os.mkdir(tempdir)

if os.path.isfile(tempfile1):
    print(f"{tempfile1} exists. remove it now")
    os.remove(tempfile1)
else:
    print(f"{tempfile1} does NOT exists")

if os.path.isdir(tempdir):
    print(f"{tempdir} exists. remove it now")
    shutil.rmtree(tempdir)
else:
    print(f"{tempdir} does NOT exists")

curpath = os.path.abspath(__file__)
print("current file:", [__file__])
print("full path of current file", [curpath])


import os
curfile = __file__
curfile_path = os.path.abspath(curfile)
print("current file:", curfile)
print("current file absolute path:", curfile_path)
filename = os.path.basename(curfile_path)
print("current file name:", filename)
pathname = os.path.dirname(curfile_path)
print("parent dir path:", pathname)
print("grand parent dir path:", os.path.dirname(pathname))
newfile = os.path.join(pathname, "newfile.txt")
print("new file name:", newfile)
with open(newfile, "w") as f:
    f.write("new file created beside python file")
newpath = os.path.join(pathname, "new", "path", "name")
print([newpath])


import os
import glob
curfile = os.path.abspath(curfile)
curdirpath = os.path.dirname(curfile)

files = os.listdir(curdirpath)
print("file list:", files)

search_pattern = os.path.join(curdirpath, "*")
print("search pattern:", search_pattern)
filelist = glob.glob(search_pattern)
print("file list:", filelist)

search_pattern = os.path.join(curdirpath, "*.py")
print("search pattern:", search_pattern)
pyfilelist = glob.glob(search_pattern)
print("python file list:", pyfilelist)

filelist = [os.path.basename(path) for path in filelist if os.path.isfile(path)]
dirlist = [os.path.basename(path) for path in filelist if os.path.isdir(path)]
print("file list in current dir:", filelist)
print("subdir list in current dir:", dirlist)
