import os
os.chdir("../external/src")
os.system("rm -rf Box2D/")
os.system("tar -zxvf Box2D.tgz")
os.chdir("./Box2D")
os.system("mkdir build296")
os.chdir("./build296")
os.system("cmake ../")
os.system("make")
os.system("make install")
os.chdir("../../../..")
os.system("make exe")
os.system("make doc")
os.system("make report")

