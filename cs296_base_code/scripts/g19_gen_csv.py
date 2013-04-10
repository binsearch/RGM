import os
import re
os.chdir('../')
os.system("make exe")
os.system("mkdir -p ./data")
os.system("rm -f ./data/*")
for i in range(100):
	for j in range(100):
		os.system("./mybins/cs296_exe_19 "+ str(j+1) + " > ./data/out-" + str(j+1) + "-" + str(i+1) + ".txt")
os.system("touch ./data/lab05_g19_data.csv")
os.system("rm ./data/lab05_g19_data.csv")
os.system("touch ./data/lab05_g19_data.csv")
writefile=open('./data/lab05_g19_data.csv','w')
writefile.write('"rerun number","iteration value","step time","collision time","velocity time","position time","looptime"')
writefile.write("\n")
numregex=re.compile('[0-9.]+ ')	
for i in range(100):
	for j in range(100):
		file=open("./data/out-"+ str(j+1) + "-"+str(i+1)+".txt",'r').read()
		li=numregex.findall(file)
		#li=li[1:]
		#print(li)
		writefile.write(str(i+1)+","+str(j+1))
		for k in li:
			writefile.write(","+str(k[:]))
		writefile.write("\n")
os.system("rm ./data/*.txt")
		
		
