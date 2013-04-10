import os
os.chdir("./scripts")
os.system("python3 g19_gen_csv.py")
os.system("python3 g19_gen_plots.py")
os.system("python3 g19_gen_html.py")
