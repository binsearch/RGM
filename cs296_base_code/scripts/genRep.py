import os
os.chdir("../doc")
os.system("latex g19_proj_report.tex")
os.system("bibtex g19_proj_report.aux")
os.system("latex g19_proj_report.tex")
os.system("latex g19_proj_report.tex")
