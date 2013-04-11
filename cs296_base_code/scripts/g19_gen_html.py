import sys,re,os
os.system("rm -rf ../doc/g19_lab09_report.html")

def main():
    
    
    f = open('../doc/g19_proj_report.tex','r')
    fo = open('../doc/g19_lab09_report.html','w')
    lines = f.readlines()
    k = len(lines)
    fo.write("<!DOCTYPE html>\n <html>\n <head> \n <title> CS 296 Group 19 Rube Goldberg Simulation</title> \n </head> \n <body>\n <h1>CS 296 Group 19 Rube Goldberg Simulation: Design and Inference Report</h1> \n")
    i = 0
    m = 0
    img1 = 0
    img2 = 0
    while  (i < k) :
        
        line = lines[i]
        match = re.search("\\\\section{([\w,\W]*)}" , line)
        match1 = re.search("\\\\subsection{([\w,\W]*)}" , line)
        
        #if m == 3:
         #   break
        
        if match:
            if m == 9:
                break
           # if m == 0:
            #    m = m + 1
             #   i = i + 1
              #  continue
            
            fo.write("<h2>")    
            fo.write(match.group(1))
            fo.write("</h2>\n")
            fo.write("<p>\n")
            while(i < k-1):
                        i = i + 1
                        line = lines[i]
                        match3 = re.search("\\\\subsection{([\w,\W]*)}" , line)
                        match4 = re.search("\\\\section{([\w,\W]*)}" , line)
                        match5 = re.search("([\w,\W]+)\\\\[\w,\W]+{([\w,\W]+)}([\w,\W]+)" , line)
                        match6 = re.search("bibliographystyle" , line)
                        #if img1 == 0:
                         #   fo.write("<img src=\"images/mainsim.png\" width=560 height=460 >")
                        
                        
                        #if match5:
                        #    print (match5.group(3))
                        if match3:
                            fo.write("</p>\n")
                            i = i - 1
                            break
                        
                        if match4:
                            fo.write("</p>\n")
                            i = i - 1
                            break
                        
                        if match6:
                            
                            i = k
                            break
                        
                        if match5:
                        
                            fo.write(match5.group(1))
                            fo.write("(")
                            fo.write(match5.group(2))
                            fo.write(")")
                            fo.write(match5.group(3))
                        else :
                            fo.write(line)
                                             
                         
            
            
            m = m + 1
            img1 = img1 + 1
        if match1:
           # print (img1)
           # print (img2)
            img2 = img2 + 1
            fo.write("<h3>")    
            fo.write(match1.group(1))
            fo.write("</h3>\n")
            
            #get the paragraph in subsection.
                
            while(i < k):
                i = i + 1
                line = lines[i]
                match2 = re.search("\\\\begin" , line)
                
                if match2:
                        
                      if img1 ==5:
                        
                        if img2 == 8:
                            fo.write("<img src=\"g19_proj_plot01.png\" width=560 height=460 >")
                            #fo.write("<img src=\"plot03.png\" width=560 height=460 >")    
                        
                        if img2 == 9:
                            fo.write("<img src=\"g19_proj_plot03.png\" width=600 height=560>")
                            #fo.write("<img src=\"../plots/g19_lab09_plot06.png\" width=600 height=560>")     
                        
                        if img2 == 10:
                            fo.write("<img src=\"g19_proj_plot05.png\" width=660 height=560>")    
                        
                        if img2 == 11:
                            fo.write("<img src=\"g19_proj_plot02.png\" width=660 height=560>") 
                        
                        if img2 == 12:
                            fo.write("<img src=\"g19_proj_plot04.png\" width=660 height=560>")   
                            
                      if img1 == 6:
                        if img2 == 13:
                          fo.write("<img src=\"callgraphdebug.png\" width=860 height=260>") 
                        if img2 == 14:
                          fo.write("<img src=\"callgraphrelease.png\" width=860 height=260>")   
                      fo.write("<p>")
                      
                      while(i < k):
                        i = i + 1
                        line = lines[i]
                        match3 = re.search("\\\\item\ +([\w.\W]*)" , line)
                        match4 = re.search("\\\\end" , line)
                        
                        if match4:
                            fo.write("</p>\n")
                            break
                        
                        if match3:     
                            fo.write(match3.group(1))
                     
                      break   
        
        
        
        i = i + 1




if __name__ == '__main__':
    main()
