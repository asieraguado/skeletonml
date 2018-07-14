N_SUBJECTS = 10
N_EXPERIMENTS = 2
N_ACTIONS = 10

class ActionRange:
    def __init__(self,action,start,end):
        try:
            self.action = int(action)
            self.start = int(start)
            self.end = int(end)
        except:
            self.action = 0
            self.start = 0
            self.end = 0

dataDir = "datasets/UTKA/"
fileActions = [[[] for i in range(N_EXPERIMENTS+1)] for j in range(N_SUBJECTS+1)]
    
with open(dataDir+"actionLabel.txt") as f:
    count = 0
    lines = f.readlines()
    i = 0
    while i<len(lines)-2:
        fileId = lines[i]
        subject = int(fileId[1:3])
        experiment = int(fileId[5:7])
        print subject,experiment
        actions = fileActions[subject][experiment]
        for num in range(1,11):
            line = lines[i+num].strip()
            columns = line.split()
            print columns
            actions.append(ActionRange(num,columns[1],columns[2]))
        i = i+11

for subject in range(1,N_SUBJECTS+1):
    for experiment in range(1,N_EXPERIMENTS+1):
        dataFile = dataDir+'joints/joints_s'+`subject`.zfill(2)+'_e'+`experiment`.zfill(2)+'.txt'
        with open(dataFile) as f:
            for line in f:
                values = line.split()
                row = int(values[0])
                valid = False
                for a in fileActions[subject][experiment]:
                    if (row >= (a.start-10) and row <= (a.end+10)):
                        print row,'is betweeen',a.start,'and',a.end
                        action = a.action
                        valid = True
                        break
                if valid:
                    rewFile = dataDir+'joints_reformat/a'+`action`.zfill(2)+'_s'+`subject`.zfill(2)+'_e'+`experiment`.zfill(2)+'_skeleton3D.txt'
                    rewPointer = open(rewFile, 'a')                
                    values.pop(0)
                    i = 0
                    while i < len(values)-2:
                        rewPointer.write(values[i]+' '+values[i+1]+' '+values[i+2]+'\n')
                        i = i+3
                    rewPointer.close()
                
            