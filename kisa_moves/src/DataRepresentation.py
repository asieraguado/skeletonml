# -*- coding: utf-8 -*-
"""
Experiments for Data Representation in KisaActionRecognizer

Dataset: MSRA3D
20 actions: (1)high arm wave (O), (2)horizontal arm wave (O), (3)hammer, (4)hand catch,
(5)forward punch, (6)high throw, (7)draw x, (8)draw tick, (9)draw circle, (10)hand clap (O),
(11)two hand wave (O), (12)side-boxing, (13)bend, (14)forward kick, (15)side kick, (16)jogging (O),
(17)tennis swing, (18)tennis serve, (19)golf swing, (20)pickup & throw

AS1: 2, 3, 5, 6, 10, 13, 18, 20
AS2: 1, 4, 7, 8, 9, 11, 14, 12
AS3: 6, 14, 15, 16, 17, 18, 20

7 subjects
3 experiments/subject
15 FPS
20 skeleton points (joints), as described in the picture

CURRENT RESULTS:
Features: spherical coordinates of joints with absolute speed by joint and coordinate
Best: Random Forest (150 predictors)
Accy: ~54% (all actions)
Confussion matrix shows actions 3-9 are bad classified, while others are quite accurate

Accy: ~74% with 13 actions (without accions 3-9)

Histogram classifier with Random Forest
Accy: ~90% with 13 actions (without accions 3-9) - Improved to ~92-94%!
Accy: ~70% with all (20) actions
SVM+Histogram: faster, but slightly less accurate (around 5% less)

With the UTKA dataset, Histogram classifier gives similar or worse results with RF as parent
classifier. With other classifiers (e.g. NBC or Decision Tree), accuracy is sligtly better
Best (RF or RF+Hist): ~80%
Interesting results: Decision Tree+Hist: From 70% to 80%
            Best improvement of Histogram with NBC: From 57% to 71%
RF+Hist (adding the training set to the test set): 92%
    --- the remaining set was maybe too small (39 individuals) ----
RF, 10 frame TW: ~85% (No Histogram)

-----
Note: this is just a prototype, not a well structured software file!
Recommended: open with Python2-Spyder
"""
import subprocess
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization! 
import math
from sklearn.preprocessing import normalize
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
import collections
import numpy as np

#%%%%%%%%% Constants %%%%%%%%%%%%

# General constants
C_POSE_JOINTS   = 20
X = 0
Y = 1
Z = 2

# Joint indexes
RIGHT_SHOULDER  = 0
LEFT_SHOULDER   = 1
NECK            = 2
CENTRAL_TORSO   = 3
RIGHT_HIP       = 4
LEFT_HIP        = 5
RIGHT_ELBOW     = 7
LEFT_ELBOW      = 8
RIGHT_HAND      = 9
LEFT_HAND       = 10
RIGHT_KNEE      = 13
LEFT_KNEE       = 14
RIGHT_FOOT      = 15
LEFT_FOOT       = 16
HEAD            = 19

USED_JOINTS = [RIGHT_SHOULDER, LEFT_SHOULDER, NECK, CENTRAL_TORSO, RIGHT_HIP,
              LEFT_HIP, RIGHT_ELBOW, LEFT_ELBOW, RIGHT_HAND, LEFT_HAND,
              RIGHT_KNEE, LEFT_KNEE, RIGHT_FOOT, LEFT_FOOT, HEAD]
              
# Used joint indexes
U_RIGHT_SHOULDER  = 0
U_LEFT_SHOULDER   = 1
U_NECK            = 2
#U_CENTRAL_TORSO   = --- reference, not used in training/test
U_RIGHT_HIP       = 3
U_LEFT_HIP        = 4
U_RIGHT_ELBOW     = 5
U_LEFT_ELBOW      = 6
U_RIGHT_HAND      = 7
U_LEFT_HAND       = 8
U_RIGHT_KNEE      = 9
U_LEFT_KNEE       = 10
U_RIGHT_FOOT      = 11
U_LEFT_FOOT       = 12
U_HEAD            = 13

# Data load constants
N_ACTIONS = 20
N_SUBJECTS = 10
N_EXPERIMENTS = 3

#%%%%%%%% Body joint adjacency model %%%%%%
ADJACENT =  [RIGHT_ELBOW,
             LEFT_ELBOW,
             HEAD,
             0,
             RIGHT_KNEE,
             LEFT_KNEE,
             0,
             RIGHT_HAND,
             LEFT_HAND,
             0,
             0,
             0,
             0,
             RIGHT_FOOT,
             LEFT_FOOT,
             0,
             0,
             0,
             0,
             0
            ]
REFERENCE =  [CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              0,
              RIGHT_SHOULDER,
              LEFT_SHOULDER,
              RIGHT_ELBOW,
              LEFT_ELBOW,
              0,
              0,
              RIGHT_HIP,
              LEFT_HIP,
              RIGHT_KNEE,
              LEFT_KNEE,
              0,
              0,
              NECK
             ]
             
#%%%%%%% Data load %%%%%%%%%
             
def loadPoseXYZSeries(action, subject, experiment):
    dataDir = "datasets/MSRA3D/skeleton/"
    dataFile = "a"+`action`.zfill(2)+"_s"+`subject`.zfill(2)+"_e"+ \
        `experiment`.zfill(2)+"_skeleton3D.txt"
    joints_x = []
    joints_y = []
    joints_z = []
    f = open(dataDir+dataFile)
    for line in f:
        values = line.split()
        joints_x.append(float(values[0]))
        joints_y.append(float(values[1]))
        joints_z.append(float(values[2]))
    poses_x = []
    poses_y = []
    poses_z = []
    for i in xrange(0,len(joints_x),20):
        poses_x.append(joints_x[i:i+20])
        poses_y.append(joints_y[i:i+20])
        poses_z.append(joints_z[i:i+20])
    return [poses_x, poses_y, poses_z]

def loadPoseSeries(action, subject, experiment):
    posesXYZ = loadPoseXYZSeries(action, subject, experiment)
    poses_x = posesXYZ[X]
    poses_y = posesXYZ[Y]
    poses_z = posesXYZ[Z]
    poses = []
    for i in range(0,len(poses_x)):
        poses.append([poses_x[i], poses_y[i], poses_z[i]])
    return poses
    
def loadPoseSamples(action, subject, experiment):
    posesXYZ = loadPoseXYZSeries(action, subject, experiment)
    poses_x = posesXYZ[X]
    poses_y = posesXYZ[Y]
    poses_z = posesXYZ[Z]
    poses = []
    for i in range(0,len(poses_x)):
        poses.append(poses_x[i] + poses_y[i] + poses_z[i])
    return poses

def loadAllPoseSamples():
    allPoses = []
    for action in range(1,N_ACTIONS):
        for subject in range(1,N_SUBJECTS):
            for experiment in range(1,N_EXPERIMENTS):
                try:
                    series = loadPoseSamples(action,subject,experiment)
                    allPoses = allPoses + series
                except IOError:
                    print "IOError on action,subject,experiment: ",action,subject,experiment
    return allPoses
    
poses = loadPoseSeries(7,1,1)
                
#%% Scatter plots of motion pose series as videos
for i in range(0,len(poses)):
    fig = plt.figure()
    ax = Axes3D(fig)
    min_ax = min(min(poses[i][X]),min(poses[i][Y]),min(poses[i][Z]))/2
    max_ax = max(max(poses[i][X]),max(poses[i][Y]),max(poses[i][Z]))/2
    ax.set_xlim(min_ax,max_ax)
    ax.set_ylim(min_ax,max_ax)
    ax.set_zlim(min_ax,max_ax)
    ax.scatter(poses[i][X], poses[i][Z], poses[i][Y])
    plt.savefig('frame'+`i`.zfill(2))
# ffmpeg -framerate 15 -i frame%02d.png -r 15 output.avi -y
subprocess.call(['sh', 'createmp4.sh'])

#%% Convert pose series to spherical coordinates
def metricToSpherical(pose):
    pose_spherical = [[],[]]
    for joint in USED_JOINTS:
        ## convert joint's neighbor to local spherical coordinates
        ## base (torso) joints: use CENTRAL_TORSO as reference system
        reference = REFERENCE[joint]
        if joint != reference: #dont add zeros for the central reference
            local_x = pose[X][joint] - pose[X][reference]
            local_y = pose[Y][joint] - pose[Y][reference]
            local_z = pose[Z][joint] - pose[Z][reference]
            r = math.sqrt(local_x**2 + local_y**2 + local_z**2)
            # theta:
            pose_spherical[0].append(math.atan2(local_y, local_x))
            # phi: 
            pose_spherical[1].append(math.atan2(local_z, r))
    return pose_spherical
    
def seriesMetricToSpherical(pose_series):
    spherical_series = []    
    for pose in pose_series:
        pose_spherical = metricToSpherical(pose)
        spherical_series.append(pose_spherical)
    return spherical_series
    
#spherical = seriesMetricToSpherical(poses)
    
#%% Calculate spherical distance
# Implement only if SVM will be used
    # joints: d(u,v) = | arccos ( sin theta_u * cos theta_v + cos theta_u * cos theta_v * cos |theta_u - theta_v| ) |
    # poses:  d(u,v) = sum_i=1to9 (d(u_i,v_i))^2   ----> return this metric

#%% Calculate angular relative distance
# For motion model

def angularRelativeDistance(a,b):
    return (abs(a % math.pi - b % math.pi)) / math.pi


def angularDistance(a,b):
    return (abs(a % math.pi - b % math.pi))
    
# in radians
def geodesicDistance(a0,a1,b0,b1):
    return abs(math.acos(math.sin(a0)*math.sin(b0)+math.cos(a0)*math.cos(b0)*math.cos(angularDistance(a1,b1))))
    
# in rad/s
def angularSpeed(a,b,tw):
    FPS = 15.0
    sec = tw/FPS
    return angularDistance(a,b)/sec
    
#%% Calculate moving averages for spherical coordinate time series
    # (soften the data)

def movingAverages(spherical):
    movingAvgs = []
    for joint in range(0,14):
        jointAvgSeries = []
        movingAvgs.append(jointAvgSeries)
        values_0 = []
        values_1 = []
        for frame in spherical:
            values_0.append(frame[0][joint])
            values_1.append(frame[1][joint])
            if len(values_0)>5:
                values_0.pop(0)
                values_1.pop(0)
            if len(values_0)==5:
                avg0 = sum(values_0)/5
                avg1 = sum(values_1)/5
                avg = [avg0,avg1]
                jointAvgSeries.append(avg)
    return movingAvgs
    
#movingAvgs = movingAverages(spherical)
    

#%% Compress motion data

def compressMotionData(movingAvgs):
    for series in movingAvgs:
        i = 0
        while i < len(series)-1:
            if geodesicDistance(series[i][0],series[i+1][0],series[i][1],series[i+1][1]) < math.pi/12:
                series.pop(i+1)
            else:
                i = i+1
                
#compressMotionData(movingAvgs)
                
#%% Remove initial and final stand poses

def removeFirstSimilarPoses(movingAvgs):
    size = len(movingAvgs[0])
    i = 0
    while i < size-1:
        distances = []
        for series in movingAvgs:
            distances.append(geodesicDistance(series[0][0],series[0][1],series[1][0],series[1][1]))
        if max(distances) < math.pi/16:
            for series in movingAvgs:
                series.pop(0)
            i = i+1
        else:
            #print "FirstRemoved: ", i
            break

def removeFinalSimilarPoses(movingAvgs):  
    size = len(movingAvgs[0])
    i = size-1
    while i > 0:
        distances = []
        for series in movingAvgs:
            distances.append(geodesicDistance(series[i][0],series[i][1],series[i-1][0],series[i-1][1]))
        if max(distances) < math.pi/16:
            for series in movingAvgs:
                series.pop(i)
            i = i-1
        else:
            #print "LastRemoved: ", i
            break              
                
def removeStandPoses(movingAvgs):
    removeFirstSimilarPoses(movingAvgs)
    removeFinalSimilarPoses(movingAvgs)
                

#%% Add angular speed info for each angle (list values 2,3) to the movingAvgs series

def addSpeedInfo(movingAvgs):
    TIME_WINDOWS = [2,4,6]
    for series in movingAvgs:
        enum = enumerate(series)
        for i,value in enum:
            acc_0 = 0
            acc_1 = 0
            for tw in TIME_WINDOWS:
                if i>tw-1:
                    speed_0 = angularSpeed(value[0],series[i-tw][0], tw)
                    speed_1 = angularSpeed(value[1],series[i-tw][1], tw)
                    acc_0 = acc_0 + speed_0
                    acc_1 = acc_1 + speed_1
                    value.append(speed_0)
                    value.append(speed_1)
                #else:
                    #value.append(None)
                    #value.append(None)
                    # First rows will be removed (missing values strategy is discarded)
            dist_00 = angularDistance(value[0],series[0][0])
            dist_01 = angularDistance(value[1],series[0][1])
            value.append(acc_0)
            value.append(acc_1)
            value.append(dist_00)
            value.append(dist_01)
        for i in range(0,min(len(series),max(TIME_WINDOWS))):
            series.pop(0)

#addSpeedInfo(movingAvgs)

#%% Convert time series by skeleton point to global time series (60 features)

def convertSkelToGlobal(skelTs):
    globalTs = []
    for series in skelTs:
        for i,values in enumerate(series):
            if i>len(globalTs)-1:
                globalTs.append([])
            for value in values:
                globalTs[i].append(value)
    return globalTs

#globalTs = convertSkelToGlobal(movingAvgs)

#%% Some filters

# Remove 'hard' cases (actions/classes 3 to 9)
def removeHardCases(featureData,classData):
    if len(featureData) != len(classData):
        return None
    else:
        size = len(featureData)
        i = 0
        while i<size:
            if (classData[i]>=3 and classData[i]<=9):
                featureData.pop(i)
                classData.pop(i)
                size = size-1
            else:
                i = i+1

# Remove "empty" (less than 10 frames) series
def removeEmptySeries(featureSeries,classSeries):
    if len(featureSeries) != len(classSeries):
        return None
    else:
        size = len(featureSeries)
        i = 0
        while i<size:
            if (len(featureSeries[i])<10):
                featureSeries.pop(i)
                classSeries.pop(i)
                size = size-1
            else:
                i = i+1
            
#%% Load+preprocess the training and test sets

# 4 first subjects
def trainingSet():
    trainingSet = []
    classes = []
    for action in range(1,N_ACTIONS+1):
        for subject in range(1,N_SUBJECTS-2):
            for experiment in range(1,N_EXPERIMENTS+1):
                try:
                    series = loadPoseSeries(action,subject,experiment)
                    spherical = seriesMetricToSpherical(series)
                    avgs = movingAverages(spherical)
                    removeStandPoses(avgs)
                    addSpeedInfo(avgs)
                    setPart = convertSkelToGlobal(avgs)
                    trainingSet = trainingSet + setPart
                    for i in range(0,len(setPart)):
                        classes.append(action)
                except IOError:
                    print "IOError on action,subject,experiment: ",action,subject,experiment
    removeHardCases(trainingSet,classes)
    return trainingSet,classes
    
# subjects 5,6,7
def testSet():
    testSet = []
    classes = []
    for action in range(1,N_ACTIONS+1):
        for subject in range(N_SUBJECTS-2,N_SUBJECTS+1):
            for experiment in range(1,N_EXPERIMENTS+1):
                try:
                    series = loadPoseSeries(action,subject,experiment)
                    spherical = seriesMetricToSpherical(series)
                    avgs = movingAverages(spherical)
                    removeStandPoses(avgs)
                    addSpeedInfo(avgs)
                    setPart = convertSkelToGlobal(avgs)
                    testSet = testSet + setPart
                    for i in range(0,len(setPart)):
                        classes.append(action)
                except IOError:
                    print "IOError on action,subject,experiment: ",action,subject,experiment
    removeHardCases(testSet,classes)
    return testSet,classes
    
train,trainClass = trainingSet()
test,testClass = testSet()

#%% Test set in second format (time-series)
# subjects 5,6,7, 1 experiment-1 series
def testSeries():
    testSet = []
    classes = []
    for action in range(1,N_ACTIONS+1):
        for subject in range(N_SUBJECTS-2,N_SUBJECTS+1):
            for experiment in range(1,N_EXPERIMENTS+1):
                try:
                    series = loadPoseSeries(action,subject,experiment)
                    spherical = seriesMetricToSpherical(series)
                    avgs = movingAverages(spherical)
                    removeStandPoses(avgs)
                    addSpeedInfo(avgs)
                    setPart = convertSkelToGlobal(avgs)
                    testSet.append(setPart)
                    classes.append(action)
                except IOError:
                    print "IOError on action,subject,experiment: ",action,subject,experiment
    removeHardCases(testSet,classes)
    removeEmptySeries(testSet,classes)
    return testSet,classes

testSeries,testSeriesClass = testSeries()

#%%  RandomForest Classifier

from sklearn.ensemble import RandomForestClassifier
clf = RandomForestClassifier(n_estimators=100, criterion='entropy', n_jobs=4)
clf.fit(train, trainClass)
print "Accuracy: ", clf.score(test,testClass)
from sklearn.metrics import confusion_matrix
predClass = clf.predict(test)
# Compute confusion matrix
cm = confusion_matrix(testClass, predClass)
np.set_printoptions(precision=2)
print('Confusion matrix, without normalization')
print(cm)

#%% Decision tree

from sklearn.tree import DecisionTreeClassifier
clf = DecisionTreeClassifier()
clf.fit(train,trainClass)
clf.score(test,testClass)
print "Accuracy", clf.score(test,testClass)
predClass = clf.predict(test)
# Compute confusion matrix
cm = confusion_matrix(testClass, predClass)
np.set_printoptions(precision=2)
print('Confusion matrix, without normalization')
print(cm)

#%% SVM
from sklearn import svm
    
clf = svm.SVC(kernel='linear')
clf.fit(train,trainClass)
clf.score(test,testClass)
print "Accuracy", clf.score(test,testClass)
predClass = clf.predict(test)
# Compute confusion matrix
cm = confusion_matrix(testClass, predClass)
np.set_printoptions(precision=2)
print('Confusion matrix, without normalization')
print(cm)

#%% Naive bayes
from sklearn.naive_bayes import GaussianNB
clf = GaussianNB()
clf.fit(train,trainClass)
print "Accuracy", clf.score(test,testClass)
predClass = clf.predict(test)
# Compute confusion matrix
cm = confusion_matrix(testClass, predClass)
np.set_printoptions(precision=2)
print('Confusion matrix, without normalization')
print(cm)

#%% Histogram classifier

# needs a trained base classifier (clf)

# Histogram
predictions = []
for action in testSeries:
    hist =  [0]*21
    if len(action)>0:
        framePredictions = clf.predict(action)
        for pred in framePredictions:
            hist[pred] = hist[pred]+1
        predictedAction = hist.index(max(hist))
    else:
        predictedAction = None
    print "a"
    predictions.append(predictedAction)
    
# Score
same = 0
for i in range(0,len(predictions)):
    if predictions[i] == testSeriesClass[i]:
        same = same+1
print same/float(len(predictions))
    

#%% Data loading and normalization for clustering
poseSamples = loadAllPoseSamples()
normSamples = normalize(poseSamples, norm='l2', axis=0)
    
#%% Cluster analysis
# Sklearn k-means
# http://scikit-learn.org/stable/modules/generated/sklearn.cluster.KMeans.html
range_n_clusters = range(100,112)
scores = []
counters = []
# Select the number of clusters that maximizes quality (silhouette average)
# Good previous results: 35, 37. Better results >16
for n_clusters in range_n_clusters:
    # same seed of 10 for all clusterers
    clusterer = KMeans(n_clusters=n_clusters, random_state=10)
    cluster_labels = clusterer.fit_predict(normSamples)
    silhouette_avg = silhouette_score(normSamples, cluster_labels)
    scores.append(silhouette_avg)
    counter = collections.Counter(cluster_labels)
    counters.append(counter)

#%% Clustering
clusterer = KMeans(n_clusters=35)
cluster_labels = clusterer.fit_predict(normSamples)
count_cluster_instances = collections.Counter(cluster_labels)
    
    
#%% Train classifier
   # Sklearn alternatives:
    # SVM: http://scikit-learn.org/stable/modules/classes.html#module-sklearn.svm
    # Random forest: http://scikit-learn.org/stable/modules/generated/sklearn.ensemble.RandomForestClassifier.html    
    
############################################################################
#%% MOVEMENT TRAINING SETUP
    # Obtain classification data from static pose classifier
    # Prepare dataset for training in time-windows
    
#%% Train classifier
    # HMM: http://scikit-learn.sourceforge.net/stable/modules/hmm.html
    # Decision forest: maybe random forest?
    #  idea: use trivial measure for variable selection (i.e. time order)

    
    
    
    
    
    
    
    
    
    
    
    
    


    