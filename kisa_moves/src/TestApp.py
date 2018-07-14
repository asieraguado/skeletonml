import data_load
from structures.PoseSet import PoseSet
import numpy as np

# Data load constants
N_ACTIONS = 20
N_SUBJECTS = 10
N_EXPERIMENTS = 3

AS1 = [2, 3, 5, 6, 10, 13, 18, 20]
AS2 = [1, 4, 7, 8, 9, 11, 14, 12]
AS3 = [6, 14, 15, 16, 17, 18, 20]
AS4 = [1, 2, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]
ONLINE = [1, 2, 10, 11, 16]
ASET = ONLINE

CLASSIFIER='rf'

def run(leave_out):
    
    # %% Functions
    
    # Remove 'hard' cases (actions/classes 3 to 9) -- not in set
    def removeHardCases(featureData,classData):
        if len(featureData) != len(classData):
            return None
        else:
            size = len(featureData)
            i = 0
            while i<size:
                if not (classData[i] in ASET):
                    featureData.pop(i)
                    classData.pop(i)
                    size = size-1
                else:
                    i = i+1

    # plot example
    plot_series = data_load.loadPoseSeries(2, 2, 2)
    plot_series.createMp4()

    def preprocess(spherical):
        spherical.calculateMovingAverages()
        spherical.removeFirstSimilarPoses()
        spherical.removeFinalSimilarPoses()
        spherical.addSpeedInfo()
        #spherical.addCosines()

    # 4 first subjects
    def trainingSet():
        training_set = PoseSet()
        classes = []
        for action in range(1, N_ACTIONS + 1):
            for subject in range(1, leave_out) + range(leave_out+1, N_SUBJECTS+1):
                for experiment in range(1, N_EXPERIMENTS + 1):
                    try:
                        series = data_load.loadPoseSeries(action, subject, experiment)
                        spherical = series.getSphericalSeries()
                        preprocess(spherical)
                        set_part = spherical.getSet()
                        training_set.addSubSet(set_part)
                        for i in range(0, len(set_part)):
                            classes.append(action)
                    except IOError:
                        pass
                        #print "IOError on action,subject,experiment: ", action, subject, experiment
        training_data = training_set.getDataFrame()
        removeHardCases(training_data, classes)
        return training_data, classes


    # subjects 5,6,7
    def testSet():
        test_set = PoseSet()
        classes = []
        for action in range(1, N_ACTIONS + 1):
            for subject in [leave_out]:
                for experiment in range(1, N_EXPERIMENTS + 1):
                    try:
                        series = data_load.loadPoseSeries(action, subject, experiment)
                        spherical = series.getSphericalSeries()
                        preprocess(spherical)
                        set_part = spherical.getSet()
                        test_set.addSubSet(set_part)
                        for i in range(0, len(set_part)):
                            classes.append(action)
                    except IOError:
                        pass
                        #print "IOError on action,subject,experiment: ", action, subject, experiment
        test_data = test_set.getDataFrame()
        removeHardCases(test_data, classes)
        return test_data, classes

    # Remove "empty" (less than 10 frames) series
    def removeEmptySeries(featureSeries,classSeries):
        if len(featureSeries) != len(classSeries):
            return None
        else:
            size = len(featureSeries)
            i = 0
            while i<size:
                if featureSeries[i].getSize()<10:
                    featureSeries.pop(i)
                    classSeries.pop(i)
                    size = size-1
                else:
                    i = i+1

    def testMultiSeries():
        test_set = []
        classes = []
        for action in range(1, N_ACTIONS + 1):
            for subject in [leave_out]:
                for experiment in range(1, N_EXPERIMENTS + 1):
                    try:
                        series = data_load.loadPoseSeries(action, subject, experiment)
                        spherical = series.getSphericalSeries()
                        preprocess(spherical)
                        test_set.append(spherical)
                        classes.append(action)
                    except IOError:
                        pass
                        #print "IOError on action,subject,experiment: ", action, subject, experiment
        removeHardCases(test_set, classes)
        removeEmptySeries(test_set, classes)
        return test_set, classes

    def persistModel(clf):
        import pickle
        f = open("model.pkl", 'w')
        pickle.dump(clf, f)

    train, trainClass = trainingSet()
    test, testClass = testSet()
    scaler = None     # initialize scaler to None, only needed by MLP
    score = None

    if (CLASSIFIER == 'rf'):
        # %% Random Forest
        from sklearn.ensemble import RandomForestClassifier
        clf = RandomForestClassifier(n_estimators=50, criterion='gini', n_jobs=4, random_state=1)
        clf.fit(train, trainClass)
        persistModel(clf)
        score = clf.score(test,testClass)
        predClass = clf.predict(test)
        #%%
    if (CLASSIFIER == 'svm'):
        # %% SVM
        from sklearn import svm
        clf = svm.SVC(kernel='poly', degree=2)
        clf.fit(train, trainClass)
        clf.score(test, testClass)
        score = clf.score(test, testClass)
        predClass = clf.predict(test)
        #%%
    if (CLASSIFIER == 'mlp'):
        #%% MLP: Multilayer Perceptron (Neural Network)
        # Normalization for Neural Network
        from sklearn.preprocessing import StandardScaler
        scaler = StandardScaler()
        scaler.fit(train)
        train_s = scaler.transform(train)
        test_s = scaler.transform(test)
        # Classifier
        from sklearn.neural_network import MLPClassifier
        clf = MLPClassifier(random_state=3, learning_rate="adaptive", learning_rate_init=0.3, tol=0.0001, momentum=0.9, algorithm='sgd',  hidden_layer_sizes=[30], validation_fraction=0.33, activation='logistic', max_iter=10000, alpha=0.001)
        clf.fit(train_s, trainClass)
        clf.score(test_s, testClass)
        score = clf.score(test_s, testClass)
        predClass = clf.predict(test_s)
        #%%
    if (CLASSIFIER == 'nb'):
        # %% Naive Bayes
        from sklearn.naive_bayes import GaussianNB
        clf = GaussianNB()
        clf.fit(train, trainClass)
        clf.score(test, testClass)
        score = clf.score(test, testClass)
        predClass = clf.predict(test)

    #%% Print results (instant class)
    print "Accuracy: ", score

    # Compute confusion matrix
    from sklearn.metrics import confusion_matrix
    cm = confusion_matrix(testClass, predClass)
    np.set_printoptions(precision=2)
    print('Confusion matrix, without normalization')
    print(cm)
    persistModel(clf)

    #%% Custom histogram time-series classifier
    from HistogramClassifier import HistogramClassifier
    histogram_clf = HistogramClassifier(clf, scaler)
    histo_set, histo_classes = testMultiSeries()
    same = 0
    for i in range(0,len(histo_set)):
        prediction = histogram_clf.classifySeries(histo_set[i])
        if prediction == histo_classes[i]:
            same += 1
        #else:
            #print "Error! Predicted class: ", prediction, " / True class: ", histo_classes[i]
    histoScore = same/float(len(histo_classes))
    print same, "/", len(histo_classes), " = Accuracy: ", histoScore

    return [score, histoScore]

#%% MAIN PROGRAM
sums = [0, 0]
print "Classifier = ", CLASSIFIER
for leave_out in range(1, N_SUBJECTS+1):
    print "==== LEAVE OUT SUBJECT ", leave_out, " ===="
    scores = run(leave_out)
    sums[0] += scores[0]
    sums[1] += scores[1]
clf_score = sums[0]/N_SUBJECTS
time_score = sums[1]/N_SUBJECTS
print "Instant classification score", clf_score
print "Complete time-series score", time_score

print "==== COMPLETE TRAINING ===="
run(11)