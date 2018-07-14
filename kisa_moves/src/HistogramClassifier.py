import numpy as np
from matplotlib import pyplot as plt

class HistogramClassifier:
    hist = []

    def __init__(self, classifier, scaler):
        self.hist = [0] * 21
        self.classifier = classifier
        self.scaler = scaler


    def addInstant(self, extended_pose):
        clf_data = np.asarray(extended_pose.getDataRow())
        clf_data = clf_data.reshape(1, -1)
        if (self.scaler != None):
            clf_data = self.scaler.transform(clf_data)
        inst_prediction = int(self.classifier.predict(clf_data))
        self.hist[inst_prediction] += 1
        #print "Instant prediction: ", inst_prediction

    def forgetHistory(self):
        self.hist = [0] * 21
        
    def plot(self):
        #fig = plt.figure()
        ax = plt.subplot(111)
        width = 1
        ax.bar(np.arange(len(self.hist)), self.hist, width=width)
        ax.set_xticks(np.arange(len(self.hist)) + width/2)
        ax.set_xticklabels(np.arange(len(self.hist)), rotation=0)
        plt.savefig("figure.pdf")

    def predictAction(self):
        #self.plot()
        prediction = self.hist.index(max(self.hist))
        return prediction

    def classifySeries(self, pose_series):
        self.forgetHistory()
        for instant in pose_series.series:
            self.addInstant(instant)
        prediction = self.predictAction()
        #print "Time-series prediction: ", prediction
        return prediction