#!/usr/bin/python

import unittest
import math

# euclidean distance between two points
def dist_pt(pA,pB):
    d=[pA[i]-pB[i] for i in range(0,len(pA))]
    return math.sqrt(reduce(lambda x,y:x+y,map(lambda x:math.pow(x,2),d)))

# compute the median of a list of values. If the number of points is even, returns the average between the two points around the median.
def median(l):
    s=sorted(l)
    i=len(l)/2
    if(len(l)%2==0):
        return (s[i]+s[i-1])/2.0
    else:
        return s[i]

# computes the arithmetic mean of a set of values
def mean(l):
    if (len(l)==0):
        return 0
    return reduce(lambda x,y:x+y,l)/float(len(l))

# computes the variance of a set of points
def variance(l):
    m=mean(l)
    ld=map(lambda x:(x-m)*(x-m),l)
    return reduce(lambda x,y:x+y,ld)/len(l)

class TestStatFunctions(unittest.TestCase):

    def setUp(self):
        pass

    def test_dist_pt(self):
        self.assertEqual(dist_pt([0,0],[1,0]), 1.0)
        self.assertEqual(dist_pt([0,0,0,0],[1,1,1,1]),2)
        self.assertEqual(dist_pt([0,0,0,0],[2,2,2,2]),4)

    def test_median(self):
        self.assertEqual(median([1,2,3,4,5]),3)
        self.assertEqual(median([4,5,1,3,2]),3)
        self.assertEqual(median([1,2,3,4]),2.5)
        self.assertEqual(median([4,1,2,3]),2.5)

    def test_mean(self):
        self.assertEqual(mean([4,1,2,3]),2.5)

    def test_variance(self):
        self.assertEqual(variance([1,1,1,1]),0)
        self.assertEqual(variance([1,-1,1,-1]),1)

if __name__ == '__main__':
    print("Test of my stat functions")
    unittest.main()
