#!/usr/bin/python

import math
import unittest
import random
import sys

from my_stat import dist_pt, median, mean, variance

#number of candidate actions to choose from
nba=100

# mex number of point
nb_max_points=250

#nb of points used to compute the average of error
theta=25

#delay to compute error
tau=15

# Class corresponding to an expert represented as a set of point. The prediction of an expert is the value of the nearest neighbor (or the mean over the set of n nearest neighbors).
class Expert(object):
    def __init__(self, _min,_max, _type,_nb_points):
        #print "Creating Expert "

        # min and max are the cut values in the input (points) space to decide whether a point is covered by the expert or not.
        self.min = _min
        self.max = _max

        # set of points belonging to this expert
        self.points=[]

        # corresponding output values
        self.output=[]

        # corresponding errors (history)
        self.errors=[]

        # corresponding errors (updated each time a point is added)
        self.error_points=[]

        # string describing the output (used only for stat, for printing purpose only)
        self.type=_type

        # sets the number of neighbors to consider when making a prediction
        self.nb_points=_nb_points

    # number of points
    def size(self):
        return len(self.points)
    
    # reinitialisation of the expert
    def reinit(self):
        self.points=[]
        self.output=[]
        self.errors=[]
        self.error_points=[]

    # is the point within the range of values covered by this expert
    def within_range(self,inputs):
        #print("within range: "+str(point)+" min="+str(self.min)+" max="+str(self.max))
        for i in range(0,len(inputs)):
            if (inputs[i]<self.min[i]):
                return 0
            if (inputs[i]>=self.max[i]):
                return 0
        return 1


    # adds a point (it is supposed to be within the range of covered values)
    def update(self,ninput,noutput):
        old_output=self.predict_n(ninput)
        error=abs(old_output-noutput)
        #print("update: output="+str(noutput)+" old_output="+str(old_output)+" error="+str(error)+" ninput="+str(ninput))
        self.errors.append(error)
        self.points.append(ninput)
        self.output.append(noutput)
        return error


    # gets the set of the n closest points
    def get_n_closest(self,ninput):
        if(len(self.points)==0):
            print "Point database empty"
            sys.exit(1)
        
        d=[[dist_pt(ninput,self.points[0]),0]]
        for i in range(1,len(self.points)):
            d.append([dist_pt(ninput,self.points[i]),i])
        d.sort(key=lambda t:t[0])
        return d[0:self.nb_points]

    # computes the predicted output value for ninput
    def predict_n(self,ninput):
        if (len(self.points)>0):
            li=self.get_n_closest(ninput)
            p=0
            s=0
            if (0 in map(lambda x:x[0],li)):
                for i in li:
                    if (i[0]==0):
                        i[0]=1
                    else:
                        i[0]=0
            for i in li:
                if (i[0]!=0):
                    p=p+self.output[i[1]]/i[0]
                    s=s+1.0/i[0]
            return p/s
        return 0

    # mean value of error over the following time window: [t-dt-theta, t-dt]
    def mean_error(self,dt):
        imin=len(self.errors)-dt-theta
        if (imin<0):
            imin=0
        imax=len(self.errors)-dt
        if (imax<0):
            imax=0
            return 0
        em=mean(self.errors[imin:imax])
        return em


    # function to split an expert.
    # to split, we consider each dimension, find its median value and compute the variance of outputs when we split it. We split along the dimension that minimizes the variance of the output
    def split_median(self):
        _var_min=0
        _d_min=-1
        _cut_min=0
        for d in range(0,len(self.min)):
            _p=[self.points[i][d] for i in range(0,len(self.points))]
            _cut=median(_p)
            _output1=[]
            _output2=[]
            for p in range(0,len(self.points)):
                if (self.points[p][d]>_cut):
                    _output1.append(self.output[p])
                else:
                    _output2.append(self.output[p])

            if (len(_output1)<0.1*len(self.points)) or (len(_output2)<0.1*len(self.points)) or (abs(len(_output1)-len(_output2))>0.2*len(self.points)):

                #print("split, dim="+str(d)+" output1="+str(_output1)+" output2="+str(_output2)+" cut="+str(_cut))
                continue
            var1=variance(_output1)
            var2=variance(_output2)
            #print("split, dim="+str(d)+" output1="+str(_output1)+" output2="+str(_output2)+" cut="+str(_cut)+" var1="+str(var1)+" var2="+str(var2))#" p="+str(_p))
            if (_d_min==-1) or (var1+var2<_var_min):
                _var_min=var1+var2
                _d_min=d
                _cut_min=_cut

        if (_d_min==-1):
            return 0
        _max=list(self.max)
        _max[_d_min]=_cut_min
        _min=list(self.min)
        _min[_d_min]=_cut_min

        nexpert=Expert(_min, self.max, self.type, self.nb_points)

        self.min=self.min
        self.max=_max

        npoints=self.points
        noutputs=self.output

        self.reinit()

        print("Split with dim="+str(_d_min)+" cut point: "+str(_max))

        for a in range(0,len(npoints)):
            if(self.within_range(npoints[a])==1):
                self.update(npoints[a],noutputs[a])
            else:
                if (nexpert.within_range(npoints[a])!=1):
                    print("Problem in split !")
                nexpert.update(npoints[a],noutputs[a])

        return nexpert

    # function to split an expert.
    # to split, we consider each dimension, find its median value and compute the variance of outputs when we split it. We split along the dimension that minimizes the variance of the output
    def split(self):
        _var_min=0
        _d_min=-1
        _cut_min=0
        _nb=0
            
        for d in range(0,len(self.min)):
            _p=[self.points[i][d] for i in range(0,len(self.points))]
            _p=sorted(set(_p))
            for j in range(0,len(_p)-1):
                _cut=(_p[j]+_p[j+1])/2.0
                _output1=[]
                _output2=[]
                for p in range(0,len(self.points)):
                    if (self.points[p][d]>_cut):
                        _output1.append(self.output[p])
                    else:
                        _output2.append(self.output[p])
                        
                var1=variance(_output1)
                var2=variance(_output2)
                _var=len(_output1)*var1+len(_output2)*var2
                _nb_min=min([len(_output1),len(_output2)])
                if (_d_min==-1) or (_var<_var_min) or (abs(_var-_var_min)<0.001 and (_nb_min>_nb)) :
                    _var_min=_var
                    _d_min=d
                    _cut_min=_cut
                    _nb=_nb_min
        if (_d_min==-1):
            return 0
        _max=list(self.max)
        _max[_d_min]=_cut_min
        _min=list(self.min)
        _min[_d_min]=_cut_min

        nexpert=Expert(_min, self.max, self.type, self.nb_points)

        self.min=self.min
        self.max=_max

        npoints=list(self.points)
        noutputs=list(self.output)

        errors=self.errors

        self.reinit()

        print("Split with dim="+str(_d_min)+" cut="+str(_cut_min)+" cut point: "+str(_max))

        for a in range(0,len(npoints)):
            if(self.within_range(npoints[a])==1):
                self.update(npoints[a],noutputs[a])
            else:
                if (nexpert.within_range(npoints[a])!=1):
                    print("Problem in split !")
                nexpert.update(npoints[a],noutputs[a])

        self.errors=list(errors)
        nexpert.errors=list(errors)

        return nexpert
                

# function to update the good predictor
def update_predictor(lexpert,ninput,noutput):
    a=which_expert(lexpert,ninput)
    if (a<0):
        print "ERREUR: no expert, we quit !"
        sys.exit(1)
    e=lexpert[a].update(ninput,noutput)
    if (lexpert[a].size()>nb_max_points):
        r=lexpert[a].split()
        if (not isinstance(r,int)):
            lexpert.append(r)
    info_expert(lexpert)
    return e

# prints some info about the experts
def info_expert(lexpert):
    for k in range(0,len(lexpert)):
        if (len(lexpert[k].points)==0):
            print(str(k)+" WARNING: empty expert !")
        else:
            print(str(k)+" nbpoints="+str(len(lexpert[k].points))+" error="+str(lexpert[k].mean_error(0))+" error(tau)="+str(lexpert[k].mean_error(tau))+" [min,mean,max]="+str(min(lexpert[k].output))+", "+str(mean(lexpert[k].output))+", "+str(max(lexpert[k].output))+" sound=["+str(lexpert[k].min[2])+", "+str(lexpert[k].max[2])+"]")

# which expert covers the input point
def which_expert(lexpert,ninput):
    for a in range(0,len(lexpert)):
        if(lexpert[a].within_range(ninput)==1):
            return a
    print "ERREUR: no expert for this value :"+str(ninput)
    return -1


class TestStatFunctions(unittest.TestCase):

    def setUp(self):
        self.te=Expert([0,0],[400,400],"test",4)

    def test_within_range(self):
        self.assertTrue(self.te.within_range([0.5,0.5]))
        self.assertTrue(not self.te.within_range([-0.5,0.5]))
        self.assertTrue(not self.te.within_range([0.5,-0.5]))
        self.assertTrue(not self.te.within_range([-0.5,-0.5]))
        self.assertTrue(not self.te.within_range([800,1]))
        self.assertTrue(not self.te.within_range([1,800]))
        self.assertTrue(not self.te.within_range([800,800]))

    def test_update(self):
        self.te.reinit()
        self.te.update([1,1],10)
        self.assertEqual(self.te.points[0],[1,1])
        self.assertEqual(self.te.output[0],10)
        self.assertEqual(self.te.errors[0],10)
        self.te.update([2,2],30)
        self.assertEqual(self.te.points[1],[2,2])
        self.assertEqual(self.te.output[1],30)
        self.assertEqual(self.te.errors[1],20)

    def test_get_n_closest(self):
        self.te.reinit()
        self.te.update([0,2],10)
        self.te.update([0,0],20)
        self.te.update([0,2],30)
        self.te.update([2,2],40)
        self.te.update([3,3],10000)
        self.te.update([3,0],10000)
        self.te.update([0,3],10000)
        self.assertEqual(sorted(map(lambda x:x[1],self.te.get_n_closest([1,1]))),[0,1,2,3])
        self.assertEqual(sorted(map(lambda x:x[0],self.te.get_n_closest([1,1]))),[math.sqrt(2),math.sqrt(2),math.sqrt(2),math.sqrt(2)])

    def test_predict_n(self):
        self.te.reinit()
        self.te.update([0,1],10)
        self.te.update([0,0],20)
        self.te.update([1,0],30)
        self.te.update([1,1],40)
        self.te.update([2,2],10000)
        self.te.update([2,0],20000)
        self.te.update([0,2],30000)
        self.te.update([0,4],100)
        self.te.update([1,3],200)
        self.assertEqual(self.te.predict_n([0,1]),10)
        self.assertEqual(self.te.predict_n([1,0]),30)
        s2=math.sqrt(2)
        s05=math.sqrt(0.5)
        self.assertEqual(self.te.predict_n([0.5,0.5]),(10.0/s05+20/s05+30/s05+40/s05)/(4./s05))
        self.assertEqual(self.te.predict_n([0,3]),(100/1.0+200/1.0+30000/1.+10/2.0)/(1/1+1/1+1/1+1.0/2.0))

    def test_mean_error(self):
        self.te.reinit()
        for i in range(-4,60):
            self.te.update([0,1+i],10+i)

        self.assertTrue(abs(self.te.mean_error(0)-48.0/25.0)<0.00001)

    def test_split(self):
        self.te.reinit()
        random.seed()
        for i in range(0,125):
            self.te.update([100+random.random(), 100+random.random()],10+random.random())
            self.te.update([300+random.random(), 100+random.random()],-10+random.random())
        self.te.update([200, 100],0)

        p=self.te.split_median()
        self.assertEqual(self.te.min[0],0)
        self.assertEqual(self.te.min[1],0)

        self.assertEqual(self.te.max[0],200)
        self.assertEqual(self.te.max[1],400)
        
        self.assertEqual(p.min[0],200)
        self.assertEqual(p.min[1],0)

        self.assertEqual(p.max[0],400)
        self.assertEqual(p.max[1],400)
        
        self.assertTrue(p.size()>124)
        self.assertTrue(self.te.size()>124)

        for pt in self.te.points:
            self.assertTrue(abs(pt[0]-100)<=1)
            self.assertTrue(abs(pt[1]-100)<=1)

        for pt in p.points:
            self.assertTrue((abs(pt[0]-300)<=1)or(abs(pt[0]-200)<=1))
            self.assertTrue(abs(pt[1]-100)<=1)

if __name__ == '__main__':
    print("Test of my expert functions")
    unittest.main()
