import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import multiprocessing
import time
import os
import networkx as nx
from PIL import Image
import glob

class Vertex:
    def __init__(self, id):
        self.id_=id
        self.mass_=1
        self.position_= None
        self.is_fixed_= False
        self.velocity_= None
        self.damping_direction_= None
        
        self.internal_force_= None
        self.external_force_= None

class Edge:
    def __init__(self, id):
        self.id_ = id
        self.vertices_ =[]
        self.natural_length_=1
        self.stiffness_=1

class Run:
    def __init__(self,run_directory="run/"):
        self.run_directory_=run_directory

        self.dt_=None
        self.t_f_=None
        self.t_out_=None
        self.damping_=None

        self.vertices_=None
        self.edges_=None

    def run(self):
        if not os.path.isdir(self.run_directory_):
            os.mkdir(self.run_directory_)
        self.load_conf()
        self.load_initialization()

        for i, time in enumerate(np.arange(0,self.t_f_+self.dt_,self.dt_)):
            print("time:",time)
            print("Vertices:")
            for vertexID,vertex in self.vertices_.items():
                print("Vertex ID:",vertexID,vertex.position_)
            self.calculateInternalForces()
            self.updateVelocities()
            self.updatePositions()
            if not i%self.t_out_:self.dump_config(time,i)

        return
    def calculateInternalForces(self):
        #intialize internal forces to zero at each vertex
        for vertexID, vertex in self.vertices_.items():
            vertex.internal_force_=np.array([0.,0.])
        #Calculate internal forces due to each edge
        for edgeID, edge in self.edges_.items():
            #vector from vertex1 to vertex2:
            r_10=self.vertices_[edge.vertices_[1]].position_-self.vertices_[edge.vertices_[0]].position_
            #Length of vector from vertex1 to vertex2:
            l_10=np.linalg.norm(r_10)
            #Unit vector from vertex1 to vertex2:
            n_10=r_10/np.linalg.norm(r_10)
            #Force on vertex1:
            self.vertices_[edge.vertices_[0]].internal_force_+=edge.stiffness_*(l_10-edge.natural_length_)*n_10
            #Force on vertex2:
            self.vertices_[edge.vertices_[1]].internal_force_-=edge.stiffness_*(l_10-edge.natural_length_)*n_10
        return
    def updateVelocities(self):
        for vertexID,vertex in self.vertices_.items():
            if vertex.is_fixed_:vertex.velocity_=np.array([0.,0.])
            vertex.velocity_+=vertex.internal_force_*self.dt_/vertex.mass_
        return
    def updatePositions(self):
        for vertexID,vertex in self.vertices_.items():
            if vertex.is_fixed_:continue
            vertex.position_+=vertex.velocity_*self.dt_
        return
    def load_conf(self):
        with open(self.run_directory_+'conf','r') as f:
            lines = f.readlines()
            for line in lines:
                if line.split()[0]=="dt": self.dt_=float(line.split()[1])
                if line.split()[0]=="t_f": self.t_f_=float(line.split()[1])
                if line.split()[0]=="t_out": self.t_out_=float(line.split()[1])
                if line.split()[0]=="damping": self.damping_=float(line.split()[1])
        return
    def load_initialization(self):
        self.vertices_={}
        self.edges_={}
        with open (self.run_directory_ + 'initialization','r') as f:
            lines=f.readlines()
            nodesFlag=False
            edgesFlag=False
            for line in lines:
                if not len(line.split()):continue
                if line.split()[0]=='Nodes:':
                    nodesFlag=True
                    continue
                if line.split()[0]=='Edges:':
                    nodesFlag=False
                    edgesFlag=True
                    continue
                if nodesFlag:
                    id=int(line.split()[0])
                    vertex=Vertex(id)
                    vertex.position_=np.array([float(line.split()[1]),float(line.split()[2])])
                    vertex.velocity_=np.array([float(line.split()[4]),float(line.split()[5])])
                    if line.split()[3]=="Fixed": vertex.is_fixed_=True

                    vertex.damping_direction_=[float(line.split()[6]),float(line.split()[7])]
                    self.vertices_[id]=vertex
                if edgesFlag:
                    id=int(line.split()[0])
                    edge=Edge(id)
                    edge.vertices_=[int(line.split()[1]),int(line.split()[2])]
                    edge.natural_length_=float(line.split()[3])
                    edge.stiffness_=float(line.split()[4])
                    self.edges_[id]=edge
        return
    def dump_config(self,time,i):
        with open(self.run_directory_+"{:09}.txt".format(i),"w") as f:
            f.write("time: {}\n".format(time))
            f.write("Nodes:     x   y\n")
            for vertexID,vertex in self.vertices_.items():
                f.write( "{}        {}  {}\n".format(vertexID,vertex.position_[0],vertex.position_[1]))
            f.write("Edges:\n")
            for edgeID,edge in self.edges_.items():
                f.write("{}        {}   {}\n".format(edgeID,edge.vertices_[0],edge.vertices_[1]))
        return
            

    