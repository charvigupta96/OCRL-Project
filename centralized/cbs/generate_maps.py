import numpy as np
import yaml
class Map:
    def __init__(self, *args):
        if len(args)==3:
          self.num_agents = args[0]
          self.map_dim = args[1]
          self.obstacle_density = args[2]
          self.map = {'dimensions':[self.map_dim,self.map_dim],'obstacles':[]}
          self.agents = []
          self.obstacles = []
          self.place_obstacles()
          self.create_start_goal()

        else:
          print("Map initialization")       
          # Read Map
          with open(args[0], 'r') as map_file:
            try:
              map = yaml.load(map_file, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
              print(exc) 
          self.map = map["map"]
          self.num_agents = len(map["agents"])
          self.map_dim = map["map"]["dimensions"][0]
          self.obstacles = map["map"]["obstacles"]
          self.agents = []
          if map_file.name == 'warehouse.yaml':
            self.create_start_goal()
          else:
            self.agents = map["agents"]

    def place_obstacles(self):
        num_obstacles = int(self.map_dim ** 2 * self.obstacle_density/100)
        obstacle = 0
        while obstacle<num_obstacles:
            index = (np.random.randint(self.map_dim),np.random.randint(self.map_dim))
            if index not in self.map['obstacles']:
                self.map['obstacles'].append(index)
                self.obstacles.append(index)
                obstacle = obstacle + 1

    def create_start_goal(self):
        starts = []
        goals = []
        numAgent = 0
        while numAgent<self.num_agents:
            start = (np.random.randint(self.map_dim),np.random.randint(self.map_dim))
            if (start not in self.obstacles) and (start not in starts):
                starts.append(start)
                numAgent = numAgent + 1

        numAgent = 0
        while numAgent<self.num_agents:
            goal = (np.random.randint(self.map_dim),np.random.randint(self.map_dim))
            if (goal not in self.obstacles) and (goal not in goals) and goal!=starts[numAgent]:
                goals.append(goal)
                numAgent = numAgent + 1

        numAgent = 0
        while numAgent<self.num_agents:
            agent = {'start':starts[numAgent],'goal':goals[numAgent],'name':'agent'+str(numAgent)}
            self.agents.append(agent)
            numAgent = numAgent + 1

    def __str__(self):
        return f"Map with {self.num_agents} agents, {self.map_dim}x{self.map_dim} dimensions, and {self.obstacle_density:.2%} obstacle density."
