class MotionPlanner:
  def __init__ (self, robot, ps):
    self.robot = robot
    self.ps = ps

  def solveBiRRT (self, maxIter = float("inf")):
    self.ps.prepareSolveStepByStep ()
    finished = False

    # In the framework of the course, we restrict ourselves to 2 connected components.
    nbCC = self.ps.numberConnectedComponents ()
    if nbCC != 2:
      raise Exception ("There should be 2 connected components.")

    iter = 0
    #### RRT begin

    while True:
    #### RRT begin      
      qRand=self.robot.shootRandomConfig()
      listConfigs = list ()

      #Init, récup les 2 premiers noeuds
      for i in range (nbCC):
        qNear, dist = self.ps.getNearestConfig (qRand, i)
        isGood,iPath,_= self.ps.directPath(qNear,qRand,True)
        length = self.ps.pathLength(iPath) 	
        qBaujolais = self.ps.configAtParam(iPath,length) 

        self.ps.addConfigToRoadmap(qBaujolais) 
        self.ps.addEdgeToRoadmap(qNear,qBaujolais,iPath,True) 

      ## Try connecting the new nodes together
      for i in range (len(listConfigs)):
        for j in range (i):
          isGood, i_path, _ = self.ps.directPath(listConfigs[i], listConfigs[j], True)
          if isGood:
            self.ps.addEdgeToRoadmap(listConfigs[i], listConfigs[j], iPath, True)
      
      
      #### RRT end
      ## Check if the problem is solved.
        
      nbCC = self.ps.numberConnectedComponents ()
      if nbCC == 1:
        # Problem solved
        finished = True
        break
      iter = iter + 1
      if iter > maxIter:
        break
    if finished:
      self.ps.finishSolveStepByStep ()
      return self.ps.numberPaths () - 1
      
  

  def solvePRM (self):
    self.ps.prepareSolveStepByStep ()
    #### PRM begin
    #### PRM end
    self.ps.finishSolveStepByStep ()
