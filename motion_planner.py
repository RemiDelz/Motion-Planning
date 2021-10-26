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

      
    #Récupérer les configs voulues 
   # qr = self.robot.shootRandomConfig()
   # q1, distq1 = self.ps.getNearestConfig(qr, 0)
   # q2, distq2 = self.ps.getNearestConfig(qr, 1)

    isDP = False

    while not isDP :
      qr = self.robot.shootRandomConfig()
      q1, distq1 = self.ps.getNearestConfig(qr, 0)
      q2, distq2 = self.ps.getNearestConfig(qr, 1)
      
      #faire le test direct entre q1 et q2
      isDP,iPath,strPath = self.ps.directPath(q1,q2,True)

      if isDP:
        print("path is good")
      else :
        qr = self.robot.shootRandomConfig()
        temp = self.ps.pathLength(iPath)
        qtemp12 = self.ps.configAtParam(iPath, temp)
          
      self.ps.addEdgeToRoadmap(q1,q2,iPath,True)
        
        
      #faire le test direct entre q2 et q1
      isDP,iPath,strPath = self.ps.directPath(q2,q1,True)
      if isDP:
        print("path is good")
         
      else :
        qr = self.robot.shootRandomConfig()
        temp = self.ps.pathLength(iPath)
        qtemp21 = self.ps.configAtParam(iPath, temp)
         
      q2 = qtemp21
      q1 = qtemp12

      self.ps.addConfigToRoadmap(q1)
      self.ps.addConfigToRoadmap(q2)
      self.ps.addEdgeToRoadmap(q2,q1,iPath,True)
      
        
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
