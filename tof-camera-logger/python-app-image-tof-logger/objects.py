import numpy as np
import helper as h

class ToF_Frame():
    isObject = False
    def __init__(self,matrix):
        self.isObject = False
        self.object = np.zeros((8, 8), dtype=np.float32)
        self.__labelNearestObjects__(matrix)
        self.__detectWall__(matrix)
        if not self.Wall:
            self.__isObject__()

    def getObjectDistance(self):
        if self.isObject:
            return self.objectDistance
        else:
            return None
    def getCenterCol(self):
        if self.isObject:
            return self.centerCol
        else:
            return None
    def getObjectCenter(self):
        if self.isObject:
            return self.centerRow,self.centerCol
        else:
            return None

    def getObjectSize(self):
        if self.isObject:
            return self.objectSize
        else:
            return 0

    def istObject(self):
        return self.isObject

    def printObject(self):
        if self.isObject:
            print(f"Object: {self.object}")
        else:
            print("No Object detected")

    def __isObject__(self):
        self.__isCeilingOrGround__()
        if not self.isCeilingOrGround:
            self.__getObjectSize__()
            # if the object is to small it might be noise
            # if its two big and covers most of the image (90%) it might be a wall
            if 3 < self.objectSize < 0.8*self.object.shape[0]*self.object.shape[1]:
                self.isObject = True

    def __detectWall__(self, input_matrix):
        matrix = input_matrix.copy()
        self.__detectSlidingWall__(matrix)

    def __detectSlidingWall__(self,matrix):
        leftEdge=[0,1,2]
        rightEdge=[7,6,5]
        self.__checkEdge__(matrix,leftEdge)
        if not self.Wall:
            self.__checkEdge__(matrix,rightEdge)
    def __checkEdge__(self,matrix,edge):
        self.Wall = False
        potentialWall = True
        # check if first two columns are connected
        references = []
        potentialWall, ref, height = self.__connectedCol__(matrix[:,edge[0]])
        if potentialWall:
            references.append(ref)
            potentialWall, ref, height = self.__connectedCol__(matrix[:,edge[1]],height)
            if potentialWall:
                references.append(ref)
                if potentialWall:
                    approachingAngle,ref,valid = h.computeAproachingAngle(references)
                    if valid:
                        d3=h.getNextPixelDistance(approachingAngle,ref)
                        if approachingAngle != 0:
                            self.Wall,dist,height = self.__connectedCol__(matrix[:,edge[-1]],height,d3)

    def __connectedCol__(self,column,height=0, reference = 0):
        columnCount = column[column!=-1]
        offset = 0.05
        if len(columnCount)>0:
            if reference == 0:
                center = int(len(columnCount)/2)
                reference = columnCount[center]
            if reference == 0:
                return False,0,height
        else:
            return False, reference, height
        columnTransposed = np.flip(column)
        for idx,i in enumerate(columnTransposed):
            if abs(i-reference) > offset:
                if height ==0:
                    if idx>1:
                        height = idx
                        return True, reference, height
                    else:
                        return False, reference, height
                elif height-1 <= idx <= height+1:
                    return True,reference,height
                else:
                    return False,reference,height
        return True,reference,8

    def __getObjectSize__(self):
        self.objectSize = np.sum(self.object)

    def __isCeilingOrGround__(self):
        if self.centerRow == 0 or self.centerRow == 7:
            self.isCeilingOrGround = True
        else:
            self.isCeilingOrGround = False
    def __labelNearestObjects__(self,input_matrix):
        matrix = input_matrix.copy()
        # replace every invalid entry with distance 3m
        matrix[matrix == -1] = 3
        # get closest Pixel and closest value
        closestPixel = np.argmin(matrix)
        closestValue = np.min(matrix)
        self.objectDistance = closestValue
        # everything which has a discrepancy of 1dm to the closest pixel is considered to be part of the object
        upperBound = closestValue + 0.3
        self.centerRow = int(closestPixel / 8)
        self.centerCol = int(closestPixel % 8)
        self.__RecursiveCluster__(matrix, self.object, int(closestPixel / 8), int(closestPixel % 8), upperBound)

    """
    For a given starting pixel (i,j), Recursive Cluster finds all pixel
    that are connected to the starting pixel by roughly the same distance
    -threshold is the upper bound for the distance
    -solution is the matrix that is filled with 1s for all pixels that are part of the object
    -grid is the matrix that contains the distance values
    -(i,j) is the starting pixel for the recursive search it determined by the closest pixel
    """
    def __RecursiveCluster__(self,grid, solution, i, j, threshold):
        # check if pixel is out of bounds
        if i < 0 or i >= grid.shape[0] or j < 0 or j >= grid.shape[1]:
            return
        # check if pixel is already part of the object and if the distance is within the threshold
        if solution[i, j] == 0 and grid[i, j] < threshold:
            # mark pixel as part of the object
            solution[i, j] = 1
            # recursive call for all 4 neighbors
            self.__RecursiveCluster__(grid, solution, i + 1, j, threshold)
            self.__RecursiveCluster__(grid, solution, i - 1, j, threshold)
            self.__RecursiveCluster__(grid, solution, i, j + 1, threshold)
            self.__RecursiveCluster__(grid, solution, i, j - 1, threshold)