import heapq as hq
import numpy as np
import cv2
import matplotlib.pyplot as plt
import math
import copy

def taking_inputs():

    yi = int(input("Enter y coordinate of start point = "))
    xi = int(input("Enter x coordinate of start point = "))
    thi = int (input("Enter input thetha in degree = "))

    r = int( input("Enter the radius of the robot "))
    l = int( input("Enter length of each steps "))

    yg = int(input("Enter y coordinate of GOAL point = "))
    xg = int(input("Enter x coordinate of GOAL point = "))
    thg = int(input("Enter the goal angle for goal point"))

    return xi,yi,xg,yg, thi, r,l,thg

def check_condition(img,yi,xi,yg,xg):
    
    if img[yi][xi] == 0:
        print("The start location is on an obstacle kindly re-input the points")
        exit()
    
    if img[yg][xg] == 0:
        print("The goal location is on an obstacle kindly re-input the points")
        exit()
    
    if yi >= 250 or yi <0 :
        print("y coordinate of start point is out of bounds")
        exit()
    
    if yg >= 250 or yg <0 :
        print("y coordinate of goal point is out of bounds")
        exit()
    
    if xi >= 400 or xi < 0:
        print("x coordinate of start point is out of bounds")
        exit()
    
    if xg >= 400 or xg < 0:
        print("x coordinate of goal point is out of bounds")
        exit()
    



def draw_obstacles(r=0):
    data = np.zeros((250,400), dtype=np.uint8)
    data[:,:] = [255]
    
    img = cv2.circle(data, (300,65), 45+r, (0), -1) #drawing a circle as an obstacle

    hex = np.array( [ [200,105-r], [240+r,130], [240+r, 170], [200,195+r], [160-r,170], [160-r,130] ] ) #drawing hexagonal obstacle
    cv2.fillPoly(img, pts =[hex], color=(0))
    
    poly = np.array( [ [33-r,63], [115+r,45-r], [83+r,72], [107+r,152+r] ] ) #drawing polygonal shape as obstacle
    cv2.fillPoly(img, pts =[poly], color=(0))
   
    return img


def allpossiblesteps(x,y,theta, l,xgoal,ygoal,img):
    steps = []
    y_max = 250
    x_max = 400

    for i in range(-60,61,30):
        new_thetha = theta + i
        y_n =  int(y + l*math.sin((theta+i)*0.01745))
        x_n =  int(x + l*math.cos((theta+i)*0.01745))
        #print(f"{x_n} {y_n} {i}")
        if x_n < x_max and x_n >= 0 and y_n >= 0 and y_n < y_max:

            tc = int (abs(xgoal - x_n ) + abs(ygoal - y_n)) #taking manhattan distance to calculate the cost to go
            # tc = int (math.sqrt((xgoal - x_n )**2 + (ygoal - y_n)**2))
            temp = (y_n,x_n,tc,new_thetha)
            if img[y_n][x_n] == 255:
                steps.append(temp)

    return steps

def main():

    xi,yi,xg,yg,thi,r,l,thg = taking_inputs()    
    
    xi,yi,xg,yg,thi,r,l, thg = 5,15,380,220,180,20,10, 40
    img = draw_obstacles(r)

    check_condition(img,yi,xi,yg,xg)
    
    org_img = draw_obstacles()


    Goal_var = (yg,xg)  #storing goal location
 
    Q = [] #DS to fetch the lowest c2c
    cost_2_go = abs(xg - xi) + abs(yg - yi)
    d1 = (cost_2_go,(-1,-1),(yi,xi),thi,0)  #initialised -1 as parents of start point
    hq.heappush(Q, d1)
    hq.heapify(Q)
    
    cq = {} #initialising closed list
    
    oq = {}
    oq[(yi,xi)] = [cost_2_go,[-1,-1],0]  #this can be treated as visited
   
    def backtrack(tup):
        
        y,x = tup
        ans = []
        
        sum = oq[(y,x)][0]
        sum3 = oq[(y,x)][2]
        ans.append([y,x])
        #print(oq)
        while x!=xi or y!=yi:
            org_img[y][x] = 10
            #print(oq[(y,x)][1])
            #print(y,x)
            y_new, x_new = oq[(y,x)][1]
            ans.append([y_new,x_new])
            cv2.line(org_img,(x,y),(x_new,y_new),140,1)
            y,x = y_new, x_new
        
        print("Total cost : ", sum)
        print("Cost to come : ", sum3)
        print("Backtracking Path from goal point to start point")
        print(ans)

        cv2.imshow("Final Path", org_img)
        cv2.waitKey(0)

        cv2.destroyAllWindows()
        exit()

    while Q:
        ele = hq.heappop(Q)
        #print(ele[2])
        cq[ele[2]] = 1  #adding into the closed queue should include parent as well

        y, x = ele[2]

        if (xg - x)**2 + (yg - y)**2 <= 3 and (ele[3] <= thg+30 or ele[3] >= thg-30):        #if the coordinates of current node is near to goal node
            #backtracking, successful return
            cv2.destroyAllWindows() 
            print("Found")
            backtrack(ele[2])
            

        else:
            total_cost = ele[0]
            cost2come = ele[4]
            #print(cost2come)
            if (y,x) in oq:  #this condition can be removed
                
                if oq[(y,x)][0] < total_cost:
                    total_cost = oq[(y,x)][0]
                
                if oq[(y,x)][2] < total_cost:
                    cost2come = oq[(y,x)][2]
                
            #there are 5 location for the coordinate moment each with constant cost to come
            new_thetha = ele[3]
            for child_y,child_x,c2c , angle in allpossiblesteps(x,y,new_thetha,l ,xg ,yg ,img):
                gg = 8
                if img[child_y,child_x] == 255:

                    cv2.line(img,(x,y),(child_x,child_y), 150, 1)
                    cv2.imshow("Animation", img)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                    
                    if (child_y,child_x) not in cq and img[y][x] != 0:

                        if (child_y,child_x) not in oq:
                            fc2c = c2c + cost2come + gg
                            hq.heappush(Q,(fc2c,(y,x),(child_y,child_x),angle,cost2come+gg)) #putting in openlist
                            hq.heapify(Q) 
                            oq[(child_y,child_x)] = [fc2c,[y,x],cost2come+gg]
                            
                        else:
                            if oq[(child_y,child_x)][0] >  c2c:
                                oq[(child_y,child_x)][1] = [y,x]
                                oq[(child_y,child_x)][0] =  c2c
                            
                            if oq[(child_y,child_x)][2] >  cost2come+gg:
                                oq[(child_y,child_x)][1] = [y,x]
                                oq[(child_y,child_x)][2] =  cost2come+gg


    print("no solution found")
    
    cv2.destroyAllWindows()     

if __name__ == "__main__":
    main()