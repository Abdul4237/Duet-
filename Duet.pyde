add_library('minim')
add_library('sound')
player=Minim(this)
import os
import math
import random
import time
RES_X=500
RES_Y=900
circle_v=0.08
NUM_OF_PARTICLES = 130
NUM_OF_COL_PARTICLES=50
NUM_OF_SPLASH_PARTICLES=70
gameiteration=0

def check_equal(lst):
    return all(ele == lst[0] for ele in lst)
def sign(num):
    if num<0:
        return -1
    elif num>0:
        return 1
    elif num==0:
        return random.random()
def calculate_distance(point1, point2):
    return math.sqrt((point2[0] - point1[0]) ** 2 + (point2[1] - point1[1]) ** 2)
def checkCollision(vector_list,ball_list):
    for vec in vector_list:
        #converting each quadlilateral into a set of 4 vector lines for collision detection
        for ball in ball_list:
        #calculating the coordinate of the projection of ball center onto vector line
            alph=(vec[1]*ball[0]-vec[0][0]*vec[1]+ball[1]*vec[2]-vec[0][1]*vec[2])/(vec[2]**2+vec[1]**2)
            proj_coord=[vec[0][0]+alph*vec[1],vec[0][1]+alph*vec[2]]
            dist_vec=[ball[0]-proj_coord[0], ball[1]-proj_coord[1]]
            #line(vec[0][0],vec[0][1],vec[-1][0]+alph*10*vec[1],vec[-1][1]+alph*10*vec[2])
            #line(proj_coord[0],proj_coord[1],ball[0],ball[1])
            proj_c1=[vec[0][0]-proj_coord[0],vec[0][1]-proj_coord[1]]
            proj_c2=[vec[-1][0]-proj_coord[0],vec[-1][1]-proj_coord[1]]
            if math.sqrt(dist_vec[0]**2 + dist_vec[1]**2)<=balls.r_ball and (sign(proj_c1[1])==-sign(proj_c2[1]) or math.sqrt(proj_c1[0]**2+proj_c1[1]**2)<=balls.r_ball or math.sqrt(proj_c2[0]**2+proj_c2[1]**2)<=balls.r_ball or sign(proj_c1[0])==-sign(proj_c2[0])):
                stroke(0)
                line(proj_coord[0],proj_coord[1],ball[0],ball[1])
                game.game_over=True
                game.proj_coord=proj_coord
                if math.sqrt(proj_c1[0]**2+proj_c1[1]**2)<=balls.r_ball:
                    game.proj_coord=vec[0]
                elif math.sqrt(proj_c2[0]**2+proj_c2[1]**2)<=balls.r_ball:
                    game.proj_coord=vec[-1]
                return ball

def defineVectors(points):
        #format of points:[UL,UR,LL,LR]
        #format of each vector:[[initialx,initialy], dirx, diry,[endx,endy]]
        UL_UR=[points[0],-points[0][0]+points[1][0],-points[0][1]+points[1][1],points[1]]
        LL_LR=[points[2],-points[2][0]+points[3][0],-points[2][1]+points[3][1],points[3]]
        UL_LL=[points[0],-points[0][0]+points[2][0],-points[0][1]+points[2][1],points[2]]
        UR_LR=[points[1],-points[1][0]+points[3][0],-points[1][1]+points[3][1],points[3]]
        return[UL_LL,UL_UR,LL_LR,UR_LR]
class Particle:
    def __init__(self, x, y, xSpd=0.8,ySpd=[1,1],dia=[1,3.5]):
        self.x = random.uniform(0, RES_X)
        self.y = random.uniform(0, RES_Y)
        self.xSpd = random.uniform(-xSpd, xSpd)
        self.ySpd = random.uniform(-ySpd[0],ySpd[1])
        self.dia = random.uniform(dia[0], dia[1])
        self.color = (172, 48, 32) if random.random() < 0.5 else (173, 200, 255)
        self.gamma=0.32
        self.lasty=self.y
        self.display_status=True
    def move(self):
        self.lasty=self.y
        self.x += 1.3*self.xSpd
        self.y -= self.ySpd

    def bounce(self):
        #function for background particles
        distance = calculate_distance((self.x, self.y), (RES_X / 2, RES_Y / 2))
        if distance > 200:
            self.xSpd *= -1
        if distance > 450:
            self.ySpd *= -1
    def decellerate(self):
        #simulating gravity for collision particles
        self.ySpd-=self.gamma
    
    def checkBounds(self):
        #function to not display splash particles outside of obstacle region
        if self.display_status==True:
            col=get(int(self.x),int(self.y))
            if red(col)==0 and green(col)==0 and blue(col)==0 or red(col)==175 and green(col)==48 and blue(col)==32 or red(col)==174 and green(col)==200 and blue(col)==255:
                self.display_status=False
    def slow(self):
        #setting max radius for splash with some stochasticity
        if calculate_distance([self.x,self.y],game.proj_coord)>=16 or random.random()<0.13:
            self.xSpd=0
            self.ySpd=0

    def display(self):
        if self.display_status==True:
            fill(*self.color)
            noStroke()
            ellipse(self.x, self.y, self.dia, self.dia)
            
class Game:
    def __init__(self,x,v,iteration=0,game_started=True,reset=False,mode=0):
        self.x=x
        self.v=v
        self.keyHandler= {'clockwise':False,'anticlockwise':False}
        self.game_started=game_started
        self.gameiteration=iteration
        self.game_mode=mode
        self.score_diff=0
        #self.obstacles_list=[Obstacles(type=random.randint(0,2))]
        self.obstacles_list=[Obstacles(4)]
        self.obstacles_list[0].dialog=True
        if self.game_mode==1:
            self.obstacles_list=[Obstacles(type=random.randint(6,11))]
            self.obstacles_list[0].dialog_text='You are not ready for this'
        self.score=0
        self.last_score=0
        self.stageCrossed=[0,0,0,0]
        self.game_over=False
        if self.game_started==False:
            self.keyHandler['anticlockwise']=True
        self.collidedBall=0
        self.file=open('highscore.txt','r')
        self.file2=open('highscore2.txt','r')
        self.high_score=int(self.file.readline().strip())
        self.hs_setter=self.file.readline().strip()
        self.encore_hs=int(self.file2.readline().strip())
        self.encore_hs_setter=self.file2.readline().strip()
        self.bg_particles = [Particle(RES_X / 2, RES_Y / 2) for _ in range(NUM_OF_PARTICLES)]
        self.collision_particles=[]
        self.splash_particles=[]
        self.displayingText=False
        self.bgMusic=player.loadFile(os.getcwd()+'/track2.mp3')
        #self.bgMusic.loop()
        self.collMusic=player.loadFile(os.getcwd()+'/coll.mp3')
        self.game_end_col=None
        self.inputting_score=False
        self.new_hs_setter=""
        self.resetAnimation=reset
        self.t=0
        self.proj_coord=None
    
    def display(self):
        if game.game_over==True:
            for particle in self.splash_particles:
                particle.slow()
                particle.move()
                particle.checkBounds()
                particle.display()
            for particle in self.collision_particles:
                particle.move()
                particle.display()
                particle.decellerate()
        for particle in self.bg_particles:
            particle.move()
            particle.bounce()
            particle.display()
            
    def updateObstaclesList(self):
        last_obst=self.obstacles_list[-1]
        last_name=self.obstacles_list[-1].name
        #checking whether the latest object that hasn't yet spawned another obstacle, passed the minimum threshold for a new obstacle to be spawned
        if last_obst.new_obst==False:
            add_new_obstacle=False
            if last_name=='rectangle array' or last_name=='squares':
                if last_obst.coords[-1][1]>=2*balls.r_circ:
                    add_new_obstacle=True
            elif last_name=='translating rectangles':
                if last_obst.coords[-1][1]>=3*balls.r_circ:
                    add_new_obstacle=True    
            elif last_name=='big rectangle followed by small rectangle':
                index=-1
                if last_obst.coords[-1][1]>last_obst.coords[-2][1]:
                       index=-2
                if last_obst.coords[index][1]>=2*balls.r_circ:
                    add_new_obstacle=True
            elif last_name=='duplicating rectangle':
                if last_obst.coords[0][1]>=2*balls.r_circ:
                    add_new_obstacle=True
            elif last_name=='rotating rectangle':
                if last_obst.y>=RES_Y/2.5:
                    add_new_obstacle=True
            elif last_name=='rotating circles':
                if last_obst.circle1[1]>=RES_Y/2.5:
                    add_new_obstacle=True
            elif last_name=='oscillating rotating rectangle pair':
                if last_obst.y2>=RES_Y/2:
                    add_new_obstacle=True
            elif last_name=='horizontal rectangle array rotated':
                if last_obst.y>=RES_Y/2:
                    add_new_obstacle=True
            elif last_name=='randomly rotated rectangles':
                #if last_obst.y-last_obst.num*math.sin(PI/2 -last_obst.theta)*(last_obst.sep+last_obst.dims[1])>=RES_Y/5:\\
                if last_obst.y-last_obst.num*(last_obst.sep+last_obst.dims[1])>=RES_Y/13:
                    add_new_obstacle=True
            elif last_name=='rotating rectangle pair':
                if last_obst.y>=RES_Y-100:
                    add_new_obstacle=True
            elif last_name=='rotating seesaw':
                if last_obst.y>=RES_Y/2:
                    add_new_obstacle=True
            #adding new obstacle
            if add_new_obstacle==True:
                last_obst.new_obst=True
                #removing old obstacles from list of obstacles
                if len(self.obstacles_list)>6:
                    self.obstacles_list.pop(0)
                #Initiating obstacles randomly if Encore Mode(game.game_mode=1), otherwise will implement difficulty accension based on score(more difficult obstacles only spawned later)
                if self.score>250 or self.game_mode==1:
                    type=random.randint(9+self.stageCrossed[2],11)
                    if self.game_mode==1:
                        type=random.randint(0,11)
                    print(self.stageCrossed[2])
                    if self.stageCrossed[2]!=-9:
                        self.stageCrossed[2]-=1
                    self.obstacles_list.append(Obstacles(type=type))
                    self.obstacles_list[-1].randomizeObstacle()
                elif self.score>150:
                    type=random.randint(6+self.stageCrossed[1],8)
                    print(self.stageCrossed[1])
                    if self.stageCrossed[1]!=-6:
                        self.stageCrossed[1]-=1
                    self.obstacles_list.append(Obstacles(type=type))
                    self.obstacles_list[-1].randomizeObstacle()
                elif self.score>70:
                    type=random.randint(3+self.stageCrossed[0],5)
                    print(self.stageCrossed[0])
                    if self.stageCrossed[0]!=-3:
                        self.stageCrossed[0]-=1
                    self.obstacles_list.append(Obstacles(type=type))
                    self.obstacles_list[-1].randomizeObstacle()
                elif self.score>0:
                    type=random.randint(0,2)
                    self.obstacles_list.append(Obstacles(type=type))
                    self.obstacles_list[-1].randomizeObstacle()
        
class Balls:
    def __init__(self,r_circ,r_ball=17):
        self.r_circ=r_circ
        self.r_ball=r_circ/7
        self.circ_center=[RES_X/2,(19*RES_Y)/20 -self.r_circ]
        self.theta_pos=0 
        #random noise for trail animation  
        self.phase=random.random()*2*PI
        self.ball1=[self.circ_center[0]+self.r_circ*math.cos(self.theta_pos),self.circ_center[1]+self.r_circ*math.sin(self.theta_pos)]
        new_theta=(self.theta_pos-math.pi)%(2*math.pi)
        self.ball2=[self.circ_center[0]+self.r_circ*math.cos(new_theta),self.circ_center[1]+self.r_circ*math.sin(new_theta)]
        #recording last 14 coordinates of both balls for trail animation
        self.history1 = [self.ball1]*14
        self.history2 = [self.ball2]*14
            
    def changePos(self,game):
        #Updating ball position parameter(theta) based on input
        if game.keyHandler['clockwise']==True and game.game_over==False:
            self.theta_pos+=game.v
        elif game.keyHandler['anticlockwise']==True and game.game_over==False:
            self.theta_pos-=game.v
        #Assigning the ith history coordinate to the i+1 element at each timestep
        for i in range(len(self.history1)-1,0,-1):
            self.history1[i]=self.history1[i-1]
            self.history2[i]=self.history2[i-1]
        self.history1[0]=self.ball1    
        self.history2[0]=self.ball2   
        #Updating ball coordinates
        self.ball1=[self.circ_center[0]+self.r_circ*math.cos(self.theta_pos),self.circ_center[1]+self.r_circ*math.sin(self.theta_pos)]
        new_theta=(self.theta_pos-math.pi)%(2*math.pi)
        self.ball2=[self.circ_center[0]+self.r_circ*math.cos(new_theta),self.circ_center[1]+self.r_circ*math.sin(new_theta)]                
    def display(self):
        #printing circular arc
        noFill()
        stroke(70)
        ellipse(self.circ_center[0],self.circ_center[1],2*self.r_circ,2*self.r_circ)
        #not displaying ball if it has collided
        if game.collidedBall!=1:
            fill(172,48,32)
            noStroke()
            ellipse(self.ball1[0],self.ball1[1],2*self.r_ball,2*self.r_ball)
            #if all history coordinates are equal, then display a straight trail with some random noise
            if check_equal(self.history1):
                for i in range(len(self.history1)):
                    fill(172,48,32,200-10*i)
                    ellipse(self.history1[0][0]+(0.4+0.08*i)*math.sin(self.phase+0.5*i),self.history1[0][1]+7*i,self.r_ball-i,self.r_ball-i)
                self.phase+=PI/10
            else:
                for i in range(len(self.history1)):
                    fill(172,48,32,200-10*i)
                    ellipse(self.history1[i][0],self.history1[i][1],self.r_ball-i,self.r_ball-i)
        if game.collidedBall!=2:
            noStroke()
            fill(173,200,255)
            ellipse(self.ball2[0],self.ball2[1],2*self.r_ball,2*self.r_ball)
            if check_equal(self.history2):
                for i in range(len(self.history2)):
                    fill(173,200,255,200-10*i)
                    ellipse(self.history2[0][0]+(0.4+0.08*i)*math.sin(self.phase+0.5*i),self.history2[0][1]+7*i,self.r_ball-i,self.r_ball-i)
            else:
                for i in range(len(self.history2)):
                    fill(173,200,255,200-10*i)
                    ellipse(self.history2[i][0],self.history2[i][1],self.r_ball-i,self.r_ball-i)
                    
class Obstacles():
    def __init__(self,type,v=4,invis=False):
        self.v=v
        self.rand_obst=obstacle_profiles[type]
        self.dims=self.rand_obst['dims']
        self.name=self.rand_obst['name']
        self.sep=self.rand_obst['separation']
        self.coords=[]
        self.invis=invis
        self.new_obst=False
        self.dialog=False
        self.dialogTime=0
        self.dialog_text=dialogs[0]
        
    #new update    
    def vary_velocity(self):
        delta_v = random.uniform(-0.5, 0.5)  
        self.v = max(0.2, self.v + delta_v)  
                 
    def randomizeObstacle(self):
        #method to call after every instance of Obstacles class is created to randomly assign one of 11 obstacle types and its assocatied parameters
        if random.random()>0.5 and (game.game_mode==1 or game.game_mode==0 and game.score>300):
            self.invis=True
        #setting dialog to be displayed based on which mode being played + some additional metrics
        if game.gameiteration>=5 and game.score_diff<=10:
            self.dialog_text=dialogs[1]
        elif game.game_started==True and game.game_mode==0:
            self.dialog_text=dialogs[random.randint(2,len(dialogs)-1)]
        elif game.game_mode==1:
            self.dialog_text='Your resilience is admirable\n but mistaken'
        if self.name=='rectangle array':
            self.offset=self.rand_obst['offset']
            #decides how many rectangles in array
            self.num=random.randint(2,5)
            #decides if rectangles on left or right(stochastic parameter)
            randint1=random.randint(0,1)
            x_coord=self.offset+randint1*(RES_X-2*self.offset-self.dims[0])
            for i in range(self.num):
                self.coords.append([x_coord,-i*(self.sep+self.dims[1])-self.dims[1]])
                
        elif self.name=='big rectangle followed by small rectangle':
            self.offset=self.rand_obst['offset']
            self.num=random.randint(1,3)
            if self.num>1:
                self.sep=self.sep*1.5
            #Initializing one of 4 random variations of this obstacle using 2 random integers
            randint1=random.randint(0,1)
            randint2=random.randint(0,1)
            for i in range(0,self.num):
                self.coords.append([self.offset+randint1*(RES_X-2*self.offset-self.dims[1][0]),-self.dims[1][1]-randint2*(self.sep+self.dims[1][1])-i*(2*self.sep+2*self.dims[0][1]),self.dims[1][0]])
                self.coords.append([RES_X-(self.offset+self.dims[0][0])-randint1*(RES_X-self.dims[0][0]-2*self.offset),-(self.dims[1][1]+self.dims[0][1]+self.sep)+randint2*(self.sep+self.dims[0][1])-i*(2*self.sep+2*self.dims[0][1] ),self.dims[0][0]])
        elif self.name=='rotating rectangle':
            self.theta=PI
            self.rot_v=0.07
            self.y=-self.dims[1]/2
            #setting rotation direction randomly
            self.dir=2*random.randint(0,1)-1
            upper_left=[RES_X/2 -(self.dims[0]/2 *math.cos(self.theta) +math.sin(self.theta)*self.dims[1]/2), self.y +(self.dims[0]/2 * math.sin(self.theta)-self.dims[1]/2*math.cos(self.theta)) ]
            lower_left=[RES_X/2 -(self.dims[0]/2 *math.cos(self.theta) -math.sin(self.theta)*self.dims[1]/2), self.y +(self.dims[0]/2 * math.sin(self.theta)+self.dims[1]/2*math.cos(self.theta)) ]
            upper_right=[RES_X/2 +(self.dims[0]/2 *math.cos(self.theta) -math.sin(self.theta)*self.dims[1]/2), self.y -(self.dims[0]/2 * math.sin(self.theta)+self.dims[1]/2*math.cos(self.theta)) ]
            lower_right=[RES_X/2 +(self.dims[0]/2 *math.cos(self.theta) +math.sin(self.theta)*self.dims[1]/2), self.y -(self.dims[0]/2 * math.sin(self.theta)-self.dims[1]/2*math.cos(self.theta)) ]
            self.coords=[upper_left,lower_left,upper_right,lower_right]      
        elif self.name=='duplicating rectangle':
            #will be set true when rectangle propelled forwards
            self.phase=False
            self.i=0
            self.offset=self.rand_obst['offset']
            #random parameter for initializing one of two possible obstacle variations
            self.layer_rects=random.randint(1,2)
            randint1=random.randint(0,1)
            self.coords.append([self.offset+randint1*(RES_X-self.dims[0]-2*self.offset),-2*self.dims[0]])
            if self.layer_rects==1:
                self.coords.append([self.offset+randint1*(RES_X-self.dims[0]-2*self.offset),-2*self.dims[0]])
            else:
                self.coords.append([RES_X-self.dims[0]-self.offset-randint1*(RES_X-2*self.offset-self.dims[0])  ,-2*self.dims[0]])
        
        elif self.name=='translating rectangles':
            self.num = random.randint(1,4)
            self.offset=self.rand_obst['offset']-20
            #constants for simple harmonic motion
            self.amplitude=RES_X/2-self.dims[0]/2-self.offset
            self.phase_shift= PI/4
            self.frequency=0.35
            self.omega=2*PI*self.frequency
            self.initialx=200
            #random parameter to decide translation direction
            self.dir=random.randint(0,1)*2-1
            self.coords.append([self.initialx,-self.dims[1]])
            self.time=math.asin((self.initialx+self.dims[0]/2 -RES_X/2)/self.amplitude)/self.omega
            for i in range(0,self.num-1):
                self.coords.append([ RES_X/2+ self.amplitude*math.sin(self.omega*self.time+(i+1)*self.phase_shift)-self.dims[0]/2    ,-self.dims[1]-(i+1)*(self.sep+self.dims[1])   ])
        
        elif self.name=='rotating circles':
            self.ball_diam=self.rand_obst['diam']
            self.offset=self.rand_obst['offset']
            self.gap=RES_X/5
            self.circle_rad=(RES_X-2*self.offset-self.gap)/4
            self.circle1=[self.offset+self.circle_rad,-self.circle_rad]
            self.circle2=[RES_X-self.offset-self.circle_rad,-self.circle_rad]
            #position parameters for both circles
            self.theta1=random.random()*2*PI
            self.theta2=random.random()*2*PI
            for i in range(2):
                self.coords.append([self.circle1[0]+self.circle_rad*math.cos(self.theta1-i*PI),self.circle1[1]+self.circle_rad*math.sin(self.theta1-i*PI)])
            for i in range(2):
                self.coords.append([self.circle2[0]+self.circle_rad*math.cos(self.theta2-i*PI),self.circle2[1]+self.circle_rad*math.sin(self.theta2-i*PI)])
            
        elif self.name=='oscillating rotating rectangle pair':
            #amplitude of simple harmonic motion
            self.amp=PI/2 - 0.4
            self.theta1=random.random()*self.amp
            self.theta2=random.random()*self.amp
            self.rot_speed=3
            self.time=math.asin(self.theta1/self.amp)/self.rot_speed
            self.time2=math.asin(self.theta2/self.amp)/self.rot_speed
            self.time=0
            self.time2=0
            self.y1=-self.dims[0]
            self.y2=self.y1-300
            self.x=70
            #x and y1,y2 are UL corners of both rectangles
            UR=[-self.x+self.dims[0]*math.cos(self.theta1),self.y1-self.dims[0]*math.sin(self.theta1)]
            LL=[-self.x+self.dims[1]*math.sin(self.theta1),self.y1+self.dims[1]*math.cos(self.theta1)]
            LR=[-self.x+self.dims[0]*math.cos(self.theta1)+self.dims[1]*math.sin(self.theta1),self.y1-self.dims[0]*math.sin(self.theta1)+self.dims[1]*math.cos(self.theta1)]
            UL=[-self.x,self.y1]
            UL2=[RES_X+self.x,self.y2]
            UR2=[UL2[0]-self.dims[0]*math.cos(self.theta2),self.y2-self.dims[0]*math.sin(self.theta2) ]
            LL2=[UL2[0]+self.dims[1]*math.sin(self.theta2),self.y2-self.dims[1]*math.cos(self.theta2)]
            LR2=[UL2[0]+self.dims[1]*math.sin(self.theta2)-self.dims[0]*math.cos(self.theta2),self.y2-self.dims[0]*math.sin(self.theta2)-self.dims[1]*math.cos(self.theta2)]
            self.coords=[UL,LL,UR,LR,UL2,LL2,UR2,LR2]
        
        elif self.name=='randomly rotated rectangles':
            #random parameters for movement direction and number of rectangles
            self.dir=random.randint(0,1)
            self.num=random.randint(1,3)
            self.theta=PI/40
            self.vx=self.v*math.sin(self.theta)
            self.vy=self.v*math.cos(self.theta)
            self.x=0+self.dir*(RES_X-self.dims[0])
            self.y=-self.dims[1]
         
        elif self.name=='horizontal rectangle array rotated':
            #random parameter for one of two obstacle types
            self.t_dir=random.randint(0,1)
            self.theta=(self.t_dir*2-1)*random.randint(30,60)*2*PI/360
            self.y=-self.dims[0]
            self.x=10
            self.vx=4.1
            self.coords=[self.x,self.x+self.sep]
        elif self.name=='squares':
            self.y=-self.dims
            self.offset=self.rand_obst['offset']
            self.num=random.randint(2,5)
            for i in range(self.num):
                #random parameter to decide left,center or right position for squares
                random_num=random.randint(0,2)
                if random_num==0:
                    self.coords.append([self.offset,self.y-i*(self.dims+self.sep)])
                elif random_num==1:
                    self.coords.append([RES_X-self.offset-self.dims,self.y-i*(self.dims+self.sep)])
                else:
                    self.coords.append([0.5*(RES_X-self.dims),self.y-i*(self.dims+self.sep)])
            
            self.dims=[self.dims,self.dims]
        
        elif self.name=='rotating rectangle pair':
            #iterative variable to initiate rotation after dialog display
            self.t=0
            self.circ_rad=self.dims[1]
            #gap between both rectangles
            self.gap=2.5*self.dims[0]
            self.w=self.dims[2]
            self.coords=[
                         [RES_X/2-self.gap/2,0 ],
                         [RES_X/2-self.gap/2-self.w,0 ],
                         [RES_X/2-self.gap/2-self.w,-2*self.circ_rad],
                         [RES_X/2-self.gap/2,-2*self.circ_rad ],
                         [RES_X/2+self.gap/2,0],
                         [RES_X/2+self.gap/2+self.w,0 ],
                         [RES_X/2+self.gap/2+self.w,-2*self.circ_rad ],
                         [RES_X/2+self.gap/2,-2*self.circ_rad ]
                         ] 
            self.dir=random.randint(0,1)*2-1
            self.rot_v=0.015*self.dir
            self.y=-self.circ_rad-50
            for i in self.coords:
                i[1]-=50
            #variable to randomly switch direction once
            self.direction_changed=False
            #coordinate number to track 
            self.i=3 if -self.dir==1 else 0
        elif self.name=='rotating seesaw':
            #lower theta will make parallelogram sides more diagonal
            self.theta_quad=65*2*PI/360
            self.dir=random.randint(0,1)*2-1
            self.rot_v=0.065*self.dir  
            #dimensions of parallelogram and squares on edges   
            self.y=-50
            self.h=self.dims[1]
            self.l=self.dims[0]-15
            self.h2=self.dims[2]
            self.l2=self.h/math.sin(self.theta_quad)
            #4 pairs of coordinates that correspond to vertices
            self.coords=[[RES_X/2+self.l2*math.cos(self.theta_quad)-self.l/2,self.y-self.h/2],
                         [RES_X/2+self.l/2,self.y-self.h/2],
                         [RES_X/2-self.l2*math.cos(self.theta_quad)+self.l/2,self.y+self.h/2],
                         [RES_X/2-self.l/2,self.y+self.h/2],
                         #left rectangle
                         [RES_X/2+self.l2*math.cos(self.theta_quad)-self.l/2-math.sin(self.theta_quad)*self.h2, self.y-self.h/2-math.cos(self.theta_quad)*self.h2],
                         [RES_X/2-self.l/2-math.sin(self.theta_quad)*self.h2,self.y+self.h/2-math.cos(self.theta_quad)*self.h2],
                         #right rectangle
                         [RES_X/2+self.l/2+math.sin(self.theta_quad)*self.h2,self.y-self.h/2+math.cos(self.theta_quad)*self.h2],
                         [RES_X/2-self.l2*math.cos(self.theta_quad)+self.l/2+math.sin(self.theta_quad)*self.h2,self.y+self.h/2+math.cos(self.theta_quad)*self.h2]
                         ]
    
    def display(self):
        fill(255)
        noStroke()
        if self.name=='rotating seesaw':
            fill(255)
            quad(self.coords[0][0],self.coords[0][1],self.coords[1][0],self.coords[1][1],self.coords[2][0],self.coords[2][1],self.coords[3][0],self.coords[3][1])
            quad(self.coords[0][0],self.coords[0][1],self.coords[3][0],self.coords[3][1],self.coords[5][0],self.coords[5][1],self.coords[4][0],self.coords[4][1])
            quad(self.coords[1][0],self.coords[1][1],self.coords[6][0],self.coords[6][1],self.coords[7][0],self.coords[7][1],self.coords[2][0],self.coords[2][1])

        elif self.name=='rotating rectangle pair':
            fill(255)
            quad(self.coords[0][0],self.coords[0][1],self.coords[1][0],self.coords[1][1],self.coords[2][0],self.coords[2][1],self.coords[3][0],self.coords[3][1])
            quad(self.coords[4][0],self.coords[4][1],self.coords[5][0],self.coords[5][1],self.coords[6][0],self.coords[6][1],self.coords[7][0],self.coords[7][1])
        elif self.name=='rectangle array' or self.name=='duplicating rectangle' or self.name=='translating rectangles' or self.name=='squares':
            for i in self.coords:
                if RES_Y>=i[1]>=-self.dims[1]:
                    if self.invis==True:
                        #decreasing transparency gradually as the obstacle arrives closer to the balls if invisble property set to true
                        fill(255,255,255,700-i[1])
                    else:
                        fill(255,255,255)
                    rect(int(i[0]),int(i[1]),self.dims[0],self.dims[1])
        elif self.name=='rotating rectangle':
            if self.invis==True:
                fill(255,255,255,600-self.y)
            else:
                fill(255,255,255) 
            pushMatrix()
            translate(self.coords[0][0],self.coords[0][1])
            rotate(-self.theta)
            rect(RES_X/2 -(self.coords[0][0]+self.dims[0]/2 *math.cos(self.theta) +math.sin(self.theta)*self.dims[1]/2),0,self.dims[0],self.dims[1])
            popMatrix()
        elif self.name=='big rectangle followed by small rectangle':
            for i in self.coords:
                if RES_Y>=i[1]>=-self.dims[1][1]:
                    if self.invis==True:
                        fill(255,255,255,700-i[1])
                    else:
                        fill(255,255,255)
                    rect(i[0],i[1],i[2],self.dims[0][1])
        elif self.name=='rotating circles':
            noFill()
            stroke(255)
            fill(255)
            for i in self.coords:
                ellipse(i[0],i[1],self.ball_diam,self.ball_diam)
        elif self.name=='oscillating rotating rectangle pair':
            fill(255)
            pushMatrix()
            translate(-self.x,self.y1)
            rotate(-self.theta1)
            rect(0,0,self.dims[0],self.dims[1])
            popMatrix()
            pushMatrix()
            translate(RES_X+self.x,self.y2)
            rotate(self.theta2+PI)
            rect(0,0,self.dims[0],self.dims[1])
            popMatrix()
            
        elif self.name=='randomly rotated rectangles':
            fill(255)
            for i in range(self.num):
                pushMatrix()
                translate(self.x+(2*self.dir-1)*i*math.cos(PI/2-self.theta)*(self.sep+self.dims[1]),self.y-i*math.sin(PI/2-self.theta)*(self.sep+self.dims[1]))
                rotate((2*self.dir-1)*self.theta)
                rect(0,0,self.dims[0],self.dims[1])
                popMatrix()
        
        elif self.name=='horizontal rectangle array rotated':
            if self.invis==True:
                fill(255,255,255,700-self.y)
            else:
                fill(255,255,255)
            for i in self.coords:
                pushMatrix()
                translate(i,self.y)
                rotate(self.theta)
                rect(0,0,self.dims[0],self.dims[1])
                popMatrix()
            
    def updateCoords(self,balls):
        #for coord in self.coords:
           #if 450 <= coord[1] <= 500: #and game.resetAnimation==False:
               #self.vary_velocity()
               #break
        #increasing obstacle velocity in Encore Mode
        if game.game_mode==1 and game.resetAnimation==False:
            self.v=5.3
        #if no dialog to be printed, update coords normally, otherwise stop updating positions and print dialog
        if self.dialog==False or self.dialogTime>140:
            #for each class of obstacle, we update positions of all obstacle vertices based on rotation matrices and position/angle parameters
            if self.name=='rectangle array' or self.name=='big rectangle followed by small rectangle' or self.name=='squares':
                for i in self.coords:
                    i[1]+=self.v
            elif self.name=='rotating rectangle':
                #rotating in different directions
                if self.dir==1:
                    self.theta+=self.rot_v
                else:
                    self.theta-=self.rot_v
                self.y+=self.v
                upper_left=[RES_X/2 -(self.dims[0]/2 *math.cos(self.theta) +math.sin(self.theta)*self.dims[1]/2), self.y +(self.dims[0]/2 * math.sin(self.theta)-self.dims[1]/2*math.cos(self.theta)) ]
                lower_left=[RES_X/2 -(self.dims[0]/2 *math.cos(self.theta) -math.sin(self.theta)*self.dims[1]/2), self.y +(self.dims[0]/2 * math.sin(self.theta)+self.dims[1]/2*math.cos(self.theta)) ]
                upper_right=[RES_X/2 +(self.dims[0]/2 *math.cos(self.theta) -math.sin(self.theta)*self.dims[1]/2), self.y -(self.dims[0]/2 * math.sin(self.theta)+self.dims[1]/2*math.cos(self.theta)) ]
                lower_right=[RES_X/2 +(self.dims[0]/2 *math.cos(self.theta) +math.sin(self.theta)*self.dims[1]/2), self.y -(self.dims[0]/2 * math.sin(self.theta)-self.dims[1]/2*math.cos(self.theta)) ]
                self.coords=[upper_left,upper_right,lower_left,lower_right]      
            elif self.name=='duplicating rectangle':
                #update normally if not reached 1/8 of the screen, then phase a predetermined distance with decreasing acceleration
                if (self.coords[0][1]<RES_Y*1/8 and self.phase==False) or self.coords[1][1]-self.coords[0][1]>=self.sep:
                    for i in self.coords:
                        i[1]+=self.v
                else:
                    self.coords[0][1]+=self.v
                    #update second rectangle at greater velocity compared to first rectangle
                    self.coords[1][1]+=self.v+0.06*self.i*self.v
                    self.i+=1
                    self.phase=True
            elif self.name=='translating rectangles':
                for i in self.coords:
                    i[1]+=self.v
                    #simple harmonic motion with phase shift on successive rectangles
                    i[0]=self.amplitude*math.sin(self.omega*self.time+self.coords.index(i)*self.phase_shift)+RES_X/2-self.dims[0]/2
                self.time+=0.02
            elif self.name=='rotating circles':
                self.theta1+=0.05
                self.theta2+=0.05
                self.circle1[1]+=self.v
                self.circle2[1]+=self.v
                for i in range(2):
                    self.coords[i]=[self.circle1[0]+self.circle_rad*math.cos(self.theta1-i*PI),self.circle1[1]+self.circle_rad*math.sin(self.theta1-i*PI)]
                for i in range(2):
                    self.coords[i+2]=[self.circle2[0]+self.circle_rad*math.cos(self.theta2-i*PI),self.circle2[1]+self.circle_rad*math.sin(self.theta2-i*PI)]
            
            elif self.name=='oscillating rotating rectangle pair':
                self.y1+=self.v
                self.y2+=self.v
                self.theta1=self.amp*math.sin(self.rot_speed*self.time)
                self.theta2=self.amp*math.sin(self.rot_speed*self.time2)
                UR=[-self.x+self.dims[0]*math.cos(self.theta1),self.y1-self.dims[0]*math.sin(self.theta1)]
                LL=[-self.x+self.dims[1]*math.sin(self.theta1),self.y1+self.dims[1]*math.cos(self.theta1)]
                LR=[-self.x+self.dims[0]*math.cos(self.theta1)+self.dims[1]*math.sin(self.theta1),self.y1-self.dims[0]*math.sin(self.theta1)+self.dims[1]*math.cos(self.theta1)]
                UL=[-self.x,self.y1]
                UL2=[RES_X+self.x,self.y2]
                UR2=[UL2[0]-self.dims[0]*math.cos(self.theta2),self.y2-self.dims[0]*math.sin(self.theta2) ]
                LL2=[UL2[0]+self.dims[1]*math.sin(self.theta2),self.y2-self.dims[1]*math.cos(self.theta2)]
                LR2=[UL2[0]+self.dims[1]*math.sin(self.theta2)-self.dims[0]*math.cos(self.theta2),self.y2-self.dims[0]*math.sin(self.theta2)-self.dims[1]*math.cos(self.theta2)]
                self.coords=[UL,UR,LL,LR,UL2,UR2,LL2,LR2]
                self.time+=0.01
                self.time2+=0.01
            elif self.name=='randomly rotated rectangles':
                if game.game_mode==1 or game.resetAnimation==True:
                    self.vx=self.v*math.sin(self.theta)
                    self.vy=self.v*math.cos(self.theta)
                self.y+=self.vy
                self.x-=(self.dir*2-1)*self.vx
                
            elif self.name=='horizontal rectangle array rotated':
                for i in range(len(self.coords)):
                    self.coords[i]+=self.vx
                #if one rectangle touches the screen, append another rectangle from opposite side of the screen
                if self.coords[0]+self.dims[0]*math.cos(self.theta)+(self.t_dir*2-1)*self.dims[1]*math.sin(self.theta)>=self.sep:
                    self.coords.insert(0,-self.dims[0]*math.cos(self.theta)-1*(self.t_dir*2-1)*self.dims[1]*math.sin(self.theta))
                if self.coords[-1]>=RES_X:
                    self.coords.pop()    
                self.y+=self.v
            elif self.name=='rotating rectangle pair':
                bool=None
                #bool checks whether rectangle pair has rotated back to vertical orientation after random rotation phases are over
                if self.i==0:
                    bool= self.y+self.circ_rad+1>=self.coords[self.i][1]>=self.y+self.circ_rad-1 and RES_X/2-self.gap/2 -3<=self.coords[self.i][0]<=RES_X/2-self.gap/2+3
                else:
                    bool= self.y-self.circ_rad-1<=self.coords[self.i][1]<=self.y-self.circ_rad+1 and RES_X/2-self.gap/2 -3<=self.coords[self.i][0]<=RES_X/2-self.gap/2+3
                #if random rotation over or rectangle pair not yet reached circle center, then translate vertically
                if self.y<=balls.circ_center[1] or (self.direction_changed==True and bool) or game.resetAnimation==True:
                    self.y+=self.v
                    for i in range(len(self.coords)):
                        self.coords[i][1]+=self.v
                    if RES_Y+105>=self.y>=RES_Y+100:
                        print('yo')
                        game.v=circle_v
                else:
                    self.t+=1
                    game.v=0.0154
                    if self.t>120:
                        #start random rotation phase
                        game.score-=0.025
                        for i in range(len(self.coords)):
                            self.coords[i]=[RES_X/2+ math.cos(self.rot_v)*(-RES_X/2+self.coords[i][0])-math.sin(self.rot_v)*(-self.y+self.coords[i][1]),self.y+math.sin(self.rot_v)*(-RES_X/2+self.coords[i][0])+(-self.y+self.coords[i][1])*math.cos(self.rot_v)]
                        #allow rotation reversal once
                        if self.t%100==0 and self.direction_changed==False:
                            rand=random.randint(1,4)==1
                            if rand:
                                self.rot_v=-self.rot_v
                                self.direction_changed=True
                    else:
                        #give warning
                        game.score-=0.05
                        textSize(50)
                        fill(255,255,255)
                        textAlign(CENTER, CENTER)
                        textFont(customFont)
                        text("Remain Vigilant", RES_X/2, RES_Y/2-20)
            elif self.name=='rotating seesaw':
                for i in range(len(self.coords)):
                    #applying rotation matrix on all polygon vertexes
                    self.coords[i]=[RES_X/2+ math.cos(self.rot_v)*(-RES_X/2+self.coords[i][0])-math.sin(self.rot_v)*(-self.y+self.coords[i][1]),self.y+math.sin(self.rot_v)*(-RES_X/2+self.coords[i][0])+(-self.y+self.coords[i][1])*math.cos(self.rot_v)]
                for i in self.coords:
                    i[1]+=self.v
                self.y+=self.v
        else:
            game.score-=0.05
            fill(255,255,255)
            if game.game_mode==1:
                fill(255,0,0)
            textAlign(CENTER, CENTER)
            textFont(customFont)
            textSize(30)
            text(self.dialog_text, RES_X/2, RES_Y/2-20)
            self.dialogTime+=1
                
    def detectCollisions(self,balls):
        ball_coords=[balls.ball1,balls.ball2]
        collidedBall=None
        if self.name=='rectangle array' or self.name=='duplicating rectangle' or self.name=='translating rectangles' or self.name=='squares':
            for i in self.coords:
                for ball in ball_coords:
                    #simple collision check for horizontally oriented rectangles/squares
                    if i[0]-balls.r_ball<=ball[0]<=i[0]+self.dims[0]+balls.r_ball and i[1]-balls.r_ball<=ball[1]<=i[1]+self.dims[1]+balls.r_ball:
                        game.game_over=True
                        collidedBall=ball
                        vector_list=defineVectors([i,[i[0]+self.dims[0],i[1]],[i[0],i[1]+self.dims[1]],[i[0]+self.dims[0] ,i[1]+self.dims[1] ]   ])
                        collidedBall=checkCollision(vector_list,ball_coords)
        elif self.name=='big rectangle followed by small rectangle':
            for i in self.coords:
                for ball in ball_coords:
                    if i[0]-balls.r_ball<=ball[0]<=i[0]+i[-1]+balls.r_ball and i[1]-balls.r_ball<=ball[1]<=i[1]+self.dims[0][1]+balls.r_ball:
                        game.game_over=True
                        collidedBall=ball
                        vector_list=defineVectors([i,[i[0]+i[-1],i[1]],[i[0],i[1]+self.dims[0][1]],[i[0]+i[-1] ,i[1]+self.dims[0][1] ]   ])
                        collidedBall=checkCollision(vector_list,ball_coords)
        elif self.name=='rotating rectangle':
            vector_list=defineVectors(self.coords)
            ball_list=[balls.ball1,balls.ball2]
            collidedBall=checkCollision(vector_list,ball_list)        
        elif self.name=='rotating circles':
            for i in self.coords:
                for ball in ball_coords:
                    #computing distance between ball centers
                    if calculate_distance(i,ball)<=balls.r_ball+self.ball_diam/2:
                        game.game_over=True
                        collidedBall=ball
        elif self.name=='oscillating rotating rectangle pair':
            vector_list=defineVectors(self.coords[:4])[1:]+defineVectors(self.coords[4:])[1:]
            ball_list=[balls.ball1,balls.ball2]
            collidedBall=checkCollision(vector_list,ball_list)
        elif self.name=='horizontal rectangle array rotated':
            vector_list=[]
            ball_list=[balls.ball1,balls.ball2]
            for i in range(len(self.coords)):
                UL=[self.coords[0]+i*self.sep,self.y]
                UR=[self.coords[0]+i*self.sep+math.cos(self.theta)*self.dims[0],self.y+math.sin(self.theta)*self.dims[0]]
                LL=[self.coords[0]+i*self.sep-math.sin(self.theta)*self.dims[1],self.y+math.cos(self.theta)*self.dims[1]]
                LR=[self.coords[0]+i*self.sep+math.cos(self.theta)*self.dims[0]-math.sin(self.theta)*self.dims[1],self.y+math.sin(self.theta)*self.dims[0]+math.cos(self.theta)*self.dims[1]]
                vector_list=vector_list+defineVectors([UL,UR,LL,LR])
            collidedBall=checkCollision(vector_list,ball_list)
        elif self.name=='randomly rotated rectangles':
            ball_list=[balls.ball1,balls.ball2]
            for i in range(self.num):
                UL=[self.x+(2*self.dir-1)*i*math.cos(PI/2-self.theta)*(self.sep+self.dims[1]),self.y-i*math.sin(PI/2-self.theta)*(self.sep+self.dims[1])]
                UR=[UL[0]+math.cos(self.theta)*self.dims[0],UL[1]+(2*self.dir-1)*math.sin(self.theta)*self.dims[0] ]
                LL=[UL[0]-(2*self.dir-1)*math.sin(self.theta)*self.dims[1],UL[1]+math.cos(self.theta)*self.dims[1]]
                LR=[UR[0]-(2*self.dir-1)*math.sin(self.theta)*self.dims[1],UR[1]+math.cos(self.theta)*self.dims[1]]
                vector_list=defineVectors([UL,UR,LL,LR])
                result=checkCollision(vector_list,ball_list)
                if result!=None:
                    collidedBall=result 
        elif self.name=='rotating rectangle pair':
            vector_list=defineVectors([self.coords[1],self.coords[2],self.coords[0],self.coords[3]])+defineVectors([self.coords[6],self.coords[5],self.coords[7],self.coords[4]])
            collidedBall=checkCollision(vector_list,ball_coords)
        elif self.name=='rotating seesaw':
            vector_list=defineVectors([self.coords[0],self.coords[1],self.coords[3],self.coords[2]])+defineVectors([self.coords[1],self.coords[6],self.coords[7],self.coords[2]])+defineVectors([self.coords[4],self.coords[0],self.coords[5],self.coords[3]])
            ball_list=[balls.ball1,balls.ball2]
            collidedBall=checkCollision(vector_list,ball_list)
        if game.game_over==True:
            #play collision sound effect
            game.collMusic.play()
            col=[]
            #record collided ball to remove it from display and display screen statistics and collision/splash particles in its color
            if ball_coords.index(collidedBall)==1:
                game.collidedBall=2
                col=[173,200,255]
            else:
                col=[172,48,32]
                game.collidedBall=1
            #initiate collision particles
            for i in range(NUM_OF_COL_PARTICLES):
                particle=Particle(RES_X / 2, RES_Y / 2,dia=[5,8],ySpd=[-8,15],xSpd=3)
                particle.x=collidedBall[0]
                particle.y=collidedBall[1]
                particle.color=(col[0],col[1],col[2])
                game.collision_particles.append(particle)
            if self.name!='rotating circles':
                #initiate splash particles
                for i in range(NUM_OF_SPLASH_PARTICLES) :
                    particle=Particle(RES_X / 2, RES_Y / 2,dia=[1,6],ySpd=[6,6],xSpd=6)
                    particle.x=game.proj_coord[0]
                    particle.y=game.proj_coord[1]
                    particle.color=(col[0],col[1],col[2])
                    game.splash_particles.append(particle)
            game.game_end_col=col
    
balls=Balls(RES_X/4)
obstacle_profiles=[{'name':'squares','dims':RES_X/4.3,'separation':RES_X/3,'offset':RES_X/5},
           {'name':'big rectangle followed by small rectangle','shape':'rect','dims':[[RES_X/2,RES_Y/17],[RES_X/4,RES_Y/17]],'separation':1.8*RES_Y/17,'offset':RES_X/16},
           {'name':'rectangle array','dims':[RES_X/2,RES_Y/17],'separation':(2*balls.r_circ)*0.7,'offset':RES_X/16},
           {'name':'duplicating rectangle','dims':[RES_X/2,RES_Y/17],'offset':RES_X/16,'separation':(2*balls.r_circ*0.9)},
           {'name':'rotating rectangle','dims':[RES_X/2,RES_Y/18],'separation':(1.3*RES_X/2)},
           {'name':'oscillating rotating rectangle pair','dims':[RES_X/1.5,RES_Y/17],'separation':None},
           {'name':'randomly rotated rectangles','dims':[RES_X/2,RES_Y/17],'separation':(2*balls.r_circ)*0.8},
           {'name':'translating rectangles','dims':[RES_X/2.5,RES_Y/19],'offset':RES_X/20,'separation':(2*balls.r_circ)*0.95},
           {'name':'rotating circles','dims':[RES_X/4,RES_Y/19],'offset':RES_X/20,'separation':None,'diam':25},
           {'name':'horizontal rectangle array rotated','dims':[RES_X/2.5,RES_Y/19],'separation':RES_X/2},
           {'name':'rotating rectangle pair','dims':[balls.r_ball*3,1.2*balls.r_circ,RES_Y/17],'separation':None},
           {'name':'rotating seesaw','dims':[RES_X/2,RES_Y/18,RES_X/6],'separation':None}
           ]
dialogs=[
        'It is so good to see you.\n Stay close to me and don\'t \ntouch anything.',
        'Repeating the same action\n but expecting new results\n is a sign of madness',
        'I want you to\n want to keep going',
        'Confront your fears',
        'Patterns always emerge,\n even in chaos',
        'Your failures need not\n define you',
        'You must move forward']
game=Game(3,circle_v,game_started=False)
game.obstacles_list[0].randomizeObstacle()
def setup():
    size(RES_X,RES_Y)
    background(0)
    global customFont, NexaHeavy
    customFont=createFont('Nexa-ExtraLight.ttf',32)
    NexaHeavy=createFont('Nexa-Heavy.ttf',32)
    
def draw():
    background(0)
    #display screen when game newly launched
    if game.game_started==False:
        fill(255)
        textFont(customFont)
        textAlign(CENTER)
        textSize(150)
        text('DUET',RES_X/2,200)
        textSize(25)
        text('Instructions:\nRight key to rotate clockwise.\n Left key to rotate anticlockwise.\n\nGoal:\n Avoid the incoming obstacles\n for as long as possible',RES_X/2,280)
        fill(255,0,0)
        text('Press R for Encore Mode',RES_X/2,580)
        textSize(30)
        fill(255)
        text('Press\n any button\n to play',RES_X/2,685)
        game.display()
        balls.changePos(game)
        balls.display()
    #animation to display when game is restarted
    elif game.resetAnimation==True:
        #decrease ball rotation speed over time for smooth update that looks nice
        game.v=circle_v*(2-game.t*0.06)
        game.display()
        balls.changePos(game)
        balls.display()
        speed=game.obstacles_list[0].v
        for obstacle in game.obstacles_list:
            #decrease obstacle velocity over time for smooth update that looks nice
            obstacle.v=(3-game.t)*speed
            obstacle.updateCoords(balls)
            obstacle.display()
            obstacle.v=speed
        game.t+=0.2
        if game.obstacles_list[0].name=='horizontal rectangle array rotated' or game.obstacles_list[0].name=='randomly rotated rectangles':
            coord=game.obstacles_list[0].y
        else:
            coord=game.obstacles_list[0].coords[0][1]
        #if last obstacle has left the screen, stop animation
        if coord<-200 and balls.circ_center[1]-5<=balls.ball1[1]<=balls.circ_center[1]+5 and game.v<0.7*circle_v:
            if game.game_mode==0:
                game.obstacles_list=[Obstacles(type=random.randint(0,2))]
            else:
                game.obstacles_list=[Obstacles(type=random.randint(9,11))]
            #game.obstacles_list=[Obstacles(type=11)]
            game.obstacles_list[0].randomizeObstacle()
            game.obstacles_list[0].dialog=True
            game.resetAnimation=False
            game.keyHandler['anticlockwise']=False
            balls.history1=[balls.ball1]*14
            balls.history2=[balls.ball2]*14
            game.v=circle_v
    #Normal game screen
    elif game.game_over==False:
        game.score+=0.05
        textFont(NexaHeavy)
        textSize(20)
        textAlign(BASELINE,BASELINE)
        text('Game Score:'+str(int(game.score)),RES_X-170,RES_Y/20)
        if game.game_mode==0:
            text('High Score:'+str(game.high_score),RES_X-170,RES_Y/20+30)
            text('Set by '+game.hs_setter,RES_X-170,RES_Y/20+60)
        else:
            fill(255,0,0)
            text('High Score:'+str(game.encore_hs),RES_X-170,RES_Y/20+30)
            text('Set by '+game.encore_hs_setter,RES_X-170,RES_Y/20+60)
        game.display()
        balls.changePos(game)
        balls.display()
        if game.displayingText==False:
            for obstacle in game.obstacles_list:
                obstacle.updateCoords(balls)
                obstacle.display()
                obstacle.detectCollisions(balls)
                if game.game_over==True:
                    break
            game.updateObstaclesList()
    #game over screen
    else:
        balls.changePos(game)
        balls.display()
        for obstacle in game.obstacles_list:
            obstacle.invis=False
            obstacle.display()
        game.display()
        textAlign(CENTER, CENTER)
        textFont(customFont)
        fill(255,0,0)
        textSize(20)
        text('Press R to change game mode',RES_X-170,RES_Y/20)
        fill(game.game_end_col[0],game.game_end_col[1],game.game_end_col[2])
        textSize(30)
        highscore=0
        if game.game_mode==1:
            highscore=game.encore_hs
        else:
            highscore=game.high_score
        #if high score is beaten, ask for new high score for write and print on screen
        if int(game.score)> highscore:
            text("Game Over!", RES_X/2, RES_Y/2-20-50)
            text('Final score:'+str(int(game.score)),RES_X/2,RES_Y/2+20-50)
            textFont(NexaHeavy)
            file=None
            if game.game_mode==0:
                file=open('highscore.txt','w')
            else:
                file=open('highscore2.txt','w')
            text('You set a new high score!',RES_X/2,RES_Y/2+90-50)
            text('Enter your name below',RES_X/2,RES_Y/2+140-50)
            text('and press enter:',RES_X/2,RES_Y/2+173-50)
            game.inputting_score=True
            text(game.new_hs_setter,RES_X/2,RES_Y/2+270)
            file.write(str(int(game.score))+'\n')
            file.write(game.new_hs_setter+'\n')
            file.close()    
        else:
            text("Game Over!", RES_X/2, RES_Y/2-20)
            text('Final score:'+str(int(game.score)),RES_X/2,RES_Y/2+20)
def keyPressed():
    if keyCode==LEFT and game.game_started==True and game.resetAnimation==False:
        game.keyHandler['anticlockwise']=True
    elif keyCode==RIGHT and game.game_started==True and game.resetAnimation==False:
        game.keyHandler['clockwise']=True
    elif game.game_started==False:
        game.game_started=True
        game.keyHandler['anticlockwise']=False
        if keyCode==82:
            print(True)
            game.game_mode=1
            game.obstacles_list=[Obstacles(type=random.randint(0,11))]
            game.obstacles_list[0].randomizeObstacle()
            game.obstacles_list[0].dialog_text='You are not ready for this'
            game.obstacles_list[0].dialog=True
    elif game.inputting_score==True and keyCode==10:
        game.inputting_score=False
        mouseClicked()
    elif game.inputting_score==True and keyCode==8:
        game.new_hs_setter=game.new_hs_setter[:-1]
    elif game.inputting_score==True and keyCode!=20 and keyCode!=RIGHT and keyCode!=LEFT:
        game.new_hs_setter=game.new_hs_setter+str(chr(keyCode))
    if game.game_over==True and keyCode==82 and game.inputting_score==False:
        game.game_mode=(game.game_mode+1)%2
        mouseClicked()
def keyReleased():
    if keyCode==LEFT and game.game_started==True and game.resetAnimation==False:
        game.keyHandler['anticlockwise']=False
    elif keyCode==RIGHT and game.game_started==True and game.resetAnimation==False:
        game.keyHandler['clockwise']=False
    
def mouseClicked():
    if game.game_over==True and game.inputting_score==False:
        game.bgMusic.close()
        last_score=game.score
        difference=abs(last_score-game.last_score)
        iteration=game.gameiteration+1
        mode=game.game_mode
        obstacles_list=None
        if mode==0:
            obstacles_list=game.obstacles_list[-5:]
        else:
            obstacles_list=game.obstacles_list[-3:]
        ball_coords=[balls.ball1,balls.ball2]
        global balls,game
        balls=Balls(RES_X/4)
        balls.ball1=ball_coords[0]
        balls.ball2=ball_coords[1]
        game=Game(3,circle_v,iteration=iteration,reset=True,mode=mode)
        game.obstacles_list=obstacles_list
        game.last_score=last_score
        game.score_diff=difference
        game.keyHandler['anticlockwise']=True
        loop()
