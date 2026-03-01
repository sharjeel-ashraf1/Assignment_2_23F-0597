import pygame, sys, random, math, time
from heapq import heappush, heappop

CELL_SIZE = 24
FPS = 30


WHITE = (245,245,245)
BLACK = (20,20,20)
GRAY = (180,180,180)

WALL_COLOR = (40,40,40)
START_COLOR = (30,180,30)
GOAL_COLOR = (180,30,30)
PATH_COLOR = (40,200,80)
AGENT_COLOR = (20,20,200)

VISITED_COLOR = (70,130,180)
FRONTIER_COLOR = (255,210,0)

BUTTON_COLOR = (200,200,200)

DYNAMIC_SPAWN_PROB = 0.01

PANEL_HEIGHT = 200 


def manhattan(a,b):
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def euclidean(a,b):
    return math.hypot(a[0]-b[0], a[1]-b[1])

def neighbors(pos, rows, cols):
    r,c = pos
    for dr,dc in ((1,0),(-1,0),(0,1),(0,-1)):
        nr,nc = r+dr,c+dc
        if 0 <= nr < rows and 0 <= nc < cols:
            yield (nr,nc)


class PriorityQueue:
    def __init__(self):
        self.heap=[]
        self.count=0

    def push(self,p,item):
        self.count+=1
        heappush(self.heap,(p,self.count,item))

    def pop(self):
        return heappop(self.heap)[2] if self.heap else None

    def empty(self):
        return len(self.heap)==0

    def as_list(self):
        return [h[2] for h in self.heap]


def reconstruct_path(came,start,goal):
    path=[]
    cur=goal

    while cur!=start:
        path.append(cur)
        cur=came.get(cur)
        if cur is None:
            return []

    path.append(start)
    path.reverse()
    return path


def a_star(grid,start,goal,hfn):
    rows,cols=len(grid),len(grid[0])

    frontier=PriorityQueue()
    frontier.push(0,start)

    came={}
    g={start:0}
    visited=set()

    while not frontier.empty():
        cur=frontier.pop()
        visited.add(cur)

        if cur==goal:
            return reconstruct_path(came,start,goal),visited,frontier.as_list()

        for nb in neighbors(cur,rows,cols):
            if grid[nb[0]][nb[1]]==1:
                continue

            tg=g[cur]+1

            if tg < g.get(nb,float("inf")):
                came[nb]=cur
                g[nb]=tg
                frontier.push(tg+hfn(nb,goal),nb)

    return [],visited,frontier.as_list()


def gbfs(grid,start,goal,hfn):
    rows,cols=len(grid),len(grid[0])

    frontier=PriorityQueue()
    frontier.push(hfn(start,goal),start)

    came={}
    visited=set()

    while not frontier.empty():
        cur=frontier.pop()
        visited.add(cur)

        if cur==goal:
            return reconstruct_path(came,start,goal),visited,frontier.as_list()

        for nb in neighbors(cur,rows,cols):
            if grid[nb[0]][nb[1]]==1:
                continue

            if nb not in visited and nb not in came:
                came[nb]=cur
                frontier.push(hfn(nb,goal),nb)

    return [],visited,frontier.as_list()


class PathfindingApp:

    def __init__(self,rows,cols):

        pygame.init()

        self.rows=rows
        self.cols=cols

        self.grid_width=cols*CELL_SIZE
        self.grid_height=rows*CELL_SIZE

        self.window_width = max(self.grid_width, 700) 
        self.window_height = self.grid_height + PANEL_HEIGHT

        self.screen=pygame.display.set_mode(
            (self.window_width, self.window_height))

        pygame.display.set_caption("Pathfinding Visualizer")

        self.clock=pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 14)
        self.font_small = pygame.font.SysFont("arial", 12)
        self.font_bold = pygame.font.SysFont("arial", 16, bold=True)

        self.grid=[[0]*cols for _ in range(rows)]

        self.start=(0,0)
        self.goal=(rows-1,cols-1)

        self.agent_pos=self.start

        self.algorithm="A*"
        self.heuristic_fn=manhattan
        self.heuristic_name="Manhattan"

        self.running_agent=False

        self.path=[]
        self.path_index=0

        self.visited_nodes=set()
        self.frontier_nodes=[]

        self.obstacle_density=0.3
        self.generate_random_map(self.obstacle_density)
        
        self.buttons = {}
        
        self.metrics = {
            'nodes_visited': 0,
            'path_cost': 0,
            'execution_time': 0,
            'last_execution_time': 0
        }
        
        self.recalculate_path()

    def layout_rect(self,r,c):
      
        x_offset = (self.window_width - self.grid_width) // 2
        return pygame.Rect(
            x_offset + c*CELL_SIZE,
            r*CELL_SIZE,
            CELL_SIZE-1,
            CELL_SIZE-1
        )

    def generate_random_map(self,density):
        self.grid=[[1 if random.random()<density else 0
                    for _ in range(self.cols)]
                   for _ in range(self.rows)]

        self.grid[self.start[0]][self.start[1]]=0
        self.grid[self.goal[0]][self.goal[1]]=0


   
    def recalculate_path(self):

        s=self.agent_pos
        g=self.goal

    
        start_time = time.time()

        if self.algorithm=="A*":
            path,vis,front=a_star(self.grid,s,g,self.heuristic_fn)
        else:
            path,vis,front=gbfs(self.grid,s,g,self.heuristic_fn)

       
        end_time = time.time()
        
       
        self.path = path
        self.visited_nodes = vis
        self.frontier_nodes = front
        
        self.metrics['nodes_visited'] = len(vis)
        self.metrics['path_cost'] = len(path) - 1 if path else 0  
        self.metrics['execution_time'] = (end_time - start_time) * 1000  
        self.metrics['last_execution_time'] = time.strftime('%H:%M:%S') 

        self.path_index = 0


    def spawn_obstacles(self):

        if random.random()>DYNAMIC_SPAWN_PROB:
            return []

        free=[(r,c) for r in range(self.rows)
              for c in range(self.cols)
              if self.grid[r][c]==0
              and (r,c) not in {self.start,self.goal,self.agent_pos}]

        if not free:
            return []

        cell=random.choice(free)
        self.grid[cell[0]][cell[1]]=1
        return [cell]


    def agent_step(self):

        if not self.path or self.path_index>=len(self.path)-1:
            self.running_agent=False
            return

        spawned=self.spawn_obstacles()

        if spawned:
            path_set=set(self.path[self.path_index+1:])
            for s in spawned:
                if s in path_set:
                    self.recalculate_path()
                    return

        self.path_index+=1
        self.agent_pos=self.path[self.path_index]


    def draw_button(self, text, x, y, w=110, h=30):
        rect = pygame.Rect(x, y, w, h)
        pygame.draw.rect(self.screen, BUTTON_COLOR, rect)
        pygame.draw.rect(self.screen, BLACK, rect, 2)
        
        txt = self.font_bold.render(text, True, BLACK)
        text_rect = txt.get_rect(center=rect.center)
        self.screen.blit(txt, text_rect)
        
        return rect


    def draw(self):

        self.screen.fill(WHITE)

       
        x_offset = (self.window_width - self.grid_width) // 2

       
        for r in range(self.rows):
            for c in range(self.cols):
                rect=self.layout_rect(r,c)
                color=WHITE if self.grid[r][c]==0 else WALL_COLOR
                pygame.draw.rect(self.screen,color,rect)
  
                pygame.draw.rect(self.screen, (220,220,220), rect, 1)

        for r,c in self.visited_nodes:
            if (r,c) != self.start and (r,c) != self.goal:
                pygame.draw.rect(self.screen, VISITED_COLOR,
                              self.layout_rect(r,c))

        
        for r,c in self.frontier_nodes:
            if (r,c) != self.start and (r,c) != self.goal:
                pygame.draw.rect(self.screen, FRONTIER_COLOR,
                              self.layout_rect(r,c))

       
        for r,c in self.path:
            if (r,c) != self.start and (r,c) != self.goal:
                pygame.draw.rect(self.screen, PATH_COLOR,
                              self.layout_rect(r,c))

       
        pygame.draw.rect(self.screen, START_COLOR,
                         self.layout_rect(*self.start))
        pygame.draw.rect(self.screen, GOAL_COLOR,
                         self.layout_rect(*self.goal))
        pygame.draw.rect(self.screen, AGENT_COLOR,
                         self.layout_rect(*self.agent_pos))

        panel_rect = pygame.Rect(0, self.grid_height, self.window_width, PANEL_HEIGHT)
        pygame.draw.rect(self.screen, (240,240,240), panel_rect)
        pygame.draw.rect(self.screen, BLACK, panel_rect, 2)

      
        button_y = self.grid_height + 15
        button_x = 20
        
        a_btn = self.draw_button("A*", button_x, button_y, w=90)
        gb_btn = self.draw_button("GBFS", button_x + 100, button_y, w=90)
        h_btn = self.draw_button(self.heuristic_name[:10], button_x + 200, button_y, w=110)
      
        button_y2 = self.grid_height + 55
        sp_btn = self.draw_button("Start/Pause", button_x, button_y2, w=150)
        reset_btn = self.draw_button("Reset", button_x + 160, button_y2, w=90)
        clear_btn = self.draw_button("Clear", button_x + 260, button_y2, w=90)

        self.buttons = {
            "A": a_btn,
            "GB": gb_btn,
            "H": h_btn,
            "S": sp_btn,
            "RESET": reset_btn,
            "CLEAR": clear_btn
        }

        metrics_x = self.window_width - 280
        metrics_y = self.grid_height + 15
        
   
        title = self.font_bold.render("METRICS DASHBOARD", True, BLACK)
        self.screen.blit(title, (metrics_x, metrics_y))

        metrics_rect = pygame.Rect(metrics_x - 5, metrics_y - 5, 260, 130)
        pygame.draw.rect(self.screen, (255,255,255), metrics_rect)
        pygame.draw.rect(self.screen, BLACK, metrics_rect, 1)
        
       
        metrics_data = [
            f"Nodes Visited: {self.metrics['nodes_visited']}",
            f"Path Cost: {self.metrics['path_cost']}",
            f"Execution Time: {self.metrics['execution_time']:.2f} ms",
            f"Algorithm: {self.algorithm}",
            f"Heuristic: {self.heuristic_name}"
        ]
        
        for i, text in enumerate(metrics_data):
            color = (0,100,0) if "Path Cost" in text and self.metrics['path_cost'] > 0 else BLACK
            txt = self.font.render(text, True, color)
            self.screen.blit(txt, (metrics_x, metrics_y + 25 + i*18))

      
        status_y = self.grid_height + 160
        status_color = (0,150,0) if self.running_agent else (150,0,0)
        status_text = f"STATUS: {'AGENT MOVING' if self.running_agent else 'PAUSED'}"
        if not self.path:
            status_text = "STATUS: NO PATH FOUND"
            status_color = (200,0,0)
            
        status_display = self.font_bold.render(status_text, True, status_color)
        self.screen.blit(status_display, (20, status_y))

        
        inst_y = self.grid_height + 180
        inst_text = "SPACE: Start/Pause | R: Reset | A: Switch Algo | H: Switch Heuristic"
        inst = self.font_small.render(inst_text, True, (100,100,100))
        self.screen.blit(inst, (20, inst_y))

        pygame.display.flip()

    def run(self):

        while True:

            self.clock.tick(FPS)

            for event in pygame.event.get():

                if event.type==pygame.QUIT:
                    pygame.quit()
                    sys.exit()

                elif event.type==pygame.KEYDOWN:

                    if event.key==pygame.K_SPACE:
                        self.running_agent=not self.running_agent

                    elif event.key==pygame.K_a:
                        self.algorithm="GBFS" if self.algorithm=="A*" else "A*"
                        self.recalculate_path()

                    elif event.key==pygame.K_h:
                        if self.heuristic_name=="Manhattan":
                            self.heuristic_name="Euclidean"
                            self.heuristic_fn=euclidean
                        else:
                            self.heuristic_name="Manhattan"
                            self.heuristic_fn=manhattan
                        self.recalculate_path()
                    
                    elif event.key==pygame.K_r:
                        self.agent_pos = self.start
                        self.running_agent = False
                        self.recalculate_path()
                    
                    elif event.key==pygame.K_c:
                        self.grid = [[0]*self.cols for _ in range(self.rows)]
                        self.grid[self.start[0]][self.start[1]] = 0
                        self.grid[self.goal[0]][self.goal[1]] = 0
                        self.agent_pos = self.start
                        self.running_agent = False
                        self.recalculate_path()

                elif event.type==pygame.MOUSEBUTTONDOWN:

                    mx,my=pygame.mouse.get_pos()

                    for key, rect in self.buttons.items():
                        if rect.collidepoint((mx, my)):
                            if key == "A":
                                self.algorithm = "A*"
                                self.recalculate_path()
                            elif key == "GB":
                                self.algorithm = "GBFS"
                                self.recalculate_path()
                            elif key == "H":
                                if self.heuristic_name == "Manhattan":
                                    self.heuristic_name = "Euclidean"
                                    self.heuristic_fn = euclidean
                                else:
                                    self.heuristic_name = "Manhattan"
                                    self.heuristic_fn = manhattan
                                self.recalculate_path()
                            elif key == "S":
                                self.running_agent = not self.running_agent
                            elif key == "RESET":
                                self.agent_pos = self.start
                                self.running_agent = False
                                self.recalculate_path()
                            elif key == "CLEAR":
                                self.grid = [[0]*self.cols for _ in range(self.rows)]
                                self.grid[self.start[0]][self.start[1]] = 0
                                self.grid[self.goal[0]][self.goal[1]] = 0
                                self.agent_pos = self.start
                                self.running_agent = False
                                self.recalculate_path()

                    x_offset = (self.window_width - self.grid_width) // 2
                    if x_offset <= mx <= x_offset + self.grid_width and my < self.grid_height:
                        c = (mx - x_offset) // CELL_SIZE
                        r = my // CELL_SIZE

                        if 0 <= r < self.rows and 0 <= c < self.cols:
                            if (r,c) not in {self.start, self.goal}:
                                self.grid[r][c] = 1 - self.grid[r][c]
                                self.recalculate_path()

            if self.running_agent:
                self.agent_step()

            self.draw()



if __name__=="__main__":

    print("="*60)
    print("PATHFINDING VISUALIZER")
    print("="*60)
    print("\nCONTROLS:")
    print("- Click grid cells to toggle walls")
    print("- Click buttons below grid")
    print("- SPACE: Start/Pause agent")
    print("- R: Reset agent to start")
    print("- A: Switch algorithm (A*/GBFS)")
    print("- H: Switch heuristic (Manhattan/Euclidean)")
    print("- C: Clear all walls")
    print("\nMETRICS DASHBOARD:")
    print("- Nodes Visited: Count of expanded nodes")
    print("- Path Cost: Length of final path")
    print("- Execution Time: Computation time in ms\n")

    while True:
        try:
            rows=int(input("Rows (5-40): "))
            cols=int(input("Cols (5-60): "))
            density=float(input("Density (0-0.9): "))

            if not (5<=rows<=40 and 5<=cols<=60 and 0<=density<=0.9):
                print("Invalid ranges! Use 5-40 rows, 5-60 cols, 0-0.9 density")
                continue

            print(f"\nCreating {rows}x{cols} grid with {density*100:.1f}% obstacles...")
            app=PathfindingApp(rows,cols)
            app.obstacle_density=density
            app.generate_random_map(density)
            app.run()

        except ValueError:
            print("Enter valid numeric values!")
        except KeyboardInterrupt:
            print("\nGoodbye!")
            sys.exit()