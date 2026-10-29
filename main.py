import pygame
import math
import random
import sys

# --- Configuration & Constants ---
WINDOW_WIDTH = 1280
WINDOW_HEIGHT = 720
PLAY_WIDTH = 900
PLAY_HEIGHT = 720
DASHBOARD_X = 900
FPS = 60

# Colors
BG_COLOR = (30, 30, 30)
WALL_COLOR = (100, 100, 100)
ROBOT_COLOR = (0, 200, 150)
OBSTACLE_COLOR = (150, 50, 50)
HUMAN_COLOR = (200, 150, 50)
GOAL_COLOR = (50, 200, 50)
TEXT_COLOR = (220, 220, 220)
RAY_COLOR = (255, 255, 0, 100)
FOV_COLOR = (0, 150, 255, 50)
PATH_COLOR = (0, 200, 150, 100)
BOUNDING_BOX_COLOR = (255, 100, 100)

class Wall:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
        
    def draw(self, surface):
        pygame.draw.rect(surface, WALL_COLOR, self.rect)

class Obstacle:
    def __init__(self, x, y, w, h):
        self.rect = pygame.Rect(x, y, w, h)
        
    def draw(self, surface):
        pygame.draw.rect(surface, OBSTACLE_COLOR, self.rect)

class Human:
    def __init__(self, x, y, w, h, speed, path_points):
        self.rect = pygame.Rect(x, y, w, h)
        self.x = float(x)
        self.y = float(y)
        self.speed = speed
        self.path_points = path_points
        self.target_idx = 0
        self.state = "WALKING"
        self.pause_time = 0

    def update(self, dt):
        if self.state == "PAUSED":
            self.pause_time -= dt
            if self.pause_time <= 0:
                self.state = "WALKING"
            return

        if len(self.path_points) == 0:
            return

        target = self.path_points[self.target_idx]
        dx = target[0] - self.x
        dy = target[1] - self.y
        dist = math.hypot(dx, dy)

        if dist < 5:
            self.target_idx = (self.target_idx + 1) % len(self.path_points)
            self.state = "PAUSED"
            self.pause_time = random.uniform(0.5, 1.5)
            return

        # Move
        move_dist = self.speed * dt
        self.x += (dx / dist) * move_dist
        self.y += (dy / dist) * move_dist
        self.rect.x = int(self.x)
        self.rect.y = int(self.y)

    def draw(self, surface):
        pygame.draw.rect(surface, HUMAN_COLOR, self.rect)
        pygame.draw.rect(surface, (255, 255, 255), self.rect, 2) # Adding a border to make it look nicer

def line_rect_intersection(p1, p2, rect):
    # Returns closest intersection point of line segment (p1,p2) and rect, or None
    lines = [
        ((rect.left, rect.top), (rect.right, rect.top)),
        ((rect.right, rect.top), (rect.right, rect.bottom)),
        ((rect.right, rect.bottom), (rect.left, rect.bottom)),
        ((rect.left, rect.bottom), (rect.left, rect.top))
    ]
    closest = None
    min_dist = float('inf')
    
    for l1, l2 in lines:
        pt = line_intersection(p1, p2, l1, l2)
        if pt:
            dist = math.hypot(pt[0]-p1[0], pt[1]-p1[1])
            if dist < min_dist:
                min_dist = dist
                closest = pt
    return closest

def line_intersection(p1, p2, p3, p4):
    x1, y1 = p1; x2, y2 = p2
    x3, y3 = p3; x4, y4 = p4
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if den == 0: return None
    t = ((x1-x3)*(y3-y4) - (y1-y3)*(x3-x4)) / den
    u = -((x1-x2)*(y1-y3) - (y1-y2)*(x1-x3)) / den
    if 0 <= t <= 1 and 0 <= u <= 1:
        return (x1 + t*(x2-x1), y1 + t*(y2-y1))
    return None

class Robot:
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
        self.radius = 15
        self.angle = 0.0 # radians
        
        self.base_speed = 80.0
        self.turn_speed = 2.0
        self.speed = 0.0
        
        self.state = "START"
        self.decision = "FORWARD"
        self.safety = "SAFE"
        self.path = []
        
        # Sensors
        self.sensor_angles = [-90, -45, -20, 0, 20, 45, 90]
        self.sensor_max_dist = 250
        self.sensor_distances = [250] * 7
        self.sensor_points = [None] * 7
        
        # Camera
        self.fov = 70
        self.camera_max_dist = 300
        self.ai_detections = []
        
        # Sensor Fusion State
        self.front_dist = 250
        self.left_clearance = 250
        self.right_clearance = 250
        self.nearest_human_dist = 999
        self.nearest_obstacle_dist = 999
        self.goal_dist = 999
        
        # Metrics
        self.time_elapsed = 0.0
        self.dist_travelled = 0.0
        self.obstacles_avoided = 0
        self.humans_avoided = 0
        self.e_stops = 0
        self.avoidance_cooldown = 0.0
        self.chosen_avoid_dir = 0
        
        self.goal_reached = False
        
    def raycast(self, env):
        # Update ultrasonics
        for i, a in enumerate(self.sensor_angles):
            rad = self.angle + math.radians(a)
            end_x = self.x + math.cos(rad) * self.sensor_max_dist
            end_y = self.y + math.sin(rad) * self.sensor_max_dist
            p1 = (self.x, self.y)
            p2 = (end_x, end_y)
            
            closest_pt = p2
            min_dist = self.sensor_max_dist
            
            rects = [w.rect for w in env.walls] + [o.rect for o in env.obstacles] + [h.rect for h in env.humans]
            for rect in rects:
                pt = line_rect_intersection(p1, p2, rect)
                if pt:
                    d = math.hypot(pt[0]-self.x, pt[1]-self.y)
                    if d < min_dist:
                        min_dist = d
                        closest_pt = pt
            self.sensor_distances[i] = min_dist
            self.sensor_points[i] = closest_pt
            
    def camera_perception(self, env):
        self.ai_detections = []
        self.nearest_human_dist = 999
        self.nearest_obstacle_dist = 999
        
        # Check humans
        for h in env.humans:
            dx = h.rect.centerx - self.x
            dy = h.rect.centery - self.y
            dist = math.hypot(dx, dy)
            if dist < self.camera_max_dist:
                angle_to = math.atan2(dy, dx)
                diff = (angle_to - self.angle + math.pi) % (2*math.pi) - math.pi
                if abs(diff) <= math.radians(self.fov/2):
                    self.ai_detections.append({'type': 'HUMAN', 'dist': dist, 'rect': h.rect, 'conf': random.randint(90, 98)})
                    if dist < self.nearest_human_dist:
                        self.nearest_human_dist = dist
                        
        # Check obstacles
        for o in env.obstacles:
            dx = o.rect.centerx - self.x
            dy = o.rect.centery - self.y
            dist = math.hypot(dx, dy)
            if dist < self.camera_max_dist:
                angle_to = math.atan2(dy, dx)
                diff = (angle_to - self.angle + math.pi) % (2*math.pi) - math.pi
                if abs(diff) <= math.radians(self.fov/2):
                    self.ai_detections.append({'type': 'STATIC OBSTACLE', 'dist': dist, 'rect': o.rect, 'conf': random.randint(95, 99)})
                    if dist < self.nearest_obstacle_dist:
                        self.nearest_obstacle_dist = dist

    def fuse_sensors(self, env):
        self.raycast(env)
        self.camera_perception(env)
        
        self.front_dist = min(self.sensor_distances[2:5]) # -20, 0, 20
        self.left_clearance = min(self.sensor_distances[0:3]) # -90 to -20
        self.right_clearance = min(self.sensor_distances[4:7]) # 20 to 90
        
        dx = env.goal[0] - self.x
        dy = env.goal[1] - self.y
        self.goal_dist = math.hypot(dx, dy)

    def decide(self, env, dt):
        if self.goal_dist < 20:
            self.state = "GOAL_REACHED"
            self.decision = "STOP"
            self.speed = 0
            self.goal_reached = True
            return
            
        self.safety = "SAFE"
        self.state = "FORWARD"
        self.decision = "FORWARD"
        self.speed = self.base_speed
        
        # 1. Physical Crash Recovery (prevent getting completely stuck)
        if self.front_dist < 18:
            self.safety = "EMERGENCY STOP"
            self.state = "REVERSE"
            self.decision = "REVERSE"
            self.speed = -self.base_speed * 0.5
            if self.left_clearance > self.right_clearance:
                self.angle -= self.turn_speed * dt
            else:
                self.angle += self.turn_speed * dt
            return

        # Base target is the goal
        target_angle = math.atan2(env.goal[1] - self.y, env.goal[0] - self.x)
        
        # 2. Very Close Front Obstacle -> max 45 degree steer
        if self.front_dist < 45:
            self.state = "AVOID"
            self.decision = "TURN 45"
            # Steer 45 degrees away from current heading
            if self.left_clearance > self.right_clearance:
                target_angle = self.angle - math.radians(45)
            else:
                target_angle = self.angle + math.radians(45)
                
        # 3. Very Close Side Obstacles -> 20 degree steer
        elif self.left_clearance < 25:
            self.state = "AVOID"
            self.decision = "TURN 20 (L)"
            target_angle = self.angle + math.radians(20)
        elif self.right_clearance < 25:
            self.state = "AVOID"
            self.decision = "TURN 20 (R)"
            target_angle = self.angle - math.radians(20)

        # Smooth steering towards target_angle
        diff = (target_angle - self.angle + math.pi) % (2*math.pi) - math.pi
        if abs(diff) > 0.05:
            if diff > 0:
                self.angle += min(self.turn_speed * dt, diff)
            else:
                self.angle -= min(self.turn_speed * dt, -diff)

    def move(self, dt, env):
        if self.speed != 0:
            nx = self.x + math.cos(self.angle) * self.speed * dt
            ny = self.y + math.sin(self.angle) * self.speed * dt
            
            # Simple collision prevention
            robot_rect = pygame.Rect(nx - self.radius, ny - self.radius, self.radius*2, self.radius*2)
            collision = False
            for w in env.walls:
                if robot_rect.colliderect(w.rect): collision = True
            for o in env.obstacles:
                if robot_rect.colliderect(o.rect): collision = True
                
            if not collision:
                self.dist_travelled += math.hypot(nx - self.x, ny - self.y)
                self.x = nx
                self.y = ny
                if len(self.path) == 0 or math.hypot(self.x - self.path[-1][0], self.y - self.path[-1][1]) > 5:
                    self.path.append((self.x, self.y))

    def update(self, env, dt):
        self.time_elapsed += dt
        self.fuse_sensors(env)
        self.decide(env, dt)
        self.move(dt, env)

    def draw(self, surface, flags):
        # Draw path
        if len(self.path) > 1:
            pygame.draw.lines(surface, PATH_COLOR, False, [(int(p[0]), int(p[1])) for p in self.path], 2)
            
        # Draw Sensor Rays
        if flags['sensors']:
            for pt in self.sensor_points:
                if pt:
                    pygame.draw.line(surface, RAY_COLOR, (self.x, self.y), pt, 1)
                    pygame.draw.circle(surface, (255,0,0), (int(pt[0]), int(pt[1])), 3)

        # Draw Camera FOV
        if flags['camera']:
            p2_x = self.x + math.cos(self.angle - math.radians(self.fov/2)) * self.camera_max_dist
            p2_y = self.y + math.sin(self.angle - math.radians(self.fov/2)) * self.camera_max_dist
            p3_x = self.x + math.cos(self.angle + math.radians(self.fov/2)) * self.camera_max_dist
            p3_y = self.y + math.sin(self.angle + math.radians(self.fov/2)) * self.camera_max_dist
            pygame.draw.polygon(surface, FOV_COLOR, [(self.x, self.y), (p2_x, p2_y), (p3_x, p3_y)])

        # Draw AI Boxes
        if flags['ai'] and len(self.ai_detections) > 0:
            font = pygame.font.SysFont("Arial", 12)
            for d in self.ai_detections:
                pygame.draw.rect(surface, BOUNDING_BOX_COLOR, d['rect'], 2)
                txt = font.render(f"{d['type']} {d['conf']}%", True, BOUNDING_BOX_COLOR)
                surface.blit(txt, (d['rect'].x, d['rect'].y - 15))

        # Draw Robot
        pygame.draw.circle(surface, ROBOT_COLOR, (int(self.x), int(self.y)), self.radius)
        end_x = self.x + math.cos(self.angle) * self.radius * 1.5
        end_y = self.y + math.sin(self.angle) * self.radius * 1.5
        pygame.draw.line(surface, (255,255,255), (self.x, self.y), (end_x, end_y), 3)

class Environment:
    def __init__(self):
        self.walls = [
            Wall(0, 0, PLAY_WIDTH, 20),
            Wall(0, 0, 20, PLAY_HEIGHT),
            Wall(0, PLAY_HEIGHT-20, PLAY_WIDTH, 20),
            Wall(PLAY_WIDTH-20, 0, 20, PLAY_HEIGHT)
        ]
        
        # Randomize obstacles to make every run unique but less cluttered
        self.obstacles = []
        for _ in range(3):
            ox = random.randint(200, PLAY_WIDTH - 200)
            oy = random.randint(150, PLAY_HEIGHT - 150)
            ow = random.randint(40, 80)
            oh = random.randint(40, 80)
            self.obstacles.append(Obstacle(ox, oy, ow, oh))
        
        # Randomize human patrols
        self.humans = []
        for _ in range(2):
            hx = random.randint(100, PLAY_WIDTH - 100)
            hy = random.randint(100, PLAY_HEIGHT - 100)
            paths = [(random.randint(100, PLAY_WIDTH-100), random.randint(100, PLAY_HEIGHT-100)) for _ in range(4)]
            self.humans.append(Human(hx, hy, 24, 24, random.randint(40, 70), paths))
        
        self.goal = (800, 100)
        
    def draw(self, surface):
        surface.fill(BG_COLOR, (0, 0, PLAY_WIDTH, PLAY_HEIGHT))
        for w in self.walls: w.draw(surface)
        for o in self.obstacles: o.draw(surface)
        for h in self.humans: h.draw(surface)
        
        # Draw Goal
        pygame.draw.circle(surface, GOAL_COLOR, self.goal, 20)
        pygame.draw.circle(surface, (255,255,255), self.goal, 20, 2)

class Simulation:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        pygame.display.set_caption("SmartNav AI Obstacle Avoidance")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 16)
        self.title_font = pygame.font.SysFont("Arial", 20, bold=True)
        
        self.flags = {
            'dashboard': True,
            'sensors': True,
            'camera': True,
            'ai': True
        }
        self.paused = False
        self.reset()
        
    def reset(self):
        self.env = Environment()
        self.robot = Robot(100, 600)
        
    def draw_dashboard(self):
        dash_rect = pygame.Rect(PLAY_WIDTH, 0, WINDOW_WIDTH-PLAY_WIDTH, WINDOW_HEIGHT)
        pygame.draw.rect(self.screen, (40, 40, 40), dash_rect)
        pygame.draw.line(self.screen, (100, 100, 100), (PLAY_WIDTH, 0), (PLAY_WIDTH, WINDOW_HEIGHT), 2)
        
        y = 20
        def t(text, color=TEXT_COLOR):
            nonlocal y
            surf = self.font.render(text, True, color)
            self.screen.blit(surf, (PLAY_WIDTH + 20, y))
            y += 25
            
        def title(text):
            nonlocal y
            y += 10
            surf = self.title_font.render(text, True, (0, 200, 255))
            self.screen.blit(surf, (PLAY_WIDTH + 20, y))
            y += 35
            
        title("SMARTNAV DASHBOARD")
        t(f"ROBOT STATE: {self.robot.state}")
        
        ai_det = "NONE"
        conf = "0%"
        if len(self.robot.ai_detections) > 0:
            d = self.robot.ai_detections[0]
            ai_det = d['type']
            conf = f"{d['conf']}%"
            
        t(f"AI DETECTION: {ai_det}")
        t(f"CONFIDENCE: {conf}")
        t("")
        t(f"FRONT DISTANCE: {int(self.robot.front_dist)} px")
        t(f"LEFT CLEARANCE: {int(self.robot.left_clearance)} px")
        t(f"RIGHT CLEARANCE: {int(self.robot.right_clearance)} px")
        t(f"NEAREST HUMAN: {int(self.robot.nearest_human_dist)} px")
        t(f"NEAREST OBSTACLE: {int(self.robot.nearest_obstacle_dist)} px")
        t(f"GOAL DISTANCE: {int(self.robot.goal_dist)} px")
        t("")
        
        color_dec = (0, 255, 0) if self.robot.decision == "FORWARD" else (255, 200, 0)
        if "STOP" in self.robot.decision: color_dec = (255, 0, 0)
        t(f"DECISION: {self.robot.decision}", color_dec)
        
        color_safe = (0, 255, 0) if self.robot.safety == "SAFE" else (255, 0, 0)
        t(f"SAFETY: {self.robot.safety}", color_safe)
        
        t("")
        title("SYSTEM STATUS")
        t("CAMERA: ACTIVE")
        t("ULTRASONIC: ACTIVE")
        t("SENSOR FUSION: ACTIVE")
        t("AI PERCEPTION: SIMULATED", (255, 150, 0))
        t("PROCESSOR: Raspberry Pi (SIMULATED)")
        
        t("")
        title("METRICS")
        t(f"Sim Time: {self.robot.time_elapsed:.1f} s")
        t(f"Distance: {self.robot.dist_travelled:.0f} px")
        t(f"Obstacles Avoided: {self.robot.obstacles_avoided}")
        t(f"Humans Avoided: {self.robot.humans_avoided}")
        t(f"Emergency Stops: {self.robot.e_stops}")
        
        if self.robot.goal_reached:
            y += 20
            surf = self.title_font.render("DESTINATION REACHED", True, (50, 255, 50))
            self.screen.blit(surf, (PLAY_WIDTH + 20, y))

    def run(self):
        running = True
        while running:
            dt = self.clock.tick(FPS) / 1000.0
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        self.paused = not self.paused
                    elif event.key == pygame.K_r:
                        self.reset()
                    elif event.key == pygame.K_d:
                        self.flags['dashboard'] = not self.flags['dashboard']
                    elif event.key == pygame.K_s:
                        self.flags['sensors'] = not self.flags['sensors']
                    elif event.key == pygame.K_c:
                        self.flags['camera'] = not self.flags['camera']
                    elif event.key == pygame.K_a:
                        self.flags['ai'] = not self.flags['ai']
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.pos[0] < PLAY_WIDTH:
                        self.env.goal = event.pos
                        self.robot.goal_reached = False
                        self.robot.state = "START"
                        self.robot.decision = "FORWARD"
                        
            if not self.paused:
                for h in self.env.humans:
                    h.update(dt)
                if not self.robot.goal_reached:
                    self.robot.update(self.env, dt)
                    
            self.screen.fill((0,0,0))
            self.env.draw(self.screen)
            self.robot.draw(self.screen, self.flags)
            
            if self.flags['dashboard']:
                self.draw_dashboard()
                
            pygame.display.flip()
            
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    sim = Simulation()
    sim.run()
