from math import pi, sin, cos, atan2, atan, sqrt
import pygame
from pygame.locals import *

BLACK = [0, 0, 0]
WHITE = [255,255,255]
BLUE = [ 0, 0,255]
GREEN = [ 0,255, 0]
RED = [255, 0, 0]

SCREENSIZE = [800,600]

class Car():
  """ Car class, to simulate a simple bicycle model of a car """
  def __init__(self):
    # Initialize car attributes
    self.m = 3600.0 / 2.2 # Kg
    self.L = 7.0 / 3.28 # m
    self.b = self.L / (1 + (0.85)) 
    self.a = self.L - self.b
    self.I = 0.5 * self.m * self.a * self.b
    self.C_f = 210.0 * (4.44822) * (180/pi) #Newtons per radian
    self.K_u = .1 # Oversteer coefficient
    self.C_r = self.K_u * (self.a/self.b) * self.C_f
    self.timestep = 0.005
    self.steer_angle = 10 # degrees
    self.linearTireModel = False

    # Calculate critical speed
    U_crit_top = ((self.a + self.b)**2) * self.C_f * self.C_r
    U_crit_bottom = self.m*(self.a*self.C_f - self.b * self.C_r)
    if U_crit_bottom > 0:
      self.U_crit = sqrt(U_crit_top / U_crit_bottom)
    else:
      self.U_crit = 0

    # State variables
    self.theta = 0
    self.x = 100
    self.y = -300
    self.U = 0
    self.V = 0
    self.r = 0
    self.delta = 0
    self.F_t = 0 # Thrust force

    self.__pygame_init()

  def __pygame_init(self):
    self.screensize = SCREENSIZE
    self.screen = pygame.display.set_mode(self.screensize)
    self.done = False
    self.clock = pygame.time.Clock()

  def step(self):
    """ Update the state variables by one timestep"""
    if self.U >= 0:
      alpha_f = self.delta - atan2((self.V + self.r*self.a), self.U)
    else:
      alpha_f = self.delta - atan2((-self.V - self.r*self.a), -self.U)
      alpha_f = -alpha_f
    alpha_r = -atan2((self.V - self.r*self.b), self.U)

    C_f_linearLimit = 15 * pi/180

    if self.linearTireModel:
      F_f = self.C_f * alpha_f
      F_r = self.C_r * alpha_r
    else:
      if alpha_f > C_f_linearLimit:
        F_f = self.C_f * C_f_linearLimit
      elif alpha_f < -C_f_linearLimit:
        F_f = self.C_f * -C_f_linearLimit
      else:
        F_f = self.C_f * alpha_f

      if alpha_r > C_f_linearLimit:
        F_r = self.C_r * C_f_linearLimit
      elif alpha_r < -C_f_linearLimit:
        F_r = self.C_r * -C_f_linearLimit
      else:
        F_r = self.C_r * alpha_r

    r_dot_top = self.a * F_f * cos(self.delta) - self.b * F_r
    r_dot = r_dot_top / self.I

    u_dot = (-F_f * sin(self.delta) + self.F_t)/self.m + self.r*self.V

    v_dot = (F_f * cos(self.delta) + F_r)/self.m - self.r*self.U
    self.V_dot = v_dot

    # Update state variables
    self.r = self.r + r_dot*self.timestep
    self.theta = self.theta + self.r*self.timestep

    self.U = self.U + u_dot * self.timestep
    self.V = self.V + v_dot * self.timestep

    self.x = self.x + (self.U*cos(self.theta) - self.V*sin(self.theta)) * self.timestep
    self.y = self.y + (self.U*sin(self.theta) + self.V*cos(self.theta)) * self.timestep

  def draw(self):
    scale_factor = 3
    """ Draw the pygame graphics """
    if self.x > 800/ scale_factor:
      self.x = 0
    if self.x < 0:
      self.x = 800 / scale_factor
    if self.y < -600/ scale_factor:
      self.y = 0
    if self.y > 0:
      self.y = -600 / scale_factor

    car_back_x = -2*scale_factor * cos(self.theta) + (self.x*scale_factor)
    car_back_y = 0 + 2*scale_factor*sin(self.theta) - (self.y*scale_factor)

    car_front_x = 2 *scale_factor* cos(self.theta) + (self.x*scale_factor)
    car_front_y = 0 + -2*scale_factor*sin(self.theta) - (self.y*scale_factor)

    # draw line of car
    pygame.draw.line(self.screen, GREEN, [car_back_x, car_back_y], [car_front_x, car_front_y], 2)

  def drawInstrumentation(self):
    """ This function draws the speedometer and accelerometer bars on the
    pygame window."""
    # Draw speedometer
    for i in range (0, 100, 10):
      pygame.draw.line(self.screen, RED, [650, 400-(i*5)], [750, 400-(i*5)], 1)

    pygame.draw.line(self.screen, RED, [700, 400], [700, 400 - 5*(self.U*2.238)], 3)
    pygame.draw.line(self.screen, BLUE, [710, 400], [710, 400-(9.8*self.V_dot)], 1)

    # Draw U_crit line
    pygame.draw.line(self.screen, WHITE, 
        [650, 400 - 5*(self.U_crit*2.238)], 
        [750, 400 - 5*(self.U_crit*2.238)], 3)

  def run(self):
    """ Run the simulation """
    self.done = False
    index = 0
    #fontMgr = cFontManager.cFontManager(((None, 24), (None, 48), ('arial', 24)))
    while self.done==False:
      self.clock.tick(40)

      # Handle keyboard events to control the car
      for event in pygame.event.get():
        if event.type == pygame.QUIT:
          self.done = True
        if event.type == pygame.KEYDOWN:
          if event.key == pygame.K_UP:
            self.F_t = 10000
          if event.key == pygame.K_LEFT:
            self.delta = self.steer_angle*pi/180
          if event.key == pygame.K_RIGHT:
            self.delta = -self.steer_angle*pi/180
          if event.key == pygame.K_DOWN:
            self.F_t = -10000
        if event.type == pygame.KEYUP:
          if event.key == pygame.K_UP:
            self.F_t = 0;
          if event.key == pygame.K_DOWN:
            self.F_t = 0;
          if event.key == pygame.K_LEFT:
            self.delta = 0
          if event.key == pygame.K_RIGHT:
            self.delta = 0
    
      # Clear the screen, set background
      self.screen.fill(BLACK)

      for i in range (0, 10):
        self.step()
      
      self.draw()
      self.drawInstrumentation()
      #fontMgr.Draw(self.screen, None, 24, self.theta, (0, 0), RED)

      pygame.display.flip()

      index = index+1

    pygame.quit()

if __name__ == "__main__":
  car = Car()
  car.run()
