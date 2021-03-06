import Tkinter
from Tkinter import *
import ttk
from control import *



world_size = 100.0
landmarks  = [[0.0, 100.0], [0.0, 0.0], [100.0, 0.0], [100.0, 100.0]] # position of 4 landmarks in (y, x) format.


class displayParticleFilter(Tk):
    ''' Used to demonstrate a particle filter and robot over a predetermined number of steps '''

    def __init__(self,motions,measurements, N=500):
        Tk.__init__(self)
        self.title('Display Particle Filter')
        self.motions = motions
        self.measurements = measurements
        self.N = N
        #initiate the particle filter
        self.initFilter()
        #Drawing
        self.margin = 100
        self.zoom_factor = 2
        self.can = DisplayParticles( self.margin, self.zoom_factor)
        self.can.configure(bg = 'ivory', bd = 2, relief = SUNKEN)
        self.can.pack(side = TOP, padx = 5, pady = 5)
        self.can.draw_all(self.p)
        #Buttons
        self.button1 = Button(self, text = 'Reset', command = self.resetFilter)
        self.button1.pack(side = LEFT, padx = 5, pady=5)
        self.button2 = Button(self, text = 'Next Step', command = self.nextStep)
        self.button2.pack(side = LEFT, padx = 5, pady = 5)
        #Label
        textLabel = 'Current state = ' + str(self.actualState) + '/' + str(len(motions)-1)
        self.label = Label(self, text = textLabel)
        self.label.pack(side =BOTTOM, padx = 5, pady = 5)

    def resetFilter(self):
        self.initFilter()
        #Replot All
        self.robot = get_position(self.p)
        self.can.draw_all(self.p)

    def initFilter (self):
        self.p = []
        for i in range(self.N):
            r = robot.Robot()
            r.set_noise(bearing_noise, steering_noise, distance_noise)
            self.p.append(r)
        self.actualState = 0

    def nextStep (self, event = None):
        self.actualState = self.actualState + 1
        if self.actualState < len(self.motions):
            #Label
            stateString = 'Actual State = ' + str(self.actualState) + '/' +str(len(motions)-1)
            self.label.configure( text = stateString )
            # motion update (prediction)
            p2 = []
            for i in range(self.N):
                p2.append(self.p[i].move(self.motions[self.actualState]))
            self.p = p2
            # measurement update
            w = []
            for i in range(self.N):
                w.append(self.p[i].measurement_prob(self.measurements[self.actualState]))
            # resampling
            p3 = []
            index = int(random.random() * self.N)
            beta = 0.0
            mw = max(w)
            for i in range(self.N):
                beta += random.random() * 2.0 * mw
                while beta > w[index]:
                    beta -= w[index]
                    index = (index + 1) % self.N
                p3.append(self.p[index])
            self.p = p3
            #Replot all
            self.robot = get_position(self.p)
            self.can.draw_all(self.p)

class DisplayParticles(Canvas):
    def __init__(self, margin, zoom_factor):
        Canvas.__init__(self)
        self.margin = margin
        self.zoom_factor = zoom_factor
        self.larg = (2*margin + world_size) * zoom_factor
        self.haut = self.larg
        self.configure(width=self.larg, height=self.haut)
        self.larg, self.haut = (2*margin + world_size) * zoom_factor, (2*margin + world_size)
        #landmarks
        self.landmarks_radius = 2
        self.landmarks_color = 'green'
        #Particles
        self.particle_radius = 1
        self.particle_color = 'red'
        #Robot
        self.robot_radius = 4
        self.robot_color = 'blue'

    def draw_all(self, p):
        #print len(p)
        self.configure(bg ='ivory', bd =2, relief=SUNKEN)
        self.delete(ALL)
        self.p = p
        self.plot_particles()
        self.plot_landmarks( landmarks, self.landmarks_radius, self.landmarks_color )
        self.robot = get_position(self.p)
        self.plot_robot( self.robot_radius, self.robot_color)

    def plot_landmarks(self, lms, radius, l_color ):
        for lm in lms:
            x0 = (self.margin + lm[1] - radius) * self.zoom_factor
            y0 = (self.margin + lm[0] - radius) * self.zoom_factor
            x1 = (self.margin + lm[1] + radius) * self.zoom_factor
            y1 = (self.margin + lm[0] + radius) * self.zoom_factor
            self.create_oval( x0, y0, x1, y1, fill = l_color )

    def plot_particles(self):
        for particle in self.p:
            self.draw_particle( particle, self.particle_radius, self.particle_color )

    def draw_particle(self, particle, radius, p_color):
        x2 = (self.margin + particle.x) * self.zoom_factor
        y2 = (self.margin + particle.y) * self.zoom_factor
        x3 = (self.margin + particle.x + 2*radius*cos(particle.orientation)) * self.zoom_factor
        y3 = (self.margin + particle.y + 2*radius*sin(particle.orientation)) * self.zoom_factor
        self.create_line( x2, y2, x3, y3, fill = p_color, width =self.zoom_factor,
                          arrow=LAST, arrowshape=(2*self.zoom_factor,
                                                  3*self.zoom_factor,
                                                  1*self.zoom_factor) )

    def plot_robot(self, radius, r_color):
        x0 = (self.margin + self.robot[0] - radius) * self.zoom_factor
        y0 = (self.margin + self.robot[1] - radius) * self.zoom_factor
        x1 = (self.margin + self.robot[0] + radius) * self.zoom_factor
        y1 = (self.margin + self.robot[1] + radius) * self.zoom_factor
        self.create_oval( x0, y0, x1, y1, fill = r_color )
        x2 = (self.margin + self.robot[0]) * self.zoom_factor
        y2 = (self.margin + self.robot[1]) * self.zoom_factor
        x3 = (self.margin + self.robot[0] + 2*radius*cos(self.robot[2])) * self.zoom_factor
        y3 = (self.margin + self.robot[1] + 2*radius*sin(self.robot[2])) * self.zoom_factor
        self.create_line( x2, y2, x3, y3, fill = r_color, width =self.zoom_factor, arrow=LAST )




if __name__ == "__main__":
    #motions and measurements
    number_of_iterations = 20
    motions = [[2. * pi / 20, 12.] for row in range(number_of_iterations)]
    x = generate_ground_truth(motions)
    final_robot = x[0]
    measurements = x[1]
    #Display window
    wind = displayParticleFilter( motions, measurements, 500 )
    wind.mainloop()









