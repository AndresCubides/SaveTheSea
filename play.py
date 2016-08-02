from __future__ import division
from scipy import stats, constants
from scipy.ndimage import measurements, morphology, filters, interpolation
from skimage.measure import regionprops
from skimage.filters import rank
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as pl
from collections import Counter
from pykinect import nui
from ePuck import ePuck
import statsmodels.formula.api as smf
import skimage.morphology as morp
import statsmodels.api as sm
import pandas as pd
import numpy as np
import time as ti
import sys
import ctypes
import thread
import pygame
import random
import cython

#Save the sea by Andres Cubides
#In case of any question write to andrescamiloc@hotmail.com

#---------------------------------------Global Variables-----------------------------------

#E-pucks dictionary to asociate an ePuck ID with its MAC Address
#If you want to connect other e-pucks add the number and the MAC addres from the devicee information

epucks = {
    '2486' : '10:00:E8:AD:75:D6',
    '3067' : '10:00:E8:C5:61:18',
    '3286' : '10:00:E8:C5:64:42',
    '3204' : '10:00:E8:C5:61:5B',
    '2720' : '10:00:E8:AD:69:B6',
    '2454' : '10:00:E8:AD:69:B5',
    '3047' : '10:00:E8:C5:6B:02',
    '3078' : '10:00:E8:C5:61:6C'
}

#Colors RGB
BLACK  = (  0,   0,   0)
WHITE = (255, 255, 255)
GREEN = (0, 247, 0)
RED = (254, 0, 0)
BLUE = (1, 175, 247)

#Fixed size of surface for kinect depth data
DEPTH_WINSIZE = 320,240
tmp_s = pygame.Surface(DEPTH_WINSIZE, 0, 16)

#Game screen size
SCREEN_WIDTH = 1280
SCREEN_HEIGHT = 697

tableWidth = 800

projectionLength = 1180

screen_lock = thread.allocate()
screen = None
#----------------------------------------------------------------------------------------

#-----------------------------Needed to make kinect depth work---------------------------
#Source: https://github.com/Microsoft/PTVS/blob/master/Python/Product/PyKinect/PyKinect/PyGameDemo.py

if hasattr(ctypes.pythonapi, 'Py_InitModule4'):
   Py_ssize_t = ctypes.c_int
elif hasattr(ctypes.pythonapi, 'Py_InitModule4_64'):
   Py_ssize_t = ctypes.c_int64
else:
   raise TypeError("Cannot determine type of Py_ssize_t")

_PyObject_AsWriteBuffer = ctypes.pythonapi.PyObject_AsWriteBuffer
_PyObject_AsWriteBuffer.restype = ctypes.c_int
_PyObject_AsWriteBuffer.argtypes = [ctypes.py_object,
                                  ctypes.POINTER(ctypes.c_void_p),
                                  ctypes.POINTER(Py_ssize_t)]

def surface_to_array(surface):
   buffer_interface = surface.get_buffer()
   address = ctypes.c_void_p()
   size = Py_ssize_t()
   _PyObject_AsWriteBuffer(buffer_interface,
                          ctypes.byref(address), ctypes.byref(size))
   bytes = (ctypes.c_byte * size.value).from_address(address.value)
   bytes.object = buffer_interface
   return bytes
#----------------------------------------------------------------------------------------

#--------------------------------------Sprites Classes-----------------------------------

#Type of sprite to display images of titles (Score: and Contamination:)
class Title(pygame.sprite.Sprite):
    global titles #Sprite group
    
    def __init__(self, image, pos):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 4 #Define to be blitted in the layered sprite group all_sprites
        self.image = image
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y

        #Define screen position
        if pos == 1:
            #Score: 
            self.rect.x  = SCREEN_WIDTH/4 - self.rect.width/2
            self.rect.y = pause.rect.height/2 - self.rect.height/2 #Pause button position is used to align the titles with it
        elif pos == 2:
            #Contamination
            self.rect.x  = SCREEN_WIDTH*3/4 - self.rect.width/2
            self.rect.y = pause.rect.height/2 - self.rect.height/2

        titles.add(self) #Adds sprites to the Titles group


#Type of sprite to display the points of Score and Contamination
class Text(pygame.sprite.Sprite):
    global texts #Sprite group
    
    def __init__(self, x, y, color, pos):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 4 #Define to be blitted in the layered sprite group all_sprites
        self.font = pygame.font.Font(None, 60)
        self.image = self.font.render(str(0)+'   ', 1, color) #The spaces are in case the number increments to 3 digits they dont get out of the screen
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y
        self.rect.x  = x
        self.rect.y = y - self.rect.height/2

        texts.add(self) #Adds sprites to the Texts group

    #Updates the points achieve recieving the value to show and the color
    def update_counter(self, counter, color):
        self.image = self.font.render(str(counter), 1, color)


#Type of sprites to draw invisible rectangles in each side of the screen to avoid trash being blitted in the borders
class Wall(pygame.sprite.Sprite):
    global walls #Sprite group
    global all_sprites #Layered sprite group containig active sprites
    
    def __init__(self,x,y,width,height):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 0 #Define to be blitted in the layered sprite group all_sprites
        self.image = pygame.Surface((width,height))
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE) #Sets the color to be transparent
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y
        self.rect.x = x
        self.rect.y = y

        walls.add(self) #Adds sprites to the Walls group
        all_sprites.add(self, layer = self.layer) #Adds sprites to the layered group of active sprites


#Sprite to control the contamination image that is changing its transparency
class ContaminationScreen(pygame.sprite.Sprite):
    global all_sprites #Layered sprite group containig active sprites
    
    def __init__(self, image, contamination):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 3 #Define to be blitted in the layered sprite group all_sprites
        self.image = image
        self.image.set_alpha(contamination*5) #Set transparency proportional to the contamination points
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y
        self.rect.x = 0
        self.rect.y = 0

        all_sprites.add(self, layer = self.layer) #Adds sprites to the layered group of active sprites


#Type of sprites to display the ripples in the e-pucks path
class Ripple(pygame.sprite.Sprite):
    global all_sprites #Layered sprite group containig active sprites
    global ripples #Sprite group
    
    def __init__(self, x, y):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 1 #Define to be blitted in the layered sprite group all_sprites
        self.counter = 0 #Number of the current ripple image to blit
        self.repeat = False #Indicates if is the first time to show the ripple (False) or second time (True)
        self.image = pygame.Surface([160,160]) #Size of the ripple images
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y

        #Recieves X,Y coordinates of the robot so its relocated to the Screen coordinates
        self.rect.x = (x*relocate)-(160/2) #160/2 is needed because the robots coordinate is centered and the X,Y from the rect is a corner
        self.rect.y = (y*relocate)-(160/2)

        ripples.add(self)

    #Updates the image of the new ripple size
    def update_ripple(self):
        if self.counter < len(ripples_images):
            #Allows to keep the ripple image 2 loops so it doesn't move too fast
            if not self.repeat:
                self.image = ripples_images[self.counter]
                self.counter += 1 #Updates the ripple image after 2 loops
            all_sprites.add(self, layer = self.layer) #Adds ripple to the layered group of active sprites because each loop they are all removed to delete previous ones
        else:
            #When the sequence is over it deletes the ripple
            ripples.remove(self)
            all_sprites.remove(self)


#Type of sprites to control the images of the trash
class Trash(pygame.sprite.Sprite):
    global inactive_trash #Sprite group with all the trash that is not in the screen
    
    def __init__(self, image, trash_counter):
        pygame.sprite.Sprite.__init__(self)

        #These numbers depend on the number of images loaded per type of garbages, now ther are 6 per type
        if trash_counter <= 5:
            #Paper
            self.trash_type = 0
        elif trash_counter <= 11:
            #Aluminium
            self.trash_type = 1
        elif trash_counter <= 17:
            #Plastic
            self.trash_type = 2
        elif trash_counter <= 23:
            #Glass
            self.trash_type = 3

        self.layer = 2 #Define to be blitted in the layered sprite group all_sprites
        self.array_position = None #Define the position in the objectives array if the trash is active
        self.cleaner_id = None #Indicates which robot has been asigned to clean it, goes from 0 to 3 to index the robots, will be asigned if the robot has the same type of trash
        self.image = image
        self.time = None #Stores the time since it appears or since it decomposed to know if it have to be decomposed again
        self.decomposing = 0 #Number of times the trash has contaminated the screen, maximum set to 4 and then it disappears
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y

        inactive_trash.add(self) # Adds the trash to the inactive list


#Type of invisible sprite that indicates in the screen the actual position of the robot and its base
class ScreenRobot(pygame.sprite.Sprite):
    global screen_robots #Sprite group
    global all_sprites #Layered sprite group containig active sprites
    
    def __init__(self, x, y, sprite_id):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 0 #Define to be blitted in the layered sprite group all_sprites
        self.sprite_id = sprite_id #Indicates if it is a robot actual position or the base position
        if sprite_id < 4:
            self.size = 100 #Size of the robot sprite used for collitions
        else:
            self.size = 180 #Size of the robot base so trashes are no located too near
        self.image = pygame.Surface([self.size,self.size])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE) #Sets the color to be transparent
        self.rect = self.image.get_rect()#Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y
        #Half of the diameter is substracted to locate the corner of the bounding box       
        self.rect.x = (x*relocate)-(self.size/2)
        self.rect.y = (y*relocate)-(self.size/2)

        if sprite_id < 4:
            screen_robots.add(self) #Only actual robots position is used to locate collitions not the base sprite
        all_sprites.add(self, layer = self.layer) #Adds sprites to the layered group of active sprites


# Sprite to control the red pointer that selects
class Selection(pygame.sprite.Sprite):
    global all_sprites #Layered sprite group containig active sprites

    def __init__(self, radius, x, y, color):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 5 #Define to be blitted in the layered sprite group all_sprites
        self.image = pygame.Surface([2*radius, 2*radius])
        self.image.fill(WHITE)
        self.image.set_colorkey(WHITE) #Sets the color to be transparent
        pygame.draw.circle(self.image, color, (radius,radius), radius, 0) #Draws the circle and the coordinates are radius because its origin is referenced to the new surface created
        self.rect = self.image.get_rect() #Rectangle with the dimensions of the image and lets update position in the screen with rect.x and rect.y
        self.rect.x = x
        self.rect.y = y

        all_sprites.add(self, layer = self.layer)#Adds sprites to the layered group of active sprites


#Type of sprite to display the buttons
class Button(pygame.sprite.Sprite):
    
    def __init__(self, image, pos):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 4 #Define to be blitted in the layered sprite group all_sprites
        self.image = image
        self.position = pos #Defines button position in the screen
        self.rect = self.image.get_rect()
    
        if pos == 1:
            #Play
            self.rect.x  = SCREEN_WIDTH/3 - self.rect.width/2
            self.rect.y = SCREEN_HEIGHT*2/4 - self.rect.height/2
        elif pos == 2:
            #Quit
            self.rect.x  = SCREEN_WIDTH*2/3 - self.rect.width/2
            self.rect.y = SCREEN_HEIGHT*2/4 - self.rect.height/2 
        elif pos == 3:
            #Pause
            self.rect.x  = SCREEN_WIDTH*2/4 - self.rect.width/2
            self.rect.y = 0
        elif pos == 4:
            #Resume
            self.rect.x  = SCREEN_WIDTH/4 - self.rect.width/2
            self.rect.y = SCREEN_HEIGHT*2/4 - self.rect.height/2 
        elif pos == 5:
            #Exit
            self.rect.x  = SCREEN_WIDTH*3/4 - self.rect.width/2
            self.rect.y = SCREEN_HEIGHT*2/4 - self.rect.height/2 
        elif pos == 6:
            #Restart
            self.rect.x  = SCREEN_WIDTH*2/4 - self.rect.width/2
            self.rect.y = SCREEN_HEIGHT*2/4 - self.rect.height/2


# Sprite to display the image in the base of each e-puck to indicate wich trash he can pick
class Type(pygame.sprite.Sprite):
    global types #Sprite group of types of trash
    
    def __init__(self, image, pos):
        pygame.sprite.Sprite.__init__(self)
        self.layer = 1 #Define to be blitted in the layered sprite group all_sprites
        self.image = image
        self.position = pos #Defines icon position in the screen
        self.rect = self.image.get_rect()
    
        if pos == 0:
            #Play
            self.rect.x  = 0
            self.rect.y = SCREEN_HEIGHT - self.rect.height
        elif pos == 1:
            #Quit
            self.rect.x  = 0
            self.rect.y = score_title.rect.y + score_title.rect.height
        elif pos == 2:
            #Pause
            self.rect.x  = SCREEN_WIDTH - self.rect.width
            self.rect.y =  score_title.rect.y + score_title.rect.height
        elif pos == 3:
            #Resume
            self.rect.x  = SCREEN_WIDTH - self.rect.width
            self.rect.y = SCREEN_HEIGHT - self.rect.height

        types.add(self) #Adds sprites to the types group
#----------------------------------------------------------------------------------------

#Method to connect with each E-Puck by its 4 digit ID, taken from E-puck library for python
def initRobots(robot_id):

    #Check if the ID exist in the dictionary of MAC addresses
    if epucks.has_key(robot_id):
        log('Connecting with the ePuck')
        try:
            robot = ePuck(epucks[robot_id])

            robot.connect() #Method from epuck class that connects the e-puck

            log('Conection complete')
            log('Library version: ' + robot.version)

        except Exception, e:
            error(e)
            sys.exit(1)
    else:
        error('ID of robot not found in MAC addresses') #If the ID is not in the dictionary

    return robot

#Print if the connection with the e-puck was succesful, taken from E-puck library for python
def log(text):
    print(''.join(('\033', '[Log] ', str(text))))

#Print if the connection with the e-puck failed, taken from E-puck library for python
def error(text):
    print(''.join(('\033', '[Error] ', str(text))))

#Use mathematical morphologic operations to delete small areas form the robot image, ite defines the number of iterarions to clean
def deleting(im, ite):
    im = 1*(im <128)
    im_open = morphology.binary_opening(im,np.ones((2,2)),iterations=ite) #Deletes small areas
    im_close = morphology.binary_closing(im_open,np.ones((2,2)),iterations=4) #Make the areas bigger again
    return im_close 

#Deletes the position of trash from the objectives vector when it disappears, recieve the position of the trash in the objectives vector
def remove_trash_position(array_position):
    global sel #Vector to store the sequence of selections to fin the most common
    global freq #Stores the value of the most frequent selection in the first 10 selections, in case there is a mistake due to the 
    #3D pointing vector uncertainty showing no selection, if the correct selections of an objective is at least 7 from 10 the tentative selection is made
    global s #Selection counter
    global newZones #Vector with the positions of the trash in the screen
    global active_trash #Sprite group with all the trash in the screen
    
    #The objectives vector includes the 4 robots and the pause button (positioned from 0 to 4th index) so thats why 5 need to be substracted
    #so the position its remapped to the vector of only trash called newZones in case of being the first trash will be 5th index in the objectives and 0 in newZones
    newZones = np.delete(newZones, array_position-5, 1) #Deletes the trash coordinates from the vector, the argument 1 is to delete the column because coordinates X and Y are stored per column
    #EX: if the position deleted is 6th in objectives the its 6-5 =1 in newZones, and if current newZones = [coors0 , coors1, coors2, coors3] then the result will be newZones = [coors0, coors2, coors3]
   
    #sel is a vector that contains a sequence of the objectives selected if the most common value in the vector have enough repetitions (at least 7 from 10)
    #an objective can be selected, if there is a selection taking place while a trash is deleted the indexes that has been stored in the vector may change because one position was deleted
    #sel stored position start in 1 not in 0 because 0 indicates no selection, thats why 1 needs to be added
    sel[(sel == array_position + 1)] = 0 #if the vector have any selection of the deleted trash, the selection will be changed by 0 so theres nothing to select
    sel[(sel > array_position + 1)] -= 1 #if the vector have any selection of a trash ordered in the vector in a position after the deleted trash, the selection is
    #changed to -1 because the deleted trash will affect all the positions after it but not the ones before it
    #EX: the trash deleted is the 6th position in objectives so 7 in sel vector and current sel vector is [6 8 8 7 8 8 8 8 8 8 8] so the new sel will be [6 7 7 0 7 7 7 7 7 7 7]

    if freq[0][0] == (array_position+1): #freq[0][0] have the current tentative selection (the most common until the 10 iteration)
        s = 0 #If the most common is the one currently being deleted then the counter of selection will set to 0 so it restarts the selection
    elif freq[0][0] > (array_position+1): 
        freq = Counter(sel[0][0:9]).most_common(1) #If the most common is one position after the one being deleted then, the most common one is calculated again with the updated sel vector
    #If the most common is a position before the one being deleted nothing needs to be done because its position won't change

    for trash in active_trash: #Iterates all the trash in the screen
        if trash.array_position > array_position: #Checks if the position in the vector, each trash have stored, is located after the one being deleted
            trash.array_position -= 1 #Updates the trash position with its new location in the objectives vector

#Detects the robots position and orientation, arm is recieved only for plotting purposes in case of an ERROR
def detecting(depth, arm):
    global robots #Objects of each robot
    global firstLoop #Indicates if is the first loop of the game after starting or restarting
    global screen_robots #Sprite group with the robots screen position
    global all_sprites #Layered sprite group with all the sprites to be blitted
    global score #Saves the score points
    global score_text #Sprite with the number to blit with the points

    kernel = morp.disk(14) #Defines the size of the disk use for the equalization
    img_local = rank.equalize(depth, selem=kernel) #Equalize the image
    img_mean = filters.uniform_filter(img_local, size=3) #Applies local mean filter to make smaller the undesired areas
    toZoom = img_mean.copy() #Copy the image to then be able to cut only the epuck position from the image and fin it's orientation
    
    #Threshold the image with a value porportional to the tables average for the first time
    img_mean[(img_mean <= np.mean(depth[(depth > 0) & (depth < 2000)])/2.55)]= 255
    img_mean[(img_mean > np.mean(depth[(depth > 0) & (depth < 2000)])/2.55)]= 0

    # #Uncomment to plot images
    # pl.figure(figsize=(18,10))
    # pl.subplot(1,4,1)
    # pl.imshow(depth, vmin = 1700, vmax = 1900)
    # pl.colorbar()
    # pl.subplot(1,4,2)
    # pl.imshow(img_local)
    # pl.colorbar()
    # pl.subplot(1,4,3)
    # pl.imshow(toZoom)
    # pl.colorbar()
    # pl.subplot(1,4,4)
    # pl.imshow(img_mean)
    # pl.colorbar()
    # pl.show()
    # raw_input("Press Enter to terminate.")
    
    labels_bin, nbr_objects = measurements.label(img_mean) #Method to obtain the number of objects detected and the labels where each object is located
    regions = regionprops(labels_bin) #Allows to extract area, centroid and orientation

    area = 0 #Variable to store the biggest area
    for props in regions:
        if props.area > area:
            area = props.area

    img_mean = toZoom.copy() #Copy the image before being thresholded
    threshold = (31680/area)+402 #New threshold depending if the biggest area detected was to small or to big

    #The image is thresholded again with the new threshold
    img_mean[(img_mean <= threshold)]= 255
    img_mean[(img_mean > threshold)]= 0

    labels_bin, nbr_objects = measurements.label(img_mean) #Method to obtain the number of objects detected and the labels where each object is located
    regions = regionprops(labels_bin) #Allows to extract area, centroid and orientation

    #Image with a bigger threshold to have the e-pucks tail and find the orientation
    toZoom[(toZoom <= threshold+65)]= 0
    toZoom[(toZoom > threshold+65)]= 255

    area = 0 #Variable to store the biggest area of the new threshold
    for props in regions:
        if props.area > area:
            area = props.area

    ite = (np.rint(area/35)).astype(int) #Define the number of iterations to delete small areas proportionally to the biggest area detected
    #Thresholded image values are inverted 0->255 and 255->0 to correctly delete the small areas
    img_mean = img_mean/255
    img_mean[(img_mean == 0)]= 255
    img_mean[(img_mean == 1)]= 0
    real = deleting(img_mean, ite) #Method to delete the small areas

    labels_bin, nbr_objects = measurements.label(real) #Method to obtain the number of objects detected and the labels where each object is located
    regions = regionprops(labels_bin) #Allows to extract area, centroid and orientation

    # #Uncomment to plot images
    # pl.figure(figsize=(18,10))
    # pl.subplot(1,4,1)
    # pl.imshow(depth, vmin = 1700, vmax = 1900)
    # pl.colorbar()
    # pl.subplot(1,4,2)
    # pl.imshow(toZoom)
    # pl.colorbar()
    # pl.subplot(1,4,3)
    # pl.imshow(img_mean)
    # pl.colorbar()
    # pl.subplot(1,4,4)
    # pl.imshow(real)
    # pl.colorbar()
    # pl.show()
    # raw_input("Press Enter to terminate.")

    area = 0 #Variable to store the biggest area of the new image with less undesired areas
    for props in regions:
        if props.area > area:
            area = props.area

    lim = (np.rint(area*6/10)).astype(int) #Calculates de 60% of the biggest area detected
    
    k = 0 #Counter that defines the robot position from 0 to 3
    coors = np.zeros((2,4)) #Vector that stores the robots positions

    #Each loop the robots sprites are created
    all_sprites.remove(screen_robots.sprites())
    screen_robots.empty()

    for props in regions: 
        #If the detected area is less than the 60% of the biggest area is deleted
        if props.area <= lim: 
            for coord in props.coords:
                real[coord[0], coord[1]] = 0
        else:
            #Cuts an square where the robot is to find its orientation
            test = toZoom[props.centroid[0] - 12:props.centroid[0] + 12,props.centroid[1] - 12:props.centroid[1] + 12]
            
            test = 1*(test <128) 

            #POSSIBLE ERROR with 1 iteration the e-puck's tail is too small and with 0 can be joined to a small area nex to him
            test = morphology.binary_opening(test,np.ones((2,2)),iterations=1)
            
            # #Uncomment to plot images
            # pl.figure(figsize=(17,10))
            # pl.subplot(1,2,1)
            # pl.imshow(toZoom)
            # pl.colorbar()
            # pl.subplot(1,2,2)
            # pl.imshow(test)
            # pl.colorbar()
            # pl.show()
            # raw_input("Press Enter to terminate.")

            labels, nb = measurements.label(test) #Method to obtain the number of objects detected and the labels where each object is located

            try:
                closeUps = regionprops(labels) #Allows to extract area, centroid and orientation
            except Exception, e:
                print 'ROBOT IN RISK!!!!!!!!' #Robot it's too close to the edge so it couldn't be cut out when creating test
                print props.centroid[0], props.centroid[1]
                for robot in robots:
                    robot.set_motors_speed(0,0) #Stop the robots so they don't fall
                
                pl.figure(figsize=(15,10))
                pl.subplot(1,6,1)
                pl.imshow(arm, vmin = 1700, vmax = 1900)
                pl.colorbar()
                pl.subplot(1,6,2)
                pl.imshow(toZoom)
                pl.colorbar()
                pl.subplot(1,6,3)
                pl.imshow(img_local)
                pl.colorbar()
                pl.subplot(1,6,4)
                pl.imshow(img_mean)
                pl.colorbar()
                pl.subplot(1,6,5)
                pl.imshow(real)
                pl.colorbar()
                pl.subplot(1,6,6)
                pl.imshow(test)
                pl.colorbar()
                pl.show()
                raw_input("Press Enter to terminate.")

            ori = 0 #Variable to store the orientation of the robot
            ar = 0 #Variable to store the biggest area so the small undesired areas are ignored
            for closeUp in closeUps:
                if closeUp.area > ar:
                    ar = closeUp.area
                    ori = closeUp.orientation

            #More than 4 can be detected if something is at the same height of the robots like a han or other object
            if k >= 4:
                print 'More than 4 detected :('
                break #If theres an extra object detected it will be ignored

            loc = np.array([[props.centroid[0]],[props.centroid[1]]]) #Stores current robot position
            asign = loc.copy() #A copy is made so the robots position doesn't change the next time a robot's position is asigned to loc

            if firstLoop: #Run only the first loop after the game started
                robots[k].set_initialPos(asign) #Save the robots base position
                robots[k].set_coors(asign[:,0]) #Save the robots actual position in this case is the same
                robots[k].set_orientation(np.rad2deg(ori)) #Saves robot's orientation
                robots[k].set_test(test) #This is only for plotting purposes to see if the angle is right in case of necessary
                robots[k].set_cleaner_type(k) #Robots vector position also defines the type of trash he will clean 0=Paper, 1=Aluminium, 2=Plastic and 3=Glass
                robot_sprite = ScreenRobot(props.centroid[0], props.centroid[1], k) #Creates the sprite that will move with the robot
                robot_base = ScreenRobot(props.centroid[0], props.centroid[1], 4) #Creates the sprite that will stay in the base to avoid trash being positioned there
            else:
                for i in range(0,len(robots)):
                    coors[:,i] = robots[i].get_coors() #If is not the first loop the previous robots positions are extracted they will be in the same order first created 2486->0, 3047->1, 3067->2 and 3078->3

                distance = np.sqrt((props.centroid[0] - coors[0,0:4])**2 + (props.centroid[1] - coors[1,0:4])**2) #Calculates the distance between the actual position of a robot to the previous position of the 4 robots
                index = np.argmin(distance) #Extracts the position of the minimum distance (from 0 to 3 indicating the same robot position in Robots[]) indicating thats the robot currently being detected

                if robots[index].get_returning(): #This checks if the robot is currently coming back from his objective
                    for trash in active_trash: #Iterates all the trashes in the screen
                        if trash.cleaner_id == index: #Checks if there's a trash that is asigned to the robot
                            #Calculates the distance between the trash and the previous robot position already multiplied by relocate to have everything in screen coordinates
                            dif_x = trash.rect.x - robots[index].get_coors()[0]*relocate
                            dif_y = trash.rect.y - robots[index].get_coors()[1]*relocate
                            #Move the trash to a location separated from the new robot position exactly the same distance calculated before so the trash looks like its being pulled by the robot
                            trash.rect.x = props.centroid[0]*relocate + dif_x
                            trash.rect.y = props.centroid[1]*relocate + dif_y
                            break #One robot can only have asigned one trash so its not necessary to finish the FOR loop

                elif robots[index].get_arrived(): #Checks if the robot arrived to its base
                    for trash in active_trash: #Iterates all the trashes in the screen
                        if trash.cleaner_id == index: #Checks if the robot arrived with a trash
                            score += 1 #Add a point to the score
                            point_sound.play(loops = 0) #Plays the scoring sound, loops indicates how many times the sound is going to be repeated 0 means no repetition only played once
                            score_text.update_counter(score, BLUE) #Update the score text to be displayed in the screen
                            active_trash.remove(trash) #Removes the trash of the screen trash
                            all_sprites.remove(trash) #Removes the trash from the all active sprites to be blitted so it disappears from the screen
                            remove_trash_position(trash.array_position) #Removes the trash position from the objectives vector and update any current selection is being made
                            trash.cleaner_id = None #Deletes the previous robot ID
                            trash.decomposing = 0 #Sets the trash decomposition to 0 again in case of being displayed again
                            inactive_trash.add(trash) #Add the trash to inactive so it can be randomly picked in the available trash
                            break #One robot can only have asigned one trash so its not necessary to finish the FOR loop

                    robots[index].set_arrived(False) #Arrived is set to False when the robot have finish all his trajectory and points are updated in case its needed

                robots[index].set_coors(asign[:,0]) #The robot save its new position
                robots[index].set_orientation(np.rad2deg(ori)) #The robot save its new orientation
                robots[index].set_test(test) #This is only for plotting purposes to see if the angle is right in case of necessary
                robot_sprite = ScreenRobot(props.centroid[0], props.centroid[1], index) #Creates the sprite that will move with the robot
                
            k += 1 #Updates the robot counter (from 0 to 3)

    firstLoop = False #It have to be changed outside of the for loop because for each robot it have to enter to the firstloop case

    for i in range(0,len(robots)):
        coors[:,i] = robots[i].get_coors() #creates a vector with the robots position in order (from 0 to 3)

    labels_bin, nbr_objects = measurements.label(real) #Method to obtain the number of objects detected and the labels where each object is located

    #Less than 4 objects can be detected in case an arm its hiding the robot from the kinect
    if nbr_objects != 4: 
        print '!!!!!!!!!!!!!!Less than 4!!!!!!!!!!!!!' #Printed in the command line just for debugging purposes
        
        # #Uncomment to plot images
        # pl.figure(figsize=(15,10))
        # pl.subplot(1,5,1)
        # pl.imshow(arm, vmin = 1700, vmax = 1900)
        # pl.colorbar()
        # pl.subplot(1,5,2)
        # pl.imshow(img_local)
        # pl.colorbar()
        # pl.subplot(1,5,3)
        # pl.imshow(toZoom)
        # pl.colorbar()
        # pl.subplot(1,5,4)
        # pl.imshow(img_mean)
        # pl.colorbar()
        # pl.subplot(1,5,5)
        # pl.imshow(real)
        # pl.colorbar()
        # pl.show()
        # raw_input("Press Enter to terminate.")

    return coors #Return the vector with the 4 robots positions ordered

#Remove the last pointer from the screen
def clearing_selection():
    global selection_sprite #Sprite with a circle indicating where the user is pointing
    global all_sprites #Layered sprite group containig active sprites

    if selection_sprite is not None: #Check if there is any pointer active
        all_sprites.remove(selection_sprite) #Remove the pointer image from the group of sprites being blitted
        selection_sprite = None #The sprite its reseted

#This method blits the new pointer position
def updating_selection(radius, x, y, color):
    global selection_sprite #Sprite with a circle indicating where the user is pointing

    clearing_selection() #Remove the previous pointer

    selection_sprite = Selection(radius, x-radius, y-radius, color) #The coordinates need to be moved to the corner of the bounding box thats why its necessary to substract the radius
    selection_sprite.image.set_alpha(80) #This alpha value set the transparency of the pointer so the circle let the user see what is being slected behind

#This method allows to know the position in the objectives vector of the selected object by the user
def selecting(im, zones, floor):
    global s #Selection counter
    global sel #Vector to store the sequence of selections to fin the most common
    global freq #Stores the value of the most frequent selection in the first 10 selections, in case there is a mistake due to the 
    #3D pointing vector uncertainty showing no selection, if the correct selections of an objective is at least 7 from 10 the tentative selection is made
    global objective #Stores the position of the trash that was selected
    global freq2 #Stores the value of the most frequent selection in the 22 selections
    global cleaners #Save the current robot selected or pause button
    
    if s is None: 
        s = 0 #If the selection counter hasn't started then its initialized in 0
        sel = np.zeros((1,23)) #Sel will store all the sequence of selections to check if the selection is valid or the user change of selection
    elif (s == 0):
        sel = np.zeros((1,23)) #If the selection has been restarted the selection vector is emptied

    highArm = floor/2.27 #Threshold proportional to the floor average to find the highest part of the arm, a smaller value means its closer to the kinect so its higher
    lowArm = floor/1.48 #Threshold proportional to the floor average to find the lowest part of the arm, a bigger value means its closer to the floor so its lower
    hand = np.where((im >= highArm) & (im < lowArm)) #Variable that stores only the indexes of the values between that threshold
    
    if hand[0].any(): #Check if there is any value to see if there is an arm to analyze
        #Converts the depth values in mm to px, first the values of the arm are substracted from the lowest position posible (meaning the biggest depth value) to locate the depth 0 at the lowest point of the arm, 
        #the closest one to the robot. Then that mm value its multiplied for the reason between the width of the table in pixels and mm, each -3 is substracted due to 3 extra columns of pixels taken in each side of the table when cutting it
        han = (lowArm-(im[(im >= highArm) & (im < lowArm)]))*((len(im[0])-3-3)/tableWidth)
        
        # #Uncomment to plot image
        # pl.figure(figsize=(15,10))
        # pl.imshow(im, vmin = highArm-5, vmax = lowArm+5)
        # pl.colorbar()
        # pl.show()
        # raw_input("Press Enter to terminate.")
    
        #The image is thresholded so only the arm is left in the image
        im[((im < highArm) | (im >= lowArm))] = 0
        im[((im >= highArm) & (im < lowArm))] = 255
        labels_c, objects_c = measurements.label(im) #Method to obtain the number of objects detected and the labels where each object is located
        handc = regionprops(labels_c) #Allows to extract area, centroid and orientation
        ang = handc[0].orientation + constants.pi/2  #hand angle referenced to Y axis
        
        #Dictionary with the coordinates of the arm in pixels
        data = {'x': hand[0], 
            'y': hand[1], 
            'z': han}
        df = pd.DataFrame(data) #panda library organizes the data 

        slope = np.tan(ang) #slope of the 2D line of the arm
        intercept = ((handc[0].centroid)[1] - (np.tan(ang)*(handc[0].centroid)[0])) #intercepth of the 2D line of the arm using the angle and the centroid coordinates

        # #Uncomment to plot image of the 2D arm and the line created
        # y2 = hand[0]*slope + intercept 
        # pl.subplot(1,3,1)
        # pl.scatter((handc[0].centroid)[0],(handc[0].centroid)[1])
        # pl.subplot(1,3,2)
        # pl.scatter(hand[0],hand[1])
        # pl.subplot(1,3,3)
        # pl.plot(hand[0],y2)
        # pl.show()

        lm = smf.ols(formula = 'z ~ x + y', data = df).fit() #Stats model calculate the linear regression model in 3D
        #Parameters extracted from the model 0 = Intercept, 1 = A*(x), 2 = B*(y) 
        A = lm.params[1]
        B = lm.params[2]
        C = lm.params[0]

        #Uncomment to plot image of the 3D arm and the 3D line created
        # y2 = hand[0]*slope + intercept 
        # z = A*hand[0] + B*y2 + C
        # #z1 = A*hand[0] + B*slope*hand[0] + B*intercept + C
        # z1 = A*hand[0] + B*hand[1] + C 
        # fig = pl.figure(figsize=(15,10))
        # ax = fig.add_subplot(2,1,1, projection='3d')
        # ax.plot(hand[0], y2, z, label='Hand line')
        # ax.legend()
        # ax.set_xlabel('X axis')
        # ax.set_xlim(0, 255)
        # ax.set_ylabel('Y axis')
        # ax.set_ylim(0, 100) 
        # ax.set_zlabel('Z axis')
        # ax.set_zlim(10, 60) 
        # ax = fig.add_subplot(2,1,2, projection='3d')
        # ax.plot(hand[0],hand[1],z1)
        # ax.set_xlabel('X axis')
        # ax.set_xlim(0, 255) 
        # ax.set_ylabel('Y axis')
        # ax.set_ylim(0, 100)
        # ax.set_zlabel('Z axis')
        # ax.set_zlim(10, 60)
        # pl.show()
        # raw_input("Press Enter to terminate.")

        #Putting the 2 models together and defining Z = 0 we can find the X,Y coordinates the user is pointing
        y0 = (A*intercept - C*slope)/(A + B*slope)
        x0 = (y0 - intercept)/slope
        distance = np.sqrt((zones[0] - x0)**2 + (zones[1] - y0)**2) #Calculates the distance between the X,Y pointed and all the objectives
        minimum = np.min(distance) #Finds the closes objective to this coordinates
       
        if minimum <= 50: #Checks if the minimum distance is lower than 50 pixels
            #El orden de las zonas define el numero del objeto
            gr = 1 + np.argmin(distance) #Finds the position of the minimum distance and adds 1 because 0 is set when nothing is selected
            #EX: if the minimum distance is for the second robot its position will be 1-> 1+1 = 2 so the number for selection will be from 1 to the length of active objectives 
        else:
            gr = 0 # 0 when is nothing selected because the pointer is too far from any objective

        if s < 10:
            #When the user starts pointing the 10 first loops the size of the selector will be smaller (Radius = 27)
            updating_selection(27,(np.rint(x0*relocate)).astype(int),(np.rint(y0*relocate)).astype(int), RED) #The coordinates sent can't be decimal and they need to be relocated to screen coordinate system

        sel[0][s] = gr #The actual selection is added to the selection vector

        if s == 10: #Checks if the selection loop is the 11th
            freq = Counter(sel[0][0:9]).most_common(1) #Finds which is the most common number and how many times have been repeated
            if freq[0][0] != 0: #Checks if the most repeated value is not zero
                if freq[0][0] == freq2[0][0]: 
                    s = 0 #if the objective inmediatly selected before was the same no need to continue because is already selected
                    print 'RE-SELECTING', freq[0][0]
                else: 
                    if freq[0][1] < 7:
                        s = 0 #If the most common selection hasn't been at least selected 7 times the selection is restarted
                        print 'THE SELECTION IS NOT SUFFICIENT < 7'
                    else:
                        print 'PRE-SELECTION', freq[0][0]
                        s = s + 1 #If the selection has been repeated at least 7 times then an objective starts being selected and the selection pointer is bigger (radius s*3)
                        updating_selection(s*3,(zones[0][freq[0][0]-1]*relocate).astype(int),(zones[1][freq[0][0]-1]*relocate).astype(int), RED)
            else:
                s = 0 #If the most common is 0 the selection restarts
                print 'THERE IS NO SELECTION THE COUNTER IS RESETED'
        else:

            if s == 22:
                s = 0 #If selection counter reach 22 the selection will be restarted no matter a definitive selection is made or not
                freq2 = Counter(sel[0]).most_common(1) #Finds which is the most common number and how many times have been repeated
                if freq2[0][0] == freq[0][0]: #If the most common number is the same that was pre-selected then the selection is correct and continues
                    
                    if freq2[0][1] >= 16: #If the most common has at least being correctly selected 16 times from 22 the selection is correct and can continue
                        if freq[0][0] <= 5: #If the selection is less or equal than 5 then its a robot or the pause button
                            if (cleaners != 0) & (cleaners != 5): #If there is a new selection and the cleaners is not 0 (no objective) or 5 (pause button) 
                            #that means a robot was selected and not assigned and now the selection is being changed
                                for i in range(8):
                                    robots[(cleaners-1).astype(int)].set_led(i,0) # i goes from 0 to 7 to address all the leds and 0 to turn off the e-puck selected before
                            
                            print freq2[0][0], 'GROUP SELECTED!!!!!!!!!!!!!!!!!!!!'
                            cleaners = freq2[0][0] #The position of the new selected robot is stored
                            #This selecting method is called also in the start or pause menus so in case its pause 
                            #cleaners will be 5 or in case its start or game over menu, the length of the objectives are the buttons so the maximum is 3
                            if (len(zones[0]) >= 5) & (cleaners != 5):
                                for i in range(8): #If its not in a menu then the selected robot turn on his leds
                                    robots[(cleaners-1).astype(int)].set_led(i,1) # i goes from 0 to 7 to address all the leds and 0 to turn off the e-puck selected before
                        else:
                            #If the selection is more than 5 its a trash the one being selected
                            print freq2[0][0], 'OBJECTIVE SELECTED!!!!!!!!!!!!!!!!!!!!'
                            objective = freq2[0][0] #The position of the trash is stored
                        
                        #The selection is succesful so the objective is selected and the selection pointer is bigger (Radius=75) and green
                        updating_selection(75,(zones[0][freq[0][0]-1]*relocate).astype(int),(zones[1][freq[0][0]-1]*relocate).astype(int), GREEN)
                    else:
                        #If the selection doesn't have at least 16 repetitions then the selection is not efective the user didn't keep his selection constant
                        print 'ERROR: SELECTION NOT CONSTANT' 
                        freq2 = np.zeros((1,2)) #freq2 is reseted so when checking if its RE-SELECTING doesn't think he is because the selection wasn't effective
                else:
                    #If the selection is not the same as the one pre-selected then the user change of selection
                    print 'ERROR: DIFFERENT SELECTION'
                    freq2 = np.zeros((1,2))#freq2 is reseted so when checking if its RE-SELECTING doesn't think he is because the selection wasn't effective
            else:
                if s > 10:
                    #If the selection counter is greater than 10 it means an objective have been pre-selected so the selection is fixed to this objective until the selection is effective or not (s=22)
                    #the size of the pointer is proportional to the actual number of selections (radius = s*3)
                    updating_selection(s*3,(zones[0][freq[0][0]-1]*relocate).astype(int),(zones[1][freq[0][0]-1]*relocate).astype(int), RED)
                    
                s = s + 1 #Each loop if the counter is not 22 the counter is increased

    else:
        #If there is no arm to analyze the user is not selecting any more or changed its mind so the selection is cleared
        clearing_selection()
        s = 0 #The selection is restarted

    return cleaners #If there is a selection returns the robot position+1 or 0 if there is no selection

#This method allows to put a new trash in the screen correctly
def positioning_trash():
    global appearing_time #Time when the new trash should appear
    global offset_time #Miliseconds between each trash appearance
    global inactive_trash #Sprite group with all the trash sprites that are not in the screen
    global active_trash #All the trash that is in the screen
    global all_sprites #Layered sprite group containig active sprites
    global newZones #Vector with the positions of the trash in the screen

    relocate_trash = True #Variable to control if a trash was succesfully positioned or not

    running_time = ti.time()*1000 - start_time #Calculates the time game have been running in miliseconds

    if (running_time) >= appearing_time: #Checks if the current running time is greater or equal thant the asigned time for a new trash
        trash_sprite = random.choice(inactive_trash.sprites()) #A sprite of the inactive trash is randomly selected
        all_sprites.remove(contaminated_water_1) #The sprite with the conatminated water image is removed because if not there will be always a collition of any trash with this image

        while relocate_trash: 
            #Randomly ubicates the trash in the screen
            trash_sprite.rect.x = random.randrange(SCREEN_WIDTH)
            trash_sprite.rect.y = random.randrange(SCREEN_HEIGHT)

            #This method checks if the given sprite (trash_sprite) collides with any of the active sprites (all_sprites) the third argument is set to True if you want to remove from the group the sprites that collided with given sprite
            overlapping_list = pygame.sprite.spritecollide(trash_sprite, all_sprites, False)

            if not overlapping_list:
                relocate_trash = False #The while loop will continue until the trash is position without colliding with anything meaning the overlapping_list is empty

        if newZones is None: 
            newZones = np.zeros((2,1)) #If the vector with the trash positions hasn't been initialized then its set to zeros
            #The first trash is stored in the first column, the x+width/2 and y+heigth/2 is to save the center of the trash not the corner of the bounding box
            newZones[:,0] = np.array([[(trash_sprite.rect.x+trash_sprite.rect.width/2)/relocate],[(trash_sprite.rect.y+trash_sprite.rect.height/2)/relocate]])[:,0]
            i = 0 #Variable used to indicate the trash wich is its position
        else:
            i = len(newZones[0]) #The position of the actual trash will be the length of the vector if there is already 3 trashes (from position 0 to 2) then its position would be the 3rd
            aux = np.zeros((2,1)) #Auxiliar variable just to concatenate the trash position to the vector
            #The x+width/2 and y+heigth/2 is to save the center of the trash not the corner of the bounding box
            aux[:,0] = np.array([[(trash_sprite.rect.x+trash_sprite.rect.width/2)/relocate],[(trash_sprite.rect.y+trash_sprite.rect.height/2)/relocate]])[:,0]
            newZones = np.concatenate((newZones, aux), axis = 1) #Concatenates the auxiliar vector to the newZones and axis indicates to concatenate as a column

        inactive_trash.remove(trash_sprite) #After succesfully positioning the trash, it is removed from the inactive group so it can't be placed twice
        trash_sprite.time = ti.time()*1000 #This saves in each trash the time when it was positioned in the screen in miliseconds
        trash_sprite.array_position = i + 5 #The 5 is added because in the objective vector the trash will always start in the 5th position after the four robots (from 0 to 3) and the pause button (position 4)
        active_trash.add(trash_sprite) #The trash is added to the active trash
        all_sprites.add(trash_sprite, layer = trash_sprite.layer) #The trash is added to the sprites that are going to be blitted
        all_sprites.add(contaminated_water_1, layer = contaminated_water_1.layer) #The contaminated water image is placed back so it can be correctly blitted
        
        if offset_time <= 100: #Defines the minimum time between the appearances of trashes
            offset_time = 100
        else:
            #Defines how fast is going to appear next trash, if is greater than the minimum (100 miliseconds) then 0.1 seconds will be substracted in each appeareance, so next trash appear always faster than the before
            offset_time-=100

        appearing_time = running_time + offset_time #Sets the time for positioning a new trash adding the offset time to the time that the game have been running

#This method update the contamination points and the contamination image 
def updating_contamination():
    global active_trash #Sprites group with the trash in the screen
    global contamination #Contamination points
    global robots #All the robots objects
    global contaminated_water_1 #Sprite with the image of the contaminated water
    global contamination_text #Number of the contamination points

    for trash in active_trash: #Each trash in the screen is checked
        if (ti.time()*1000 - trash.time) >= 20000: #If the time since the trash was positioned or since it last contaminated the screen is greater or equal to 20 second is time to contaminate again 
            contamination += 1 #Adds one point of contamination to the contamination points
            contamination_text.update_counter(contamination, BLUE) #Updates the number to be displayd with the contamination points
            contaminated_water_1.image.set_alpha(contamination*5) #The factor (*5) defines how fast the water start getting contaminated
            trash.decomposing += 1 #Updates the decomposing counter of the trash
            trash.time = ti.time()*1000 #Updates the time of last contamination in miliseconds
            if trash.decomposing == 4: #Number of contaminations before totally decomposing and disappearing
                disappear_sound.play(loops = 0) #Plays a sound to indicate the trash was decomposed
                active_trash.remove(trash) #The trash is removed from the active trash
                all_sprites.remove(trash) #The trash is removed from the group of sprites to be blitted
                remove_trash_position(trash.array_position) #Remove the trash position from the trash vector and updates the selection
                trash.decomposing = 0 #Sets the decomposing value for the next time the trash is active again
                inactive_trash.add(trash) #The trash is added to the inactive trash so it can be randomly selected to be put in the screen again
                
                if trash.cleaner_id is not None: #Checks if the trash had an asigned robot to clean it
                    if not robots[trash.cleaner_id].get_returning(): #Checks if the robot is not coming back yet
                        robots[trash.cleaner_id].set_motors_speed(0, 0) #Stops the robot
                        #Reset the variables for the movement of the robot
                        robots[trash.cleaner_id].set_integral(0) 
                        robots[trash.cleaner_id].set_last_proportional(0)
                        robots[trash.cleaner_id].set_intDist(0)
                        robots[trash.cleaner_id].set_last_propDist(0)
                        robots[trash.cleaner_id].set_backwards(False)
                        robots[trash.cleaner_id].set_cnt(0)
                        robots[trash.cleaner_id].set_firstDist(0)
                        robots[trash.cleaner_id].set_moving(False)

                        robots[trash.cleaner_id].set_returning(True) #Set the robot to start coming back
                        robots[trash.cleaner_id].set_trash_picked(True) #Set as if the robot has already tried to pick up the trash
                    trash.cleaner_id = None #Delete the robots id from the trash so it becomes unasigned for the next time it is active

#This method detects if a robot has bumped into an other robot, or if they are too close
def detecting_collision():
    global screen_robots #Robots invisible sprites for collisions
    collision = None #Variable that save the position of the robot being checked in case there is a collition

    for robot in screen_robots: #Check every robot
        screen_robots.remove(robot) #First the root being checked is removed from the group so it doesn't collides with itself

        collision_list = pygame.sprite.spritecollide(robot, screen_robots, False) #Checks if there is any collition between the robot and 
        #the rest of the robot, third argument set to False so it doesn't eliminate the other robot of the group in case there is a collition
        
        if collision_list:
            collision = [robot.rect.x - robot.rect.width/2, robot.rect.y - robot.rect.height/2] #Saves the position of the robot centered not in the corner of the bounding box
            return collision #If there is one collition there is no need to check for more the game is over

        screen_robots.add(robot)

    return collision

#Main loop of the game, this method is repeated each iteration
def depth_frame_ready(frame):
    with screen_lock:
        global cleaners #Save the current robot selected or pause button
        global objective #Save the current trash selected
        global robots #Object of each robot
        global screen #Surface to blit on
        global active_trash #Sprite group with all the trash in the screen
        global relocate #Value to re-map from the screen px to the kinect px or vice versa
        global ripples #Sprite group with the active ripples in the game
        global start_time #Time when the game started or re-started
        global projection #Variable to control when to return to the game and start the splash sequence of images
        global start_counter #Counter to control wich splash image to blit
        global start #Variable that control if the game starts, gets paused or restarted
        global guide_size #Variable that controls the changing font size of the guide_text
        global paused #Variable that controls when the game is paused
        global restarting #Variable that controls when the game is re-starts
        global game_over #Variable that controls when the game is over
        global win #Variable that control when the user win
        global freq2 #Variable that stores the most common value of the selection
        global paused_time #Time in milliseconds the game have been paused
        global contaminated_water_image #Sprite that contains the image of the contaminated water
        global s #Selection counter
        global change #Checks if is time to increase or decrease the guide text font
        global collision #Variable that save the position of the robot being checked in case there is a collition
        
        #------------------------Taken from https://bitbucket.org/snippets/VitoGentile/Eoze/depth-data-visualization-with-pykinect-and------------------------
        address = surface_to_array(tmp_s)
        frame.image.copy_bits(address) #Copy raw data in a temp surface
        del address

        depth = (pygame.surfarray.pixels2d(tmp_s) >> 3) & 4095 #Get actual depth data in mm
        #------------------------------------------------------------------------------------------------------------------------------------------------------

        floor = np.mean(depth[depth > 2100]) #Mean value of the floor to use as reference for the game heights

        # #Uncomment to plot the whole depth image 
        # pl.imshow(depth)
        # pl.colorbar()
        # pl.show()
        # raw_input("Press Enter to terminate.")

        #------------------------------------------Table Cutting and Cleanning---------------------------------------------------------------------------------------------------------------------------------------------------
        #If there is a tall object before the table, to cut the table correctly the image checking will have to manually be set to start after the object position in the image and not in 0 (for columnI->160,X:120 and for rowI->X:160,120)
        #To cut the table it is assumed that the middle of the kinect image (160,120) is on the table and not the floor
        columnI = np.where((depth[160,0:120] >= floor/2.261) & (depth[160,0:120]<= floor/1.31)) #Checks in the row 160 from the column at the beggining of the image (0) to the column in the middle of the image (120), where pixels in the table range can be found
        rowI = np.where((depth[0:160,120] >= floor/2.261) & (depth[0:160,120] <= floor/1.31)) #Checks in the row 160 from the column at the beggining of the image (0) to the column in the middle of the image (120), where pixels in the table range can be found
        columnF = np.where((depth[rowI[0][0]+10,120:239]<= floor/2.261) | (depth[rowI[0][0]+10,120:239]>= floor/1.31)) #The final column is taken from a border of the table (rowI+10) so the hands or the body won't interfere in the cutting as they will if the row selected was the middle were the user is
        rowF = np.where((depth[160:319,((((columnF[0][0]+120)-columnI[0][0])/2)+columnI[0][0]).astype(int)]<= floor/2.261) | (depth[160:319,((((columnF[0][0]+120)-columnI[0][0])/2)+columnI[0][0]).astype(int)]>= floor/1.31)) #The final row is checked in the column just in the middle of the table (Middle between columnF and columnI)
        
        #Cuts the table from the image. Due to the fact that coulmnF and rowF doesn't start in 0 their starting point to check have to be added, the 3 extra added and the 3 substracted from 
        #rowI and columnI is to have 3 extra lines of pixels to let the kinect see the epuck a little closer to the edge, due to the 25x25 pixel square is cutted per robot for the orientation
        #The indexing is to find the first appeareance of the value searched (table depth for initials and floor depth for finals) the rowF doesn't takes the first appearance because there was some mistake in the detection in middle of the table not sure why
        depth = depth[rowI[0][0]-3:rowF[0][2]+163,columnI[0][0]-3:columnF[0][0]+123] 
        
        #Deletes the content of the 3 extra lines of pixels per side and clean also one line of the table in each side just to be sure there is no undesired pixel values in the table
        depth[0:4,:] = floor/1.2435
        depth[:,0:4] = floor/1.2435
        depth[len(depth)-4:len(depth),:] = floor/1.2435
        depth[:,len(depth[0])-4:len(depth[0])] = floor/1.2435
        #------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

        depth = np.flipud(depth) #The image the kinect give is mirrored so it is needed to flip it

        #The factor used for the relocation between the screen coordinates and the kinect coordinates needs the pixels of the projection of the screen (theoretically screen width but there is a mistake with the projection size don't know due to what and manually 
        #I set it lower so the location of robot's is more accurate), the length of the projection in mm, the table width in mm and also in the kinect pixels, the two -3 are due to the extra lines taken before
        relocate = 1220/(projectionLength*((len(depth[0])-3-3)/tableWidth)) 
        arm = depth.copy()#A copy is generated so it can keep the arm for selection

        depth[((depth <= floor/1.463 ) & (depth >= 0))] = floor/1.2435 #Threshold the image proportionallye to the floor so it deletes the arm

        # #Uncomment to plot the depth image ignoring the arm
        # pl.figure(figsize=(8,10))
        # pl.imshow(depth, vmin = 1700, vmax = 1900)
        # pl.colorbar()
        # pl.show()
        # raw_input("Press Enter to terminate.")
 
        if start: #Controls if the game continues or is paused

            #The pause button, the scores titles and their number and the icons of trash for the base of the e-puck, they all should only appear 
            #while playing not in the pause menu that's why they are blitted here, they all have the same layer to be blitted except the trash icons they should be under the trash
            all_sprites.add(pause, layer = pause.layer)
            all_sprites.add(texts, layer = pause.layer)
            all_sprites.add(titles, layer = pause.layer)
            all_sprites.add(types, layer = 1)

            coors = detecting(depth, arm) #Method that detects the position and orientation of the robots
            pause_location = np.array([[(pause.rect.x+pause.rect.width/2)/relocate],[(pause.rect.y+pause.rect.height/2)/relocate]]) #Defines the position of the pause button in the top centered
            coors = np.concatenate((coors, pause_location), axis = 1) #Adds the pause button to the possible objectives
            
            positioning_trash() #Method that activates a new trash
            updating_contamination() #Method that decompose each trash and the contamination points

            if cleaners != 0: #This checks if a robot have been selected, because only in this case a trash can be selected, if there is no robot selected no trash can be selected
                if newZones is not None: #Checks if there is any trash
                    coors = np.concatenate((coors, newZones), axis = 1) #Add the trash to the objectives vector

                # #Uncomment to check the projection and the robots position coherence, this will project green circle in each robot position
                # #It was used to calibrate the correct placement in the screen
                # for i in range(0,4):
                #     pygame.draw.circle(screen,(0,247,0),((coors[0][i]*relocate).astype(int),(coors[1][i]*relocate).astype(int)),75,0)
                #     pygame.display.update()
                #     #raw_input("Press Enter to terminate.")
                # pl.imshow(depth, vmin = 1700, vmax = 1900)
                # pl.colorbar()
                # pl.show()
                # raw_input("Press Enter to terminate.")

            cleaners = selecting(arm, coors, floor) #This method checks what is the selection of the user

            collision = detecting_collision() #This method checks if there is any collition between robots
            
            #Game is over if the contamination reach 40 points or if there is a collition
            if (contamination >= 40) | (collision is not None): 
                if collision is not None:
                    collision_sound.play(loops = 0) #In case of collition the collition sound is played, loops=0 indicates no repetitions only played once
                s = 0 #The game will change to a menu so the selection must be restart
                cleaners = 0 #Clean the previous selection to allow the buttons to be selected
                freq2 = np.zeros((1,2)) #Clean the previous most common selection to allow the buttons to be selected
                game_over_sound.play(loops = 0) #The game over sound is played, loops=0 indicates no repetitions only played once
                start = False #Stops the game to allow the menu to pop up
                game_over = True #Defines that the game is over so the right images and menu are displayed
                for robot in robots:
                    robot.set_motors_speed(0,0) #All the robots are stopped

            #If the user survives 3 minutes without getting to a critical contamination he wins
            elif (ti.time()*1000 - start_time - paused_time) >= 180000: #The time the game have been paused is deleted from the time it have been running so only the real playing time is taken in consideration
                s = 0 #The game will change to a menu so the selection must be restart
                cleaners = 0 #Clean the previous selection to allow the buttons to be selected
                freq2 = np.zeros((1,2)) #Clean the previous most common selection to allow the buttons to be selected
                win_sound.play(loops = 0) #The winning sound is played, loops=0 indicates no repetitions only played once
                start = False #Stops the game to allow the menu to pop up
                win = True #Defines that the user win so the right images and menu are displayed
                for robot in robots:
                    robot.set_motors_speed(0,0) #All the robots are stopped

            #Checks if the button paused was selected
            elif cleaners == 5:
                s = 0 #The game will change to a menu so the selection must be restart
                cleaners = 0 #Clean the previous selection to allow the buttons to be selected
                freq2 = np.zeros((1,2)) #Clean the previous most common selection to allow the buttons to be selected
                start = False #Stops the game to allow the menu to pop up
                paused = True #Defines that the game was paused so the right images and menu are displayed
                for robot in robots:
                    robot.set_motors_speed(0,0) #All the robots are stopped
            else:
                if objective is not None: #Checks if an objective was just selected
                    obj = np.zeros((2,1)) #Auxiliar variable to save the objective coordinates
                    obj[0] = coors[0, objective-1] #Objective row
                    obj[1] = coors[1, objective-1] #Objective column

                    for trash in active_trash: #Iterates all the trash in the screen
                        if trash.array_position == objective-1: #Checks if the trash have the position that was just selected, the -1 its because in selection the selected position starts in 1 not 0
                            trash.cleaner_id = (cleaners-1).astype(int) #If this is the trash selected then the robot selected position is saved in the trash (robot in charge of this trash)
                            break #No trash can share the same position so it is not necessary to finish the FOR loop
                    robots[(cleaners-1).astype(int)].set_objective(obj) #The robot saves the trash coordinates he have to clean
                    robots[(cleaners-1).astype(int)].set_cleaning(True) #Set the robot control variable true to indicate the robot started the cleaning process
                    cleaners = 0 #Once the robot have been asigned to a trash the robot is erased from the selection so next time his leds won't turn off
                    objective = None #Cleans the objective variable so it enter here to assign the trash to a robot only once per trash

                for i in range(0,len(robots)): #Iterates all the robots
                    if robots[i].get_cleaning(): #Checks if the robot is in the process of cleaning something so he will have to continue moving
                        ripple = Ripple(robots[i].get_coors()[0], robots[i].get_coors()[1]) #Creates a ripple in the current position of the robot

                        if robots[i].get_trash_picked(): #Checks if the robot just arrived to the trash
                            print '!!!!!!!!!!!!!!!!!!RETURNING!!!!!!!!!!!!!!!!!!'
                            robots[i].set_objective(robots[i].get_initialPos()) #Setss the robot new objective to its base so he goes back to his position to dispose the trash
                            robots[i].set_trash_picked(False) #Indicates the trash was already picked if is the correct or left there if is the incorrect, so each robot only enter in this loop once
                            for trash in active_trash: #Iterates all the trashin the screen
                                if trash.cleaner_id == i: #Finds the trash that is assigned to the robot
                                    if trash.trash_type != robots[i].get_cleaner_type(): #Checks if the robot type and the trash type are not the same
                                        trash.cleaner_id = None #If the type is incorrect the trash robot ID is deleted so it can be assigned to an other robot
                                        wrong_sound.play(loops = 0) #Sound that indicates that the trash type are not compatible
                                        break #One robot can't have more than one trash assigned at the same time so it is not necessary to finish the FOR loop
                        
                        robots[i] = robots[i].move(depth) #Method in the E-puck class that control the movement of the robot, the depth argument is only for plotting purposes in case of an ERROR

                for ripple in ripples: #Iterate all the ripples in the screen
                    ripple.update_ripple() #Update th ripple image to a bigger ripple in case a ripple image has already been blitted twice, so the movement of the ripples is not to fast
                    if not ripple.repeat:
                        ripple.repeat = True #Sets the actual ripple image to be repeated once more before being changed
                    else:
                        ripple.repeat = False #After showing the same ripple twice allows the ripple image to be changed

                screen.blit(background, (0, 0)) #Blit the background image
                all_sprites.draw(screen) #Blit every active sprite that sould appear in the screen
                pygame.display.update() #Actually updates the screen so everything appears
                all_sprites.remove(ripples) #Remove the ripples from the blitting group so it won't be taken in consideration for collisitions with trashes and because all ripple images can change and new ones can appear
        else: #In case the game is paused, the user won or lost
            if not restarting: #Checks if the game is not being restarted
                if paused: #Checks if the game was paused 
                    menu_locations = np.zeros((2,3)) #Creates an array to sent the pause menu buttons
                    pause_beginning = ti.time()*1000 #Saves the time the pause started
                    for button in buttons_2: #Iterates the buttons for the pause menu
                        menu_locations[:, button.position-4] = np.array([[(button.rect.x+button.rect.width/2)/relocate],[(button.rect.y+button.rect.height/2)/relocate]])[:,0] #Adds each button position to the objective vector already in kinect coordinate system and centered
                else:
                    menu_locations = np.zeros((2,2)) #Creates an array to sent the start, game over or win menu buttons
                    for button in buttons_1: #Iterates the buttons for the other menus
                        menu_locations[:, button.position-1] = np.array([[(button.rect.x+button.rect.width/2)/relocate],[(button.rect.y+button.rect.height/2)/relocate]])[:,0] #Adds each button position to the objective vector already in kinect coordinate system and centered

                cleaners = selecting(arm, menu_locations, floor) #Checks with button is being selected
                if cleaners == 1: #Checks if the first button is pressed
                    if game_over | win: #Checks if the current menu is of game over or winning screen
                        restarting = True #Sets the game to restart
                        #NEW!!!!!!!!!!!!
                        paused = False #Sets paused to false so in case of a previous pause it doesn't try to rest the paused time at the end, is not needed because the game will restart
                        for i in range(0,len(robots)): #Iterates all the robots
                            for j in range(8): #Iterates the robots leds
                                robots[i].set_led(j,0) #Turn off each led

                            if robots[i].get_objective() is not None: #Checks if the robot have an objective
                                initial = robots[i].get_initialPos() #Saves the robots initial position
                                actual = robots[i].get_coors() #Saves the robots current position
                                robots[i].reseting() #Reset robot variables
                                robots[i].set_initialPos(initial) #Assigns back the initial position
                                robots[i].set_coors(actual) #Assigns back the current position
                                robots[i].set_objective(robots[i].get_initialPos()) #Set the objective now to its base
                                robots[i].set_returning(True) #Set the robot to return
                    else: #In case the first button is selected in the start menu or the pause menu
                        projection = True #Variable that indicates if is time to start showing th splash sequence of images to start or continue the game
                        splash_sound.play(loops = 0) #Plays a splash sound
                        cleaners = 0 #Deletes the last selection so it doesn't affect the game selections when it starts      
                elif cleaners == 2: #The exit or X button is always number 2 regardless its screen position
                    pygame.quit() #Exits the game
                elif cleaners == 3: #Checks if the button selected is restart in the pause menu
                    restarting = True #Sets the game to restart
                    paused = False #Sets paused to false so in case of a previous pause it doesn't try to rest the paused time at the end, is not needed because the game will restart
                    for i in range(0,len(robots)): #Iterates all the robots
                        for j in range(8): #Iterates the robots leds
                            robots[i].set_led(j,0) #Turn off each led

                        if robots[i].get_objective() is not None: #Checks if the robot have an objective
                            initial = robots[i].get_initialPos() #Saves the robots initial position
                            actual = robots[i].get_coors() #Saves the robots current position
                            robots[i].reseting() #Reset robot variables
                            robots[i].set_initialPos(initial) #Assigns back the initial position
                            robots[i].set_coors(actual) #Assigns back the current position
                            robots[i].set_objective(robots[i].get_initialPos()) #Set the objective now to its base
                            robots[i].set_returning(True) #Set the robot to return
            else:
                coors = detecting(depth, arm) #Method that detects the robots position and orientation so the can be correctly move to their original position
                ready = 0 #Variable that save how many robots are ready to restart the game
                for i in range(0,len(robots)): #Iterates all the robots
                    if robots[i].get_objective() is None: #Checks if the robot don't have any objective he is moving towards
                        ready +=1 #If he doesn't have any objective it means he don't need to be redirected to it's base
                    else:
                        robots[i] = robots[i].move(depth) #If the robot have an objective is because he have to return so it is set to move

                if ready == 4: #Checks if the 4 robots are already in their base without objectives
                    initialization() #Set game variables to default
                    projection = True #Variable that indicates if is time to start showing th splash sequence of images to start or continue the game
                    splash_sound.play(loops = 0) #Plays a splash sound
                    for robot in robots:
                        robot.reseting() #Set robots variables to default

            if projection: #Checks if the sequence of splash images should begin so the game can start
                pygame.time.wait(10) #This allows the splash no to be too fast, the wait time is set to 10 miliseconds
                if start_counter < 13: #Checks if the splash image counter is not greater than the total number of splash images
                    screen.blit(sea_images[start_counter], (0, 0)) #Blits the splash image indicated by the counter
                    start_counter += 1 #Updates the image so next time the image is changed
                else:
                    pygame.time.wait(20) #Sets an extra wait time so the sea foam image of dipping into the water stays a little longer before starting
                    screen.blit(background, (0, 0)) #Blits the background image
                    start = True #Allows the game to start
                    start_counter = 0 #Sets the splash image sequence to 0 so next time a menu pops up it will start again in the first image
                    cleaners = 0 #Reset the las selection so it won't affect the game selection
                    s = 0 #Resets teh selection so a robot or trash can be selected correctly
                    freq2 = np.zeros((1,2)) #Clean the previous most common selection to allow the robots and trash to be selected

                    if not paused:
                        start_time = ti.time()*1000 #If the game is being restarted or just started, the initial time is set
                    else: 
                        paused = False #If it was a pause the sets that the pause is over so next time the menu pops up for a game over or win, it doesn't think is a pause

                    projection = False #Sets the splash sequence to finish so next time the menu pops up and the splash image doesn't starts inmediatly
            else: #If the game is not starting yet
                if restarting: #Checks if the game is in the restarting process
                    screen.blit(sea_images[0], (0, 0)) #Blit the menu background
                    #Locates the image of the word Restarting centered in the screen
                    rect = restarting_image.get_rect()
                    rect.x  = SCREEN_WIDTH/2 - rect.width/2
                    rect.y  = SCREEN_HEIGHT/2 - rect.height/2
                    screen.blit(restarting_image, (rect.x, rect.y)) #Blits the Restarting word image

                    if start_counter == 36: #Checks if the counter of loading image sequence reached its maximum
                        start_counter = 0 #Sets the sequence of loading image to restart

                    rect0 = loading[start_counter].get_rect() 
                    rect0.x  = rect.x + rect.width + 50 #Position the corner of the loading image next to the restarting image
                    rect0.y  = SCREEN_HEIGHT/2 - rect0.height/2 #Center the image in the Y axis
                    screen.blit(loading[start_counter], (rect0.x, rect0.y)) #Blits the loading image
                    start_counter += 1 #Updates the counter of the loading image sequence
                else: #If the game is not in the restarting process
                    if change: #Checks if is time to make the guide text smaller
                        guide_size -= 1 #Makes the font size of the guide text smaller
                        if guide_size == 35: #Checks if the size of the font is the minimum
                            change = False #Set change to false so the font size starts to increase again
                    else: 
                        guide_size += 1 #Makes the font size of the guide text bigger
                        if guide_size == 55: #Checks if the size of the font is the maximum
                            change = True #Set change to true so the font size starts to decrease again

                    if game_over: #Checks if the game is over
                        contaminated_water_image.set_alpha(255) #Sets the transparency to 255 so the image is fully visible
                        screen.blit(contaminated_water_image, (0, 0)) #Blits the contaminated water background
                        screen.blit(fishes, (0, 0)) #Blits dead fishes
                        if collision is not None: #Checks if there is a collition
                            screen.blit(collision_image, (collision[0], collision[1])) #Blit a collision image in the robot position
                        #Positions the game over title centered in the X axis and in the first fourth of the Y axis
                        rect1 = game_over_image.get_rect()
                        rect1.x  = SCREEN_WIDTH/2 - rect1.width/2 
                        rect1.y  = SCREEN_HEIGHT/4 - rect1.height/2
                        screen.blit(game_over_image, (rect1.x, rect1.y)) #Blits the game over title
                    elif win:
                        screen.blit(sea_images[0], (0, 0)) #Blits the usual menu background
                        #Positions the thank you title centered in the X axis and in the first fourth of the Y axis
                        rect1 = thanks.get_rect()
                        rect1.x  = SCREEN_WIDTH/2 - rect1.width/2
                        rect1.y  = SCREEN_HEIGHT/4 - rect1.height/2
                        screen.blit(thanks, (rect1.x, rect1.y)) #Blits the thank you title
                        #Positions the happy fish next to the title
                        rect = thanks_fish.get_rect()
                        rect.x  = rect1.x - rect.width - 20 #Locates the fish to the left of the title
                        rect.y  = rect1.y #Locates the fish at the same height of the title
                        screen.blit(thanks_fish, (rect.x, rect.y)) #Blits the fish
                    else:
                        screen.blit(sea_images[0], (0, 0)) #Blits the menu background
                        #Positions the game title centered in the X axis and in the first fourth of the Y axis
                        rect1 = title.get_rect()
                        rect1.x  = SCREEN_WIDTH/2 - rect1.width/2
                        rect1.y  = SCREEN_HEIGHT/4 - rect1.height/2
                        screen.blit(title, (rect1.x, rect1.y)) #Blits the game title

                    if paused: #Checks if the game was paused
                        buttons_2.draw(screen) #Blits the pause buttons
                    else:
                        buttons_1.draw(screen) #Blits the other buttons menu (Check and X button)
                    
                    font = pygame.font.Font(None, guide_size) #Defines the guide text font size so it changes to call the users attention
                    guide_text = font.render('Point to select an option', 1, BLACK) #Creates the text surface
                    #Positions the text centered in the X axis and in the third fourth of the Y axis
                    rect2 = guide_text.get_rect()
                    rect2.x  = SCREEN_WIDTH/2 - rect2.width/2
                    rect2.y  = SCREEN_HEIGHT*3/4 - rect2.height/2
                    screen.blit(guide_text, (rect2.x, rect2.y)) #Blits the guide text
                    
                    if selection_sprite is not None: #Checks if the user is pointing
                        screen.blit(selection_sprite.image, (selection_sprite.rect.x, selection_sprite.rect.y)) #Blits the pointer
            
            pygame.display.update() #Updates the screen so all the blitted images actually appear
            if paused: #Checks if the game was paused
                paused_time += (ti.time()*1000 - pause_beginning) #Adds the milliseconds that this pause loop took

#Method that sets game variables to default    
def initialization():
    global s #Selection counter
    global cleaners #Save the current robot selected or pause button
    global objective #Save the current trash selected
    global firstLoop #Indicates if is the first loop of the game after starting or restarting
    global offset_time #Miliseconds between each trash appearance
    global appearing_time #Time when the new trash should appear
    global inactive_trash #Sprite group with all the trash sprites that are not in the screen
    global active_trash #Sprite group with all the trash in the screen
    global all_sprites #Layered sprite group containig active sprites
    global contamination #Contamination points
    global score #Saves the score points
    global contaminated_water_1 #Sprite with the image of the contaminated water
    global ripples #Sprite group with the active ripples in the game
    global freq2 #Variable that stores the most common value of the selection
    global newZones #Vector with the positions of the trash in the screen
    global start #Variable that control if the game starts, gets paused or restarted
    global start_counter #Counter to control wich splash image to blit
    global projection #Variable to control when to return to the game and start the splash sequence of images
    global guide_size #Variable that controls the changing font size of the guide_text
    global change #Checks if is time to increase or decrease the guide text font
    global paused #Variable that controls when the game is paused
    global restarting #Variable that controls when the game is restarting
    global selection_sprite #Sprite with a circle indicating where the user is pointing
    global game_over #Variable that controls when the game is over
    global win #Variable that controls when the user win
    global contamination_text #Number of the contamination points
    global score_text #Sprite with the number to blit with the points
    global paused_time #Time in milliseconds the game have been paused
    global collision #Variable that save the position of the robot being checked in case there is a collition
    

    # Global variables initialization
    s = None
    cleaners = 0
    objective = None
    firstLoop = True
    contamination = 0
    score = 0
    freq2 = np.zeros((1,2))
    newZones = None
    start = False
    start_counter = 0
    projection = False
    guide_size = 35
    change = False
    paused = False
    game_over = False
    win = False
    paused_time = 0
    collision = None

    #Time for the trash first appearing 9s (9000 ms)
    offset_time = 9000
    #Initial time for next appeareance
    appearing_time = 9000

    selection_sprite = None

    if restarting:
        contaminated_water_1.image.set_alpha(contamination*5)
        score_text.update_counter(score, BLUE)
        contamination_text.update_counter(contamination, BLUE)
        ripples.empty()
        all_sprites.empty()
        all_sprites.add(walls, layer = 0)
        all_sprites.add(contaminated_water_1, layer = contaminated_water_1.layer)
        inactive_trash.add(active_trash)
        active_trash.empty()

    restarting = False

def main():
    # Initialize PyGame
    pygame.init()

    # Global variables values that need to be kept for more than one loop
    global screen #Surface to blit on
    global s #Selection counter
    global sel #Vector to store the sequence of selections to fin the most common
    global freq #Stores the value of the most frequent selection in the first 10 selections, in case there is a mistake due to the 
    #3D pointing vector uncertainty showing no selection, if the correct selections of an objective is at least 7 from 10 the tentative selection is made
    global cleaners #Save the current robot selected or pause button
    global robots #Object of each robot
    global objective #Save the current trash selected
    global firstLoop #Indicates if is the first loop of the game after starting or restarting
    global background #Background image of the sand floor in the sea
    global offset_time #Miliseconds between each trash appearance
    global start_time #Time when the game started or re-started
    global appearing_time #Time when the new trash should appear
    global screen_robots #Sprite group with the robots screen position
    global inactive_trash #Sprite group with all the trash sprites that are not in the screen
    global active_trash #Sprite group with all the trash in the screen
    global all_sprites #Layered sprite group containig active sprites
    global walls #Sprites group with the invisible walls in the screen
    global relocate #Value to re-map from the screen px to the kinect px or vice versa
    global contamination #Contamination points
    global score #Saves the score points
    global contaminated_water_1 #Sprite with the image of the contaminated water
    global contaminated_water_image #Sprite that contains the image of the contaminated water
    global texts #Sprites group with the points of the score and contamination to be displayed
    global score_text #Sprite with the number to blit with the points
    global contamination_text #Number of the contamination points
    global waves_sound #Sound of the whole game, sea waves
    global ripples #Sprite group with the active ripples in the game
    global ripples_images #Ripple sequence of images
    global freq2 #Variable that stores the most common value of the selection
    global newZones #Vector with the positions of the trash in the screen
    global start #Variable that control if the game starts, gets paused or restarted
    global sea_images #Splash sequence of images
    global start_counter #Counter to control wich splash image to blit
    global projection #Variable to control when to return to the game and start the splash sequence of images
    global buttons_1 #Sprite group with the buttons for start, win or game over screen (Check and X)
    global buttons_2 #Sprite group with the pause menu buttons (Resume, Restart and Exit)
    global splash_sound #Sound played when the game starts or continues after a menu
    global guide_size #Variable that controls the changing font size of the guide_text
    global change #Checks if is time to increase or decrease the guide text font
    global pause #Sprite with the button image
    global title #Image with the game title
    global paused #Variable that controls when the game is paused
    global loading #Loading sequence of images
    global restarting_image #Image with the word Restarting
    global restarting #Variable that controls when the game is restarting
    global fishes #Image with the dead fishes
    global win #Variable that controls when the user win
    global game_over #Variable that controls when the game is over
    global game_over_image #Game over title image
    global thanks #Thank you imae title
    global thanks_fish #Image of happy fish
    global game_over_sound #Sound played when the game is over
    global win_sound #Sound played when the user win
    global titles #Sprite group of the images of Score: and Contamination:
    global contamination_text #Sprite with the number to blit with the contamination points
    global disappear_sound #Sound played when a trash disappear
    global point_sound #Sound played when the trash was succesfully cleaned by the robot
    global wrong_sound #Sound when the robot tries to pick up a trash of a different type
    global score_title #Sprite with the image saying Score:
    global types #Sprite group with the images of icons in the bases of the robots indicating the type of trash they can clean
    global collision_sound #Sound played when a robot bump into an other
    global collision #Variable that save the position of the robot being checked in case there is a collition
    global collision_image #Image blitted when a robot bump into an other
    global paused_time #Time in milliseconds the game have been paused

    restarting = False
    trash_counter = 0

    #Set variables to default values
    initialization()

    #Connecting robots in the same order the kinect will detect them (left to right, top to bottom), keep in mind kinect image is going to be flip upside down 
    robots = [initRobots('2486'), initRobots('3047'), initRobots('3067'), initRobots('3078')]
    
    #Screen size
    screen = pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 24)
    
    #Screen title
    pygame.display.set_caption('Save the sea!')


    #------------------------------------Sprites groups-------------------------------
    #List of walls to avoid trash projected in the border of the window
    walls = pygame.sprite.Group()
    #List of the robots actual locations
    screen_robots = pygame.sprite.Group()
    #List of the ripples active
    ripples = pygame.sprite.Group()
    #List of the start buttons
    buttons_1 = pygame.sprite.Group()
    #List of the pause buttons
    buttons_2 = pygame.sprite.Group()
    #List of the points (Score and contamination) projected
    texts = pygame.sprite.Group()
    #List of the images of titles projected
    titles = pygame.sprite.Group()
    #Type of trash icons
    types = pygame.sprite.Group()
    #List with the trash not being projected
    inactive_trash = pygame.sprite.Group()
    #List with the trash currently projected
    active_trash = pygame.sprite.Group()
    #List of all sprites
    all_sprites = pygame.sprite.LayeredUpdates()
    #---------------------------------------------------------------------------------

    #--------------------------------------Image Loading------------------------------------------------------------------------------------------------
    background = pygame.image.load("Images/floor.png").convert_alpha()
    title = pygame.image.load("Images/save.png").convert_alpha()
    restarting_image = pygame.image.load("Images/restarting.png").convert_alpha()
    contamination_image = pygame.image.load("Images/contamination.png").convert_alpha()
    score_image = pygame.image.load("Images/score.png").convert_alpha()
    game_over_image = pygame.image.load("Images/gameover.png").convert_alpha()
    collision_image = pygame.image.load("Images/collision.png").convert_alpha()
    fishes = pygame.image.load("Images/fishes.png").convert_alpha()
    thanks = pygame.image.load("Images/thanks.png").convert_alpha()
    thanks_fish = pygame.image.load("Images/thanks_fish.png").convert_alpha()
    contaminated_water_image = pygame.image.load("Images/contaminated.png")
    
    #Ripple sequence of images
    ripples_images = [pygame.image.load("Images/Ripples/ripple0.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple1.png").convert_alpha(), 
                pygame.image.load("Images/Ripples/ripple2.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple3.png").convert_alpha(), 
                pygame.image.load("Images/Ripples/ripple4.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple5.png").convert_alpha(), 
                pygame.image.load("Images/Ripples/ripple6.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple7.png").convert_alpha(),
                pygame.image.load("Images/Ripples/ripple8.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple9.png").convert_alpha(),
                pygame.image.load("Images/Ripples/ripple10.png").convert_alpha(), pygame.image.load("Images/Ripples/ripple11.png").convert_alpha()]

    #Splash sequence of images
    sea_images = [pygame.image.load("Images/Splash/sea0.png").convert_alpha(), pygame.image.load("Images/Splash/sea1.png").convert_alpha(), 
                pygame.image.load("Images/Splash/sea2.png").convert_alpha(), pygame.image.load("Images/Splash/sea3.png").convert_alpha(), 
                pygame.image.load("Images/Splash/sea4.png").convert_alpha(), pygame.image.load("Images/Splash/sea5.png").convert_alpha(), 
                pygame.image.load("Images/Splash/sea6.png").convert_alpha(), pygame.image.load("Images/Splash/sea7.png").convert_alpha(),
                pygame.image.load("Images/Splash/sea8.png").convert_alpha(), pygame.image.load("Images/Splash/sea9.png").convert_alpha(),
                pygame.image.load("Images/Splash/sea10.png").convert_alpha(), pygame.image.load("Images/Splash/sea11.png").convert_alpha(),
                pygame.image.load("Images/Splash/sea12.png").convert_alpha()] 

    #Loading icon sequence of images
    loading = [pygame.image.load("Images/Loading/loading0.png").convert_alpha(), pygame.image.load("Images/Loading/loading1.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading2.png").convert_alpha(), pygame.image.load("Images/Loading/loading3.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading4.png").convert_alpha(), pygame.image.load("Images/Loading/loading5.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading6.png").convert_alpha(), pygame.image.load("Images/Loading/loading7.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading8.png").convert_alpha(), pygame.image.load("Images/Loading/loading9.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading10.png").convert_alpha(), pygame.image.load("Images/Loading/loading11.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading12.png").convert_alpha(), pygame.image.load("Images/Loading/loading13.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading14.png").convert_alpha(), pygame.image.load("Images/Loading/loading15.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading16.png").convert_alpha(), pygame.image.load("Images/Loading/loading17.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading18.png").convert_alpha(), pygame.image.load("Images/Loading/loading19.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading20.png").convert_alpha(), pygame.image.load("Images/Loading/loading21.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading22.png").convert_alpha(), pygame.image.load("Images/Loading/loading23.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading24.png").convert_alpha(), pygame.image.load("Images/Loading/loading25.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading26.png").convert_alpha(), pygame.image.load("Images/Loading/loading27.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading28.png").convert_alpha(), pygame.image.load("Images/Loading/loading29.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading30.png").convert_alpha(), pygame.image.load("Images/Loading/loading31.png").convert_alpha(),
                pygame.image.load("Images/Loading/loading32.png").convert_alpha(), pygame.image.load("Images/Loading/loading33.png").convert_alpha(), 
                pygame.image.load("Images/Loading/loading34.png").convert_alpha(), pygame.image.load("Images/Loading/loading35.png").convert_alpha()]
     
    #Buttons images
    play_image = pygame.image.load("Images/play.png").convert_alpha()
    close_image = pygame.image.load("Images/close.png").convert_alpha()
    pause_image = pygame.image.load("Images/pause.png").convert_alpha()
    resume_image = pygame.image.load("Images/resume.png").convert_alpha()
    exit_image = pygame.image.load("Images/exit.png").convert_alpha()
    restart_image = pygame.image.load("Images/restart.png").convert_alpha()
    #---------------------------------------------------------------------------------------------------------------------------------------------------


    #-----------------------------------Loading sounds--------------------------------
    splash_sound = pygame.mixer.Sound("Sounds/splash.wav")
    wrong_sound = pygame.mixer.Sound("Sounds/wrong.wav")
    win_sound = pygame.mixer.Sound("Sounds/win.wav")
    point_sound = pygame.mixer.Sound("Sounds/point.wav")
    collision_sound = pygame.mixer.Sound("Sounds/collision.wav")
    disappear_sound = pygame.mixer.Sound("Sounds/disappear.wav")
    game_over_sound = pygame.mixer.Sound("Sounds/gameover.wav")
    waves_sound = pygame.mixer.Sound("Sounds/waves.wav")
    waves_sound.play(loops = -1)
    #---------------------------------------------------------------------------------

    #-----------------------Initialization of some individual sprites---------------------------------------------------------------------
    
    #Game walls to avoid images too close to the border of the screen
    wall_1 = Wall(0, 0, SCREEN_WIDTH, 60)
    wall_2 = Wall(0, 0, 60, SCREEN_HEIGHT)
    wall_3 = Wall(0, SCREEN_HEIGHT-60, SCREEN_WIDTH, 60)
    wall_4 = Wall(SCREEN_WIDTH-60, 0, 60, SCREEN_HEIGHT)

    play = Button(play_image, 1)
    close = Button(close_image, 2)
    buttons_1.add(play,close)

    pause = Button(pause_image, 3)
    resume = Button(resume_image, 4)
    exit = Button(exit_image, 5)
    restart = Button(restart_image, 6)
    buttons_2.add(resume,restart,exit)

    score_title = Title(score_image, 1)
    contamination_title = Title(contamination_image, 2)

    score_text = Text(score_title.rect.x + score_title.rect.width + 10, pause.rect.height/2, BLUE, 1)
    contamination_text = Text(contamination_title.rect.x + contamination_title.rect.width + 10, pause.rect.height/2, BLUE, 2)

    contaminated_water_1 = ContaminationScreen(contaminated_water_image, contamination)
    #------------------------------------------------------------------------------------------------------------------------
    
    #Types of trash image for each e-puck
    type0 = Type(pygame.image.load("Images/type 0.png").convert_alpha(), 0)
    type1 = Type(pygame.image.load("Images/type 1.png").convert_alpha(), 1)
    type2 = Type(pygame.image.load("Images/type 2.png").convert_alpha(), 2)
    type3 = Type(pygame.image.load("Images/type 3.png").convert_alpha(), 3)

    #Position of the game title, includes width, height, x and y coordinates
    rect1 = title.get_rect()
    rect1.x  = SCREEN_WIDTH/2 - rect1.width/2
    rect1.y  = SCREEN_HEIGHT/4 - rect1.height/2

    #Text to indicate the user how to start
    font = pygame.font.Font(None, 35)
    guide_text = font.render('Point to select an option', 1, BLACK)
    #Position of the guide text
    rect2 = guide_text.get_rect()
    rect2.x  = SCREEN_WIDTH/2 - rect2.width/2
    rect2.y  = SCREEN_HEIGHT*3/4 - rect2.height/2

    #Blitting the START background
    screen.blit(sea_images[0], (0, 0))
    #Blitting game title
    screen.blit(title, (rect1.x, rect1.y)) 
    #Blitting Point to select an option
    screen.blit(guide_text, (rect2.x, rect2.y))
    #Blitting buttons
    buttons_1.draw(screen)
    #Blitting all the sprites
    all_sprites.draw(screen)
    #Updating screen to actually show changes
    pygame.display.update()
    
    #Uploads all the trash images wich are named only with number from 0 to 23 and are ordered by types PAPER, ALUMINIUM, PLASTIC and GLASS
    for i in range(24):
        image = pygame.image.load('Images/Trash/'+str(i)+'.png').convert_alpha()
        #Trash sprite created with image and number of position to classify the type of trash
        trash = Trash(image, trash_counter)
        #Counter to upload the next image
        trash_counter += 1

    with nui.Runtime() as kinect:
        
        kinect.depth_frame_ready += depth_frame_ready
        kinect.depth_stream.open(nui.ImageStreamType.Depth, 2, nui.ImageResolution.Resolution320x240, nui.ImageType.Depth)
        
        # Main game loop
        while True:
            event = pygame.event.wait()
            if event.type == pygame.QUIT:
                pygame.quit()

            if event.type == pygame.KEYDOWN:
                #Close the game when pressing Q
                if event.key == pygame.K_q:
                    pygame.quit()
                #Kinect camera angle can't be changed while hanging looking to the table it will move more than one degree to another fixed position
                #Increase kinect camera angle in 1 degree when W is pressed and prints the current angle in the command line
                elif event.key == pygame.K_w:
                    #kinect.camera.elevation_angle = kinect.camera.elevation_angle + 1
                    print kinect.camera.elevation_angle
                #Decrease kinect camera angle in 1 degree when S is pressed and prints the current angle in the command line
                elif event.key == pygame.K_s:
                    #kinect.camera.elevation_angle = kinect.camera.elevation_angle - 1 
                    print kinect.camera.elevation_angle
                #Set kinect camera angle to zero when Z is pressed and prints the current angle in the command line
                elif event.key == pygame.K_z:
                    #kinect.camera.elevation_angle = 0
                    print kinect.camera.elevation_angle

if __name__ == '__main__':
    main()